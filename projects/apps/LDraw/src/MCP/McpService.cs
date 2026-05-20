using System;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Connections;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging;
using ModelContextProtocol.Server;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Utility;

namespace LDraw.MCP;

/// <summary>The lifecycle owner for the embedded LDraw MCP service</summary>
public sealed class McpService :IDisposable, INotifyPropertyChanged
{
	private readonly Model m_model;
	private readonly InstanceRegistry m_registry;
	private readonly InstancePipeClient m_client;
	private readonly SemaphoreSlim m_gate;
	private McpConfiguration m_configuration;
	private LDrawInstanceHost? m_instance_host;
	private WebApplication? m_broker_app;
	private Timer? m_reconfigure_timer;
	private Timer? m_retry_timer;
	private Timer? m_settings_reload_timer;
	private FileSystemWatcher? m_settings_watcher;
	private bool m_started;

	/// <summary>Create the MCP service controller for 'model'</summary>
	public McpService(Model model)
	{
		m_model = model;
		m_registry = new InstanceRegistry(model.StartupOptions.UserDataDir);
		m_client = new InstancePipeClient();
		m_gate = new SemaphoreSlim(1, 1);
		m_configuration = McpConfiguration.Disabled;
		StatusMessage = "MCP disabled";

		// Create the token up front so the Options UI can copy a complete client configuration without starting the server first.
		model.Settings.MCP.EnsureAccessToken();
		model.Settings.SettingChange += HandleSettingChange;
	}

	/// <summary>A user-facing status string for the Options UI</summary>
	public string StatusMessage
	{
		get => field;
		private set
		{
			if (field == value) return;
			field = value;
			NotifyPropertyChanged(nameof(StatusMessage));
		}
	} = string.Empty;

	/// <summary>Start the MCP service if enabled</summary>
	public async Task StartAsync()
	{
		await m_gate.WaitAsync().ConfigureAwait(false);
		try
		{
			if (m_started)
				return;

			m_started = true;
			StartSettingsWatcher();
		}
		finally
		{
			m_gate.Release();
		}

		// Apply settings outside the lock because the settings snapshot is marshalled to the UI thread.
		await ApplyCurrentSettingsAsync().ConfigureAwait(false);
	}

	/// <summary>Stop all MCP service activity</summary>
	public async Task StopAsync()
	{
		await m_gate.WaitAsync().ConfigureAwait(false);
		try
		{
			m_started = false;
			DisposeTimers();
			StopSettingsWatcher();
			await StopBrokerUnlockedAsync().ConfigureAwait(false);
			await StopInstanceUnlockedAsync().ConfigureAwait(false);
			StatusMessage = "MCP stopped";
		}
		finally
		{
			m_gate.Release();
		}
	}

	/// <summary>Dispose of the MCP service</summary>
	public void Dispose()
	{
		m_model.Settings.SettingChange -= HandleSettingChange;
		StopAsync().GetAwaiter().GetResult();
		m_gate.Dispose();
	}

	/// <summary>Handle changes to MCP settings</summary>
	private void HandleSettingChange(object? sender, SettingChangeEventArgs e)
	{
		if (e.Before)
			return;

		// Nested settings bubble through the root settings set, so only react to changes from the MCP set itself or replacement of the MCP set.
		if (!ReferenceEquals(e.SettingSet, m_model.Settings.MCP) && e.Key != nameof(SettingsData.MCP))
			return;

		QueueReconfigure();
	}

	/// <summary>Queue a debounced reconfiguration pass</summary>
	private void QueueReconfigure()
	{
		m_reconfigure_timer ??= new Timer(_ => _ = ApplyCurrentSettingsAsync(), null, Timeout.InfiniteTimeSpan, Timeout.InfiniteTimeSpan);
		m_reconfigure_timer.Change(TimeSpan.FromMilliseconds(500), Timeout.InfiniteTimeSpan);
	}

	/// <summary>Apply the current in-memory MCP settings</summary>
	private async Task ApplyCurrentSettingsAsync()
	{
		try
		{
			// Read the mutable settings on the UI thread once, then use the immutable snapshot below.
			var configuration = await CaptureSettingsAsync().ConfigureAwait(false);

			await m_gate.WaitAsync().ConfigureAwait(false);
			try
			{
				if (!m_started)
					return;

				var previous = m_configuration;
				m_configuration = configuration;

				// Disabled means no registry entry and no broker. Copilot should see no endpoint while the option is off.
				if (!configuration.Enabled)
				{
					DisposeRetryTimer();
					await StopBrokerUnlockedAsync().ConfigureAwait(false);
					await StopInstanceUnlockedAsync().ConfigureAwait(false);
					StatusMessage = "MCP disabled";
					return;
				}

				// Keep the instance pipe available for diagnostics, but do not attempt to bind Kestrel to an invalid port.
				if (!configuration.ValidPort)
				{
					DisposeRetryTimer();
					await StopBrokerUnlockedAsync().ConfigureAwait(false);
					await StartInstanceUnlockedAsync().ConfigureAwait(false);
					StatusMessage = $"MCP enabled, but port {configuration.Port} is invalid.";
					return;
				}

				// Every enabled process hosts its own pipe and registry entry; only the successful port binder becomes the broker.
				await StartInstanceUnlockedAsync().ConfigureAwait(false);
				if (m_broker_app != null && previous.Port == configuration.Port)
				{
					StatusMessage = $"MCP broker running at {configuration.Endpoint}";
					return;
				}

				// A port change requires rebuilding Kestrel because the listen endpoint is fixed at startup.
				await StopBrokerUnlockedAsync().ConfigureAwait(false);
				await TryStartBrokerUnlockedAsync(configuration).ConfigureAwait(false);
			}
			finally
			{
				m_gate.Release();
			}
		}
		catch (Exception ex)
		{
			Log.Write(ELogLevel.Error, ex, "Failed to configure LDraw MCP service.");
			StatusMessage = $"MCP configuration failed: {ex.Message}";
		}
	}

	/// <summary>Capture the MCP settings on the UI thread</summary>
	private Task<McpConfiguration> CaptureSettingsAsync()
	{
		return m_model.InvokeAsync(() =>
		{
			var settings = m_model.Settings.MCP;
			var token = settings.EnsureAccessToken();
			return new McpConfiguration(settings.Enabled, settings.Port, token);
		}, TimeSpan.FromSeconds(2));
	}

	/// <summary>Start the local instance host if needed</summary>
	private Task StartInstanceUnlockedAsync()
	{
		if (m_instance_host == null)
		{
			m_instance_host = new LDrawInstanceHost(m_model, m_registry);
			m_instance_host.Start();
		}

		return Task.CompletedTask;
	}

	/// <summary>Stop the local instance host if it is running</summary>
	private async Task StopInstanceUnlockedAsync()
	{
		if (m_instance_host == null)
			return;

		await m_instance_host.StopAsync().ConfigureAwait(false);
		Util.Dispose(ref m_instance_host);
	}

	/// <summary>Attempt to become the broker by binding the configured localhost port</summary>
	private async Task TryStartBrokerUnlockedAsync(McpConfiguration configuration)
	{
		try
		{
			// Broker election is the localhost port bind. This avoids needing a separate lock file or persisted "broker" flag.
			var local_instance_id = m_instance_host?.InstanceId ?? throw new InvalidOperationException("The LDraw MCP instance host is not running.");
			var broker = new McpBroker(m_registry, m_client, local_instance_id, () => m_configuration.Endpoint);

			// Build an isolated ASP.NET Core host inside the WPF process. Ignore inherited URL configuration and bind only to loopback.
			var builder = WebApplication.CreateBuilder(new WebApplicationOptions
			{
				Args = [],
				ApplicationName = typeof(McpService).Assembly.FullName,
			});
			builder.Logging.ClearProviders();
			builder.Configuration["AllowedHosts"] = "localhost;127.0.0.1";
			builder.WebHost.UseKestrel(options => options.ListenLocalhost(configuration.Port));
			builder.WebHost.UseSetting(WebHostDefaults.SuppressStatusMessagesKey, "true");

			// The broker is registered as a singleton because the SDK creates tool instances via dependency injection.
			builder.Services.AddSingleton(broker);
			builder.Services
				.AddMcpServer()
				.WithHttpTransport(options =>
				{
					// The first MCP slice is read-only and does not require server-initiated notifications.
					options.Stateless = true;
				})
				.WithTools<LDrawTools>(McpJson.Options);

			// Authentication is deliberately outside the MCP SDK so all MCP JSON-RPC methods use the same local bearer-token gate.
			var app = builder.Build();
			app.Use(AuthenticateRequestAsync);
			app.MapMcp("/mcp");
			await app.StartAsync().ConfigureAwait(false);

			m_broker_app = app;
			DisposeRetryTimer();
			StatusMessage = $"MCP broker running at {configuration.Endpoint}";
			Log.Write(ELogLevel.Info, $"LDraw MCP broker started at {configuration.Endpoint}");
		}
		catch (Exception ex) when (IsAddressInUse(ex))
		{
			// Another LDraw instance is already the broker. Stay registered and periodically retry so failover happens after it exits.
			ScheduleBrokerRetry();
			StatusMessage = $"MCP instance registered. Another broker is using port {configuration.Port}.";
		}
	}

	/// <summary>Stop the broker HTTP host if it is running</summary>
	private async Task StopBrokerUnlockedAsync()
	{
		if (m_broker_app == null)
			return;

		var app = m_broker_app;
		m_broker_app = null;
		try
		{
			// Keep shutdown bounded because this can run during application close.
			using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(1));
			await app.StopAsync(cts.Token).ConfigureAwait(false);
			await app.DisposeAsync().ConfigureAwait(false);
			Log.Write(ELogLevel.Info, "LDraw MCP broker stopped.");
		}
		catch (Exception ex)
		{
			Log.Write(ELogLevel.Warn, ex, "Stopping the LDraw MCP broker failed.");
		}
	}

	/// <summary>Schedule a retry so this instance can become broker after the current broker exits</summary>
	private void ScheduleBrokerRetry()
	{
		m_retry_timer ??= new Timer(_ => _ = ApplyCurrentSettingsAsync(), null, Timeout.InfiniteTimeSpan, Timeout.InfiniteTimeSpan);
		m_retry_timer.Change(TimeSpan.FromSeconds(5), TimeSpan.FromSeconds(5));
	}

	/// <summary>Dispose of the broker retry timer</summary>
	private void DisposeRetryTimer()
	{
		m_retry_timer?.Dispose();
		m_retry_timer = null;
	}

	/// <summary>Dispose all timers owned by this service</summary>
	private void DisposeTimers()
	{
		m_reconfigure_timer?.Dispose();
		m_reconfigure_timer = null;
		DisposeRetryTimer();
		m_settings_reload_timer?.Dispose();
		m_settings_reload_timer = null;
	}

	/// <summary>Start watching the settings file for changes from other LDraw instances</summary>
	private void StartSettingsWatcher()
	{
		if (m_settings_watcher != null)
			return;

		// Multiple LDraw processes can share the same settings file; watching it keeps token and port changes in sync.
		var filepath = m_model.StartupOptions.SettingsPath;
		var dir = Path.GetDirectoryName(filepath);
		var file = Path.GetFileName(filepath);
		if (dir == null || file.Length == 0)
			return;

		m_settings_watcher = new FileSystemWatcher(dir, file)
		{
			NotifyFilter = NotifyFilters.LastWrite | NotifyFilters.Size | NotifyFilters.FileName,
			EnableRaisingEvents = true,
		};
		m_settings_watcher.Changed += HandleSettingsFileChanged;
		m_settings_watcher.Created += HandleSettingsFileChanged;
		m_settings_watcher.Renamed += HandleSettingsFileChanged;
	}

	/// <summary>Stop watching the settings file</summary>
	private void StopSettingsWatcher()
	{
		if (m_settings_watcher == null)
			return;

		m_settings_watcher.Changed -= HandleSettingsFileChanged;
		m_settings_watcher.Created -= HandleSettingsFileChanged;
		m_settings_watcher.Renamed -= HandleSettingsFileChanged;
		Util.Dispose(ref m_settings_watcher);
	}

	/// <summary>Handle external changes to the settings file</summary>
	private void HandleSettingsFileChanged(object sender, FileSystemEventArgs e)
	{
		// Settings saves can emit multiple file notifications. Debounce so one save produces one reload.
		m_settings_reload_timer ??= new Timer(_ => _ = ReloadMcpSettingsFromFileAsync(), null, Timeout.InfiniteTimeSpan, Timeout.InfiniteTimeSpan);
		m_settings_reload_timer.Change(TimeSpan.FromMilliseconds(750), Timeout.InfiniteTimeSpan);
	}

	/// <summary>Reload MCP settings written by another LDraw process</summary>
	private async Task ReloadMcpSettingsFromFileAsync()
	{
		try
		{
			var loaded = new SettingsData(m_model.StartupOptions.SettingsPath);

			// Copy only MCP fields from the file so an external save does not unexpectedly replace the active profile or UI layout.
			await m_model.InvokeAsync(() =>
			{
				m_model.Settings.MCP.Enabled = loaded.MCP.Enabled;
				m_model.Settings.MCP.Port = loaded.MCP.Port;
				m_model.Settings.MCP.AccessToken = loaded.MCP.AccessToken;
				return 0;
			}, TimeSpan.FromSeconds(2)).ConfigureAwait(false);
		}
		catch (Exception ex)
		{
			Log.Write(ELogLevel.Warn, ex, "Reloading LDraw MCP settings failed.");
		}
	}

	/// <summary>Authenticate one HTTP MCP request</summary>
	private async Task AuthenticateRequestAsync(HttpContext context, Func<Task> next)
	{
		if (!context.Request.Path.StartsWithSegments("/mcp"))
		{
			await next().ConfigureAwait(false);
			return;
		}

		// Reject browser-originated requests that do not come from loopback or VS Code. This defends against DNS rebinding.
		if (!IsAllowedOrigin(context.Request.Headers.Origin.FirstOrDefault()))
		{
			context.Response.StatusCode = StatusCodes.Status403Forbidden;
			return;
		}

		// The server also binds to loopback, but host validation keeps absolute URL generation and proxy behaviour constrained.
		if (!IsLocalHost(context.Request.Host.Host))
		{
			context.Response.StatusCode = StatusCodes.Status400BadRequest;
			return;
		}

		// The token is copied into the user's MCP client configuration and is required even for localhost requests.
		if (!IsAuthorised(context.Request.Headers.Authorization.FirstOrDefault()))
		{
			context.Response.StatusCode = StatusCodes.Status401Unauthorized;
			context.Response.Headers.WWWAuthenticate = "Bearer";
			return;
		}

		await next().ConfigureAwait(false);
	}

	/// <summary>Return true if the HTTP Origin header is allowed</summary>
	private static bool IsAllowedOrigin(string? origin)
	{
		if (string.IsNullOrWhiteSpace(origin))
			return true;
		if (!Uri.TryCreate(origin, UriKind.Absolute, out var uri))
			return false;
		if (uri.Scheme.Equals("vscode", StringComparison.OrdinalIgnoreCase))
			return true;
		return IsLocalHost(uri.Host);
	}

	/// <summary>Return true if 'host' is a loopback host name</summary>
	private static bool IsLocalHost(string? host)
	{
		return
			string.Equals(host, "localhost", StringComparison.OrdinalIgnoreCase) ||
			string.Equals(host, "127.0.0.1", StringComparison.OrdinalIgnoreCase) ||
			string.Equals(host, "[::1]", StringComparison.OrdinalIgnoreCase) ||
			string.Equals(host, "::1", StringComparison.OrdinalIgnoreCase);
	}

	/// <summary>Return true if 'authorization' contains the current bearer token</summary>
	private bool IsAuthorised(string? authorization)
	{
		const string prefix = "Bearer ";
		if (authorization == null || !authorization.StartsWith(prefix, StringComparison.OrdinalIgnoreCase))
			return false;

		var provided = authorization.Substring(prefix.Length).Trim();
		var expected = m_configuration.AccessToken;
		if (provided.Length == 0 || expected.Length == 0)
			return false;

		var lhs = Encoding.UTF8.GetBytes(provided);
		var rhs = Encoding.UTF8.GetBytes(expected);

		// Compare in constant time so local timing does not reveal token prefixes.
		return lhs.Length == rhs.Length && CryptographicOperations.FixedTimeEquals(lhs, rhs);
	}

	/// <summary>Return true if 'ex' indicates the configured broker port is already in use</summary>
	private static bool IsAddressInUse(Exception ex)
	{
		for (var e = ex; e != null; e = e.InnerException!)
		{
			switch (e)
			{
				case AddressInUseException:
				{
					return true;
				}
				case SocketException se when se.SocketErrorCode == SocketError.AddressAlreadyInUse:
				{
					return true;
				}
			}
		}
		return false;
	}

	/// <inheritdoc/>
	public event PropertyChangedEventHandler? PropertyChanged;

	/// <summary>Raise property changed for 'prop_name'</summary>
	private void NotifyPropertyChanged(string prop_name)
	{
		PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}

	/// <summary>An immutable snapshot of MCP settings</summary>
	private sealed class McpConfiguration
	{
		public static McpConfiguration Disabled { get; } = new(false, McpSettingsData.DefaultPort, string.Empty);

		/// <summary>Create an immutable configuration snapshot</summary>
		public McpConfiguration(bool enabled, int port, string access_token)
		{
			Enabled = enabled;
			Port = port;
			AccessToken = access_token;
		}

		public bool Enabled { get; }
		public int Port { get; }
		public string AccessToken { get; }
		public bool ValidPort
		{
			get { return Port is >= 1 and <= 65535; }
		}
		public string Endpoint
		{
			get { return $"http://127.0.0.1:{Port}/mcp"; }
		}
	}
}
