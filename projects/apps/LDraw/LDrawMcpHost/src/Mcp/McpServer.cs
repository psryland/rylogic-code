using System;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
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

namespace LDraw.MCP.Host;

/// <summary>
/// Owns the loopback HTTP MCP endpoint for the tray host.
/// The host is the sole broker: it serves 'tools/list' from the reflected LDrawTools
/// catalog without any LDraw running, and forwards 'tools/call' to a target LDraw
/// process over its named pipe. Unlike the previous in-LDraw service there is no port
/// election or failover - the host owns the port and a bind clash is a hard error.
///
/// Security/threat model (local developer tool):
/// - Trust boundary is the Windows user account. The endpoint binds 127.0.0.1 only, and
///   the named pipes use CurrentUserOnly, so only same-user processes can reach either side.
///   A same-user process can already drive the user's apps, so this grants no new authority.
/// - 'tools/call' requires the pre-shared access token (constant-time compared, DPAPI at rest);
///   discovery (initialize/ping/*list) stays open so clients can enumerate tools before configuring.
/// - Origin and Host headers are validated to defend a browser-based DNS-rebinding attacker that
///   tries to reach the loopback endpoint; non-loopback Host or unexpected Origin is rejected.
/// - Request bodies are size-bounded (Kestrel) and pipe request lines are length-bounded so a
///   malformed or hostile local client cannot force an unbounded allocation.
/// - Not defended: another process running as the same user (out of scope by design); network
///   peers (never bound off loopback).</summary>
public sealed class McpServer :IDisposable
{
	private readonly HostSettings m_settings;
	private readonly InstanceRegistry m_registry;
	private readonly InstancePipeClient m_client;
	private readonly InstanceLauncher m_launcher;
	private readonly CancellationTokenSource m_lifetime;
	private readonly SemaphoreSlim m_gate;
	private WebApplication? m_app;

	/// <summary>Create the MCP endpoint host for 'settings'</summary>
	public McpServer(HostSettings settings)
	{
		m_settings = settings;
		m_registry = new InstanceRegistry(settings.UserDataDir);
		m_client = new InstancePipeClient();
		m_launcher = new InstanceLauncher(settings);
		m_lifetime = new CancellationTokenSource();
		m_gate = new SemaphoreSlim(1, 1);
		StatusMessage = "MCP host stopped";
	}

	/// <summary>A user-facing status string for the tray UI</summary>
	public string StatusMessage { get; private set; }

	/// <summary>Raised when <see cref="StatusMessage"/> changes</summary>
	public event EventHandler? StatusChanged;

	/// <summary>Start listening on the configured loopback port</summary>
	public async Task StartAsync()
	{
		await m_gate.WaitAsync().ConfigureAwait(false);
		try
		{
			if (m_app != null)
				return;

			if (!m_settings.Listening)
			{
				SetStatus("MCP host disabled");
				return;
			}
			if (!m_settings.ValidPort)
			{
				SetStatus($"MCP host error: port {m_settings.Port} is invalid.");
				return;
			}

			await StartUnlockedAsync().ConfigureAwait(false);
		}
		finally
		{
			m_gate.Release();
		}
	}

	/// <summary>Stop listening</summary>
	public async Task StopAsync()
	{
		await m_gate.WaitAsync().ConfigureAwait(false);
		try
		{
			await StopUnlockedAsync().ConfigureAwait(false);
			SetStatus("MCP host stopped");
		}
		finally
		{
			m_gate.Release();
		}
	}

	/// <summary>Apply changed settings by restarting the endpoint (e.g. after a port or listening change)</summary>
	public async Task RestartAsync()
	{
		await m_gate.WaitAsync().ConfigureAwait(false);
		try
		{
			await StopUnlockedAsync().ConfigureAwait(false);

			if (!m_settings.Listening)
			{
				SetStatus("MCP host disabled");
				return;
			}
			if (!m_settings.ValidPort)
			{
				SetStatus($"MCP host error: port {m_settings.Port} is invalid.");
				return;
			}

			await StartUnlockedAsync().ConfigureAwait(false);
		}
		finally
		{
			m_gate.Release();
		}
	}

	/// <summary>Dispose the host endpoint</summary>
	public void Dispose()
	{
		// Cancel the host-lifetime token first so any in-flight auto-launch wait unblocks instead of holding teardown.
		try { m_lifetime.Cancel(); } catch (ObjectDisposedException) { }

		// Stop/dispose the ASP.NET Core host on a thread-pool thread, never the caller's thread.
		// Dispose is reached from WPF OnExit on the UI thread; invoking the host's async teardown there
		// lets an internal continuation capture the Dispatcher SynchronizationContext and post back to the
		// UI thread, which is blocked waiting here - a deadlock. Running on the pool avoids that context.
		// Bound the wait so process shutdown can never hang: any remaining host threads are background,
		// so abandoning a stalled teardown still lets the process exit cleanly.
		try
		{
			var stop = Task.Run(StopAsync);
			if (!stop.Wait(TimeSpan.FromSeconds(5)))
			{
				System.Diagnostics.Trace.TraceWarning("LDraw MCP host endpoint did not stop within 5s; abandoning teardown.");

				// Observe the abandoned task's exception so it is not raised as unobserved on finalization.
				_ = stop.ContinueWith(t => _ = t.Exception, TaskContinuationOptions.OnlyOnFaulted);
				return;
			}
		}
		catch (Exception ex)
		{
			System.Diagnostics.Trace.TraceWarning($"LDraw MCP host endpoint teardown failed: {ex.GetBaseException().Message}");
		}
		m_gate.Dispose();
		m_lifetime.Dispose();
	}

	/// <summary>Build and start the Kestrel MCP endpoint</summary>
	private async Task StartUnlockedAsync()
	{
		try
		{
			var broker = new McpBroker(m_registry, m_client, m_settings, m_launcher, m_lifetime.Token);

			// Build an isolated ASP.NET Core host. Ignore inherited URL configuration and bind only to loopback.
			var builder = WebApplication.CreateBuilder(new WebApplicationOptions
			{
				Args = [],
				ApplicationName = typeof(McpServer).Assembly.FullName,
			});
			builder.Logging.ClearProviders();
			builder.Configuration["AllowedHosts"] = "localhost;127.0.0.1";
			builder.WebHost.UseKestrel(options =>
			{
				options.ListenLocalhost(m_settings.Port);

				// Bound request bodies on the always-on endpoint so a malformed or hostile local client cannot
				// stream an unbounded body; the limit comfortably exceeds the largest legitimate tool payload.
				options.Limits.MaxRequestBodySize = 64L * 1024 * 1024;
			});
			builder.WebHost.UseSetting(WebHostDefaults.SuppressStatusMessagesKey, "true");

			// The broker is a singleton because the SDK creates tool instances via dependency injection.
			builder.Services.AddSingleton(broker);
			builder.Services
				.AddMcpServer()
				.WithHttpTransport(options =>
				{
					// The read-only tool slice does not require server-initiated notifications.
					options.Stateless = true;
				})
				.WithTools<LDrawTools>(McpJson.Options);

			// Authentication is outside the MCP SDK so every MCP JSON-RPC method uses the same local bearer-token gate.
			var app = builder.Build();
			app.Use(AuthenticateRequestAsync);
			app.MapMcp("/mcp");
			await app.StartAsync().ConfigureAwait(false);

			m_app = app;
			SetStatus($"MCP host running at {m_settings.Endpoint}");
		}
		catch (Exception ex) when (IsAddressInUse(ex))
		{
			// The host owns the port; a clash means a stale LDraw broker or another host is bound. This is a hard error, not failover.
			SetStatus($"MCP host error: port {m_settings.Port} is already in use.");
			throw new InvalidOperationException($"LDraw MCP host cannot bind loopback port {m_settings.Port}; it is already in use.", ex);
		}
	}

	/// <summary>Stop and dispose the Kestrel host if running</summary>
	private async Task StopUnlockedAsync()
	{
		if (m_app == null)
			return;

		var app = m_app;
		m_app = null;
		try
		{
			// Bound graceful stop; a hung in-flight request must not stall shutdown.
			using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(1));
			await app.StopAsync(cts.Token).ConfigureAwait(false);
		}
		catch (Exception ex)
		{
			// A stop timeout or transient error must not prevent the host from disposing and exiting.
			System.Diagnostics.Trace.TraceWarning($"Stopping the LDraw MCP host endpoint failed: {ex.Message}");
		}
		finally
		{
			// WebApplication.DisposeAsync can stall on a service's async disposal; bound it so teardown completes.
			var dispose = app.DisposeAsync().AsTask();
			if (await Task.WhenAny(dispose, Task.Delay(TimeSpan.FromSeconds(2))).ConfigureAwait(false) != dispose)
			{
				System.Diagnostics.Trace.TraceWarning("Disposing the LDraw MCP host endpoint timed out; continuing shutdown.");

				// Observe the abandoned dispose task's exception so it is not raised as unobserved.
				_ = dispose.ContinueWith(t => _ = t.Exception, TaskContinuationOptions.OnlyOnFaulted);
			}
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

		// The token is required for tool calls; discovery (initialize/ping/list) stays open so clients can enumerate tools.
		if (!IsAuthorised(context))
		{
			if (await IsUnauthorisedDiscoveryRequestAsync(context).ConfigureAwait(false))
			{
				await next().ConfigureAwait(false);
				return;
			}

			await WriteAccessDeniedAsync(context).ConfigureAwait(false);
			return;
		}

		await next().ConfigureAwait(false);
	}

	/// <summary>Return true if an unauthorised MCP request is safe discovery-only traffic</summary>
	private static async Task<bool> IsUnauthorisedDiscoveryRequestAsync(HttpContext context)
	{
		if (!HttpMethods.IsPost(context.Request.Method) || context.Request.ContentLength == 0)
			return false;

		context.Request.EnableBuffering();
		using var reader = new StreamReader(context.Request.Body, Encoding.UTF8, detectEncodingFromByteOrderMarks: false, leaveOpen: true);
		var content = await reader.ReadToEndAsync().ConfigureAwait(false);
		context.Request.Body.Position = 0;

		try
		{
			using var json = JsonDocument.Parse(content);
			return json.RootElement.ValueKind switch
			{
				JsonValueKind.Object => IsUnauthorisedDiscoveryRequest(json.RootElement),
				JsonValueKind.Array => json.RootElement.EnumerateArray().All(IsUnauthorisedDiscoveryRequest),
				_ => false,
			};
		}
		catch (JsonException)
		{
			return false;
		}
	}

	/// <summary>Return true if a JSON-RPC request element is allowed before the access token is configured in the client</summary>
	private static bool IsUnauthorisedDiscoveryRequest(JsonElement request)
	{
		if (!request.TryGetProperty("method", out var method_element) || method_element.ValueKind != JsonValueKind.String)
			return false;

		var method = method_element.GetString();
		return method switch
		{
			"initialize" => true,
			"notifications/initialized" => true,
			"ping" => true,
			"tools/list" => true,
			"resources/list" => true,
			"resources/templates/list" => true,
			"prompts/list" => true,
			_ => false,
		};
	}

	/// <summary>Reject MCP access without triggering OAuth client flows in MCP clients</summary>
	private static async Task WriteAccessDeniedAsync(HttpContext context)
	{
		// LDraw uses a local pre-shared token, not OAuth. Some MCP clients treat HTTP 401 as an OAuth challenge even without WWW-Authenticate,
		// so use 403 with an explicit diagnostic message when the local token is missing or invalid.
		context.Response.StatusCode = StatusCodes.Status403Forbidden;
		context.Response.ContentType = "text/plain; charset=utf-8";
		await context.Response.WriteAsync($"Missing or invalid {McpProtocol.AccessTokenHeaderName} header. Copy the MCP configuration from the LDraw MCP host.").ConfigureAwait(false);
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

	/// <summary>Return true if 'context' contains the current local MCP access token</summary>
	private bool IsAuthorised(HttpContext context)
	{
		var provided = context.Request.Headers[McpProtocol.AccessTokenHeaderName].FirstOrDefault()?.Trim();
		if (string.IsNullOrWhiteSpace(provided))
		{
			const string prefix = "Bearer ";
			var authorization = context.Request.Headers.Authorization.FirstOrDefault();
			if (authorization == null || !authorization.StartsWith(prefix, StringComparison.OrdinalIgnoreCase))
				return false;

			provided = authorization.Substring(prefix.Length).Trim();
		}

		var expected = m_settings.AccessToken;
		if (provided.Length == 0 || expected.Length == 0)
			return false;

		var lhs = Encoding.UTF8.GetBytes(provided);
		var rhs = Encoding.UTF8.GetBytes(expected);

		// Compare in constant time so local timing does not reveal token prefixes.
		return lhs.Length == rhs.Length && CryptographicOperations.FixedTimeEquals(lhs, rhs);
	}

	/// <summary>Return true if 'ex' indicates the configured port is already in use</summary>
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

	/// <summary>Update the status message and notify listeners</summary>
	private void SetStatus(string message)
	{
		StatusMessage = message;
		StatusChanged?.Invoke(this, EventArgs.Empty);
	}
}
