using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Http;
using System.Net.Http.Headers;
using System.Text.Json.Nodes;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;

namespace Rylogic.Audio.Authoring;

/// <summary>What is known about the Freesound connection, for display.</summary>
public sealed record FreesoundStatus(bool Configured, bool Connected);

/// <summary>The descriptive fields of a Freesound sound, read from its public page.</summary>
public sealed record FreesoundSoundInfo(string Number, string Title, string User, string Licence, string Url, string PreviewUrl);

/// <summary>
/// Freesound access for the authoring tool: public sound pages for browsing, and the OAuth2 API for downloading original recordings. The credentials
/// file is per user and outside version control; it holds 'client_id', 'client_secret' and, once connected, 'access_token', 'refresh_token' and
/// 'expires_at' (Unix seconds).
/// </summary>
public sealed partial class FreesoundClient
{
	/// <summary>The Freesound API root.</summary>
	public const string ApiRoot = "https://freesound.org/apiv2";

	/// <summary>The public site root.</summary>
	public const string SiteRoot = "https://freesound.org";

	private readonly HttpClient m_http;
	private readonly string? m_credentials_path;
	private readonly SemaphoreSlim m_token_lock = new(1, 1);

	/// <summary>Use 'http' for all requests and 'credentials_path' for the per-user credentials, or null when Freesound downloads are not configured.</summary>
	public FreesoundClient(HttpClient http, string? credentials_path)
	{
		m_http = http;
		m_credentials_path = credentials_path;
	}

	/// <summary>The current connection state. Reads only the local credentials file.</summary>
	public FreesoundStatus Status
	{
		get
		{
			// Connected means a token was obtained; an expired one is refreshed when it is next used
			var credentials = LoadCredentials();
			var configured = !string.IsNullOrEmpty(Text(credentials, "client_id")) && !string.IsNullOrEmpty(Text(credentials, "client_secret"));
			return new FreesoundStatus(configured, configured && !string.IsNullOrEmpty(Text(credentials, "refresh_token")));
		}
	}

	/// <summary>Store the API credentials, discarding any tokens obtained with earlier ones.</summary>
	public void SetCredentials(string client_id, string client_secret)
	{
		// Tokens belong to the credentials that obtained them
		var path = m_credentials_path ?? throw new InvalidOperationException("The project does not configure a Freesound credentials file.");
		Directory.CreateDirectory(Path.GetDirectoryName(path)!);
		SaveCredentials(new JsonObject { ["client_id"] = client_id.Trim(), ["client_secret"] = client_secret.Trim() });
	}

	/// <summary>The Freesound page where the user approves access; Freesound then redirects to the registered callback with a code.</summary>
	public string AuthorizeUrl()
	{
		// The callback address is registered with the credentials on Freesound, so it is not sent here
		var client_id = Text(LoadCredentials(), "client_id") ?? throw new InvalidOperationException("Enter the Freesound API credentials first.");
		return $"{ApiRoot}/oauth2/authorize/?client_id={Uri.EscapeDataString(client_id)}&response_type=code";
	}

	/// <summary>Exchange the code from the approval page for tokens.</summary>
	public async Task ConnectAsync(string code, CancellationToken cancel)
	{
		await m_token_lock.WaitAsync(cancel);
		try
		{
			await RequestTokenAsync(new() { ["grant_type"] = "authorization_code", ["code"] = code }, cancel);
		}
		finally
		{
			m_token_lock.Release();
		}
	}

	/// <summary>Read the title, author, licence and preview address of sound 'number' from its public page.</summary>
	public async Task<FreesoundSoundInfo> SoundInfoAsync(string number, CancellationToken cancel)
	{
		// The public page needs no login and carries everything shown while browsing
		if (!NumberRegex().IsMatch(number))
			throw new ArgumentException($"'{number}' is not a Freesound sound number.");

		var url = $"{SiteRoot}/s/{number}/";
		using var request = new HttpRequestMessage(HttpMethod.Get, url);
		request.Headers.UserAgent.ParseAdd(BrowserAgent);
		using var response = await m_http.SendAsync(request, cancel);
		response.EnsureSuccessStatusCode();
		var page = await response.Content.ReadAsStringAsync(cancel);

		// The licence link names the Creative Commons licence and version; 'zero' is the public-domain dedication
		var licence = "?";
		var lic = LicenceRegex().Match(page);
		if (lic.Success)
			licence = lic.Groups[2].Value == "zero" ? "CC0" : $"CC-{lic.Groups[2].Value.ToUpperInvariant()} {lic.Groups[3].Value}";

		// The page title ends with " by <user>", which is dropped because the user is shown separately
		var user_match = UserRegex().Match(page);
		var user = user_match.Success ? WebUtility.HtmlDecode(user_match.Groups[1].Value) : "?";
		var title_match = TitleRegex().Match(page);
		var title = title_match.Success ? WebUtility.HtmlDecode(title_match.Groups[1].Value) : number;
		if (title.EndsWith($" by {user}", StringComparison.Ordinal))
			title = title[..^$" by {user}".Length];

		// The high-quality preview is the low-quality link with a different suffix
		var preview = PreviewRegex().Match(page);
		if (!preview.Success)
			throw new InvalidDataException($"No preview was found on the page of Freesound sound {number}.");

		return new FreesoundSoundInfo(number, title, user, licence, url, preview.Value.Replace("-lq", "-hq", StringComparison.Ordinal));
	}

	/// <summary>Download a browse preview from 'preview_url' to 'path'.</summary>
	public async Task DownloadPreviewAsync(string preview_url, string path, CancellationToken cancel)
	{
		using var request = new HttpRequestMessage(HttpMethod.Get, preview_url);
		request.Headers.UserAgent.ParseAdd(BrowserAgent);
		await DownloadAsync(request, path, cancel);
	}

	/// <summary>Download the original recording of sound 'number' into 'folder' as '&lt;number&gt;.&lt;type&gt;' and return its path. Requires a connection.</summary>
	public async Task<string> DownloadOriginalAsync(string number, string folder, CancellationToken cancel)
	{
		// The file type decides the file name, so ask for it first
		var token = await AccessTokenAsync(cancel);
		using var info_request = new HttpRequestMessage(HttpMethod.Get, $"{ApiRoot}/sounds/{number}/?fields=type");
		info_request.Headers.Authorization = new AuthenticationHeaderValue("Bearer", token);
		using var info_response = await m_http.SendAsync(info_request, cancel);
		info_response.EnsureSuccessStatusCode();
		var info = JsonNode.Parse(await info_response.Content.ReadAsStringAsync(cancel));
		var type = info?["type"]?.GetValue<string>() ?? throw new InvalidDataException($"Freesound did not report the file type of sound {number}.");
		if (!FileTypeRegex().IsMatch(type))
			throw new InvalidDataException($"Freesound reported an unexpected file type '{type}' for sound {number}.");

		// Download the original itself
		var path = Path.Combine(folder, $"{number}.{type}");
		using var request = new HttpRequestMessage(HttpMethod.Get, $"{ApiRoot}/sounds/{number}/download/");
		request.Headers.Authorization = new AuthenticationHeaderValue("Bearer", token);
		await DownloadAsync(request, path, cancel);
		return path;
	}

	/// <summary>Send 'request' and write the response body to 'path' atomically.</summary>
	private async Task DownloadAsync(HttpRequestMessage request, string path, CancellationToken cancel)
	{
		// A partial download must never be mistaken for a complete file
		using var response = await m_http.SendAsync(request, HttpCompletionOption.ResponseHeadersRead, cancel);
		response.EnsureSuccessStatusCode();
		Directory.CreateDirectory(Path.GetDirectoryName(path)!);
		var temp = $"{path}.{Guid.NewGuid():N}.tmp";
		try
		{
			await using (var file = File.Create(temp))
				await response.Content.CopyToAsync(file, cancel);

			File.Move(temp, path, overwrite: true);
		}
		finally
		{
			File.Delete(temp);
		}
	}

	/// <summary>A valid access token, refreshing it when it is about to expire.</summary>
	private async Task<string> AccessTokenAsync(CancellationToken cancel)
	{
		// One refresh at a time, so concurrent downloads do not each spend the single-use refresh token
		await m_token_lock.WaitAsync(cancel);
		try
		{
			var credentials = LoadCredentials();
			var token = Text(credentials, "access_token");
			if (string.IsNullOrEmpty(Text(credentials, "refresh_token")) || token == null)
				throw new InvalidOperationException("Connect to Freesound before downloading originals.");

			var expires_at = credentials["expires_at"]?.GetValue<double>() ?? 0;
			if (expires_at - DateTimeOffset.UtcNow.ToUnixTimeSeconds() > 60)
				return token;

			return await RequestTokenAsync(new() { ["grant_type"] = "refresh_token", ["refresh_token"] = Text(credentials, "refresh_token")! }, cancel);
		}
		finally
		{
			m_token_lock.Release();
		}
	}

	/// <summary>Post a token request with 'fields' plus the client credentials, store the new tokens and return the access token.</summary>
	private async Task<string> RequestTokenAsync(Dictionary<string, string> fields, CancellationToken cancel)
	{
		// The client credentials accompany every token request
		var credentials = LoadCredentials();
		fields["client_id"] = Text(credentials, "client_id") ?? throw new InvalidOperationException("Enter the Freesound API credentials first.");
		fields["client_secret"] = Text(credentials, "client_secret") ?? throw new InvalidOperationException("Enter the Freesound API credentials first.");
		using var response = await m_http.PostAsync($"{ApiRoot}/oauth2/access_token/", new FormUrlEncodedContent(fields), cancel);
		var body = await response.Content.ReadAsStringAsync(cancel);
		if (!response.IsSuccessStatusCode)
			throw new InvalidOperationException($"Freesound refused the token request ({(int)response.StatusCode}). Connect to Freesound again.");

		// Keep the client credentials and replace the tokens
		var reply = JsonNode.Parse(body) ?? throw new InvalidDataException("Freesound returned an empty token reply.");
		var access = Text(reply, "access_token") ?? throw new InvalidDataException("Freesound's token reply has no access token.");
		credentials["access_token"] = access;
		credentials["refresh_token"] = Text(reply, "refresh_token");
		credentials["expires_at"] = DateTimeOffset.UtcNow.ToUnixTimeSeconds() + (reply["expires_in"]?.GetValue<double>() ?? 0);
		SaveCredentials(credentials);
		return access;
	}

	/// <summary>The credentials file contents, or an empty object.</summary>
	private JsonObject LoadCredentials()
	{
		return m_credentials_path != null && File.Exists(m_credentials_path) ? JsonNode.Parse(File.ReadAllText(m_credentials_path)) as JsonObject ?? [] : [];
	}

	/// <summary>Replace the credentials file.</summary>
	private void SaveCredentials(JsonObject credentials)
	{
		var path = m_credentials_path ?? throw new InvalidOperationException("The project does not configure a Freesound credentials file.");
		AuthoringJson.WriteAtomic(path, System.Text.Encoding.UTF8.GetBytes(credentials.ToJsonString()));
	}

	/// <summary>A string property of 'node', or null.</summary>
	private static string? Text(JsonNode? node, string name)
	{
		return node?[name] is JsonValue value && value.TryGetValue<string>(out var text) ? text : null;
	}

	/// <summary>The public site serves pages to ordinary browsers only.</summary>
	private const string BrowserAgent = "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/124.0 Safari/537.36";

	[GeneratedRegex(@"^\d+$")]
	private static partial Regex NumberRegex();

	[GeneratedRegex(@"^[a-z0-9]{1,8}$")]
	private static partial Regex FileTypeRegex();

	[GeneratedRegex(@"creativecommons\.org/(licenses|publicdomain)/([a-z\-]+)/([0-9.]+)")]
	private static partial Regex LicenceRegex();

	[GeneratedRegex("<meta property=\"og:title\" content=\"([^\"]*)\"")]
	private static partial Regex TitleRegex();

	[GeneratedRegex("<meta property=\"og:url\" content=\"https://freesound\\.org/people/([^/]+)/")]
	private static partial Regex UserRegex();

	[GeneratedRegex("https://cdn\\.freesound\\.org/previews/[^\"]+\\.mp3")]
	private static partial Regex PreviewRegex();
}
