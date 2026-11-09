using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;

namespace Rylogic.Audio.Authoring;

/// <summary>A sound already reviewed for a sound ID, given to the assistant as an example of what suits it or does not.</summary>
public sealed record ReviewedSound(string Title, string? Note);

/// <summary>
/// The prompts and reply handling used to find new Freesound candidates for one sound ID. The assistant first writes search queries from the sound's
/// catalog entry and review history; the results are then given back to it to choose the best few.
/// </summary>
public static class CandidateFinder
{
	/// <summary>The most search queries used per search.</summary>
	public const int MaxQueries = 4;

	/// <summary>The results requested from Freesound per query.</summary>
	public const int ResultsPerQuery = 15;

	/// <summary>The most results given to the assistant to choose from, which keeps the prompt within the command-line limit.</summary>
	public const int MaxChoices = 40;

	/// <summary>The most candidates added per search.</summary>
	public const int MaxPicks = 5;

	/// <summary>The prompt that asks for search queries for the sound whose catalog entry is 'sound_json'.</summary>
	public static string QueryPrompt(string sound_json, IReadOnlyList<ReviewedSound> liked, IReadOnlyList<ReviewedSound> disliked)
	{
		// The catalog entry describes the need; the reviewed sounds show the author's taste
		var prompt = new StringBuilder();
		prompt.AppendLine("You help choose sound effects for a video game from Freesound.org.");
		prompt.AppendLine("The game needs this sound (its catalog entry, as JSON):");
		prompt.AppendLine(sound_json);
		AppendReviewed(prompt, liked, disliked);
		prompt.AppendLine($"Write up to {MaxQueries} different Freesound text search queries, each 1 to 4 words, that are likely to find more recordings suitable for this sound.");
		prompt.AppendLine("Reply with only a JSON object and no other text, in this form: {\"queries\":[\"query one\",\"query two\"]}");
		return prompt.ToString();
	}

	/// <summary>The queries in the assistant's reply to <see cref="QueryPrompt"/>: distinct, non-empty, and at most <see cref="MaxQueries"/>.</summary>
	public static IReadOnlyList<string> QueriesOf(JsonObject reply)
	{
		var queries = reply["queries"] as JsonArray ?? throw new InvalidOperationException("The assistant's reply has no 'queries' list.");
		return queries
			.Select(x => x is JsonValue v && v.TryGetValue<string>(out var s) ? s.Trim() : string.Empty)
			.Where(x => x.Length != 0)
			.Distinct(StringComparer.OrdinalIgnoreCase)
			.Take(MaxQueries)
			.ToList();
	}

	/// <summary>
	/// The results that could become new candidates: each Freesound sound once, in the order found, excluding every number in 'known', and at most
	/// <see cref="MaxChoices"/>.
	/// </summary>
	public static IReadOnlyList<FreesoundSearchResult> ChoicesOf(IEnumerable<FreesoundSearchResult> results, IReadOnlySet<string> known)
	{
		var seen = new HashSet<string>(known, StringComparer.Ordinal);
		return results.Where(x => seen.Add(x.Number)).Take(MaxChoices).ToList();
	}

	/// <summary>The prompt that asks the assistant to choose the best of 'choices' for the sound whose catalog entry is 'sound_json'.</summary>
	public static string PickPrompt(string sound_json, IReadOnlyList<ReviewedSound> liked, IReadOnlyList<ReviewedSound> disliked, IReadOnlyList<FreesoundSearchResult> choices)
	{
		// Each choice is one compact JSON line; long descriptions and tag lists are cut because the title and first tags carry most of the meaning
		var prompt = new StringBuilder();
		prompt.AppendLine("You help choose sound effects for a video game from Freesound.org.");
		prompt.AppendLine("The game needs this sound (its catalog entry, as JSON):");
		prompt.AppendLine(sound_json);
		AppendReviewed(prompt, liked, disliked);
		prompt.AppendLine("These Freesound recordings were found, one JSON object per line:");
		foreach (var choice in choices)
		{
			var line = new JsonObject
			{
				["number"] = choice.Number,
				["title"] = choice.Title,
				["seconds"] = Math.Round(choice.Duration, 1),
				["tags"] = new JsonArray([.. choice.Tags.Take(12).Select(x => (JsonNode)x)]),
				["description"] = choice.Description.Length > 160 ? choice.Description[..160] : choice.Description,
			};
			prompt.AppendLine(line.ToJsonString());
		}
		prompt.AppendLine($"Choose up to {MaxPicks} recordings that would best suit this sound, best first. Consider whether each can loop or be cut to the needed length.");
		prompt.AppendLine("Choose none rather than a poor match.");
		prompt.AppendLine("Reply with only a JSON object and no other text, in this form: {\"picks\":[\"number\",\"number\"]}");
		return prompt.ToString();
	}

	/// <summary>The choices the assistant picked in its reply to <see cref="PickPrompt"/>, best first. Numbers that were not offered are ignored.</summary>
	public static IReadOnlyList<FreesoundSearchResult> PicksOf(JsonObject reply, IReadOnlyList<FreesoundSearchResult> choices)
	{
		// Numbers may come back as JSON numbers or strings
		var picks = reply["picks"] as JsonArray ?? throw new InvalidOperationException("The assistant's reply has no 'picks' list.");
		return picks
			.Select(x => x is JsonValue v ? (v.TryGetValue<string>(out var s) ? s.Trim() : v.ToJsonString()) : string.Empty)
			.Distinct(StringComparer.Ordinal)
			.Select(n => choices.FirstOrDefault(c => c.Number == n))
			.OfType<FreesoundSearchResult>()
			.Take(MaxPicks)
			.ToList();
	}

	/// <summary>Describe the sounds already reviewed for the sound ID.</summary>
	private static void AppendReviewed(StringBuilder prompt, IReadOnlyList<ReviewedSound> liked, IReadOnlyList<ReviewedSound> disliked)
	{
		// Notes are the author's own words about why a sound was kept or rejected
		static string Describe(ReviewedSound x)
		{
			return x.Note is { Length: > 0 } note ? $"- {JsonSerializer.Serialize(x.Title)} (note: {JsonSerializer.Serialize(note)})" : $"- {JsonSerializer.Serialize(x.Title)}";
		}

		if (liked.Count != 0)
		{
			prompt.AppendLine("Recordings already accepted for this sound, which show what the author likes:");
			foreach (var x in liked)
				prompt.AppendLine(Describe(x));
		}
		if (disliked.Count != 0)
		{
			prompt.AppendLine("Recordings rejected for this sound, which show what the author does not want:");
			foreach (var x in disliked)
				prompt.AppendLine(Describe(x));
		}
	}
}
