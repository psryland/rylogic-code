//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Reader (pr::script::reader) - UTF-8 native preprocessor.
//
// Style Guidance:
//  - Macro expansion, conditional compilation, includes, #eval, #lit, and
//    #embedded operate on the byte-oriented 'Cursor'/'Frame' types in this folder.
//    Diagnostics use shared 'EResult' codes and 'Loc' locations.
#pragma once
#include <string>
#include <string_view>
#include <vector>
#include <deque>
#include <memory>
#include <functional>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <sstream>
#include "pr/script/forward.h"
#include "pr/script/location.h"
#include "pr/script/fail_policy.h"
#include "pr/script/includes.h"
#include "pr/filesys/resolve_path.h"
#include "pr/common/expr_eval.h"
#include "pr/str/extract.h"
#include "pr/str/string_core.h"
#include "pr/script/reader/input.h"
#include "pr/script/reader/cursor.h"
#include "pr/script/reader/macros.h"

namespace pr::script::reader
{
	// A lightweight forward cursor over a plain in-memory 'std::string', used when
	// scanning already-expanded text (e.g. inside 'RecursiveExpandMacros') for
	// nested macro invocations. Satisfies the same operator concept as 'Cursor'.
	struct StrCursor
	{
		std::string const* m_str;
		size_t m_pos;

		StrCursor(std::string const& s, size_t pos)
			: m_str(&s)
			, m_pos(pos)
		{}

		char operator *() const
		{
			return m_pos < m_str->size() ? (*m_str)[m_pos] : 0;
		}
		char operator [](size_t i) const
		{
			auto p = m_pos + i;
			return p < m_str->size() ? (*m_str)[p] : 0;
		}
		StrCursor& operator ++()
		{
			if (m_pos < m_str->size())
				++m_pos;

			return *this;
		}
		StrCursor& operator +=(size_t n)
		{
			m_pos = (m_pos + n) < m_str->size() ? m_pos + n : m_str->size();
			return *this;
		}
	};

	// Recursively expand any macro invocations found within 'text', substituting
	// each one with its (also recursively expanded) expansion text. 'ancestor' is
	// the chain of macros already being expanded higher up the call stack, used to
	// stop a macro from expanding itself, directly or transitively.
	inline std::string RecursiveExpandMacros(std::string const& text, IMacroHandler const& macros, Macro::Ancestor const* ancestor, Loc const& loc)
	{
		std::string out;
		out.reserve(text.size());

		size_t i = 0;
		while (i != text.size())
		{
			// Copy non-identifier characters through unchanged.
			if (!str::IsIdentifier(text[i], true))
			{
				out.push_back(text[i]);
				++i;
				continue;
			}

			// Extract the whole identifier so it can be looked up as a macro tag.
			size_t j = i;
			for (; j != text.size() && str::IsIdentifier(text[j], j == i);)
				++j;

			auto tag = text.substr(i, j - i);
			auto* macro = macros.Find(tag);

			// Not a macro, or already being expanded higher up the chain (recursion guard):
			// pass the identifier through unchanged.
			if (!macro || (ancestor && ancestor->IsRecursive(macro)))
			{
				out += tag;
				i = j;
				continue;
			}

			// Attempt to read call-site parameters (if the macro takes any) directly
			// from the text following the identifier.
			StrCursor cur(text, j);
			Macro::Params params;
			auto is_call = macro->m_params.empty() || macro->ReadParams<false>(cur, params, loc);
			if (!is_call)
			{
				out += tag;
				i = j;
				continue;
			}

			// Substitute parameters into the macro body, then recursively expand any
			// macros referenced from within that body (with this macro added to the guard chain).
			Macro::Ancestor me(macro, ancestor);
			auto exp = macro->Expand(params, loc);
			out += RecursiveExpandMacros(exp, macros, &me, loc);
			i = cur.m_pos;
		}
		return out;
	}

	// Find the index of the '}' that matches the '{' at 'open' within 'text', or
	// 'std::string::npos' if unterminated. Used to capture '#eval{...}' bodies,
	// which may themselves contain nested '#eval{...}' occurrences.
	inline size_t MatchingBrace(std::string const& text, size_t open)
	{
		int depth = 0;
		for (size_t i = open; i != text.size(); ++i)
		{
			if (text[i] == '{') ++depth;
			else if (text[i] == '}' && --depth == 0) return i;
		}
		return std::string::npos;
	}

	// Render a '#eval' arithmetic result as text, using an integer representation
	// when the value is exactly integral (matching the typical script convention
	// of writing whole numbers without a trailing ".0").
	inline std::string FormatEvalResult(double value)
	{
		if (std::isfinite(value) && value == std::floor(value) && std::abs(value) < 1e15)
			return std::to_string(static_cast<int64_t>(value));

		std::ostringstream out;
		out.precision(15);
		out << value;
		return out.str();
	}

	// Executes an embedded UTF-8 code block such as '#embedded(lua) ... #end'.
	struct IEmbeddedCode2
	{
		virtual ~IEmbeddedCode2() = default;

		// The language tag this handler executes (e.g. "lua").
		virtual std::string_view Lang() const = 0;

		// Execute 'code', appending any generated output text to 'result'. 'support'
		// reflects the optional directive flag. Returns false on execution failure.
		virtual bool Execute(std::string_view code, bool support, std::string& result) = 0;
	};

	// Resolves and opens '#include' targets as UTF-8 byte sources.
	struct IIncludeHandler
	{
		virtual ~IIncludeHandler() = default;

		// Add a directory to the include search path list.
		virtual void AddSearchPath(std::filesystem::path const& path)
		{
			(void)path;
		}

		// Resolve 'include' (as written in the source) to a concrete path.
		virtual std::filesystem::path ResolveInclude(std::filesystem::path const& include, EIncludeFlags flags, Loc const& loc) = 0;

		// Open 'include' (already resolved by 'ResolveInclude') as a UTF-8 byte input.
		virtual std::unique_ptr<IInput> Open(std::filesystem::path const& resolved, EIncludeFlags flags, Loc const& loc) = 0;
	};

	// Rejects includes when a reader has no include source.
	struct NoIncludes :IIncludeHandler
	{
		std::filesystem::path ResolveInclude(std::filesystem::path const&, EIncludeFlags flags, Loc const& loc) override
		{
			if (AllSet(flags, EIncludeFlags::IgnoreMissing))
				return {};

			throw ScriptException(EResult::IncludesNotSupported, loc, "includes are not supported by this reader");
		}
		std::unique_ptr<IInput> Open(std::filesystem::path const&, EIncludeFlags flags, Loc const& loc) override
		{
			if (AllSet(flags, EIncludeFlags::IgnoreMissing))
				return nullptr;

			throw ScriptException(EResult::IncludesNotSupported, loc, "includes are not supported by this reader");
		}
	};

	// The default 'IIncludeHandler': resolves '#include' targets against a list of
	// search directories (using the shared, non-wide 'pr::filesys::ResolvePath'
	// helper) and opens them as plain UTF-8 files.
	struct FileIncludeHandler :IIncludeHandler
	{
		std::vector<std::filesystem::path> m_paths;

		void AddSearchPath(std::filesystem::path const& path) override
		{
			m_paths.push_back(path);
		}
		std::filesystem::path ResolveInclude(std::filesystem::path const& include, EIncludeFlags flags, Loc const& loc) override
		{
			// A quoted '#include "file"' also checks the directory of the including
			// file first; an angle-bracket '#include <file>' searches only 'm_paths'.
			auto source_dir = loc.Filepath().parent_path();
			auto local_dir = AllSet(flags, EIncludeFlags::IncludeLocalDir) ? &source_dir : nullptr;
			auto resolved = pr::filesys::ResolvePath(include, m_paths, local_dir);
			if (resolved.empty() && !AllSet(flags, EIncludeFlags::IgnoreMissing))
				throw ScriptException(EResult::MissingInclude, loc, "cannot find include file: " + include.string() + " from " + loc.Filepath().string());

			return resolved;
		}
		std::unique_ptr<IInput> Open(std::filesystem::path const& resolved, EIncludeFlags, Loc const& loc) override
		{
			if (resolved.empty())
				return nullptr;

			auto stream = std::make_shared<std::ifstream>(resolved, std::ios::binary);
			if (!stream->is_open())
				throw ScriptException(EResult::FileNotFound, loc, "cannot open include file");

			// The returned 'IInput' owns the stream via the closure below so that it
			// stays alive for exactly as long as the include's 'Cursor' needs it.
			struct OwningStreamInput :IInput
			{
				std::shared_ptr<std::ifstream> m_owned;
				std::filesystem::path m_filepath;

				OwningStreamInput(std::shared_ptr<std::ifstream> owned, std::filesystem::path filepath)
					: m_owned(std::move(owned))
					, m_filepath(std::move(filepath))
				{}
				size_t Read(char* buf, size_t count) override
				{
					m_owned->read(buf, static_cast<std::streamsize>(count));
					auto n = static_cast<size_t>(m_owned->gcount());

					// Propagate hard include transport failures instead of treating a truncated include as clean EOF.
					if (m_owned->bad())
						throw std::ios_base::failure("failed to read script include stream");

					return n;
				}
				std::filesystem::path const& Filepath() const override
				{
					return m_filepath;
				}
			};
			return std::make_unique<OwningStreamInput>(stream, resolved);
		}
	};

	// A comment/line-continuation filtering layer over a raw 'Cursor'. Produces a
	// logical byte-addressable stream for extraction and macro parsing.
	class FilterCursor
	{
		// Memory sources without comment or continuation markers can delegate
		// directly; all other sources retain filtered output and locations.
		Cursor m_raw;
		bool m_passthrough;
		std::vector<char> m_out;
		std::vector<Loc> m_loc;
		size_t m_pos;
		bool m_in_string;
		char m_quote;

		static constexpr size_t CompactThreshold = 2 * BlockSize;

	public:

		explicit FilterCursor(Cursor raw)
			: m_raw(std::move(raw))
			, m_passthrough(m_raw.IsMemory() && m_raw.RemainingView().find_first_of("/\\") == std::string_view::npos)
			, m_out()
			, m_loc()
			, m_pos(0)
			, m_in_string(false)
			, m_quote(0)
		{}

		char operator *()
		{
			if (m_passthrough)
				return *m_raw;

			Ensure(0);
			return m_pos < m_out.size() ? m_out[m_pos] : 0;
		}
		char operator [](size_t i)
		{
			if (m_passthrough)
				return m_raw[i];

			Ensure(i);
			return (m_pos + i) < m_out.size() ? m_out[m_pos + i] : 0;
		}
		FilterCursor& operator ++()
		{
			if (m_passthrough)
			{
				++m_raw;
				return *this;
			}

			Ensure(0);
			if (m_pos < m_out.size())
				++m_pos;

			Compact();
			return *this;
		}
		FilterCursor& operator +=(size_t n)
		{
			if (m_passthrough)
			{
				m_raw += n;
				return *this;
			}

			for (; n-- != 0;)
				++*this;

			return *this;
		}
		Loc const& Location()
		{
			if (m_passthrough)
				return m_raw.Location();

			Ensure(0);
			return m_pos < m_loc.size() ? m_loc[m_pos] : m_raw.Location();
		}
		bool AtEnd()
		{
			return **this == 0;
		}

		// Return transport storage retained by the raw physical cursor.
		size_t TransportRetainedBytes() const noexcept
		{
			return m_raw.RetainedBytes();
		}

		// Return the raw physical cursor's peak transport storage.
		size_t TransportPeakBytes() const noexcept
		{
			return m_raw.PeakRetainedBytes();
		}
		bool Match(std::string_view s, bool consume, bool case_sensitive = true)
		{
			for (size_t i = 0; i != s.size(); ++i)
			{
				auto have = (*this)[i];
				auto want = s[i];
				auto eq = case_sensitive
					? have == want
					: std::tolower(static_cast<unsigned char>(have)) == std::tolower(static_cast<unsigned char>(want));

				if (!eq)
					return false;
			}
			if (consume)
				*this += s.size();

			return true;
		}

	private:

		// Append one filtered character (with the raw cursor's current location).
		void Emit(char ch)
		{
			m_out.push_back(ch);
			m_loc.push_back(m_raw.Location());
		}

		// Produce filtered output until at least 'n + 1' characters are buffered, or
		// the raw source is exhausted.
		void Ensure(size_t n)
		{
			while (m_pos + n >= m_out.size() && *m_raw != 0)
				Step();
		}

		// Consume raw characters until the filter has produced zero or more bytes of
		// logical output (line continuations produce none; comments produce at most
		// one substitute space; everything else copies straight through).
		void Step()
		{
			// Line continuations join physical lines before comment recognition.
			if (*m_raw == '\\' && (m_raw[1] == '\n' || (m_raw[1] == '\r' && m_raw[2] == '\n')))
			{
				m_raw += m_raw[1] == '\r' ? 3 : 2;
				return;
			}

			// An escape sequence inside a string literal is copied through verbatim so
			// that an escaped quote or backslash can't prematurely end the string.
			if (m_in_string && *m_raw == '\\')
			{
				Emit(*m_raw);
				++m_raw;
				if (*m_raw != 0)
				{
					Emit(*m_raw);
					++m_raw;
				}
				return;
			}

			// Comments are only recognised outside of string literals, so that (for
			// example) "http://example.com" is not mistaken for a line comment.
			if (!m_in_string && *m_raw == '/' && m_raw[1] == '/')
			{
				for (; *m_raw != 0 && *m_raw != '\n';)
					++m_raw;

				return;
			}
			if (!m_in_string && *m_raw == '/' && m_raw[1] == '*')
			{
				m_raw += 2;
				for (; *m_raw != 0 && !(*m_raw == '*' && m_raw[1] == '/');)
					++m_raw;

				if (*m_raw != 0)
					m_raw += 2;

				// A single space keeps tokens either side of the removed comment separate.
				Emit(' ');
				return;
			}

			// Script literals are line-bounded, allowing inactive conditional branches
			// containing malformed literals to resume directive recognition on the next line.
			if (m_in_string && *m_raw == '\n')
			{
				m_in_string = false;
				m_quote = 0;
			}
			else if (*m_raw == '"' || *m_raw == '\'')
			{
				if (!m_in_string) { m_in_string = true; m_quote = *m_raw; }
				else if (*m_raw == m_quote) { m_in_string = false; }
			}

			Emit(*m_raw);
			++m_raw;
		}

		// Drop the consumed prefix of the filtered buffer once it grows large enough
		// to be worth reclaiming.
		void Compact()
		{
			if (m_pos < CompactThreshold)
				return;

			m_out.erase(m_out.begin(), m_out.begin() + static_cast<ptrdiff_t>(m_pos));
			m_loc.erase(m_loc.begin(), m_loc.begin() + static_cast<ptrdiff_t>(m_pos));
			m_pos = 0;
		}
	};

	// One level of the preprocessor's source stack: either a file-backed frame
	// (reading filtered bytes from a 'FilterCursor') or a text-backed frame (the
	// in-memory result of a macro expansion, a '#eval' result, an '#embedded'
	// result, or a '#lit' block). Text frames report a single, fixed location -
	// the location of whatever introduced them - matching the common convention
	// that diagnostics raised inside expanded text point at the expansion site.
	class Frame
	{
		std::unique_ptr<FilterCursor> m_file;
		std::string m_text;
		size_t m_tpos;
		Loc m_tloc;
		size_t m_echo; // Remaining char count (from 'm_tpos') to emit verbatim, bypassing directive/macro recognition (used only for '#lit' bodies).
		bool m_no_expand; // True once this frame's text is already the final, fully-resolved output of a macro invocation - 'Pump' must not look up macros in it again.

	public:

		explicit Frame(FilterCursor file)
			: m_file(std::make_unique<FilterCursor>(std::move(file)))
			, m_tpos(0)
			, m_echo(0)
			, m_no_expand(false)
		{}
		Frame(std::string text, Loc const& loc, bool no_expand = false)
			: m_file()
			, m_text(std::move(text))
			, m_tpos(0)
			, m_tloc(loc)
			, m_echo(0)
			, m_no_expand(no_expand)
		{}

		bool IsFile() const
		{
			return m_file != nullptr;
		}
		char operator *()
		{
			return m_file ? **m_file : (m_tpos < m_text.size() ? m_text[m_tpos] : 0);
		}
		char operator [](size_t i)
		{
			if (m_file) return (*m_file)[i];
			auto p = m_tpos + i;
			return p < m_text.size() ? m_text[p] : 0;
		}
		Frame& operator ++()
		{
			if (m_file) ++*m_file;
			else if (m_tpos < m_text.size()) ++m_tpos;
			return *this;
		}
		Frame& operator +=(size_t n)
		{
			if (m_file) *m_file += n;
			else m_tpos = (m_tpos + n) < m_text.size() ? m_tpos + n : m_text.size();
			return *this;
		}
		Loc const& Location()
		{
			return m_file ? m_file->Location() : m_tloc;
		}
		bool AtEnd()
		{
			return m_file ? m_file->AtEnd() : m_tpos >= m_text.size();
		}

		// Mark the next 'n' characters of this frame as verbatim: 'Pump' bypasses
		// directive ('#') and macro-identifier recognition for them entirely. Only
		// '#lit' bodies use this, whereas '#embedded' results are re-scanned.
		void Echo(size_t n)
		{
			m_echo = n;
		}
		size_t EchoRemaining() const
		{
			return m_echo;
		}
		void ConsumeEcho()
		{
			if (m_echo != 0)
				--m_echo;
		}

		// True for a frame holding the already-resolved output of a macro invocation.
		// Such text must never be re-scanned for macro identifiers: doing so would
		// re-attempt to expand any macro tag left un-expanded by the recursion guard
		// (e.g. a mutually-recursive macro's terminal, deliberately-unexpanded tag),
		// looping forever as each attempt re-produces the same unresolved text.
		bool NoExpand() const
		{
			return m_no_expand;
		}

		// Return transport storage retained by this frame's physical source.
		size_t TransportRetainedBytes() const noexcept
		{
			return m_file ? m_file->TransportRetainedBytes() : 0;
		}

		// Return peak transport storage observed by this frame's physical source.
		size_t TransportPeakBytes() const noexcept
		{
			return m_file ? m_file->TransportPeakBytes() : 0;
		}

		bool Match(std::string_view s, bool consume, bool case_sensitive = true)
		{
			if (m_file)
				return m_file->Match(s, consume, case_sensitive);

			for (size_t i = 0; i != s.size(); ++i)
			{
				auto have = (*this)[i];
				auto want = s[i];
				auto eq = case_sensitive
					? have == want
					: std::tolower(static_cast<unsigned char>(have)) == std::tolower(static_cast<unsigned char>(want));

				if (!eq)
					return false;
			}
			if (consume)
				*this += s.size();

			return true;
		}
	};

	// A pull-based, UTF-8 native preprocessor: expands macros, resolves '#include'
	// directives, evaluates conditional-compilation blocks, and consumes the other
	// script preprocessor directives, presenting the result as a single logical
	// forward-only byte stream (satisfying the same operator concept as 'Cursor').
	//
	// Not reused from 'pr::script::Preprocessor': that type is templated on
	// 'wchar_t'-based 'Src' sources throughout, whereas this implementation's
	// source stack ('Frame') is UTF-8 byte oriented from the ground up.
	class Preprocessor
	{
		// Plain source spans bypass preprocessing until syntax requiring the full pipeline is reached.
		std::unique_ptr<Cursor> m_passthrough;
		bool m_passthrough_screened;
		size_t m_passthrough_safe;
		std::vector<Frame> m_stack;
		size_t m_transport_peak_bytes;
		MacroDB m_macros;
		std::unique_ptr<IIncludeHandler> m_default_includes;
		IIncludeHandler* m_includes;
		std::function<IEmbeddedCode2*(std::string_view)> m_embedded_lookup;
		bool m_ignore_missing;

		// One level of '#if'/'#ifdef'/'#ifndef' ... '#endif' nesting.
		struct IfFrame
		{
			bool parent_active; // Whether the enclosing scope was emitting when this level was entered (fixed for the level's lifetime).
			bool taken;          // Whether some branch in this if/elif/else chain has already been selected.
			bool active;         // Whether this level is currently emitting (parent_active && the selected branch).
		};
		std::vector<IfFrame> m_if_stack;

		char m_literal_quote;
		bool m_literal_escape;

		std::deque<char> m_look;
		std::deque<Loc> m_look_loc;

	public:

		// Construct a preprocessor reading UTF-8 bytes from 'input'. 'includes', if
		// non-null, resolves '#include' directives; otherwise includes are rejected.
		explicit Preprocessor(std::unique_ptr<IInput> input, IIncludeHandler* includes = nullptr, Loc const& loc = {})
			: m_passthrough()
			, m_passthrough_screened(false)
			, m_passthrough_safe()
			, m_stack()
			, m_transport_peak_bytes()
			, m_macros()
			, m_default_includes()
			, m_includes(includes)
			, m_embedded_lookup()
			, m_ignore_missing(false)
			, m_if_stack()
			, m_literal_quote(0)
			, m_literal_escape(false)
			, m_look()
			, m_look_loc()
		{
			if (m_includes == nullptr)
			{
				m_default_includes = std::make_unique<NoIncludes>();
				m_includes = m_default_includes.get();
			}

			// Fully screen memory sources once; streamed sources remain direct until
			// their current window reaches syntax that needs filtering or preprocessing.
			auto memory = dynamic_cast<MemoryInput const*>(input.get());
			if (memory != nullptr && memory->m_data.find_first_of("#/\\") == std::string_view::npos)
			{
				m_passthrough = std::make_unique<Cursor>(std::move(input), loc);
				m_passthrough_screened = true;
			}
			else if (memory == nullptr)
			{
				m_passthrough = std::make_unique<Cursor>(std::move(input), loc);
			}
			else
			{
				m_stack.emplace_back(FilterCursor(Cursor(std::move(input), loc)));
			}
		}

		// Convenience constructor over a caller-owned UTF-8 memory buffer, which must
		// outlive this preprocessor.
		explicit Preprocessor(std::string_view utf8, std::filesystem::path filepath = {}, IIncludeHandler* includes = nullptr)
			: Preprocessor(std::make_unique<MemoryInput>(utf8, std::move(filepath)), includes)
		{}

		Preprocessor(Preprocessor const&) = delete;
		Preprocessor& operator =(Preprocessor const&) = delete;

		// Register a handler used to execute '#embedded(lang[,support]) ... #end' blocks.
		void EmbeddedLookup(std::function<IEmbeddedCode2*(std::string_view)> lookup)
		{
			m_embedded_lookup = std::move(lookup);
		}

		// Directly define a macro programmatically (equivalent to a '#define').
		void Define(std::string_view tag, std::string_view expansion)
		{
			ActivatePipeline();
			m_macros.Add(Macro(tag, expansion));
		}

		// Access the macro table backing '#define'/'#undef' and macro expansion.
		IMacroHandler& Macros()
		{
			ActivatePipeline();
			return m_macros;
		}

		// Access the include handler resolving '#include' directives.
		IIncludeHandler& Includes() noexcept
		{
			return *m_includes;
		}

		// True when the current source is the screened contiguous memory path.
		bool HasContiguousInput() const noexcept
		{
			return m_passthrough != nullptr;
		}

		// Return a contiguous view while the direct-source path is active.
		std::string_view Contiguous(size_t count)
		{
			return m_passthrough ? m_passthrough->View(count) : std::string_view{};
		}

		// Return the direct bytes known not to contain syntax that needs the full pipeline.
		std::string_view RemainingContiguous()
		{
			if (!m_passthrough)
				return {};

			auto text = m_passthrough->RemainingView();
			if (m_passthrough_screened)
				return text;

			if (m_passthrough_safe == 0)
			{
				auto special = text.find_first_of("#/\\\"'");
				m_passthrough_safe = special != std::string_view::npos ? special : text.size();
			}
			return text.substr(0, m_passthrough_safe);
		}

		// True when the direct source was screened to its physical end.
		bool ContiguousRangeIsComplete() const noexcept
		{
			return m_passthrough_screened;
		}

		// Consume a caller-validated ASCII span from the contiguous memory path.
		void ConsumeContiguous(size_t count)
		{
			assert(m_passthrough != nullptr);
			assert(m_passthrough_screened || count <= m_passthrough_safe);
			m_passthrough->AdvanceAscii(count);
			if (!m_passthrough_screened)
				m_passthrough_safe -= count;
		}

		// Consume and validate UTF-8 from the contiguous memory path.
		void ConsumeContiguousUtf8(size_t count)
		{
			assert(m_passthrough != nullptr);
			assert(m_passthrough_screened || count <= m_passthrough_safe);
			m_passthrough->AdvanceUtf8(count);
			if (!m_passthrough_screened)
				m_passthrough_safe -= count;
		}

		// Forward-cursor access to the fully preprocessed character stream.
		char operator *()
		{
			if (m_passthrough)
			{
				if (!m_passthrough_screened && m_passthrough_safe == 0)
					ActivatePipelineForLookahead(0);
				if (m_passthrough)
					return **m_passthrough;
			}

			Fill(0);
			return m_look[0];
		}
		char operator [](size_t i)
		{
			if (m_passthrough)
			{
				ActivatePipelineForLookahead(i);
				if (m_passthrough)
					return (*m_passthrough)[i];
			}

			Fill(i);
			return m_look[i];
		}
		Preprocessor& operator ++()
		{
			if (m_passthrough)
			{
				if (!m_passthrough_screened && m_passthrough_safe == 0)
					ActivatePipelineForLookahead(0);
				if (m_passthrough)
				{
					++*m_passthrough;
					if (!m_passthrough_screened && m_passthrough_safe != 0)
						--m_passthrough_safe;
					return *this;
				}
			}

			Fill(0);
			m_look.pop_front();
			m_look_loc.pop_front();
			return *this;
		}
		Preprocessor& operator +=(size_t n)
		{
			if (m_passthrough)
			{
				if (n != 0 && !m_passthrough_screened && n > m_passthrough_safe)
				ActivatePipelineForLookahead(n - 1);
				if (m_passthrough)
				{
				assert(m_passthrough_screened || n <= m_passthrough_safe);
				*m_passthrough += n;
				if (!m_passthrough_screened)
					m_passthrough_safe -= n;
				return *this;
				}
			}

			for (; n-- != 0;)
				++*this;

			return *this;
		}
		Loc const& Location()
		{
			if (m_passthrough)
			{
				// Preserve a stable exhausted-source sentinel.
				if (m_passthrough->AtEnd())
				{
					static Loc const end_loc;
					return end_loc;
				}

				return m_passthrough->Location();
			}

			Fill(0);
			return m_look_loc[0];
		}
		bool AtEnd()
		{
			if (m_passthrough)
				return m_passthrough->AtEnd();

			return **this == 0;
		}

		// Return transport storage retained by currently active physical sources.
		size_t TransportRetainedBytes() const noexcept
		{
			auto bytes = m_passthrough ? m_passthrough->RetainedBytes() : 0;
			for (auto const& frame : m_stack)
				bytes += frame.TransportRetainedBytes();
			return bytes;
		}

		// Return the largest aggregate transport capacity observed across physical sources.
		size_t TransportPeakBytes() const noexcept
		{
			return std::max(m_transport_peak_bytes, ActiveTransportPeakBytes());
		}

	private:

		// Sum per-source high-water marks while their physical frames remain active.
		size_t ActiveTransportPeakBytes() const noexcept
		{
			auto bytes = m_passthrough ? m_passthrough->PeakRetainedBytes() : 0;
			for (auto const& frame : m_stack)
				bytes += frame.TransportPeakBytes();
			return bytes;
		}

		// Preserve the aggregate high-water mark before an exhausted frame is discarded.
		void RecordTransportPeak() noexcept
		{
			m_transport_peak_bytes = std::max(m_transport_peak_bytes, ActiveTransportPeakBytes());
		}

		// Promote a passthrough source into the full preprocessing pipeline.
		void ActivatePipeline()
		{
			if (!m_passthrough)
				return;

			m_stack.emplace_back(FilterCursor(std::move(*m_passthrough)));
			m_passthrough.reset();
			m_passthrough_screened = false;
			m_passthrough_safe = 0;
		}

		// Promote an unscreened source when requested lookahead reaches syntax that
		// needs continuation, comment, directive, or literal-aware processing.
		void ActivatePipelineForLookahead(size_t count)
		{
			if (!m_passthrough || m_passthrough_screened)
				return;

			auto text = m_passthrough->View(count + 1);
			if (text.empty())
				return;

			if (m_passthrough_safe == 0 || count >= m_passthrough_safe)
			{
				auto special = text.find_first_of("#/\\\"'");
				m_passthrough_safe = special != std::string_view::npos ? special : text.size();
			}
			if (count >= m_passthrough_safe && m_passthrough_safe != text.size())
				ActivatePipeline();
		}

		// Ensure the lookahead buffer holds at least 'n + 1' entries, pulling and
		// interpreting further preprocessor input as needed. Once the input is
		// genuinely exhausted, the buffer is padded with sentinel zero characters.
		void Fill(size_t n)
		{
			while (m_look.size() <= n)
			{
				char ch;
				Loc loc;
				if (Pump(ch, loc))
				{
					m_look.push_back(ch);
					m_look_loc.push_back(loc);
				}
				else
				{
					m_look.push_back(0);
					m_look_loc.push_back(m_look_loc.empty() ? Loc() : m_look_loc.back());
				}
			}
		}

		// True while the current position is inside an active (emitting) region of
		// the innermost conditional-compilation block, or when there is no enclosing
		// '#if' at all.
		bool Emitting() const
		{
			return m_if_stack.empty() || m_if_stack.back().active;
		}

		// Produce the next logical output character into 'ch'/'loc', looping past any
		// directives, inactive conditional regions, and macro invocations along the
		// way. Returns false once every source on the stack is exhausted.
		bool Pump(char& ch, Loc& loc)
		{
			for (;;)
			{
				if (m_stack.empty())
					return false;

				auto& top = m_stack.back();
				if (top.AtEnd())
				{
					auto end_loc = top.Location();
					RecordTransportPeak();
					m_stack.pop_back();
					if (m_stack.empty() && !m_if_stack.empty())
						throw ScriptException(EResult::UnmatchedPreprocessorDirective, end_loc, "conditional block does not have a closing #endif");

					continue;
				}

				auto c = *top;

				// '#lit'/'#embedded' bodies are streamed through completely verbatim:
				// skip directive/macro recognition entirely while an echo count remains.
				if (top.EchoRemaining() != 0)
				{
					loc = top.Location();
					++top;
					top.ConsumeEcho();
					ch = c;
					return true;
				}

				// A '#' outside a literal introduces a directive at any output position.
				if (m_literal_quote == 0 && c == '#')
				{
					auto directive_loc = top.Location();
					++top;
					DoDirective(directive_loc);
					continue;
				}

				// Recognise and expand macro invocations while in an active region. A
				// frame flagged 'NoExpand' is already a macro's fully-resolved output,
				// so it is excluded from this lookup (see 'Frame::NoExpand').
				if (Emitting() && m_literal_quote == 0 && !top.NoExpand() && str::IsIdentifier(c, true) && m_macros.CanStart(c) && TryExpandMacro())
					continue;

				loc = top.Location();
				++top;

				UpdateLiteralState(c);
				if (!Emitting())
					continue; // Swallow content inside an inactive '#if' branch.

				ch = c;
				return true;
			}
		}

		// Track quoted output so directive-looking text inside string/character
		// literals remains ordinary data.
		void UpdateLiteralState(char ch)
		{
			if (m_literal_quote != 0 && ch == '\n')
			{
				m_literal_quote = 0;
				m_literal_escape = false;
				return;
			}

			if (m_literal_escape)
			{
				m_literal_escape = false;
				return;
			}
			if (m_literal_quote != 0 && ch == '\\')
			{
				m_literal_escape = true;
				return;
			}
			if (m_literal_quote != 0)
			{
				if (ch == m_literal_quote)
					m_literal_quote = 0;

				return;
			}
			if (ch == '"' || ch == '\'')
				m_literal_quote = ch;
		}

		// If the top frame is positioned at an identifier that names a macro, consume
		// the invocation (and any call-site parameters) and push its expansion as a
		// new frame. Returns false (without consuming anything) if no macro applies.
		bool TryExpandMacro()
		{
			auto& top = m_stack.back();

			size_t n = 0;
			for (; str::IsIdentifier(top[n], n == 0);)
				++n;

			std::string tag;
			for (size_t i = 0; i != n; ++i)
				tag.push_back(top[i]);

			auto* macro = m_macros.Find(tag);
			if (macro == nullptr)
				return false;

			auto loc = top.Location();
			top += n;

			Macro::Params params;
			auto is_call = macro->m_params.empty() || macro->ReadParams<false>(top, params, loc);
			if (!is_call)
			{
				// Not actually invoked (e.g. a parameterised macro's bare tag with no
				// following '('): emit the tag text unchanged, flagged so it is not
				// looked up as a macro again.
				m_stack.emplace_back(tag, loc, true);
				return true;
			}

			// Guard the recursive expansion below with 'macro' as the chain's root, so a
			// macro that (directly or via another macro) expands back to itself is caught
			// here rather than only once a *nested* 'RecursiveExpandMacros' call re-enters -
			// omitting this let mutually-recursive macros (e.g. 'A -> B -> A') expand forever.
			auto exp = macro->Expand(params, loc);
			Macro::Ancestor ancestor(macro, nullptr);
			exp = RecursiveExpandMacros(exp, m_macros, &ancestor, loc);
			m_stack.emplace_back(std::move(exp), loc, true);
			return true;
		}

		// Consume and interpret one directive; 'top' is already positioned just past
		// the leading '#'. Side effects (defining macros, pushing include/eval/embedded
		// frames, reporting errors) only occur while 'Emitting()'. Inactive branches
		// parse conditional structure only.
		void DoDirective(Loc const& loc)
		{
			// Remember which physical frame holds the directive text: a handler such as
			// 'DoInclude'/'DoEval'/'DoLit' may push a new frame on top of the stack, and
			// the trailing-line cleanup below must still target the original frame, not
			// whatever frame is now on top.
			auto frame_index = m_stack.size() - 1;
			auto& top = m_stack[frame_index];

			for (; *top == ' ' || *top == '\t';)
				++top;

			// Read the directive keyword as a plain identifier and resolve it against
			// the shared 'EPPKeyword' reflection, so dispatch is a switch over an enum
			// (per repo style) rather than a chain of string comparisons.
			std::string word;
			for (; str::IsIdentifier(*top, word.empty());)
				word.push_back(*top), ++top;

			auto kw = pr::Enum<EPPKeyword>::TryParse(std::string_view(word), true);
			auto directive = kw ? *kw : EPPKeyword::Invalid;

			// Inactive branches only interpret conditional structure. All other text,
			// including unknown directives, is discarded without validating its syntax.
			auto conditional = false;
			switch (directive)
			{
				case EPPKeyword::If:
				case EPPKeyword::Ifdef:
				case EPPKeyword::Ifndef:
				case EPPKeyword::Elif:
				case EPPKeyword::Else:
				case EPPKeyword::Endif:
				{
					conditional = true;
					break;
				}
				default:
				{
					break;
				}
			}
			if (!Emitting() && !conditional)
			{
				SkipToLineEnd(frame_index);
				return;
			}

			switch (directive)
			{
				case EPPKeyword::IncludePath:   { DoIncludePath(loc); break; }
				case EPPKeyword::Include:       { DoInclude(loc); break; }
				case EPPKeyword::IgnoreMissing: { DoIgnoreMissing(loc); break; }
				case EPPKeyword::Defifndef:     { DoDefifndef(loc); break; }
				case EPPKeyword::Define:        { DoDefine(loc); break; }
				case EPPKeyword::Depend:        { DoDepend(loc); break; }
				case EPPKeyword::Undef:         { DoUndef(loc); break; }
				case EPPKeyword::Ifdef:         { DoIfdef(loc, false); break; }
				case EPPKeyword::Ifndef:        { DoIfdef(loc, true); break; }
				case EPPKeyword::If:            { DoIf(loc); break; }
				case EPPKeyword::Elif:          { DoElif(loc); break; }
				case EPPKeyword::Else:          { DoElse(loc); break; }
				case EPPKeyword::Endif:         { DoEndif(loc); break; }
				case EPPKeyword::End:           { throw ScriptException(EResult::UnmatchedPreprocessorDirective, loc, "#end without matching #lit or #embedded"); }
				case EPPKeyword::Eval:          { DoEval(loc); break; }
				case EPPKeyword::Embedded:      { DoEmbedded(loc); break; }
				case EPPKeyword::Lit:           { DoLit(loc); break; }
				case EPPKeyword::Line:          { DoLine(loc); break; }
				case EPPKeyword::Pragma:        { DoPragma(loc); break; }
				case EPPKeyword::Warning:       { DoWarning(loc); break; }
				case EPPKeyword::Error:         { DoError(loc); break; }
				case EPPKeyword::Defined:       { throw ScriptException(EResult::InvalidPreprocessorDirective, loc, "'defined' is only valid within an #if/#elif expression"); }
				case EPPKeyword::Invalid:
				default:
				{
					throw ScriptException(EResult::UnknownPreprocessorCommand, loc, "unknown preprocessor command");
				}
			}

			SkipToLineEnd(frame_index);
		}

		// Consume up to (and including) the next newline on frame 'frame_index'
		// (captured by 'DoDirective' before dispatch), so any directive argument text
		// left unread by its handler doesn't leak into the following output - this
		// uses the frame's stack index rather than 'm_stack.back()' because a handler
		// may have pushed a further frame (e.g. an '#include'd file or '#lit' body) on
		// top since the directive began.
		void SkipToLineEnd(size_t frame_index)
		{
			auto& top = m_stack[frame_index];
			for (; *top != 0 && *top != '\n';)
				++top;

			if (*top == '\n')
				++top;

		}

		// Read a raw identifier directly from the current top frame (bypassing macro
		// expansion), used for directive arguments such as macro tags in '#undef'.
		std::string ReadIdentifier(Loc const& loc, char const* what)
		{
			std::string id;
			for (; *m_stack.back() == ' ' || *m_stack.back() == '\t';)
				++m_stack.back();

			if (!str::ExtractIdentifier(id, m_stack.back()))
				throw ScriptException(EResult::InvalidIdentifier, loc, what);

			return id;
		}

		#pragma region Directive handlers

		void DoDefine(Loc const& loc)
		{
			Macro macro(m_stack.back(), loc);
			if (Emitting())
				m_macros.Add(macro);
		}
		void DoDefifndef(Loc const& loc)
		{
			Macro macro(m_stack.back(), loc);
			if (Emitting() && m_macros.Find(macro.m_tag) == nullptr)
				m_macros.Add(macro);
		}
		void DoUndef(Loc const& loc)
		{
			auto tag = ReadIdentifier(loc, "invalid macro name");
			if (Emitting())
				m_macros.Remove(tag);
		}
		void DoDepend(Loc const& loc)
		{
			bool quoted;
			auto path = ReadIncludePath(loc, quoted);
			if (!Emitting())
				return;

			// Opening then immediately releasing the dependency preserves include-handler
			// notifications without adding its contents to the preprocessed stream.
			auto flags = quoted ? EIncludeFlags::IncludeLocalDir : EIncludeFlags::None;
			if (m_ignore_missing)
				flags |= EIncludeFlags::IgnoreMissing;

			auto resolved = m_includes->ResolveInclude(path, flags, loc);
			m_includes->Open(resolved, flags, loc);
		}
		void DoIgnoreMissing(Loc const& loc)
		{
			auto& top = m_stack.back();
			for (; *top == ' ' || *top == '\t';)
				++top;

			if (*top != '"')
				throw ScriptException(EResult::InvalidInclude, loc, "expected a string following #ignore_missing");

			++top;
			std::string state;
			for (; *top != 0 && *top != '"';)
				state.push_back(*top), ++top;

			if (*top != '"')
				throw ScriptException(EResult::InvalidInclude, loc, "#ignore_missing string incomplete");

			++top;
			if (Emitting())
				m_ignore_missing = str::EqualI(state, "on");
		}

		// Read a '#include' path of the form '"file"' or '<file>', returning the
		// path text and whether it was quote-delimited (i.e. should also search the
		// including file's own directory).
		std::string ReadIncludePath(Loc const& loc, bool& quoted)
		{
			auto& top = m_stack.back();
			for (; *top == ' ' || *top == '\t';)
				++top;

			char close;
			     if (*top == '"') { quoted = true; close = '"'; }
			else if (*top == '<') { quoted = false; close = '>'; }
			else throw ScriptException(EResult::InvalidInclude, loc, "expected \"file\" or <file>");

			++top;
			std::string path;
			for (; *top != 0 && *top != close;)
				path.push_back(*top), ++top;

			if (*top != close)
				throw ScriptException(EResult::InvalidInclude, loc, "unterminated include path");

			++top;
			return path;
		}
		void DoInclude(Loc const& loc)
		{
			bool quoted;
			auto path = ReadIncludePath(loc, quoted);
			if (!Emitting())
				return;

			auto flags = quoted ? EIncludeFlags::IncludeLocalDir : EIncludeFlags::None;
			if (m_ignore_missing)
				flags |= EIncludeFlags::IgnoreMissing;

			auto resolved = m_includes->ResolveInclude(path, flags, loc);
			auto input = m_includes->Open(resolved, flags, loc);
			if (input == nullptr)
				return; // Missing, but 'IgnoreMissing' was requested by the handler's policy.

			auto source_path = input->Filepath().empty() ? resolved : input->Filepath();
			auto source_offset = input->InitialOffset();
			m_stack.emplace_back(FilterCursor(Cursor(std::move(input), Loc(source_path, source_offset))));
		}
		void DoIncludePath(Loc const& loc)
		{
			auto& top = m_stack.back();
			for (; *top == ' ' || *top == '\t';)
				++top;

			bool quoted;
			auto path = ReadIncludePath(loc, quoted);
			if (Emitting())
				m_includes->AddSearchPath(path);
		}

		// Shared implementation for '#if'/'#ifdef'/'#ifndef': push a new nesting
		// level whose activation depends on the enclosing scope plus this level's
		// own (only-if-enclosing-is-active) condition.
		void PushIf(bool condition_value)
		{
			auto parent_active = Emitting();
			auto value = parent_active && condition_value;
			m_if_stack.push_back(IfFrame{ parent_active, value, value });
		}
		void DoIf(Loc const& loc)
		{
			auto parent_active = Emitting();
			auto text = ReadDirectiveExpressionText();
			auto value = parent_active && EvaluateCondition(text, loc);
			m_if_stack.push_back(IfFrame{ parent_active, value, value });
		}
		void DoIfdef(Loc const& loc, bool negate)
		{
			auto tag = ReadIdentifier(loc, "invalid macro name");
			auto defined = m_macros.Find(tag) != nullptr;
			PushIf(negate ? !defined : defined);
		}
		void DoElif(Loc const& loc)
		{
			if (m_if_stack.empty())
				throw ScriptException(EResult::UnmatchedPreprocessorDirective, loc, "#elif without matching #if");

			auto& top = m_if_stack.back();
			auto text = ReadDirectiveExpressionText();
			if (!top.parent_active || top.taken)
			{
				top.active = false;
			}
			else
			{
				auto value = EvaluateCondition(text, loc);
				top.active = value;
				top.taken = value;
			}
		}
		void DoElse(Loc const& loc)
		{
			if (m_if_stack.empty())
				throw ScriptException(EResult::UnmatchedPreprocessorDirective, loc, "#else without matching #if");

			auto& top = m_if_stack.back();
			if (!top.parent_active || top.taken)
				top.active = false;
			else
				top.active = top.taken = true;
		}
		void DoEndif(Loc const& loc)
		{
			if (m_if_stack.empty())
				throw ScriptException(EResult::UnmatchedPreprocessorDirective, loc, "#endif without matching #if");

			m_if_stack.pop_back();
		}

		// Read the remainder of the current line (used as a '#if'/'#elif' condition,
		// or a '#eval'/'#embedded' argument list) without interpreting it further yet.
		std::string ReadRestOfLine()
		{
			auto& top = m_stack.back();
			std::string text;
			for (; *top != 0 && *top != '\n';)
				text.push_back(*top), ++top;

			return text;
		}
		std::string ReadDirectiveExpressionText()
		{
			auto& top = m_stack.back();
			for (; *top == ' ' || *top == '\t';)
				++top;

			return ReadRestOfLine();
		}

		// Resolve 'defined X' / 'defined(X)' occurrences in 'text' to "1"/"0" (the
		// evaluator itself has no notion of macro definitions), then macro-expand,
		// then hand the result to 'eval::Compile' for arithmetic/logical evaluation.
		bool EvaluateCondition(std::string text, Loc const& loc)
		{
			text = ResolveDefined(text);
			text = RecursiveExpandMacros(text, m_macros, nullptr, loc);
			try
			{
				auto expr = eval::Compile(std::string_view(text));
				return expr().db() != 0.0;
			}
			catch (std::exception const& ex)
			{
				throw ScriptException(EResult::ExpressionSyntaxError, loc, ex.what());
			}
		}
		std::string ResolveDefined(std::string const& text)
		{
			std::string out;
			out.reserve(text.size());

			size_t i = 0;
			while (i != text.size())
			{
				if (!str::IsIdentifier(text[i], true) || text.compare(i, 7, "defined") != 0 ||
					(i + 7 != text.size() && str::IsIdentifier(text[i + 7], false)))
				{
					out.push_back(text[i]);
					++i;
					continue;
				}

				size_t j = i + 7;
				for (; j != text.size() && (text[j] == ' ' || text[j] == '\t');)
					++j;

				bool paren = j != text.size() && text[j] == '(';
				if (paren)
					++j;

				for (; j != text.size() && (text[j] == ' ' || text[j] == '\t');)
					++j;

				size_t k = j;
				for (; k != text.size() && str::IsIdentifier(text[k], k == j);)
					++k;

				auto tag = text.substr(j, k - j);
				auto end = k;
				if (paren)
				{
					for (; end != text.size() && (text[end] == ' ' || text[end] == '\t');)
						++end;

					if (end != text.size() && text[end] == ')')
						++end;
				}

				out += (m_macros.Find(tag) != nullptr) ? "1" : "0";
				i = end;
			}
			return out;
		}

		void DoEval(Loc const& loc)
		{
			auto& top = m_stack.back();
			for (; *top == ' ' || *top == '\t';)
				++top;

			if (*top != '{')
				throw ScriptException(EResult::PreprocessError, loc, "expected '{' after #eval");

			// Consume the block's own opening brace so the capture loop starts already
			// one level deep; this keeps the capture symmetric with the terminating '}'
			// below (both delimiters are consumed but neither is added to 'body'),
			// leaving any nested '#eval{...}' braces intact for 'ResolveNestedEval'.
			++top;
			std::string body;
			int depth = 1;
			for (;; ++top)
			{
				auto c = *top;
				if (c == 0)
					throw ScriptException(EResult::UnexpectedEndOfFile, loc, "unterminated #eval block");

				if (c == '{') ++depth;
				else if (c == '}' && --depth == 0) { ++top; break; }

				body.push_back(c);
			}

			if (!Emitting())
				return;

			auto resolved = ResolveNestedEval(body, loc);
			auto value = EvaluateExpressionText(resolved, loc);
			m_stack.emplace_back(FormatEvalResult(value), loc);
		}

		// Resolve macro references and any nested '#eval{...}' occurrences within an
		// '#eval' body's text before it is handed to the arithmetic evaluator, so that
		// expressions like '#eval{1+#eval{1+1}}' evaluate the inner block first.
		std::string ResolveNestedEval(std::string body, Loc const& loc)
		{
			body = RecursiveExpandMacros(body, m_macros, nullptr, loc);
			for (auto pos = body.find("#eval{"); pos != std::string::npos; pos = body.find("#eval{"))
			{
				auto open = pos + 5;
				auto close = MatchingBrace(body, open);
				if (close == std::string::npos)
					throw ScriptException(EResult::UnexpectedEndOfFile, loc, "unterminated nested #eval block");

				auto inner = body.substr(open + 1, close - open - 1);
				auto value = EvaluateExpressionText(ResolveNestedEval(inner, loc), loc);
				body.replace(pos, close - pos + 1, FormatEvalResult(value));
			}
			return body;
		}
		double EvaluateExpressionText(std::string const& text, Loc const& loc)
		{
			try
			{
				auto expr = eval::Compile(std::string_view(text));
				return expr().db();
			}
			catch (std::exception const& ex)
			{
				throw ScriptException(EResult::ExpressionSyntaxError, loc, ex.what());
			}
		}

		void DoLit(Loc const& loc)
		{
			// Discard directive-line spacing and its line break before capturing the body.
			auto& top = m_stack.back();
			for (; *top == ' ' || *top == '\t';)
				++top;
			if (*top == '\r')
				++top;
			if (*top == '\n')
				++top;

			std::string text;
			for (;;)
			{
				if (top.AtEnd())
					throw ScriptException(EResult::UnexpectedEndOfFile, loc, "unterminated #lit block");

				if (*top == '#' && top.Match("#end", false))
				{
					top += 4;
					break;
				}
				text.push_back(*top);
				++top;
			}

			if (!Emitting())
				return;

			// Push the buffered body as a new frame flagged entirely as 'echo', so
			// 'Pump' streams it through untouched instead of re-scanning it for
			// directives or macro invocations.
			auto n = text.size();
			m_stack.emplace_back(std::move(text), loc);
			m_stack.back().Echo(n);
		}
		void DoEmbedded(Loc const& loc)
		{
			auto& top = m_stack.back();
			for (; *top == ' ' || *top == '\t';)
				++top;

			if (*top != '(')
				throw ScriptException(EResult::InvalidPreprocessorDirective, loc, "expected '#embedded(lang[,support])'");

			++top;
			std::string lang;
			for (; str::IsIdentifier(*top, lang.empty());)
				lang.push_back(*top), ++top;
			if (lang.empty())
				throw ScriptException(EResult::InvalidPreprocessorDirective, loc, "embedded language identifier expected");

			for (; *top == ' ' || *top == '\t';)
				++top;

			auto support = false;
			if (*top == ',')
			{
				++top;
				for (; *top == ' ' || *top == '\t';)
					++top;
				if (!top.Match("support", true, false))
					throw ScriptException(EResult::InvalidPreprocessorDirective, loc, "expected 'support' after ','");

				support = true;
				for (; *top == ' ' || *top == '\t';)
					++top;
			}
			if (*top != ')')
				throw ScriptException(EResult::InvalidPreprocessorDirective, loc, "expected ')' after embedded language");

			// The directive's trailing whitespace and line break are not embedded code.
			++top;
			for (; *top == ' ' || *top == '\t';)
				++top;
			if (*top == '\r')
				++top;
			if (*top == '\n')
				++top;

			std::string code;
			for (;;)
			{
				if (top.AtEnd())
					throw ScriptException(EResult::UnexpectedEndOfFile, loc, "unterminated #embedded block");

				if (*top == '#' && top.Match("#end", false))
				{
					top += 4;
					break;
				}
				code.push_back(*top);
				++top;
			}

			if (!Emitting())
				return;

			// Embedded source receives recursively expanded macro text.
			code = RecursiveExpandMacros(code, m_macros, nullptr, loc);
			auto* handler = m_embedded_lookup ? m_embedded_lookup(lang) : nullptr;
			if (handler == nullptr)
			{
				if (m_ignore_missing)
					return;

				throw ScriptException(EResult::EmbeddedCodeNotSupported, loc, "no handler registered for embedded code language");
			}

			std::string result;
			if (!handler->Execute(code, support, result))
				throw ScriptException(EResult::EmbeddedCodeError, loc, "embedded code execution failed");

			m_stack.emplace_back(std::move(result), loc);
		}

		void DoLine(Loc const&)
		{
			// '#line' is accepted and ignored: reader reports true source locations
			// rather than caller-overridden ones (an intentional, documented gap).
		}
		void DoPragma(Loc const&)
		{
			// Pragmas are consumed as informational directives.
		}
		void DoWarning(Loc const&)
		{
			// Warnings are consumed and otherwise ignored; reader has no diagnostic
			// sink wired up at this layer (an integration concern, per the plan).
		}
		void DoError(Loc const& loc)
		{
			if (!Emitting())
				return;

			auto text = ReadRestOfLine();
			throw ScriptException(EResult::PreprocessError, loc, text);
		}

		#pragma endregion
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::script::reader::testing
{
	// Drain a preprocessor to a plain string, for comparison against expected output.
	inline std::string Drain(Preprocessor& pp)
	{
		std::string out;
		for (; !pp.AtEnd();)
			out.push_back(*pp), ++pp;

		return out;
	}

	PRUnitTestClass(ReaderPreprocessorTests)
	{
		PRUnitTestMethod(ConsecutiveStrings, Quick)
		{
			// Comments are removed and replaced with a single space, macros expand normally.
			Preprocessor pp("abc // comment\ndef");
			PR_EXPECT(Drain(pp) == "abc \ndef");
		}
		PRUnitTestMethod(BlockComment, Quick)
		{
			Preprocessor pp("A/*comment*/B");
			PR_EXPECT(Drain(pp) == "A B");
		}
		PRUnitTestMethod(LineContinuation, Quick)
		{
			Preprocessor pp("A\\\nB");
			PR_EXPECT(Drain(pp) == "AB");
		}
		PRUnitTestMethod(CommentInsideString, Quick)
		{
			Preprocessor pp("\"http://example.com\"");
			PR_EXPECT(Drain(pp) == "\"http://example.com\"");
		}
		PRUnitTestMethod(SimpleMacro, Quick)
		{
			Preprocessor pp("#define TWO 2\nTWO+TWO");
			PR_EXPECT(Drain(pp) == "2+2");
		}
		PRUnitTestMethod(FunctionMacro, Quick)
		{
			Preprocessor pp("#define ADD(a,b) a+b\nADD(1,2)");
			PR_EXPECT(Drain(pp) == "1+2");
		}
		PRUnitTestMethod(HashEval, Quick)
		{
			Preprocessor pp("#eval{1+#eval{1+1}}");
			PR_EXPECT(Drain(pp) == "3");
		}
		PRUnitTestMethod(MidLineEval, Quick)
		{
			Preprocessor pp("*Value #eval{1+2}");
			PR_EXPECT(Drain(pp) == "*Value 3");
		}
		PRUnitTestMethod(IfElseEndif, Quick)
		{
			Preprocessor pp("#if 0\nA\n#elif 1\nB\n#else\nC\n#endif");
			PR_EXPECT(Drain(pp) == "B\n");
		}
		PRUnitTestMethod(DirectivesInsideInactiveStrings, Quick)
		{
			Preprocessor pp("#if 0\n*Value \"#endif\"\n#endif\nAfter");
			PR_EXPECT(Drain(pp) == "After");
		}
		PRUnitTestMethod(IfDefUndef, Quick)
		{
			Preprocessor pp("#define X\n#ifdef X\nyes\n#endif\n#undef X\n#ifndef X\nno\n#endif");
			PR_EXPECT(Drain(pp) == "yes\nno\n");
		}
		PRUnitTestMethod(LitBlock, Quick)
		{
			Preprocessor pp("#lit  \r\n#define X 1\n#end");
			PR_EXPECT(Drain(pp) == "#define X 1\n");
		}
		PRUnitTestMethod(EmbeddedBlock, Quick)
		{
			struct Handler :IEmbeddedCode2
			{
				std::string m_code;
				bool m_support = false;

				std::string_view Lang() const override
				{
					return "test";
				}
				bool Execute(std::string_view code, bool support, std::string& result) override
				{
					m_code = code;
					m_support = support;
					result = "result";
					return true;
				}
			};

			auto handler = Handler{};
			Preprocessor pp("#define VALUE 42\n#embedded(test,support)  \r\nVALUE\n#end");
			pp.EmbeddedLookup([&](std::string_view lang)
			{
				return lang == handler.Lang() ? &handler : nullptr;
			});

			PR_EXPECT(Drain(pp) == "result");
			PR_EXPECT(handler.m_code == "42\n");
			PR_EXPECT(handler.m_support);
		}
		PRUnitTestMethod(ErrorInsideStringIsNotADirective, Quick)
		{
			Preprocessor pp("\"#error not real\"");
			PR_EXPECT(Drain(pp) == "\"#error not real\"");
		}
		PRUnitTestMethod(UnmatchedEnd, Quick)
		{
			Preprocessor pp("#end");
			PR_THROWS(pp.AtEnd(), ScriptException);
		}
	};
}
#endif
