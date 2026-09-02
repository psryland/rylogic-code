//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Reader2 (pr::script::v2) - UTF-8 native preprocessor macros.
//
// Style Guidance:
//  - This mirrors the algorithm in 'pr/script/macros.h' exactly (parameter reading,
//    '#'/'##' substitution, recursion guard) but stores UTF-8 'std::string' content
//    instead of 'std::wstring'. The legacy 'Macro' type is not reusable as-is
//    because it hardcodes 'string_t' (= std::wstring) as its storage; everything
//    else about the algorithm carries over unchanged.
#pragma once
#include <string>
#include <string_view>
#include <vector>
#include <unordered_map>
#include "pr/script/forward.h"
#include "pr/script/location.h"
#include "pr/script/fail_policy.h"
#include "pr/str/extract.h"
#include "pr/str/string_core.h"
#include "pr/str/string_util.h"

namespace pr::script::v2
{
	// A single preprocessor macro definition (result of '#define').
	//
	// A macro either has no parameter list at all (a plain substitution, e.g.
	// '#define TWO 2') or has a parameter list, possibly empty (e.g. '#define
	// TAG() body' or '#define TAG(a,b) a+b'). The empty-parameter-list case is
	// distinguished from the no-parameter-list case by 'm_params' containing a
	// single blank entry (matching the legacy convention).
	struct Macro
	{
		using Params = std::vector<std::string>;

		std::string m_tag;       // The macro name.
		std::string m_expansion; // The raw substitution text (before parameter substitution).
		Params m_params;         // Parameter names; empty() means no parameter list was given at all.
		Loc m_loc;               // Where this macro was defined, for diagnostics.

		Macro() = default;

		// Construct a simple '#define TAG expansion' style macro directly.
		explicit Macro(std::string_view tag, std::string_view expansion, Params params = {}, Loc const& loc = Loc())
			: m_tag(tag)
			, m_expansion(expansion)
			, m_params(std::move(params))
			, m_loc(loc)
		{}

		// Parse a function-style macro definition of the form 'TAG(p0,p1,..,pn) expansion...'
		// from 'src', stopping at the first (unescaped) newline. 'Ptr' must satisfy the
		// forward-cursor concept used throughout 'pr::str::Extract*' (operator*, ++, +=, []).
		template <typename Ptr>
		explicit Macro(Ptr& src, Loc const& loc = Loc())
			: Macro()
		{
			m_loc = loc;

			// The macro name comes first, unconditionally.
			if (!str::ExtractIdentifier(m_tag, src))
				throw ScriptException(EResult::InvalidIdentifier, loc, "invalid macro name");

			// An immediately-following '(' introduces the (possibly empty) parameter list.
			if (*src == '(')
				ReadParams<true>(src, m_params, loc);

			// The remainder of the (already continuation-joined) line is the expansion text,
			// with surrounding whitespace trimmed.
			for (; *src == ' ' || *src == '\t';)
				++src;

			for (; *src != 0 && *src != '\n';)
				m_expansion.push_back(*src), ++src;

			str::Trim(m_expansion, str::IsWhiteSpace<char>, true, true);
		}

		// Read a comma-separated parameter list '(p0,p1,..,pn)' starting at 'src' (which must
		// be positioned at the opening '('). If 'Identifiers' is true, each parameter is parsed
		// as a plain identifier (used when reading a macro definition); otherwise each parameter
		// is the raw text up to the next comma or matching close-paren, respecting nested
		// parentheses (used when reading the arguments at a macro call site). An empty
		// parameter list "()" yields a single blank parameter, to distinguish "TAG()" from a
		// bare "TAG". Returns false (without throwing) if this macro takes no parameters and
		// none were supplied at the call site - callers use this to mean "don't expand".
		template <bool Identifiers, typename Ptr>
		bool ReadParams(Ptr& src, Params& params, Loc const& loc) const
		{
			params.clear();

			if constexpr (!Identifiers)
			{
				// A parameter-less macro is only invoked as a bare tag, never with a call.
				if (!m_params.empty())
				{
					size_t i = 0;
					for (; str::IsWhiteSpace(src[i]);)
						++i;

					if (src[i] != '(')
						return false;

					src += i;
				}
				if (m_params.empty())
					return true;
			}

			std::string param;

			// Capture the text between each pair of commas (or the opening paren and the
			// first comma, or the last comma and the closing paren) as one parameter.
			for (++src; *src != ')';)
			{
				param.clear();
				if constexpr (Identifiers)
				{
					if (!str::ExtractIdentifier(param, src))
						throw ScriptException(EResult::InvalidIdentifier, loc, "invalid macro identifier");
				}
				else
				{
					for (int nest = 0; nest || (*src != ',' && *src != ')');)
					{
						if (*src == 0)
							throw ScriptException(EResult::UnexpectedEndOfFile, loc, "macro parameter list incomplete");

						nest += *src == '(';
						nest -= *src == ')';
						param.push_back(*src);
						++src;
					}
				}

				params.push_back(param);
				if (*src != ')')
					++src;
			}

			// Skip the closing ')'.
			++src;

			if (params.empty())
				params.push_back(std::string());

			if constexpr (!Identifiers)
			{
				if (m_params.size() != params.size())
					throw ScriptException(EResult::ParameterCountMismatch, loc, "incorrect number of macro parameters");
			}

			return true;
		}

		// Produce the expansion of this macro, given the call-site 'params', substituting
		// each formal parameter occurrence within 'm_expansion'. A parameter reference
		// prefixed with '##' is pasted (the '##' is simply removed, joining adjacent text).
		// A parameter reference prefixed with '#' is stringized (replaced by a quoted,
		// escaped literal of the argument text) rather than substituted directly.
		std::string Expand(Params const& params, Loc const& loc) const
		{
			if (params.size() != m_params.size())
				throw ScriptException(EResult::ParameterCountMismatch, loc, "macro parameter count mismatch");

			std::string exp = m_expansion;
			for (size_t i = 0; i != m_params.size(); ++i)
			{
				auto const& what = m_params[i];
				auto len = what.size();
				if (len == 0)
					continue;

				std::string with;
				for (size_t j = str::FindIdentifier(exp, what, 0); j != exp.size(); j = str::FindIdentifier(exp, what, j += len))
				{
					if (j >= 2 && exp[j - 1] == '#' && exp[j - 2] == '#')
					{
						// Paste: drop the '##' and substitute the raw argument text.
						j -= 2;
						len += 2;
						with = params[i];
					}
					else if (j >= 1 && exp[j - 1] == '#')
					{
						// Stringize: substitute a quoted, escaped copy of the argument text.
						j -= 1;
						len += 1;
						with = params[i];
						str::Replace(with, "\"", "\\\"");
						str::Quotes(with, true);
					}
					else
					{
						with = params[i];
					}

					exp.erase(j, len);
					exp.insert(j, with);

					// Advance the search past the just-inserted text (which may be shorter than
					// what it replaced), so a fresh substitution can never re-match part of it.
					if (with.size() >= len)
						j += with.size() - len;
					else
						j = (j > len - with.size()) ? j - (len - with.size()) : 0;
				}
			}
			return exp;
		}

		friend bool operator ==(Macro const& lhs, Macro const& rhs)
		{
			return lhs.m_params.size() == rhs.m_params.size() && lhs.m_expansion == rhs.m_expansion;
		}
		friend bool operator !=(Macro const& lhs, Macro const& rhs)
		{
			return !(lhs == rhs);
		}

		// A singly-linked chain of macros currently being expanded, used to prevent a macro
		// from recursively expanding itself (directly or via another macro it invokes).
		struct Ancestor
		{
			Macro const* m_macro;
			Ancestor const* m_parent;

			Ancestor(Macro const* macro, Ancestor const* parent) noexcept
				: m_macro(macro)
				, m_parent(parent)
			{}

			// True if 'macro' already appears earlier in this expansion chain.
			bool IsRecursive(Macro const* macro) const
			{
				auto p = this;
				for (; p && p->m_macro != macro;)
					p = p->m_parent;

				return p != nullptr;
			}
		};
	};

	// Interface for looking up/adding/removing macro definitions, so callers can supply
	// their own macro store (mirrors 'pr::script::IMacroHandler').
	struct IMacroHandler
	{
		virtual ~IMacroHandler() = default;

		// Add a macro definition. Throws 'EResult::MacroAlreadyDefined' if a different
		// definition already exists for the same tag; an identical redefinition is allowed.
		virtual void Add(Macro const& macro) = 0;

		// Remove a macro definition, if one exists (no-op otherwise).
		virtual void Remove(std::string_view tag) = 0;

		// Find a macro definition by tag, or return nullptr if none is defined.
		virtual Macro const* Find(std::string_view tag) const = 0;
	};

	// The default 'IMacroHandler': an unordered map of tag to 'Macro'.
	struct MacroDB :IMacroHandler
	{
		std::unordered_map<std::string, Macro> m_db;

		void Add(Macro const& macro) override
		{
			auto i = m_db.find(macro.m_tag);
			if (i != std::end(m_db))
			{
				if (i->second == macro)
					return; // Identical redefinition, silently allowed.

				throw ScriptException(EResult::MacroAlreadyDefined, macro.m_loc, "macro already defined");
			}
			m_db.emplace(macro.m_tag, macro);
		}
		void Remove(std::string_view tag) override
		{
			auto i = m_db.find(std::string(tag));
			if (i != std::end(m_db))
				m_db.erase(i);
		}
		Macro const* Find(std::string_view tag) const override
		{
			auto i = m_db.find(std::string(tag));
			return i != std::end(m_db) ? &i->second : nullptr;
		}
	};
}
