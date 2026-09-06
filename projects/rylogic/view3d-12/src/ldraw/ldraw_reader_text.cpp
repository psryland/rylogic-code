//********************************
// Ldraw Script Text Reader
//  Copyright (c) Rylogic Ltd 2026
//********************************
#include "pr/view3d-12/ldraw/ldraw_reader_text.h"
#include "pr/script/reader.h"

namespace pr::rdr12::ldraw
{
	namespace
	{
		// Convert a script location to the location shape exposed by the LDraw parser.
		Location ToLocation(script::Loc const& loc, int64_t filesize)
		{
			return
			{
				.m_filepath = loc.Filepath(),
				.m_filesize = loc.FileSize() != 0 ? loc.FileSize() : filesize,
				.m_offset = loc.Pos(),
				.m_column = loc.Col(),
				.m_line = loc.Line(),
			};
		}

		// Preserve snapshot source metadata when adapting an existing stream.
		script::Loc SourceLocation(std::istream const& stream, std::filesystem::path const& src_filepath)
		{
			auto const* snapshot = dynamic_cast<filesys::FileSnapshotStream const*>(&stream);
			if (snapshot != nullptr && !snapshot->filepath().empty())
				return script::Loc(snapshot->filepath(), snapshot->file_size(), snapshot->file_offset(), snapshot->file_offset(), 1, 1, true);

			return script::Loc(src_filepath);
		}

		// Append one Unicode scalar to a UTF-8 byte string.
		void AppendUtf8(std::string& output, uint32_t code_point, script::Loc const& loc)
		{
			if (code_point <= 0x7F)
			{
				output.push_back(static_cast<char>(code_point));
			}
			else if (code_point <= 0x7FF)
			{
				output.push_back(static_cast<char>(0xC0 | (code_point >> 6)));
				output.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
			}
			else if (code_point <= 0xFFFF)
			{
				if (code_point >= 0xD800 && code_point <= 0xDFFF)
					throw script::ScriptException(script::EResult::WrongEncoding, loc, "unpaired UTF-16 surrogate");

				output.push_back(static_cast<char>(0xE0 | (code_point >> 12)));
				output.push_back(static_cast<char>(0x80 | ((code_point >> 6) & 0x3F)));
				output.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
			}
			else if (code_point <= 0x10FFFF)
			{
				output.push_back(static_cast<char>(0xF0 | (code_point >> 18)));
				output.push_back(static_cast<char>(0x80 | ((code_point >> 12) & 0x3F)));
				output.push_back(static_cast<char>(0x80 | ((code_point >> 6) & 0x3F)));
				output.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
			}
			else
			{
				throw script::ScriptException(script::EResult::WrongEncoding, loc, "Unicode scalar is out of range");
			}
		}

		// Convert Reader include flags to the shared path-resolver flags.
		IPathResolver::EFlags ToResolverFlags(script::EIncludeFlags flags)
		{
			auto result = IPathResolver::EFlags::None;
			if (AllSet(flags, script::EIncludeFlags::IncludeLocalDir))
				result |= IPathResolver::EFlags::IncludeLocalDir;
			if (AllSet(flags, script::EIncludeFlags::IgnoreMissing))
				result |= IPathResolver::EFlags::IgnoreMissing;

			return result;
		}

		// Owns converted include bytes while exposing Reader's bulk input contract.
		struct OwnedUtf8Input : script::reader::IInput
		{
			std::string m_source;
			size_t m_position;
			std::filesystem::path m_filepath;
			std::streamoff m_initial_offset;

			OwnedUtf8Input(std::string source, std::filesystem::path filepath, std::streamoff initial_offset)
				: m_source(std::move(source))
				, m_position()
				, m_filepath(std::move(filepath))
				, m_initial_offset(initial_offset)
			{
			}

			// Copy the next UTF-8 block into Reader's transport buffer.
			size_t Read(char* buffer, size_t count) override
			{
				auto available = m_source.size() - m_position;
				auto length = std::min(available, count);
				if (length != 0)
				{
					std::memcpy(buffer, m_source.data() + m_position, length);
					m_position += length;
				}
				return length;
			}

			// Return the physical identity used for source locations.
			std::filesystem::path const& Filepath() const override
			{
				return m_filepath;
			}

			// Return the physical offset retained when caller-side conversion removed a source byte-order-mark.
			std::streamoff InitialOffset() const override
			{
				return m_initial_offset;
			}
		};

		// Bridge LDraw's resolver to Reader's UTF-8 include contract.
		struct IncludeAdapter : script::reader::IIncludeHandler
		{
			IPathResolver const* m_resolver;
			std::optional<PathResolver> m_path_resolver;

			explicit IncludeAdapter(IPathResolver const& resolver)
				: m_resolver(&resolver)
				, m_path_resolver()
			{
				if (auto path_resolver = dynamic_cast<PathResolver const*>(&resolver); path_resolver != nullptr)
				{
					m_path_resolver.emplace(*path_resolver);
					m_resolver = &*m_path_resolver;
				}
			}

			// Append source-declared search paths to an isolated copy of the standard resolver.
			void AddSearchPath(std::filesystem::path const& path) override
			{
				if (m_path_resolver)
					m_path_resolver->AddSearchPath(path);
			}

			// Preserve the include spelling because the shared resolver owns file, resource, and string lookup.
			std::filesystem::path ResolveInclude(std::filesystem::path const& include, script::EIncludeFlags flags, script::Loc const& loc) override
			{
				if (m_path_resolver && AllSet(flags, script::EIncludeFlags::IncludeLocalDir))
					m_path_resolver->LocalDir(loc.Filepath().parent_path());

				return include;
			}

			// Snapshot and convert each include before Reader begins scanning it.
			std::unique_ptr<script::reader::IInput> Open(std::filesystem::path const& resolved, script::EIncludeFlags flags, script::Loc const&) override
			{
				auto stream = m_resolver->OpenStream(resolved, ToResolverFlags(flags));
				if (stream == nullptr)
					return nullptr;

				auto filepath = resolved;
				if (auto snapshot = dynamic_cast<filesys::FileSnapshotStream const*>(stream.get()); snapshot != nullptr && !snapshot->filepath().empty())
					filepath = snapshot->filepath();

				std::string bytes(std::istreambuf_iterator<char>(*stream), {});
				if (stream->bad())
					throw std::ios_base::failure("failed to read LDraw include stream");

				auto bom_size = 0;
				auto encoding = filesys::DetectFileEncoding(std::span(bytes.data(), bytes.size()), bom_size);
				auto source = ToUtf8Source(bytes, encoding, filepath);
				return std::make_unique<OwnedUtf8Input>(std::move(source), std::move(filepath), bom_size);
			}
		};

		// True when a compact object-header token has LDraw's eight-digit ARGB form.
		bool IsColour(std::string_view token)
		{
			return token.size() == 8 && std::all_of(token.begin(), token.end(), [](char ch)
			{
				return std::isxdigit(static_cast<unsigned char>(ch)) != 0;
			});
		}

		// True when a compact object-header token is a valid LDraw name.
		bool IsName(std::string_view token)
		{
			return !token.empty() && std::all_of(token.begin(), token.end(), [](char ch)
			{
				return std::isalnum(static_cast<unsigned char>(ch)) != 0 || ch == '_';
			});
		}
	}

	// Convert caller-owned LDraw source bytes to the UTF-8 contract required by Reader.
	std::string ToUtf8Source(std::string_view source, EEncoding encoding, std::filesystem::path const& filepath)
	{
		auto bom_size = 0;
		if (encoding == EEncoding::auto_detect)
			encoding = filesys::DetectFileEncoding(std::span(source.data(), source.size()), bom_size);
		else if (encoding == EEncoding::utf8)
			bom_size = static_cast<int>(script::reader::Utf8BomLength(source));
		else if (source.size() >= 2)
			bom_size =
				(encoding == EEncoding::utf16_le && static_cast<unsigned char>(source[0]) == 0xFF && static_cast<unsigned char>(source[1]) == 0xFE) ||
				(encoding == EEncoding::utf16_be && static_cast<unsigned char>(source[0]) == 0xFE && static_cast<unsigned char>(source[1]) == 0xFF)
				? 2
				: 0;

		source.remove_prefix(static_cast<size_t>(bom_size));
		auto loc = script::Loc(filepath);
		switch (encoding)
		{
			case EEncoding::ascii:
			{
				if (std::any_of(source.begin(), source.end(), [](char ch) { return static_cast<unsigned char>(ch) > 0x7F; }))
					throw script::ScriptException(script::EResult::WrongEncoding, loc, "non-ASCII byte in ASCII source");

				return std::string(source);
			}
			case EEncoding::utf8:
			{
				return std::string(source);
			}
			case EEncoding::ascii_extended:
			{
				std::string output;
				output.reserve(source.size() * 2);
				for (auto ch : source)
					AppendUtf8(output, static_cast<unsigned char>(ch), loc);

				return output;
			}
			case EEncoding::utf16_le:
			case EEncoding::utf16_be:
			{
				if ((source.size() & 1) != 0)
					throw script::ScriptException(script::EResult::WrongEncoding, loc, "truncated UTF-16 code unit");

				std::string output;
				output.reserve(source.size());
				for (size_t index = 0; index != source.size(); index += 2)
				{
					auto const byte0 = static_cast<unsigned char>(source[index + 0]);
					auto const byte1 = static_cast<unsigned char>(source[index + 1]);
					auto code_unit = encoding == EEncoding::utf16_le
						? static_cast<uint16_t>(byte0 | (byte1 << 8))
						: static_cast<uint16_t>((byte0 << 8) | byte1);
					auto code_point = static_cast<uint32_t>(code_unit);

					if (code_unit >= 0xD800 && code_unit <= 0xDBFF)
					{
						if (index + 3 >= source.size())
							throw script::ScriptException(script::EResult::WrongEncoding, loc, "truncated UTF-16 surrogate pair");

						auto const next0 = static_cast<unsigned char>(source[index + 2]);
						auto const next1 = static_cast<unsigned char>(source[index + 3]);
						auto low = encoding == EEncoding::utf16_le
							? static_cast<uint16_t>(next0 | (next1 << 8))
							: static_cast<uint16_t>((next0 << 8) | next1);
						if (low < 0xDC00 || low > 0xDFFF)
							throw script::ScriptException(script::EResult::WrongEncoding, loc, "invalid UTF-16 surrogate pair");

						code_point = 0x10000 + ((code_unit - 0xD800) << 10) + (low - 0xDC00);
						index += 2;
					}
					else if (code_unit >= 0xDC00 && code_unit <= 0xDFFF)
					{
						throw script::ScriptException(script::EResult::WrongEncoding, loc, "unpaired UTF-16 surrogate");
					}

					AppendUtf8(output, code_point, loc);
				}
				return output;
			}
			default:
			{
				throw std::runtime_error(std::format("Unsupported LDraw text encoding: {}", static_cast<int>(encoding)));
			}
		}
	}

	struct TextReader::Impl
	{
		enum class EPseudoValue
		{
			None,
			Name,
			Colour,
		};
		struct PseudoToken
		{
			EPseudoValue m_kind;
			std::string m_value;
		};

		IncludeAdapter m_includes;
		script::Reader m_reader;
		mutable Location m_location;
		string32 m_keyword;
		std::array<PseudoToken, 2> m_pseudo_tokens;
		size_t m_pseudo_count;
		PseudoToken m_pseudo_value;
		int64_t m_filesize;
		int m_section_level;
		int m_nest_level;

		Impl(std::string_view source, script::Loc const& loc, IPathResolver const& resolver)
			: m_includes(resolver)
			, m_reader(std::make_unique<script::reader::MemoryInput>(source, loc.Filepath()), loc, false, &m_includes)
			, m_location(ToLocation(loc, loc.FileSize() != 0 ? loc.FileSize() : static_cast<int64_t>(source.size())))
			, m_keyword()
			, m_pseudo_tokens()
			, m_pseudo_count()
			, m_pseudo_value{ EPseudoValue::None, {} }
			, m_filesize(loc.FileSize() != 0 ? loc.FileSize() : static_cast<int64_t>(source.size()))
			, m_section_level()
			, m_nest_level()
		{
			m_reader.ReportError = [](script::EResult, script::Loc const&, std::string_view)
			{
				return false;
			};
		}

		Impl(std::istream& stream, std::filesystem::path filepath, IPathResolver const& resolver)
			: Impl(stream, SourceLocation(stream, filepath), resolver)
		{
		}

		// Construct a stream reader after source metadata has been captured once.
		Impl(std::istream& stream, script::Loc const& loc, IPathResolver const& resolver)
			: m_includes(resolver)
			, m_reader(std::make_unique<script::reader::StreamInput>(stream, loc.Filepath()), loc, false, &m_includes)
			, m_location(ToLocation(loc, loc.FileSize()))
			, m_keyword()
			, m_pseudo_tokens()
			, m_pseudo_count()
			, m_pseudo_value{ EPseudoValue::None, {} }
			, m_filesize(loc.FileSize())
			, m_section_level()
			, m_nest_level()
		{
			m_reader.ReportError = [](script::EResult, script::Loc const&, std::string_view)
			{
				return false;
			};
		}

		// Access the preprocessed byte stream underlying the Reader facade.
		script::reader::Preprocessor& Source()
		{
			return m_reader.Source();
		}

		// Consume a data section opening brace before scalar extraction.
		void PrepareValue()
		{
			if (m_pseudo_value.m_kind != EPseudoValue::None)
				return;

			auto& source = Source();
			m_nest_level += *source == '{';
			source += *source == '{';
		}

		// Finish consuming the currently active compact-header pseudo value.
		void ConsumePseudo()
		{
			m_pseudo_value = { EPseudoValue::None, {} };
		}
	};

	TextReader::TextReader(std::string_view utf8_source, std::filesystem::path src_filepath, ReportErrorCB report_error_cb, ParseProgressCB progress_cb, IPathResolver const& resolver)
		: IReader(report_error_cb, progress_cb, resolver)
		, m_impl(std::make_unique<Impl>(utf8_source, script::Loc(src_filepath, static_cast<std::streamsize>(utf8_source.size()), 0, 0, 1, 1, true), resolver))
	{
	}

	TextReader::TextReader(std::string_view utf8_source, Location const& source_location, ReportErrorCB report_error_cb, ParseProgressCB progress_cb, IPathResolver const& resolver)
		: IReader(report_error_cb, progress_cb, resolver)
		, m_impl(std::make_unique<Impl>(utf8_source, script::Loc(source_location.m_filepath, source_location.m_filesize, source_location.m_offset, source_location.m_offset, source_location.m_line, source_location.m_column, true), resolver))
	{
	}

	TextReader::TextReader(std::istream& utf8_stream, std::filesystem::path src_filepath, ReportErrorCB report_error_cb, ParseProgressCB progress_cb, IPathResolver const& resolver)
		: IReader(report_error_cb, progress_cb, resolver)
		, m_impl(std::make_unique<Impl>(utf8_stream, std::move(src_filepath), resolver))
	{
	}

	TextReader::~TextReader() = default;

	// Return the current location in the source.
	Location const& TextReader::Loc() const
	{
		auto loc = m_impl->m_reader.Location();
		if (script::reader::IsDefaultLocation(loc) && (!m_impl->m_location.m_filepath.empty() || m_impl->m_location.m_filesize != 0))
		{
			// Preserve the source identity at EOF while reporting complete progress.
			m_impl->m_location.m_offset = m_impl->m_filesize;
			return m_impl->m_location;
		}

		m_impl->m_location = ToLocation(loc, m_impl->m_filesize);
		return m_impl->m_location;
	}

	// Move into a nested section.
	void TextReader::PushSection()
	{
		auto& source = m_impl->Source();
		script::EatDelimiters(source, m_impl->m_reader.Delimiters());
		if (*source != '{')
		{
			ReportError(EParseError::NotFound, Loc(), "section start expected");
			str::AdvanceToDelim(source, m_impl->m_reader.Delimiters());
			return;
		}

		++m_impl->m_section_level;
		++m_impl->m_nest_level;
		++source;
	}

	// Leave the current nested section.
	void TextReader::PopSection()
	{
		auto& source = m_impl->Source();
		script::EatDelimiters(source, m_impl->m_reader.Delimiters());
		if (*source != '}')
		{
			ReportError(EParseError::NotFound, Loc(), "section end expected");
			str::AdvanceToDelim(source, m_impl->m_reader.Delimiters());
			return;
		}

		--m_impl->m_section_level;
		--m_impl->m_nest_level;
		++source;
	}

	// True when the current position has reached the end of the current section.
	bool TextReader::IsSectionEnd()
	{
		m_impl->PrepareValue();
		auto& source = m_impl->Source();
		script::EatDelimiters(source, m_impl->m_reader.Delimiters());
		return *source == '}' || *source == 0;
	}

	// True when the source is exhausted.
	bool TextReader::IsSourceEnd()
	{
		m_impl->PrepareValue();
		auto& source = m_impl->Source();
		script::EatDelimiters(source, m_impl->m_reader.Delimiters());
		return *source == 0;
	}

	// Get the next keyword within the current section.
	bool TextReader::NextKeywordImpl(int& kw)
	{
		if (m_impl->m_pseudo_count != 0)
		{
			m_impl->m_pseudo_value = std::move(m_impl->m_pseudo_tokens[0]);
			--m_impl->m_pseudo_count;
			if (m_impl->m_pseudo_count != 0)
				m_impl->m_pseudo_tokens[0] = std::move(m_impl->m_pseudo_tokens[1]);

			switch (m_impl->m_pseudo_value.m_kind)
			{
				case Impl::EPseudoValue::Name: { m_impl->m_keyword = "Name"; break; }
				case Impl::EPseudoValue::Colour: { m_impl->m_keyword = "Colour"; break; }
				default: throw std::runtime_error("Invalid LDraw pseudo-token kind");
			}
			kw = HashI(m_impl->m_keyword);
			return true;
		}

		auto& source = m_impl->Source();

		// Skip completed data sections and unrelated values without crossing the current object scope.
		for (; *source && *source != '*';)
		{
			if (*source == '\"') { script::EatLiteral(source, source.Location()); continue; }
			if (*source == '{') { script::EatSection(source, source.Location()); continue; }
			if (*source == '}')
			{
				if (m_impl->m_nest_level > m_impl->m_section_level)
					--m_impl->m_nest_level;
				else
					break;
			}
			++source;
		}
		if (*source == '*')
			++source;
		else
			return false;

		m_impl->m_keyword.clear();
		if (!str::ExtractIdentifier(m_impl->m_keyword, source, m_impl->m_reader.Delimiters()))
			return false;

		kw = HashI(m_impl->m_keyword);

		// Capture optional compact name/colour fields and expose them as ordinary keyword/value pairs.
		for (int index = 0; index != 2; ++index)
		{
			script::EatDelimiters(source, m_impl->m_reader.Delimiters());
			if (*source == '{')
				break;

			std::string token;
			if (!str::ExtractToken(token, source, m_impl->m_reader.Delimiters()))
				break;

			if (IsColour(token))
				m_impl->m_pseudo_tokens[m_impl->m_pseudo_count++] = { Impl::EPseudoValue::Colour, std::move(token) };
			else if (IsName(token))
				m_impl->m_pseudo_tokens[m_impl->m_pseudo_count++] = { Impl::EPseudoValue::Name, std::move(token) };
		}

		script::EatDelimiters(source, m_impl->m_reader.Delimiters());
		if (*source != '{')
		{
			ReportError(EParseError::UnexpectedToken, Loc(), "expected '{'");
			str::AdvanceToDelim(source, m_impl->m_reader.Delimiters());
			return false;
		}
		return true;
	}

	// Read an identifier from the current section.
	string32 TextReader::IdentifierImpl(bool incl_dot)
	{
		if (m_impl->m_pseudo_value.m_kind == Impl::EPseudoValue::Name)
		{
			auto value = std::move(m_impl->m_pseudo_value.m_value);
			m_impl->ConsumePseudo();
			return value;
		}

		m_impl->PrepareValue();
		string32 value;
		if (!str::ExtractIdentifier(value, m_impl->Source(), m_impl->m_reader.Delimiters(), incl_dot))
		{
			ReportError(EParseError::InvalidValue, Loc(), "identifier expected");
			str::AdvanceToDelim(m_impl->Source(), m_impl->m_reader.Delimiters());
			return {};
		}
		return value;
	}

	// Read a UTF-8 string from the current section.
	string32 TextReader::StringImpl(char escape_char)
	{
		m_impl->PrepareValue();
		auto& source = m_impl->Source();
		script::EatDelimiters(source, m_impl->m_reader.Delimiters());
		auto quote = *source;
		string32 value;
		if (!str::ExtractString(value, source, escape_char, {}, m_impl->m_reader.Delimiters()))
		{
			ReportError(EParseError::InvalidValue, Loc(), "string expected");
			str::AdvanceToDelim(source, m_impl->m_reader.Delimiters());
			return {};
		}

		// Present whitespace-adjacent literals as one logical string.
		for (;;)
		{
			auto join = size_t{};
			for (; str::IsWhiteSpace(source[join]); ++join) {}
			if (source[join] != quote)
				break;

			source += join;
			string32 part;
			if (!str::ExtractString(part, source, escape_char, {}, m_impl->m_reader.Delimiters()))
				break;

			value.append(part);
		}
		str::ProcessIndentedNewlines(value);
		return value;
	}

	// Read an integral value from the current section.
	int64_t TextReader::IntImpl(int, int radix)
	{
		if (m_impl->m_pseudo_value.m_kind == Impl::EPseudoValue::Colour)
		{
			uint64_t value = {};
			auto const& token = m_impl->m_pseudo_value.m_value;
			auto [ptr, ec] = std::from_chars(token.data(), token.data() + token.size(), value, radix);
			if (ec == std::errc{} && ptr == token.data() + token.size())
			{
				m_impl->ConsumePseudo();
				return static_cast<int64_t>(value);
			}
			m_impl->ConsumePseudo();
		}

		m_impl->PrepareValue();
		int64_t value = {};
		if (!m_impl->m_reader.Int(value, radix))
		{
			ReportError(EParseError::InvalidValue, Loc(), "integer value expected");
			str::AdvanceToDelim(m_impl->Source(), m_impl->m_reader.Delimiters());
			return {};
		}
		return value;
	}

	// Read a dense sequence of integral values without repeated interface dispatch.
	void TextReader::IntsImpl(int byte_count, std::span<int64_t> values, int radix)
	{
		for (auto& value : values)
			value = IntImpl(byte_count, radix);
	}

	// Read a floating point value from the current section.
	double TextReader::RealImpl(int)
	{
		m_impl->PrepareValue();
		double value = {};
		if (!m_impl->m_reader.Real(value) || std::isnan(value) || !std::isfinite(value))
		{
			ReportError(EParseError::InvalidValue, Loc(), std::isnan(value) ? "real value is Not-a-Number" : !std::isfinite(value) ? "real value is not finite" : "real value expected");
			str::AdvanceToDelim(m_impl->Source(), m_impl->m_reader.Delimiters());
			return {};
		}
		return value;
	}

	// Read a dense sequence of floating-point values without repeated interface dispatch.
	void TextReader::RealsImpl(int byte_count, std::span<double> values)
	{
		for (auto& value : values)
			value = RealImpl(byte_count);
	}

	// Read an enum value from the current section.
	int64_t TextReader::EnumImpl(int, ParseEnumIdentCB parse)
	{
		auto identifier = IdentifierImpl(false);
		if (identifier.empty())
			return {};

		return parse(identifier);
	}

	// Read a boolean value from the current section.
	bool TextReader::BoolImpl()
	{
		m_impl->PrepareValue();
		bool value = {};
		if (!m_impl->m_reader.Bool(value))
		{
			ReportError(EParseError::InvalidValue, Loc(), "boolean value expected");
			str::AdvanceToDelim(m_impl->Source(), m_impl->m_reader.Delimiters());
			return {};
		}
		return value;
	}

	// Return the most recently read keyword for diagnostics.
	string32 TextReader::LastKeywordString() const
	{
		return m_impl->m_keyword;
	}
}
