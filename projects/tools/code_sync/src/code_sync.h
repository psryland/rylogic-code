//************************************
// CodeSync
//  Copyright (c) Rylogic Ltd 2025
//************************************
// Synchronises code blocks across files.
// Truth blocks:     PR_CODE_SYNC _BEGIN(name, source_of_truth) ... PR_CODE_SYNC _END()
// Reference blocks: PR_CODE_SYNC _BEGIN(name) ... PR_CODE_SYNC _END()
// Reference blocks are replaced with the content of the corresponding truth block.
#pragma once
#include "src/forward.h"

namespace code_sync
{
	// A single line from a truth block, stored as relative indent + content
	struct TruthLine
	{
		int m_indent_columns; // Indent relative to the BEGIN line
		std::string m_content; // Non-whitespace content (trimmed leading whitespace)
	};

	// A source-of-truth block
	struct TruthBlock
	{
		std::string m_name;
		std::vector<TruthLine> m_lines;
		fs::path m_filepath;
		int m_line_number; // 1-based
	};

	// A ref block found in a file
	struct RefBlock
	{
		std::string m_name;
		int m_begin_line;    // 0-based index of the BEGIN line
		int m_content_start; // 0-based index of first content line
		int m_content_end;   // 0-based index of END line (exclusive)
	};

	// File extensions to scan
	inline bool IsSyncFile(fs::path const& p)
	{
		auto ext = p.extension().string();
		for (auto& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
		return ext == ".h" || ext == ".hpp" || ext == ".cpp" || ext == ".c" || ext == ".inl" || ext == ".cs";
	}

	struct CodeSync
	{
		std::map<std::string, TruthBlock> m_truths;
		std::vector<std::string> m_errors;
		int m_tab_size;
		bool m_verbose;

		// Marker tokens (split to avoid self-matching when this file is scanned by CodeSync)
		static constexpr char const* BeginTag = "PR_CODE" "_SYNC_BEGIN";
		static constexpr char const* EndTag = "PR_CODE" "_SYNC_END";

		// Returns the full end marker including "()"
		static std::string EndTagFull() { return std::string(EndTag) + "()"; }

		CodeSync(int tab_size = 4, bool verbose = false)
			: m_tab_size(tab_size)
			, m_verbose(verbose)
		{}

		// Result of parsing a BEGIN line
		struct BeginMatch
		{
			bool matched = false;
			std::string name;
			bool is_sot = false;
		};

		// Try to parse a begin-marker line. Format: <anything>BEGIN_TAG( <name> [, source_of_truth] )<anything>
		static BeginMatch MatchBegin(std::string const& line)
		{
			auto pos = line.find(BeginTag);
			if (pos == std::string::npos)
				return {};

			// Find '(' after the tag
			auto p = pos + std::strlen(BeginTag);
			while (p < line.size() && line[p] == ' ') ++p;
			if (p >= line.size() || line[p] != '(')
				return {};
			++p;

			// Skip whitespace
			for (; p < line.size() && std::isspace(line[p]); ++p) {};

			// Read name (identifier: [a-zA-Z_][a-zA-Z0-9_-:]*)
			auto name_start = p;
			auto is_tag_char = [](char c) { return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == ':'; };
			for (; p < line.size() && is_tag_char(line[p]); ++p) {}
			if (p == name_start)
				return {};

			auto name = line.substr(name_start, p - name_start);

			// Skip whitespace
			while (p < line.size() && line[p] == ' ') ++p;

			// Check for optional ', source_of_truth'
			bool is_sot = false;
			if (p < line.size() && line[p] == ',')
			{
				++p;
				while (p < line.size() && line[p] == ' ') ++p;
				auto sot = std::string_view("source_of_truth");
				if (p + sot.size() <= line.size() && line.substr(p, sot.size()) == sot)
				{
					is_sot = true;
					p += sot.size();
				}
				while (p < line.size() && line[p] == ' ') ++p;
			}

			// Expect ')'
			if (p >= line.size() || line[p] != ')')
				return {};

			return {true, std::move(name), is_sot};
		}

		// Check if a line contains the end marker
		static bool MatchEnd(std::string const& line)
		{
			auto tag = EndTagFull();
			return line.find(tag) != std::string::npos;
		}

		// Check if a line contains either marker (for quick skip of irrelevant files)
		static bool ContainsAnyMarker(std::string const& line)
		{
			return line.find("PR_CODE" "_SYNC") != std::string::npos;
		}

		// Measure the column width of leading whitespace
		int MeasureIndent(std::string const& line) const
		{
			int cols = 0;
			for (auto c : line)
			{
				if (c == '\t') cols += m_tab_size;
				else if (c == ' ') cols += 1;
				else break;
			}
			return cols;
		}

		// Return the leading whitespace string
		static std::string GetIndentStr(std::string const& line)
		{
			size_t i = 0;
			while (i < line.size() && (line[i] == ' ' || line[i] == '\t'))
				++i;
			return line.substr(0, i);
		}

		// Detect whether indentation uses tabs
		static bool UsesTabs(std::string const& indent)
		{
			return indent.empty() || indent.find('\t') != std::string::npos;
		}

		// Build a whitespace string for the given column count
		std::string MakeIndent(int columns, bool use_tabs) const
		{
			if (use_tabs)
			{
				int tabs = columns / m_tab_size;
				int spaces = columns % m_tab_size;
				return std::string(tabs, '\t') + std::string(spaces, ' ');
			}
			return std::string(columns, ' ');
		}

		// Decompose a line into relative indent + content
		TruthLine DecomposeLine(std::string const& line, int base_columns) const
		{
			if (line.find_first_not_of(" \t\r\n") == std::string::npos)
				return {0, ""};

			auto indent_cols = MeasureIndent(line);
			auto relative_cols = std::max(0, indent_cols - base_columns);
			auto indent_str = GetIndentStr(line);
			return {relative_cols, line.substr(indent_str.size())};
		}

		// Reconstruct a line from a TruthLine with the given base indent
		std::string ReconstructLine(TruthLine const& tl, int base_columns, bool use_tabs) const
		{
			if (tl.m_content.empty())
				return "";
			return MakeIndent(base_columns + tl.m_indent_columns, use_tabs) + tl.m_content;
		}

		// Read entire file as a string using Win32 for performance
		static std::string ReadFileRaw(fs::path const& filepath)
		{
			auto h = CreateFileW(filepath.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
				OPEN_EXISTING, FILE_FLAG_SEQUENTIAL_SCAN, nullptr);
			if (h == INVALID_HANDLE_VALUE)
				return {};

			LARGE_INTEGER size;
			GetFileSizeEx(h, &size);
			std::string content(static_cast<size_t>(size.QuadPart), '\0');
			DWORD bytes_read = 0;
			ReadFile(h, content.data(), static_cast<DWORD>(content.size()), &bytes_read, nullptr);
			CloseHandle(h);
			content.resize(bytes_read);
			return content;
		}

		// Split raw file content into lines
		static std::vector<std::string> SplitLines(std::string const& content)
		{
			std::vector<std::string> lines;
			size_t start = 0;
			while (start <= content.size())
			{
				auto nl = content.find('\n', start);
				if (nl == std::string::npos)
				{
					if (start < content.size())
					{
						auto line = content.substr(start);
						if (!line.empty() && line.back() == '\r')
							line.pop_back();
						lines.push_back(std::move(line));
					}
					break;
				}
				auto line = content.substr(start, nl - start);
				if (!line.empty() && line.back() == '\r')
					line.pop_back();
				lines.push_back(std::move(line));
				start = nl + 1;
			}
			return lines;
		}

		// Recursively enumerate all sync files under a root directory using Win32 APIs for speed
		std::vector<fs::path> EnumerateFiles(fs::path const& root) const
		{
			struct L
			{
				std::vector<fs::path> out;

				// Enumerate source files in a directory tree using Win32 for speed
				void Run(std::wstring const& dir)
				{
					WIN32_FIND_DATAW fd;
					auto pattern = dir + L"\\*";
					auto h = FindFirstFileExW(pattern.c_str(), FindExInfoBasic, &fd, FindExSearchNameMatch, nullptr, FIND_FIRST_EX_LARGE_FETCH);
					if (h == INVALID_HANDLE_VALUE)
						return;

					do
					{
						if (fd.cFileName[0] == L'.' && (fd.cFileName[1] == 0 || (fd.cFileName[1] == L'.' && fd.cFileName[2] == 0)))
							continue;

						auto path = dir + L"\\" + fd.cFileName;
						if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
						{
							Run(path);
						}
						else if (IsSyncFile(fs::path(fd.cFileName)))
						{
							out.emplace_back(path);
						}
					}
					while (FindNextFileW(h, &fd));
					FindClose(h);
				}
			};

			L enum_files;
			enum_files.Run(root.wstring());
			return enum_files.out;
		}

		// Write all lines to a file
		static void WriteAllLines(fs::path const& filepath, std::vector<std::string> const& lines)
		{
			std::ofstream file(filepath, std::ios::binary);
			for (size_t i = 0; i != lines.size(); ++i)
			{
				file << lines[i];
				if (i + 1 != lines.size())
					file << '\n';
			}
		}

		// Extract truth content from lines [content_start, content_end), stripping nested BEGIN/END
		std::vector<TruthLine> ExtractTruthContent(std::vector<std::string> const& lines, int begin_line, int content_start, int content_end) const
		{
			auto base_columns = MeasureIndent(lines[begin_line]);
			std::vector<TruthLine> result;
			for (int i = content_start; i != content_end; ++i)
			{
				if (MatchBegin(lines[i]).matched || MatchEnd(lines[i]))
					continue;
				result.push_back(DecomposeLine(lines[i], base_columns));
			}
			return result;
		}

		// Validate that a truth block does not contain ref blocks
		void ValidateNoRefBlocks(std::vector<std::string> const& lines, int start, int end, std::string const& truth_name, fs::path const& filepath) const
		{
			for (int i = start; i < end; ++i)
			{
				auto bm = MatchBegin(lines[i]);
				if (bm.matched && !bm.is_sot)
					throw std::runtime_error(
						filepath.string() + "(" + std::to_string(i + 1) + "): Ref block '" +
						bm.name + "' found inside source_of_truth block '" + truth_name + "'.");
			}
		}

		// Find matching END tag, returns -1 if not found
		int FindMatchingEnd(std::vector<std::string> const& lines, int content_start, int end) const
		{
			int depth = 1;
			for (int j = content_start; j < end; ++j)
			{
				if (MatchBegin(lines[j]).matched)
					++depth;
				if (MatchEnd(lines[j]))
				{
					--depth;
					if (depth == 0)
						return j;
				}
			}
			return -1;
		}

		// Scan lines for truth blocks
		void FindTruthBlocksInLines(std::vector<std::string> const& lines, fs::path const& filepath, int start, int end, bool is_truth_scope)
		{
			for (int i = start; i < end; ++i)
			{
				auto bm = MatchBegin(lines[i]);
				if (!bm.matched)
				{
					if (MatchEnd(lines[i]))
						throw std::runtime_error(
							filepath.string() + "(" + std::to_string(i + 1) + "): Unexpected " + EndTag + " without matching BEGIN.");
					continue;
				}

				int content_start = i + 1;
				int content_end = FindMatchingEnd(lines, content_start, end);

				if (content_end < 0)
					throw std::runtime_error(
						filepath.string() + "(" + std::to_string(i + 1) + "): " + BeginTag + "('" + bm.name + "') has no matching " + EndTag + ".");

				if (bm.is_sot)
				{
					auto content = ExtractTruthContent(lines, i, content_start, content_end);

					if (m_truths.count(bm.name))
					{
						auto const& existing = m_truths[bm.name];
						throw std::runtime_error(
							filepath.string() + "(" + std::to_string(i + 1) + "): Duplicate source_of_truth block '" + bm.name +
							"'. First defined at " + existing.m_filepath.string() + "(" + std::to_string(existing.m_line_number) + ").");
					}

					ValidateNoRefBlocks(lines, content_start, content_end, bm.name, filepath);
					m_truths[bm.name] = TruthBlock{bm.name, std::move(content), filepath, i + 1};

					// Recurse for nested truth blocks
					FindTruthBlocksInLines(lines, filepath, content_start, content_end, true);
				}
				else if (is_truth_scope)
				{
					throw std::runtime_error(
						filepath.string() + "(" + std::to_string(i + 1) + "): Ref block '" + bm.name +
						"' found inside a source_of_truth block. Only nested source_of_truth blocks are allowed.");
				}

				i = content_end;
			}
		}

		// Find all ref blocks in a file
		std::vector<RefBlock> FindRefBlocks(std::vector<std::string> const& lines, fs::path const& filepath) const
		{
			std::vector<RefBlock> blocks;
			for (int i = 0; i < static_cast<int>(lines.size()); ++i)
			{
				auto bm = MatchBegin(lines[i]);
				if (!bm.matched)
					continue;

				int content_start = i + 1;
				int content_end = FindMatchingEnd(lines, content_start, static_cast<int>(lines.size()));

				if (content_end < 0)
					throw std::runtime_error(
						filepath.string() + "(" + std::to_string(i + 1) + "): " + BeginTag + "('" + bm.name + "') has no matching " + EndTag + ".");

				if (!bm.is_sot)
				{
					// Validate no nested blocks
					for (int j = content_start; j < content_end; ++j)
					{
						if (MatchBegin(lines[j]).matched)
							throw std::runtime_error(
								filepath.string() + "(" + std::to_string(j + 1) + "): Nested block found inside ref block '" + bm.name + "'.");
					}
					blocks.push_back({bm.name, i, content_start, content_end});
				}

				i = content_end;
			}
			return blocks;
		}

		// Run across multiple directories
		int Run(std::vector<fs::path> const& directories)
		{
			// Validate directories exist
			for (auto const& dir : directories)
			{
				if (!fs::exists(dir) || !fs::is_directory(dir))
					throw std::runtime_error("Directory '" + dir.string() + "' does not exist.");
			}

			if (m_verbose)
			{
				std::cout << "CodeSync: Scanning ";
				for (size_t i = 0; i != directories.size(); ++i)
				{
					if (i != 0) std::cout << ", ";
					std::cout << "'" << directories[i].string() << "'";
				}
				std::cout << "..." << std::endl;
			}

			// Enumerate all source files, then read in parallel to find those with markers
			struct FileData
			{
				fs::path filepath;
				std::string raw;           // Raw file content (only for files with markers)
				std::vector<std::string> lines; // Parsed lines (populated after parallel phase)
				bool has_markers = false;
			};

			// Enumerate files (single-threaded, fast)
			std::vector<FileData> all_files;
			for (auto const& dir : directories)
				for (auto const& filepath : EnumerateFiles(dir))
					all_files.push_back({filepath, {}, {}, false});

			// Read files and scan for markers in parallel
			static auto const marker = std::string("PR_CODE") + "_SYNC";
			std::for_each(std::execution::par, all_files.begin(), all_files.end(), [](FileData& fd)
			{
				auto raw = ReadFileRaw(fd.filepath);
				if (raw.find(marker) != std::string::npos)
				{
					fd.has_markers = true;
					fd.raw = std::move(raw);
				}
			});

			// Collect files with markers and split into lines
			std::vector<FileData*> sync_files;
			for (auto& fd : all_files)
			{
				if (!fd.has_markers)
					continue;

				fd.lines = SplitLines(fd.raw);
				fd.raw.clear(); // Free raw content
				sync_files.push_back(&fd);
			}

			// Pass 1: find truth blocks
			for (auto* fd : sync_files)
				FindTruthBlocksInLines(fd->lines, fd->filepath, 0, static_cast<int>(fd->lines.size()), false);

			if (m_verbose)
			{
				std::cout << "CodeSync: Found " << m_truths.size() << " truth block(s):";
				for (auto const& [name, _] : m_truths)
					std::cout << " " << name;
				std::cout << std::endl;
			}

			// Pass 2: replace ref blocks (using already-loaded lines)
			int files_updated = 0;
			for (auto* fd : sync_files)
			{
				bool modified = false;
				auto ref_blocks = FindRefBlocks(fd->lines, fd->filepath);

				for (auto it = ref_blocks.rbegin(); it != ref_blocks.rend(); ++it)
				{
					auto const& rb = *it;
					auto truth_it = m_truths.find(rb.m_name);
					if (truth_it == m_truths.end())
						throw std::runtime_error(
							fd->filepath.string() + "(" + std::to_string(rb.m_begin_line + 1) + "): Ref block '" + rb.m_name + "' refers to unknown truth block.");

					auto const& truth = truth_it->second;

					auto ref_indent_str = GetIndentStr(fd->lines[rb.m_begin_line]);
					auto ref_base_columns = MeasureIndent(fd->lines[rb.m_begin_line]);
					auto ref_uses_tabs = UsesTabs(ref_indent_str);

					std::vector<std::string> indented_truth;
					for (auto const& tl : truth.m_lines)
						indented_truth.push_back(ReconstructLine(tl, ref_base_columns, ref_uses_tabs));

					std::vector<std::string> current(fd->lines.begin() + rb.m_content_start, fd->lines.begin() + rb.m_content_end);

					bool same = current.size() == indented_truth.size();
					if (same)
					{
						for (size_t i = 0; i != current.size(); ++i)
						{
							auto trim = [](std::string const& s) {
								auto pos = s.find_first_not_of(" \t");
								return pos == std::string::npos ? std::string{} : s.substr(pos);
							};
							if (trim(current[i]) != trim(indented_truth[i]))
							{
								same = false;
								break;
							}
						}
					}

					if (!same)
					{
						auto ref_modified = fs::last_write_time(fd->filepath);
						auto truth_modified = fs::last_write_time(truth.m_filepath);
						if (ref_modified > truth_modified)
						{
							bool has_content = false;
							for (auto const& line : current)
							{
								if (line.find_first_not_of(" \t\r\n") != std::string::npos)
								{
									has_content = true;
									break;
								}
							}
							if (has_content)
							{
								m_errors.push_back(
									fd->filepath.string() + "(" + std::to_string(rb.m_begin_line + 1) + "): Ref block '" + rb.m_name +
									"' code is newer than source of truth in " + truth.m_filepath.string() +
									"(" + std::to_string(truth.m_line_number) + "). Check source of truth implementation is up to date.");
								continue;
							}
						}

						std::vector<std::string> new_lines;
						new_lines.insert(new_lines.end(), fd->lines.begin(), fd->lines.begin() + rb.m_content_start);
						new_lines.insert(new_lines.end(), indented_truth.begin(), indented_truth.end());
						new_lines.insert(new_lines.end(), fd->lines.begin() + rb.m_content_end, fd->lines.end());
						fd->lines = std::move(new_lines);
						modified = true;

						if (m_verbose)
							std::cout << "  Updated '" << rb.m_name << "' in " << fd->filepath.string() << std::endl;
					}
				}

				if (modified)
				{
					WriteAllLines(fd->filepath, fd->lines);
					++files_updated;
				}
			}

			if (files_updated > 0 || m_verbose)
				std::cout << "CodeSync: Updated " << files_updated << " file(s)." << std::endl;

			if (!m_errors.empty())
			{
				for (auto const& error : m_errors)
					std::cerr << "CodeSync error: " << error << std::endl;

				throw std::runtime_error(
					"CodeSync: " + std::to_string(m_errors.size()) + " ref block(s) are newer than their source of truth. Update the source of truth first.");
			}

			return files_updated;
		}
	};
}
