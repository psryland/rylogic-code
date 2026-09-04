//**********************************
// Script Reader Benchmark
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Standalone Release/x64 throughput and memory regression benchmark for 'pr::script::Reader'.
//
// Style Guidance:
//  - This benchmark exists purely to measure and report; it must never modify 'pr/script/reader.h' or
//    'pr/script/reader/*'. It owns only the fixture generators, drivers, timing/statistics, and reporting.
//  - Workload fixtures are generated once per (workload, size) before any timing starts (see
//    'workload_generators.h'); only the construct-and-parse step performed by 'reader_drivers.h' is
//    ever timed.
//  - Every measured parse is verified against the first successful parse so timing results cannot
//    hide semantic instability.
#include "src/forward.h"
#include "src/checksum.h"
#include "src/statistics.h"
#include "src/workload_generators.h"
#include "src/reader_drivers.h"

namespace pr::script_bench
{
	// Parsed command-line configuration.
	struct Options
	{
		bool m_show_help = false;
		std::vector<ESize> m_sizes = { ESize::Small };
		std::vector<EWorkload> m_workloads =
		{
			EWorkload::NumberHeavy, EWorkload::IdentEmpty, EWorkload::IdentPopulated,
			EWorkload::Strings, EWorkload::Macros, EWorkload::Boundary, EWorkload::Includes,
		};
		std::vector<EInputMode> m_input_modes = { EInputMode::Memory };
		bool m_memory_scaling = false;
		int m_warmups = 4;
		int m_reps = 16;
		uint64_t m_seed = 20240517ull;
	};

	// One measured (non-warmup) parse: what was run, how big it was, how long it took, and its checksum.
	struct TimedSample
	{
		EWorkload m_workload;
		ESize m_size;
		EInputMode m_input_mode;
		int m_repetition;
		uint64_t m_bytes;
		uint64_t m_items;
		double m_elapsed_s;
		uint64_t m_checksum;
	};

	// --- Naming ----------------------------------------------------------------------------------

	char const* WorkloadName(EWorkload w)
	{
		switch (w)
		{
			case EWorkload::NumberHeavy:     return "number-heavy";
			case EWorkload::IdentEmpty:      return "ident-empty";
			case EWorkload::IdentPopulated:  return "ident-populated";
			case EWorkload::Strings:         return "strings-utf8";
			case EWorkload::Macros:          return "directive-macro-heavy";
			case EWorkload::Boundary:        return "token-boundary";
			case EWorkload::Includes:        return "include-tree-warm";
			default: throw std::invalid_argument("unknown EWorkload value");
		}
	}
	char const* SizeName(ESize s)
	{
		switch (s)
		{
			case ESize::Small: return "small";
			case ESize::Large: return "large";
			default: throw std::invalid_argument("unknown ESize value");
		}
	}
	char const* InputModeName(EInputMode mode)
	{
		switch (mode)
		{
			case EInputMode::Memory: return "memory";
			case EInputMode::Stream: return "stream";
			case EInputMode::File:   return "file";
			default: throw std::invalid_argument("unknown EInputMode value");
		}
	}
	std::optional<EWorkload> ParseWorkloadName(std::string_view s)
	{
		if (s == "number") return EWorkload::NumberHeavy;
		if (s == "ident-empty") return EWorkload::IdentEmpty;
		if (s == "ident-populated") return EWorkload::IdentPopulated;
		if (s == "strings") return EWorkload::Strings;
		if (s == "macros") return EWorkload::Macros;
		if (s == "boundary") return EWorkload::Boundary;
		if (s == "includes") return EWorkload::Includes;
		return std::nullopt;
	}

	// --- Build/environment identification (informational CSV columns) ----------------------------

	char const* BuildConfigName()
	{
		#if defined(NDEBUG)
		return "Release";
		#else
		return "Debug";
		#endif
	}
	char const* ArchName()
	{
		#if defined(_M_X64) || defined(_M_AMD64)
		return "x64";
		#elif defined(_M_IX86)
		return "x86";
		#elif defined(_M_ARM64)
		return "arm64";
		#else
		return "unknown";
		#endif
	}
	std::string CompilerName()
	{
		#if defined(_MSC_FULL_VER)
		return "MSVC" + std::to_string(_MSC_FULL_VER);
		#else
		return "unknown";
		#endif
	}

	// --- Command line ------------------------------------------------------------------------------

	void PrintHelp()
	{
		std::cout <<
			"script_reader_benchmark - measures pr::script::Reader throughput and transport memory\n"
			"\n"
			"Usage: script_reader_benchmark [options]\n"
			"\n"
			"Options:\n"
			"  --help, -h                   Show this help and exit.\n"
			"  --size <small|large|both>    Fixture size(s) to run. Default: small.\n"
			"  --workloads <list>           Comma-separated workload names to run. Default: all.\n"
			"                               Names: number, ident-empty, ident-populated, strings, macros, boundary, includes\n"
			"  --inputs <list>              Comma-separated input modes. Default: memory.\n"
			"                               Names: memory, stream, file\n"
			"  --memory-scaling             Measure streamed transport memory at 10 and 100 MiB, then exit.\n"
			"  --warmups <N>                Warm-up parses. Default: 4, minimum: 1.\n"
			"  --reps <N>                   Measured parses. Default: 16, minimum: 3.\n"
			"  --seed <N>                   Base seed for deterministic fixture generation. Default: 20240517.\n"
			"\n"
			"Every repetition must produce the same checksum and item count. The default configuration\n"
			"(small size, all workloads) is designed to finish in about a minute.\n";
	}

	// Returns the value for an option, accepting both '--opt value' and '--opt=value' forms.
	std::string NextOptionValue(int argc, char** argv, int& i)
	{
		std::string_view arg = argv[i];
		auto eq = arg.find('=');
		if (eq != std::string_view::npos)
			return std::string(arg.substr(eq + 1));

		if (i + 1 >= argc)
			throw std::invalid_argument("missing value for option '" + std::string(arg) + "'");

		return argv[++i];
	}

	Options ParseArgs(int argc, char** argv)
	{
		Options opt;
		for (int i = 1; i != argc; ++i)
		{
			std::string_view arg = argv[i];
			if (arg == "--help" || arg == "-h")
			{
				opt.m_show_help = true;
			}
			else if (arg.starts_with("--size"))
			{
				auto value = NextOptionValue(argc, argv, i);
				if (value == "small") opt.m_sizes = { ESize::Small };
				else if (value == "large") opt.m_sizes = { ESize::Large };
				else if (value == "both") opt.m_sizes = { ESize::Small, ESize::Large };
				else throw std::invalid_argument("unknown --size value: " + value);
			}
			else if (arg.starts_with("--workloads"))
			{
				auto value = NextOptionValue(argc, argv, i);
				opt.m_workloads.clear();
				std::stringstream ss{ value };
				std::string tok;
				while (std::getline(ss, tok, ','))
				{
					auto w = ParseWorkloadName(tok);
					if (!w)
						throw std::invalid_argument("unknown workload name: " + tok);

					opt.m_workloads.push_back(*w);
				}
			}
			else if (arg.starts_with("--inputs"))
			{
				auto value = NextOptionValue(argc, argv, i);
				opt.m_input_modes.clear();
				std::stringstream ss{ value };
				std::string tok;
				while (std::getline(ss, tok, ','))
				{
					if (tok == "memory") opt.m_input_modes.push_back(EInputMode::Memory);
					else if (tok == "stream") opt.m_input_modes.push_back(EInputMode::Stream);
					else if (tok == "file") opt.m_input_modes.push_back(EInputMode::File);
					else throw std::invalid_argument("unknown input mode: " + tok);
				}
			}
			else if (arg == "--memory-scaling")
			{
				opt.m_memory_scaling = true;
			}
			else if (arg.starts_with("--warmups"))
			{
				opt.m_warmups = std::stoi(NextOptionValue(argc, argv, i));
			}
			else if (arg.starts_with("--reps"))
			{
				opt.m_reps = std::stoi(NextOptionValue(argc, argv, i));
			}
			else if (arg.starts_with("--seed"))
			{
				opt.m_seed = std::stoull(NextOptionValue(argc, argv, i));
			}
			else
			{
				throw std::invalid_argument("unknown option: " + std::string(arg) + " (use --help for usage)");
			}
		}
		return opt;
	}

	// --- Workload dispatch table ---------------------------------------------------------------

	// Plain function pointer to a workload driver.
	using DriveFn = ParseResult(*)(BenchmarkSource const&, SizeParams const&);

	// Return the parse driver for a workload.
	DriveFn GetWorkloadDriver(EWorkload w)
	{
		switch (w)
		{
			case EWorkload::NumberHeavy:
			{
				return &DriveNumberHeavy;
			}
			case EWorkload::IdentEmpty:
			case EWorkload::IdentPopulated:
			{
				return &DriveIdentifierHeavy;
			}
			case EWorkload::Strings:
			{
				return &DriveStringsUtf8;
			}
			case EWorkload::Macros:
			{
				return &DriveDirectiveMacroHeavy;
			}
			case EWorkload::Boundary:
			{
				return &DriveTokenBoundaryAdversary;
			}
			case EWorkload::Includes:
			{
				return &DriveIncludeTree;
			}
			default:
			{
				throw std::invalid_argument("unknown EWorkload value");
			}
		}
	}

	// Generates the fixture text for 'w'. Called once per (workload, size) before any timing starts.
	std::string GenerateFixture(EWorkload w, SizeParams const& p, uint64_t seed)
	{
		switch (w)
		{
			case EWorkload::NumberHeavy:     return GenerateNumberHeavy(p, seed);
			case EWorkload::IdentEmpty:      return GenerateIdentifierHeavy(p, seed, false);
			case EWorkload::IdentPopulated:  return GenerateIdentifierHeavy(p, seed, true);
			case EWorkload::Strings:         return GenerateStringsUtf8(p, seed);
			case EWorkload::Macros:          return GenerateDirectiveMacroHeavy(p, seed);
			case EWorkload::Boundary:        return GenerateTokenBoundaryAdversary(p, seed);
			case EWorkload::Includes:        return GenerateIncludeTreeRoot();
			default: throw std::invalid_argument("unknown EWorkload value");
		}
	}

	// --- Running one workload ------------------------------------------------------------------

	struct WorkloadRunOutcome
	{
		bool m_ok;
		std::vector<TimedSample> m_samples; // measured repetitions only
	};

	// Generate the fixture, warm the reader, measure complete parses, and verify stable results.
	WorkloadRunOutcome RunWorkload(EWorkload w, ESize size, EInputMode input_mode, SizeParams const& p, Options const& opt, uint64_t seed)
	{
		auto driver = GetWorkloadDriver(w);
		auto fixture = GenerateFixture(w, p, seed); // Untimed: fixture generation is never measured.
		auto filepath = std::filesystem::path{};
		auto include_root = std::filesystem::path{};
		auto total_bytes = uint64_t(fixture.size());
		bool ok = true;

		// Include-tree sources use production file resolvers while keeping all fixture generation outside timing.
		if (w == EWorkload::Includes)
		{
			auto nonce = std::chrono::high_resolution_clock::now().time_since_epoch().count();
			include_root = std::filesystem::temp_directory_path() / ("script-reader-includes-" + std::to_string(nonce));
			if (!std::filesystem::create_directory(include_root))
				throw std::runtime_error("failed to create include-tree directory: " + include_root.string());

			total_bytes += MaterializeIncludeTree(include_root, p);
			filepath = include_root / "root.ldr";
		}
		else if (input_mode == EInputMode::File)
		{
			auto nonce = std::chrono::high_resolution_clock::now().time_since_epoch().count();
			filepath = std::filesystem::temp_directory_path() / ("script-reader-benchmark-" + std::to_string(nonce) + ".ldr");
		}

		// File-backed runs materialize the generated root fixture before any timed reader construction.
		if (input_mode == EInputMode::File)
		{
			auto file = std::ofstream(filepath, std::ios::binary);
			file.write(fixture.data(), static_cast<std::streamsize>(fixture.size()));
			if (!file)
				throw std::runtime_error("failed to write benchmark fixture: " + filepath.string());
		}
		auto source = BenchmarkSource{ fixture, filepath, include_root, input_mode, total_bytes };

		// Warm-ups prime caches and branch predictors before any measured sample.
		try
		{
			for (int round = 0; round != opt.m_warmups; ++round)
			{
				auto result = driver(source, p);
				g_black_hole.fetch_xor(result.m_checksum.m_value, std::memory_order_relaxed);
			}
		}
		catch (std::exception const& e)
		{
			std::cerr << "ERROR: warm-up parse failed for workload '" << WorkloadName(w) << "' (" << SizeName(size) << "): " << e.what() << "\n";
			ok = false;
		}

		std::vector<TimedSample> samples;
		if (ok)
		{
			try
			{
				for (int round = 0; round != opt.m_reps; ++round)
				{
					// One parse per timing sample: construction and full extraction are both inside the timed region.
					auto t0 = std::chrono::steady_clock::now();
					auto result = driver(source, p);
					auto t1 = std::chrono::steady_clock::now();
					g_black_hole.fetch_xor(result.m_checksum.m_value, std::memory_order_relaxed);

					auto elapsed = std::chrono::duration<double>(t1 - t0).count();
					samples.push_back(TimedSample{ w, size, input_mode, round, result.m_bytes, result.m_items, elapsed, result.m_checksum.m_value });
				}
			}
			catch (std::exception const& e)
			{
				std::cerr << "ERROR: measured parse failed for workload '" << WorkloadName(w) << "' (" << SizeName(size) << "): " << e.what() << "\n";
				ok = false;
			}
		}

		if (ok)
		{
			// Every measured parse must produce the same semantic result.
			auto checksum = samples.front().m_checksum;
			auto items = samples.front().m_items;
			for (auto const& s : samples)
			{
				if (s.m_checksum != checksum || s.m_items != items)
				{
					std::cerr << "ERROR: inconsistent results across repetitions for workload '" << WorkloadName(w) << "' (" << SizeName(size) << ")\n";
					ok = false;
				}
			}
		}

		// Remove only the concrete temporary fixtures created for this workload.
		if (!include_root.empty())
		{
			std::error_code error;
			std::filesystem::remove_all(include_root, error);
			if (error)
				std::cerr << "WARNING: failed to remove temporary include tree '" << include_root << "': " << error.message() << "\n";
		}
		else if (!filepath.empty())
		{
			std::error_code error;
			std::filesystem::remove(filepath, error);
			if (error)
				std::cerr << "WARNING: failed to remove temporary fixture '" << filepath << "': " << error.message() << "\n";
		}
		if (!ok)
			samples.clear();

		return WorkloadRunOutcome{ ok, std::move(samples) };
	}

	// --- Reporting -----------------------------------------------------------------------------

	void PrintCsvHeader()
	{
		std::cout << "workload,size,input_mode,repetition,bytes,items,elapsed_s,bytes_per_s,items_per_s,checksum,block_size,config,arch,compiler\n";
	}

	void PrintCsvRow(TimedSample const& s)
	{
		double bytes_per_s = s.m_elapsed_s > 0.0 ? double(s.m_bytes) / s.m_elapsed_s : 0.0;
		double items_per_s = s.m_elapsed_s > 0.0 ? double(s.m_items) / s.m_elapsed_s : 0.0;

		std::cout
			<< WorkloadName(s.m_workload) << ','
			<< SizeName(s.m_size) << ','
			<< InputModeName(s.m_input_mode) << ','
			<< s.m_repetition << ','
			<< s.m_bytes << ','
			<< s.m_items << ','
			<< std::setprecision(9) << s.m_elapsed_s << ','
			<< std::setprecision(9) << bytes_per_s << ','
			<< std::setprecision(9) << items_per_s << ','
			<< "0x" << std::hex << s.m_checksum << std::dec << ','
			<< pr::script::reader::BlockSize << ','
			<< BuildConfigName() << ','
			<< ArchName() << ','
			<< CompilerName()
			<< '\n';
	}

	// Aggregate and report elapsed-time statistics for every workload, size, and input-mode combination.
	void PrintSummary(std::vector<TimedSample> const& samples)
	{
		using Key = std::tuple<EWorkload, ESize, EInputMode>;
		std::map<Key, std::vector<double>> elapsed_by_key;
		std::map<Key, uint64_t> bytes_by_key, items_by_key;

		for (auto const& s : samples)
		{
			Key key{ s.m_workload, s.m_size, s.m_input_mode };
			elapsed_by_key[key].push_back(s.m_elapsed_s);
			bytes_by_key[key] = s.m_bytes;
			items_by_key[key] = s.m_items;
		}

		std::cerr << "\n=== Summary (elapsed time per parse) ===\n";
		for (auto const& [key, elapsed] : elapsed_by_key)
		{
			auto stats = ComputeStats(elapsed);
			std::cerr
				<< WorkloadName(std::get<0>(key)) << " (" << SizeName(std::get<1>(key)) << ", " << InputModeName(std::get<2>(key)) << "), "
				<< bytes_by_key[key] << " bytes, " << items_by_key[key] << " items:\n"
				<< "  median=" << stats.m_median << "s  p10=" << stats.m_p10 << "s  p90=" << stats.m_p90
				<< "s  mean=" << stats.m_mean << "s  stddev=" << stats.m_stddev << "s  cv=" << stats.m_cv << "\n";
		}

		// Ordinary samples report wall-clock throughput; use '--memory-scaling' for transport capacity.
	}

	// Generate and parse large streamed sources whose semantic output remains one integer.
	void RunMemoryScaling()
	{
		constexpr size_t MiB = 1024 * 1024;
		auto block = std::array<char, 64 * 1024>{};
		block.fill(' ');
		std::cout << "input_bytes,elapsed_s,transport_peak_bytes,transport_retained_bytes,checksum,block_size\n";

		for (auto input_size : { 10 * MiB, 100 * MiB })
		{
			// Materialize the input without retaining an input-sized benchmark buffer.
			auto nonce = std::chrono::high_resolution_clock::now().time_since_epoch().count();
			auto filepath = std::filesystem::temp_directory_path() / ("script-reader-memory-" + std::to_string(nonce) + ".ldr");
			{
				auto file = std::ofstream(filepath, std::ios::binary);
				for (auto remaining = input_size - 1; remaining != 0;)
				{
					auto count = std::min(remaining, block.size());
					file.write(block.data(), static_cast<std::streamsize>(count));
					remaining -= count;
				}
				file.put('1');
				if (!file)
					throw std::runtime_error("failed to write memory-scaling fixture: " + filepath.string());
			}

			// Time one complete streamed parse and retain only the scalar semantic result.
			auto stream = std::ifstream(filepath, std::ios::binary);
			auto reader = pr::script::Reader(stream, false, filepath);
			auto value = int{};
			auto t0 = std::chrono::steady_clock::now();
			reader.Int(value, 10);
			auto t1 = std::chrono::steady_clock::now();
			auto elapsed = std::chrono::duration<double>(t1 - t0).count();
			auto checksum = Checksum{};
			checksum.Add(int64_t(value));

			std::cout
				<< input_size << ','
				<< std::setprecision(9) << elapsed << ','
				<< reader.TransportPeakBytes() << ','
				<< reader.TransportRetainedBytes() << ','
				<< "0x" << std::hex << checksum.m_value << std::dec << ','
				<< pr::script::reader::BlockSize << '\n';

			// Remove only the generated fixture after the stream releases its handle.
			stream.close();
			std::error_code error;
			std::filesystem::remove(filepath, error);
			if (error)
				throw std::runtime_error("failed to remove memory-scaling fixture: " + error.message());
		}
	}
}

int main(int argc, char** argv)
{
	using namespace pr::script_bench;
	try
	{
		auto opt = ParseArgs(argc, argv);
		if (opt.m_show_help)
		{
			PrintHelp();
			return 0;
		}
		if (opt.m_memory_scaling)
		{
			RunMemoryScaling();
			return 0;
		}
		if (opt.m_warmups < 1)
			throw std::invalid_argument("--warmups must be >= 1");
		if (opt.m_reps < 3)
			throw std::invalid_argument("--reps must be >= 3");

		PrintCsvHeader();

		bool overall_ok = true;
		std::vector<TimedSample> all_samples;

		for (auto size : opt.m_sizes)
		{
			auto p = GetSizeParams(size);
			for (auto input_mode : opt.m_input_modes)
			{
				for (auto w : opt.m_workloads)
				{
					// Each (workload, size) pair gets its own derived seed so results stay stable regardless
					// of which subset of workloads/sizes a given run selects.
					uint64_t seed = opt.m_seed
						+ uint64_t(w) * 0x9E3779B97F4A7C15ull
						+ uint64_t(size) * 0xD1B54A32D192ED03ull;

					auto outcome = RunWorkload(w, size, input_mode, p, opt, seed);
					for (auto const& s : outcome.m_samples)
						PrintCsvRow(s);

					all_samples.insert(all_samples.end(), outcome.m_samples.begin(), outcome.m_samples.end());
					overall_ok = overall_ok && outcome.m_ok;
				}
			}
		}

		PrintSummary(all_samples);

		std::cerr << "\nBlack-hole sink (ignore; proves outputs were consumed): 0x" << std::hex << g_black_hole.load() << std::dec << "\n";
		std::cerr << (overall_ok
			? "\nRESULT: PASS - all workload repetitions produced stable results.\n"
			: "\nRESULT: FAIL - see ERROR lines above for the failing workload(s).\n");

		return overall_ok ? 0 : 1;
	}
	catch (std::exception const& e)
	{
		std::cerr << "FATAL: " << e.what() << "\n";
		return 1;
	}
}
