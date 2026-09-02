//**********************************
// Script Reader Benchmark
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Standalone Release/x64 console benchmark that times the legacy 'pr::script::Reader' against the
// new 'pr::script::v2::Reader' ("Reader2") across deterministic memory-, stream-, and file-backed workloads.
//
// Style Guidance:
//  - This benchmark exists purely to measure and report; it must never modify 'pr/script/reader.h' or
//    'pr/script/reader2/*'. It links against both readers unmodified and only owns the fixture
//    generators, drivers, timing/statistics, and reporting in this project.
//  - Workload fixtures are generated once per (workload, size) before any timing starts (see
//    'workload_generators.h'); only the construct-and-parse step performed by 'reader_drivers.h' is
//    ever timed.
//  - Backends are always run in ABBA order (alternating which backend goes first each round) for
//    both the warm-up and measured phases, so slow drift across the run (thermal throttling, OS
//    scheduling noise, cache state) cannot systematically favour one backend.
//  - Every measured parse is verified for internal consistency (repeated parses of one backend must
//    all agree) and cross-backend equality (legacy and Reader2 must produce the same checksum and
//    item count) before any speedup is reported; the process exits non-zero if any workload fails
//    that verification or throws during parsing.
#include "src/forward.h"
#include "src/checksum.h"
#include "src/workload_generators.h"
#include "src/reader_drivers.h"

namespace pr::script_bench
{
	// Which reader implementation produced a given timed sample.
	enum class EBackend
	{
		Legacy,
		New,
	};

	// Parsed command-line configuration.
	struct Options
	{
		bool m_show_help = false;
		std::vector<ESize> m_sizes = { ESize::Small };
		std::vector<EWorkload> m_workloads =
		{
			EWorkload::NumberHeavy, EWorkload::IdentEmpty, EWorkload::IdentPopulated,
			EWorkload::Strings, EWorkload::Macros, EWorkload::Boundary,
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
		EBackend m_backend;
		EInputMode m_input_mode;
		int m_repetition;
		uint64_t m_bytes;
		uint64_t m_items;
		double m_elapsed_s;
		uint64_t m_checksum;
	};

	// Robust summary statistics over a set of elapsed-time samples.
	struct Stats
	{
		double m_median;
		double m_p10;
		double m_p90;
		double m_mean;
		double m_stddev;
		double m_cv; // coefficient of variation: stddev / mean
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
	char const* BackendName(EBackend b)
	{
		switch (b)
		{
			case EBackend::Legacy: return "legacy";
			case EBackend::New:    return "reader2";
			default: throw std::invalid_argument("unknown EBackend value");
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
			"script_reader_benchmark - times pr::script::Reader (legacy) against pr::script::v2::Reader (Reader2)\n"
			"\n"
			"Usage: script_reader_benchmark [options]\n"
			"\n"
			"Options:\n"
			"  --help, -h                   Show this help and exit.\n"
			"  --size <small|large|both>    Fixture size(s) to run. Default: small.\n"
			"  --workloads <list>           Comma-separated workload names to run. Default: all.\n"
			"                               Names: number, ident-empty, ident-populated, strings, macros, boundary\n"
			"  --inputs <list>              Comma-separated input modes. Default: memory.\n"
			"                               Names: memory, stream, file\n"
			"  --memory-scaling             Measure Reader2 transport memory at 10 and 100 MiB, then exit.\n"
			"  --warmups <N>                Warm-up parses per backend. Default: 4, minimum: 4, must be even.\n"
			"  --reps <N>                   Measured parses per backend. Default: 16, minimum: 16, must be even.\n"
			"  --seed <N>                   Base seed for deterministic fixture generation. Default: 20240517.\n"
			"\n"
			"Backends are timed in ABBA order (alternating which goes first each round). Every workload's\n"
			"legacy and Reader2 checksums and item counts must match, or the process exits with a non-zero\n"
			"status. The default configuration (small size, all workloads) is designed to finish in about a\n"
			"minute.\n";
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

	// Plain function pointer to a fully-instantiated 'Drive*<ReaderType>' specialisation: dispatching
	// through this indirection (rather than branching on backend inside the timed region) keeps the
	// instantiated parse code itself branch-free with respect to which backend is running.
	using DriveFn = ParseResult(*)(BenchmarkSource const&, SizeParams const&);

	struct WorkloadSpec
	{
		DriveFn m_legacy;
		DriveFn m_new;
	};

	WorkloadSpec GetWorkloadSpec(EWorkload w)
	{
		switch (w)
		{
			case EWorkload::NumberHeavy:
			{
				return WorkloadSpec{ &DriveNumberHeavy<pr::script::Reader>, &DriveNumberHeavy<pr::script::v2::Reader> };
			}
			case EWorkload::IdentEmpty:
			case EWorkload::IdentPopulated:
			{
				// Same extraction sequence for both; only the generated fixture text differs.
				return WorkloadSpec{ &DriveIdentifierHeavy<pr::script::Reader>, &DriveIdentifierHeavy<pr::script::v2::Reader> };
			}
			case EWorkload::Strings:
			{
				return WorkloadSpec{ &DriveStringsUtf8<pr::script::Reader>, &DriveStringsUtf8<pr::script::v2::Reader> };
			}
			case EWorkload::Macros:
			{
				return WorkloadSpec{ &DriveDirectiveMacroHeavy<pr::script::Reader>, &DriveDirectiveMacroHeavy<pr::script::v2::Reader> };
			}
			case EWorkload::Boundary:
			{
				return WorkloadSpec{ &DriveTokenBoundaryAdversary<pr::script::Reader>, &DriveTokenBoundaryAdversary<pr::script::v2::Reader> };
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
			default: throw std::invalid_argument("unknown EWorkload value");
		}
	}

	// --- ABBA scheduling ---------------------------------------------------------------------------

	// Invokes 'run_one(backend, round)' 'count' times per backend, alternating which backend runs
	// first each round (round 0: Legacy,New; round 1: New,Legacy; round 2: Legacy,New; ...), so
	// systematic drift over the run affects both backends symmetrically.
	template <typename F>
	void RunAbba(int count, F&& run_one)
	{
		for (int round = 0; round != count; ++round)
		{
			bool forward = (round % 2) == 0;
			run_one(forward ? EBackend::Legacy : EBackend::New, round);
			run_one(forward ? EBackend::New : EBackend::Legacy, round);
		}
	}

	// --- Running one workload ------------------------------------------------------------------

	struct WorkloadRunOutcome
	{
		bool m_ok;
		std::vector<TimedSample> m_samples; // measured repetitions only
	};

	// Generates the fixture, runs ABBA warm-ups (discarded) then ABBA measured repetitions (recorded),
	// and verifies both per-backend internal consistency and cross-backend equality before returning.
	WorkloadRunOutcome RunWorkload(EWorkload w, ESize size, EInputMode input_mode, SizeParams const& p, Options const& opt, uint64_t seed)
	{
		auto spec = GetWorkloadSpec(w);
		auto fixture = GenerateFixture(w, p, seed); // Untimed: fixture generation is never measured.
		auto filepath = std::filesystem::path{};
		bool ok = true;

		// File-backed runs materialize the generated fixture before any timed reader construction.
		if (input_mode == EInputMode::File)
		{
			auto nonce = std::chrono::high_resolution_clock::now().time_since_epoch().count();
			filepath = std::filesystem::temp_directory_path() / ("reader2-benchmark-" + std::to_string(nonce) + ".ldr");
			auto file = std::ofstream(filepath, std::ios::binary);
			file.write(fixture.data(), static_cast<std::streamsize>(fixture.size()));
			if (!file)
				throw std::runtime_error("failed to write benchmark fixture: " + filepath.string());
		}
		auto source = BenchmarkSource{ fixture, filepath, input_mode };

		// Warm-ups prime caches/branch predictors for both backends before any measured sample; failures
		// here still abort the workload since a warm-up that throws indicates a broken fixture/driver.
		try
		{
			RunAbba(opt.m_warmups, [&](EBackend backend, int)
			{
				auto fn = backend == EBackend::Legacy ? spec.m_legacy : spec.m_new;
				auto result = fn(source, p);
				g_black_hole.fetch_xor(result.m_checksum.m_value, std::memory_order_relaxed);
			});
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
				RunAbba(opt.m_reps, [&](EBackend backend, int round)
				{
					auto fn = backend == EBackend::Legacy ? spec.m_legacy : spec.m_new;

					// One parse per timing sample: construction and full extraction are both inside the timed region.
					auto t0 = std::chrono::steady_clock::now();
					auto result = fn(source, p);
					auto t1 = std::chrono::steady_clock::now();
					g_black_hole.fetch_xor(result.m_checksum.m_value, std::memory_order_relaxed);

					auto elapsed = std::chrono::duration<double>(t1 - t0).count();
					samples.push_back(TimedSample{ w, size, backend, input_mode, round, result.m_bytes, result.m_items, elapsed, result.m_checksum.m_value });
				});
			}
			catch (std::exception const& e)
			{
				std::cerr << "ERROR: measured parse failed for workload '" << WorkloadName(w) << "' (" << SizeName(size) << "): " << e.what() << "\n";
				ok = false;
			}
		}

		if (ok)
		{
			// Internal consistency: every repetition of a given backend must produce the same checksum/items.
			uint64_t legacy_cs = 0, new_cs = 0, legacy_items = 0, new_items = 0;
			bool have_legacy = false, have_new = false;
			for (auto const& s : samples)
			{
				bool is_legacy = s.m_backend == EBackend::Legacy;
				auto& have = is_legacy ? have_legacy : have_new;
				auto& cs = is_legacy ? legacy_cs : new_cs;
				auto& items = is_legacy ? legacy_items : new_items;
				if (!have)
				{
					cs = s.m_checksum;
					items = s.m_items;
					have = true;
				}
				else if (s.m_checksum != cs || s.m_items != items)
				{
					std::cerr << "ERROR: " << BackendName(s.m_backend) << " produced inconsistent results across repetitions for workload '"
						<< WorkloadName(w) << "' (" << SizeName(size) << ")\n";
					ok = false;
				}
			}

			// Cross-backend equality: legacy and Reader2 must agree on both checksum and item count.
			if (ok && (legacy_cs != new_cs || legacy_items != new_items))
			{
				std::cerr
					<< "ERROR: checksum/item-count mismatch for workload '" << WorkloadName(w) << "' (" << SizeName(size) << "): "
					<< "legacy checksum=0x" << std::hex << legacy_cs << std::dec << " items=" << legacy_items
					<< " vs reader2 checksum=0x" << std::hex << new_cs << std::dec << " items=" << new_items << "\n";
				ok = false;
			}
		}

		// Remove only the concrete temporary fixture created for this workload.
		if (!filepath.empty())
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

	// --- Statistics ----------------------------------------------------------------------------

	Stats ComputeStats(std::vector<double> values)
	{
		std::sort(values.begin(), values.end());
		auto n = values.size();

		// Linear-interpolated percentile between the two nearest order statistics.
		auto percentile = [&](double q)
		{
			double idx = q * double(n - 1);
			auto lo = size_t(std::floor(idx));
			auto hi = size_t(std::ceil(idx));
			double frac = idx - double(lo);
			return values[lo] + (values[hi] - values[lo]) * frac;
		};

		double mean = std::accumulate(values.begin(), values.end(), 0.0) / double(n);
		double variance = 0.0;
		for (auto v : values)
			variance += (v - mean) * (v - mean);
		variance /= double(n);

		double stddev = std::sqrt(variance);
		return Stats
		{
			.m_median = percentile(0.5),
			.m_p10 = percentile(0.10),
			.m_p90 = percentile(0.90),
			.m_mean = mean,
			.m_stddev = stddev,
			.m_cv = mean != 0.0 ? stddev / mean : 0.0,
		};
	}

	// --- Reporting -----------------------------------------------------------------------------

	void PrintCsvHeader()
	{
		std::cout << "workload,size,input_mode,backend,repetition,bytes,items,elapsed_s,bytes_per_s,items_per_s,checksum,block_size,config,arch,compiler\n";
	}

	void PrintCsvRow(TimedSample const& s)
	{
		double bytes_per_s = s.m_elapsed_s > 0.0 ? double(s.m_bytes) / s.m_elapsed_s : 0.0;
		double items_per_s = s.m_elapsed_s > 0.0 ? double(s.m_items) / s.m_elapsed_s : 0.0;

		std::cout
			<< WorkloadName(s.m_workload) << ','
			<< SizeName(s.m_size) << ','
			<< InputModeName(s.m_input_mode) << ','
			<< BackendName(s.m_backend) << ','
			<< s.m_repetition << ','
			<< s.m_bytes << ','
			<< s.m_items << ','
			<< std::setprecision(9) << s.m_elapsed_s << ','
			<< std::setprecision(9) << bytes_per_s << ','
			<< std::setprecision(9) << items_per_s << ','
			<< "0x" << std::hex << s.m_checksum << std::dec << ','
			<< pr::script::v2::BlockSize << ','
			<< BuildConfigName() << ','
			<< ArchName() << ','
			<< CompilerName()
			<< '\n';
	}

	// Aggregates every (workload, size) pair's legacy/reader2 elapsed-time samples, prints per-pair
	// stats and speedup, flags >5% regressions, and prints the geometric-mean Reader-only speedup.
	void PrintSummary(std::vector<TimedSample> const& samples)
	{
		using Key = std::tuple<EWorkload, ESize, EInputMode>;
		std::map<Key, std::vector<double>> legacy_elapsed, new_elapsed;
		std::map<Key, uint64_t> bytes_by_key, items_by_key;

		for (auto const& s : samples)
		{
			Key key{ s.m_workload, s.m_size, s.m_input_mode };
			(s.m_backend == EBackend::Legacy ? legacy_elapsed : new_elapsed)[key].push_back(s.m_elapsed_s);
			bytes_by_key[key] = s.m_bytes;
			items_by_key[key] = s.m_items;
		}

		std::cerr << "\n=== Summary (elapsed time per parse) ===\n";
		std::vector<double> speedups;
		bool any_regression = false;

		for (auto const& [key, legacy_samples] : legacy_elapsed)
		{
			auto it_new = new_elapsed.find(key);
			if (it_new == new_elapsed.end())
				continue; // Workload failed before producing any Reader2 samples; already reported above.

			auto legacy_stats = ComputeStats(legacy_samples);
			auto new_stats = ComputeStats(it_new->second);
			double speedup = legacy_stats.m_median / new_stats.m_median; // >1 means Reader2 is faster.
			speedups.push_back(speedup);

			std::cerr
				<< WorkloadName(std::get<0>(key)) << " (" << SizeName(std::get<1>(key)) << ", " << InputModeName(std::get<2>(key)) << "), "
				<< bytes_by_key[key] << " bytes, " << items_by_key[key] << " items:\n"
				<< "  legacy : median=" << legacy_stats.m_median << "s  p10=" << legacy_stats.m_p10 << "s  p90=" << legacy_stats.m_p90
				<< "s  mean=" << legacy_stats.m_mean << "s  stddev=" << legacy_stats.m_stddev << "s  cv=" << legacy_stats.m_cv << "\n"
				<< "  reader2: median=" << new_stats.m_median << "s  p10=" << new_stats.m_p10 << "s  p90=" << new_stats.m_p90
				<< "s  mean=" << new_stats.m_mean << "s  stddev=" << new_stats.m_stddev << "s  cv=" << new_stats.m_cv << "\n"
				<< "  speedup (legacy median / reader2 median): " << speedup << "x\n";

			if (speedup < (1.0 / 1.05))
			{
				any_regression = true;
				std::cerr << "  ** REGRESSION: reader2 is " << ((1.0 / speedup - 1.0) * 100.0) << "% slower than legacy (exceeds the 5% threshold) **\n";
			}
		}

		if (!speedups.empty())
		{
			double log_sum = std::accumulate(speedups.begin(), speedups.end(), 0.0, [](double acc, double s) { return acc + std::log(s); });
			double geo_mean = std::exp(log_sum / double(speedups.size()));
			std::cerr << "\nGeometric mean Reader-only speedup across " << speedups.size() << " workload/size combination(s): " << geo_mean << "x\n";
		}
		if (any_regression)
		{
			std::cerr << "\nWARNING: one or more workloads show a >5% regression for reader2 versus legacy.\n";
		}

		// Note: no memory/allocation telemetry is captured here (see the project README / final report);
		// ordinary comparative samples intentionally report only wall-clock throughput. Use
		// '--memory-scaling' for Reader2's transport peak and retained capacity.
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
			auto filepath = std::filesystem::temp_directory_path() / ("reader2-memory-" + std::to_string(nonce) + ".ldr");
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
			auto reader = pr::script::v2::Reader(stream, false, filepath);
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
				<< pr::script::v2::BlockSize << '\n';

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
		if (opt.m_warmups < 4 || (opt.m_warmups % 2) != 0)
			throw std::invalid_argument("--warmups must be even and >= 4");
		if (opt.m_reps < 16 || (opt.m_reps % 2) != 0)
			throw std::invalid_argument("--reps must be even and >= 16");

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
			? "\nRESULT: PASS - all workloads matched between legacy and reader2.\n"
			: "\nRESULT: FAIL - see ERROR lines above for the mismatching workload(s).\n");

		return overall_ok ? 0 : 1;
	}
	catch (std::exception const& e)
	{
		std::cerr << "FATAL: " << e.what() << "\n";
		return 1;
	}
}
