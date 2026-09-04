//**********************************
// LDraw Text Reader Benchmark
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Measures the production LDraw text adapters independently of renderer/model creation.
#include "src/forward.h"
#include "src/checksum.h"
#include "src/statistics.h"
#include "src/workload_generators.h"

namespace pr::script_bench
{
	using namespace pr::rdr12::ldraw;

	// Command-line controls for the dense point workload.
	struct LDrawOptions
	{
		int m_point_count = 60000;
		int m_warmups = 4;
		int m_reps = 16;
	};

	// Return the active compiler configuration for reproducible result metadata.
	char const* LDrawBuildConfig()
	{
		#if defined(NDEBUG)
		return "Release";
		#else
		return "Debug";
		#endif
	}

	// Return the active target architecture for reproducible result metadata.
	char const* LDrawArchitecture()
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

	// Generate one compact-header point object containing dense vector data.
	std::string GenerateLDrawPoints(int point_count)
	{
		auto source = std::string("*Point DensePoints FF123456\n{\n*Data\n{\n");
		source.reserve(size_t(point_count) * 34 + 64);
		for (int index = 0; index != point_count; ++index)
		{
			AppendReal(source, index * 0.125);
			AppendReal(source, index * -0.25);
			AppendReal(source, index * 0.5);
			source += '\n';
		}
		source += "}\n}\n";
		return source;
	}

	// Consume the public LDraw reader surface and return an order-sensitive semantic fingerprint.
	Checksum FingerprintLDrawPoints(IReader& reader, int point_count)
	{
		auto checksum = Checksum{};
		auto keyword = EKeyword{};
		if (!reader.NextKeyword(keyword) || keyword != EKeyword::Point)
			throw std::runtime_error("point object expected");

		// Fingerprint compact header metadata and every dense point value through the shared interface.
		{
			auto section = reader.SectionScope();
			if (!reader.NextKeyword(keyword) || keyword != EKeyword::Name)
				throw std::runtime_error("point name expected");
			auto name = reader.Identifier<pr::rdr12::string32>();
			checksum.Add(std::string_view(name.data(), name.size()));
			if (!reader.NextKeyword(keyword) || keyword != EKeyword::Colour)
				throw std::runtime_error("point colour expected");
			checksum.Add(uint64_t(reader.Int<uint32_t>(16)));
			if (!reader.NextKeyword(keyword) || keyword != EKeyword::Data)
				throw std::runtime_error("point data expected");

			// Preserve exact float results and ordering in the semantic fingerprint.
			for (int index = 0; index != point_count; ++index)
			{
				auto point = reader.Vector3f();
				checksum.Add(point.x);
				checksum.Add(point.y);
				checksum.Add(point.z);
			}
		}
		if (reader.NextKeyword(keyword))
			throw std::runtime_error("unexpected trailing LDraw object");
		return checksum;
	}

	// Parse one production-shaped borrowed UTF-8 snapshot.
	Checksum RunTextReader(std::string const& source, int point_count)
	{
		auto reader = TextReader(source);
		return FingerprintLDrawPoints(reader, point_count);
	}

	// Parse supported benchmark options.
	LDrawOptions ParseLDrawOptions(int argc, char** argv)
	{
		auto options = LDrawOptions{};
		for (int index = 1; index != argc; ++index)
		{
			auto arg = std::string_view(argv[index]);
			if (index + 1 == argc)
				throw std::invalid_argument("missing value for option: " + std::string(arg));
			auto value = std::stoi(argv[++index]);
			if (arg == "--points")
				options.m_point_count = value;
			else if (arg == "--warmups")
				options.m_warmups = value;
			else if (arg == "--reps")
				options.m_reps = value;
			else
				throw std::invalid_argument("unknown option: " + std::string(arg));
		}

		// Keep the workload and sample set meaningful.
		if (options.m_point_count <= 0)
			throw std::invalid_argument("--points must be positive");
		if (options.m_warmups < 1)
			throw std::invalid_argument("--warmups must be at least 1");
		if (options.m_reps < 3)
			throw std::invalid_argument("--reps must be at least 3");
		return options;
	}
}

int main(int argc, char** argv)
{
	using namespace pr::script_bench;
	try
	{
		// Generate once so timing includes only adapter construction and extraction.
		auto options = ParseLDrawOptions(argc, argv);
		auto source = GenerateLDrawPoints(options.m_point_count);
		auto expected = RunTextReader(source, options.m_point_count);

		// Revalidate every timed parse rather than relying only on the initial equivalence check.
		auto run = [&]
		{
			auto checksum = RunTextReader(source, options.m_point_count);
			if (checksum.m_value != expected.m_value)
				throw std::runtime_error("LDraw fingerprint changed during measurement");
			return checksum;
		};

		// Warm the parser before collecting samples.
		for (int round = 0; round != options.m_warmups; ++round)
		{
			auto checksum = run();
			g_black_hole.fetch_xor(checksum.m_value, std::memory_order_relaxed);
		}

		// Measure complete adapter construction and parsing.
		auto elapsed = std::vector<double>{};
		elapsed.reserve(options.m_reps);
		for (int round = 0; round != options.m_reps; ++round)
		{
			auto start = std::chrono::steady_clock::now();
			auto checksum = run();
			auto stop = std::chrono::steady_clock::now();
			g_black_hole.fetch_xor(checksum.m_value, std::memory_order_relaxed);
			elapsed.push_back(std::chrono::duration<double>(stop - start).count());
		}

		// Report the measured adapter-level result and semantic checksum.
		auto stats = ComputeStats(elapsed);
		std::cout
			<< "points,bytes,median_s,p10_s,p90_s,mean_s,stddev_s,cv,checksum,config,arch\n"
			<< options.m_point_count << ','
			<< source.size() << ','
			<< std::setprecision(9) << stats.m_median << ','
			<< stats.m_p10 << ','
			<< stats.m_p90 << ','
			<< stats.m_mean << ','
			<< stats.m_stddev << ','
			<< stats.m_cv << ','
			<< "0x" << std::hex << expected.m_value << std::dec << ','
			<< LDrawBuildConfig() << ',' << LDrawArchitecture() << '\n';
		return 0;
	}
	catch (std::exception const& ex)
	{
		std::cerr << "ERROR: " << ex.what() << '\n';
		return 1;
	}
}
