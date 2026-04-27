#include "src/forward.h"
#include "pr/collision/collision.h"
#include "src/diagnostics/stack_diagnostic.h"
#include "src/scene/scene.h"
#include "src/utils/scene_loader.h"

namespace physics_sandbox::diag
{
	namespace
	{
		struct PenetrationSample
		{
			float m_max_depth = 0.0f;
			int m_contact_count = 0;
			int m_body_a = -1;
			int m_body_b = -1;
			float m_kinetic_energy = 0.0f;
		};

		PenetrationSample MeasurePenetration(Scene const& scene)
		{
			auto sample = PenetrationSample{};

			for (auto const& body : scene.m_body)
				sample.m_kinetic_energy += body.KineticEnergy();

			for (int i = 0; i != std::ssize(scene.m_body); ++i)
			{
				auto const& body_a = scene.m_body[i];
				if (!body_a.HasShape())
					continue;

				for (int j = i + 1; j != std::ssize(scene.m_body); ++j)
				{
					auto const& body_b = scene.m_body[j];
					if (!body_b.HasShape())
						continue;

					auto contact = collision::Contact{};
					if (!collision::Collide(body_a.Shape(), body_a.O2W(), body_b.Shape(), body_b.O2W(), contact))
						continue;

					++sample.m_contact_count;
					if (contact.m_depth > sample.m_max_depth)
					{
						sample.m_max_depth = contact.m_depth;
						sample.m_body_a = i;
						sample.m_body_b = j;
					}
				}
			}

			return sample;
		}

		void Emit(std::ofstream& log, std::string_view text)
		{
			printf("%.*s", static_cast<int>(text.size()), text.data());
			if (log)
				log << text;
		}

		void PrintSample(std::ofstream& log, int step, double time_s, PenetrationSample const& sample)
		{
			Emit(log, std::format("{:5d}  {:8.4f}  {:4d}  {:10.6f}  {:3d}  {:3d}  {:12.6f}\n",
				step,
				time_s,
				sample.m_contact_count,
				sample.m_max_depth,
				sample.m_body_a,
				sample.m_body_b,
				sample.m_kinetic_energy));
		}
	}

	StackDiagnosticResult RunStackDiagnostic(StackDiagnosticOptions const& options)
	{
		auto log_path = AppDataPath() / "stackdiag.log";
		auto log = std::ofstream(log_path, std::ios::out | std::ios::trunc);
		auto result = StackDiagnosticResult{};

		Emit(log, std::format("Stack diagnostic log: {}\n", log_path.string()));
		Emit(log, std::format("Stack diagnostic scene: {}\n", options.m_scene_filepath.string()));
		Emit(log, std::format("steps={} dt={:.8f} report_interval={}\n", options.m_steps, options.m_dt, options.m_report_interval));
		Emit(log, " step    time_s  hits   max_depth    a    b            KE\n");

		if (!std::filesystem::exists(options.m_scene_filepath))
			throw std::runtime_error(std::format("Scene file not found: {}", options.m_scene_filepath.string()));

		auto scene_desc = scene_loader::LoadFromFile(options.m_scene_filepath);
		auto scene = Scene(nullptr);
		scene.LoadScene(std::move(scene_desc));

		for (int step = 0; step != options.m_steps; ++step)
		{
			scene.Step(options.m_dt);

			auto sample = MeasurePenetration(scene);
			if (sample.m_max_depth > result.m_max_depth)
			{
				result.m_max_depth = sample.m_max_depth;
				result.m_max_depth_step = step + 1;
				result.m_body_a = sample.m_body_a;
				result.m_body_b = sample.m_body_b;
			}

			auto report_interval = std::max(options.m_report_interval, 1);
			if ((step + 1) % report_interval == 0 || step + 1 == options.m_steps)
				PrintSample(log, step + 1, scene.m_clock, sample);
		}

		Emit(log, std::format("worst: step={} max_depth={:.6f} pair=({},{})\n",
			result.m_max_depth_step,
			result.m_max_depth,
			result.m_body_a,
			result.m_body_b));

		return result;
	}
}
