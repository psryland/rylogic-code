#include "src/forward.h"
#include "pr/collision/collision.h"
#include "src/diagnostics/scene_diagnostic.h"
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

		struct BodyTraceState
		{
			m4x4 m_o2w = {};
			v4 m_pos = {};
			v4 m_lin_vel = {};
			v4 m_ang_vel = {};
			float m_mass = 0.0f;
			float m_kinetic_energy = 0.0f;
			bool m_sleeping = false;
		};

		struct BodyTraceContact
		{
			int m_contact_index = 0;
			int m_body_a = -1;
			int m_body_b = -1;
			float m_depth = 0.0f;
			v4 m_axis_a = {};
			v4 m_point_a = {};
			int m_feature_count = 0;
		};
		struct SleepScanSample
		{
			int m_dynamic_count = 0;
			int m_sleeping_count = 0;
			int m_low_velocity_count = 0;
			int m_never_sleep_count = 0;
			int m_max_lin_body = -1;
			int m_max_ang_body = -1;
			float m_avg_lin_speed = 0.0f;
			float m_avg_ang_speed = 0.0f;
			float m_p50_lin_speed = 0.0f;
			float m_p90_lin_speed = 0.0f;
			float m_p95_lin_speed = 0.0f;
			float m_p99_lin_speed = 0.0f;
			float m_p50_ang_speed = 0.0f;
			float m_p90_ang_speed = 0.0f;
			float m_p95_ang_speed = 0.0f;
			float m_p99_ang_speed = 0.0f;
			float m_max_lin_speed = 0.0f;
			float m_max_ang_speed = 0.0f;
		};
		struct ColumnMetricState
		{
			int m_top_body = -1;
			int m_body_count = 0;
			float m_ground_height = 0.0f;
			float m_ideal_top_height = 0.0f;
			float m_initial_top_height = 0.0f;
			float m_min_top_height = std::numeric_limits<float>::max();
			double m_physics_ms = 0.0;
			int m_sample_count = 0;

			bool Enabled() const
			{
				return m_top_body != -1;
			}
		};
		struct PyramidMetricBody
		{
			int m_body = -1;
			v4 m_initial_pos = {};
			float m_height = 0.0f;
			float m_fallen_radius = 0.0f;
		};
		struct PyramidMetricState
		{
			std::vector<PyramidMetricBody> m_bodies = {};
			float m_initial_top_height = 0.0f;
			float m_initial_spread = 0.0f;
			double m_physics_ms = 0.0;
			int m_sample_count = 0;

			bool Enabled() const
			{
				return !m_bodies.empty();
			}
		};

		struct EngineProfileAccumulator
		{
			int m_sample_count = 0;
			int m_contact_count = 0;
			double m_scene_step_ms = 0.0;
			double m_physics_ms = 0.0;
			physics::Engine::StepProfile m_engine = {};

			void Add(Scene::StepProfile const& profile, int contact_count)
			{
				++m_sample_count;
				m_contact_count += contact_count;
				m_scene_step_ms += profile.m_total_ms;
				m_physics_ms += profile.m_physics_ms;
				m_engine.m_new_frame_ms += profile.m_engine.m_new_frame_ms;
				m_engine.m_pack_ms += profile.m_engine.m_pack_ms;
				m_engine.m_upload_ms += profile.m_engine.m_upload_ms;
				m_engine.m_integrate_ms += profile.m_engine.m_integrate_ms;
				m_engine.m_sleepwake_ms += profile.m_engine.m_sleepwake_ms;
				m_engine.m_broadphase_ms += profile.m_engine.m_broadphase_ms;
				m_engine.m_collide_ms += profile.m_engine.m_collide_ms;
				m_engine.m_resolve_ms += profile.m_engine.m_resolve_ms;
				m_engine.m_selective_ms += profile.m_engine.m_selective_ms;
				m_engine.m_sleepupdate_ms += profile.m_engine.m_sleepupdate_ms;
				m_engine.m_readback_ms += profile.m_engine.m_readback_ms;
				m_engine.m_gpu_run_ms += profile.m_engine.m_gpu_run_ms;
				m_engine.m_unpack_ms += profile.m_engine.m_unpack_ms;
			}

			void Reset()
			{
				*this = {};
			}
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
		float BoxHeight(scene_loader::BodyDesc const& body)
		{
			return body.box_dimensions.z;
		}
		bool SameColumnXY(scene_loader::BodyDesc const& lhs, scene_loader::BodyDesc const& rhs)
		{
			auto const dx = lhs.position.x - rhs.position.x;
			auto const dy = lhs.position.y - rhs.position.y;
			auto const xy_tolerance = 0.01f;
			return dx * dx + dy * dy < xy_tolerance * xy_tolerance;
		}
		ColumnMetricState CreateColumnMetric(scene_loader::SceneDesc const& scene_desc)
		{
			auto metric = ColumnMetricState{};
			metric.m_ground_height = scene_desc.ground ? scene_desc.ground->height : 0.0f;

			auto top_body = -1;
			auto top_height = -std::numeric_limits<float>::max();
			for (int i = 0; i != std::ssize(scene_desc.bodies); ++i)
			{
				auto const& body = scene_desc.bodies[i];
				if (body.mass <= 0.0f || body.shape_type != scene_loader::BodyDesc::EShape::Box)
					continue;

				if (body.position.z > top_height)
				{
					top_height = body.position.z;
					top_body = i;
				}
			}
			if (top_body == -1)
				throw std::runtime_error("Column metric requires at least one dynamic box body");

			auto const& top = scene_desc.bodies[top_body];
			auto supporting_height = 0.0f;
			auto body_count = 0;
			for (int i = 0; i != std::ssize(scene_desc.bodies); ++i)
			{
				auto const& body = scene_desc.bodies[i];
				if (body.mass <= 0.0f || body.shape_type != scene_loader::BodyDesc::EShape::Box)
					continue;
				if (!SameColumnXY(body, top))
					continue;
				if (body.position.z > top.position.z + 0.01f)
					continue;

				++body_count;
				if (i != top_body)
					supporting_height += BoxHeight(body);
			}

			metric.m_top_body = top_body;
			metric.m_body_count = body_count;
			metric.m_initial_top_height = top.position.z;
			metric.m_ideal_top_height = metric.m_ground_height + supporting_height + 0.5f * BoxHeight(top);
			metric.m_min_top_height = top.position.z;
			return metric;
		}
		float BoxMaxDimension(scene_loader::BodyDesc const& body)
		{
			return std::max(body.box_dimensions.x, std::max(body.box_dimensions.y, body.box_dimensions.z));
		}
		PyramidMetricState CreatePyramidMetric(scene_loader::SceneDesc const& scene_desc)
		{
			auto metric = PyramidMetricState{};
			metric.m_initial_top_height = -std::numeric_limits<float>::max();

			for (int i = 0; i != std::ssize(scene_desc.bodies); ++i)
			{
				auto const& body = scene_desc.bodies[i];
				if (body.mass <= 0.0f || body.shape_type != scene_loader::BodyDesc::EShape::Box)
					continue;

				// The pyramid metric treats the initial dynamic-box layout as the intended lattice. Persistent drift, sag, or lateral
				// spread then becomes a whole-structure stability signal instead of a single transient top-body sample.
				auto const xy_radius_sq = body.position.x * body.position.x + body.position.y * body.position.y;
				metric.m_initial_spread = std::max(metric.m_initial_spread, std::sqrt(xy_radius_sq));
				metric.m_initial_top_height = std::max(metric.m_initial_top_height, body.position.z);
				metric.m_bodies.push_back(PyramidMetricBody{
					.m_body = i,
					.m_initial_pos = body.position,
					.m_height = body.box_dimensions.z,
					.m_fallen_radius = std::max(0.5f, 0.75f * BoxMaxDimension(body)),
				});
			}

			if (!metric.Enabled())
				throw std::runtime_error("Pyramid metric requires at least one dynamic box body");

			return metric;
		}

		void Emit(std::ofstream& log, std::string_view text)
		{
			printf("%.*s", static_cast<int>(text.size()), text.data());
			if (log)
				log << text;
		}

		BodyTraceState CaptureBodyState(physics::RigidBody const& body)
		{
			auto vel = body.VelocityWS();
			return BodyTraceState{
				.m_o2w = body.O2W(),
				.m_pos = body.O2W().pos,
				.m_lin_vel = vel.lin,
				.m_ang_vel = vel.ang,
				.m_mass = body.Mass(),
				.m_kinetic_energy = body.KineticEnergy(),
				.m_sleeping = body.Sleeping(),
			};
		}

		float TotalKineticEnergy(Scene const& scene)
		{
			auto kinetic_energy = 0.0f;
			for (auto const& body : scene.m_body)
				kinetic_energy += body.KineticEnergy();

			return kinetic_energy;
		}
		SleepScanSample MeasureSleep(Scene const& scene, bool non_spheres_only)
		{
			auto sample = SleepScanSample{};
			auto const& config = scene.m_physics.Config();
			auto lin_speeds = std::vector<float>{};
			auto ang_speeds = std::vector<float>{};
			lin_speeds.reserve(scene.m_body.size());
			ang_speeds.reserve(scene.m_body.size());

			for (int i = 0; i != std::ssize(scene.m_body); ++i)
			{
				auto const& body = scene.m_body[i];
				if (body.InvMass() <= 0.0f)
					continue;
				if (non_spheres_only && body.HasShape() && body.Shape().m_type == collision::EShape::Sphere)
					continue;

				auto vel = body.VelocityWS();
				auto lin_speed = Length(vel.lin);
				auto ang_speed = Length(vel.ang);
				auto low_velocity =
					lin_speed < config.sleep_velocity_threshold_lin &&
					ang_speed < config.sleep_velocity_threshold_ang;

				sample.m_dynamic_count += 1;
				sample.m_sleeping_count += body.Sleeping() ? 1 : 0;
				sample.m_low_velocity_count += low_velocity ? 1 : 0;
				sample.m_never_sleep_count += body.NeverSleep() ? 1 : 0;
				sample.m_avg_lin_speed += lin_speed;
				sample.m_avg_ang_speed += ang_speed;
				lin_speeds.push_back(lin_speed);
				ang_speeds.push_back(ang_speed);
				if (lin_speed > sample.m_max_lin_speed)
				{
					sample.m_max_lin_speed = lin_speed;
					sample.m_max_lin_body = i;
				}
				if (ang_speed > sample.m_max_ang_speed)
				{
					sample.m_max_ang_speed = ang_speed;
					sample.m_max_ang_body = i;
				}
			}
			if (sample.m_dynamic_count != 0)
			{
				auto percentile = [](std::vector<float>& speeds, float pct)
				{
					std::sort(std::begin(speeds), std::end(speeds));
					auto idx = static_cast<int>(std::round(pct * (std::ssize(speeds) - 1)));
					return speeds[std::clamp(idx, 0, static_cast<int>(std::ssize(speeds) - 1))];
				};

				sample.m_avg_lin_speed /= sample.m_dynamic_count;
				sample.m_avg_ang_speed /= sample.m_dynamic_count;
				sample.m_p50_lin_speed = percentile(lin_speeds, 0.50f);
				sample.m_p90_lin_speed = percentile(lin_speeds, 0.90f);
				sample.m_p95_lin_speed = percentile(lin_speeds, 0.95f);
				sample.m_p99_lin_speed = percentile(lin_speeds, 0.99f);
				sample.m_p50_ang_speed = percentile(ang_speeds, 0.50f);
				sample.m_p90_ang_speed = percentile(ang_speeds, 0.90f);
				sample.m_p95_ang_speed = percentile(ang_speeds, 0.95f);
				sample.m_p99_ang_speed = percentile(ang_speeds, 0.99f);
			}

			return sample;
		}

		int BodyIndex(Scene const& scene, physics::RigidBody const* body)
		{
			for (int i = 0; i != std::ssize(scene.m_body); ++i)
			{
				auto const* candidate = static_cast<physics::RigidBody const*>(&scene.m_body[i]);
				if (candidate == body)
					return i;
			}

			return -1;
		}

		char const* ShapeName(physics::RigidBody const& body)
		{
			return body.HasShape() ? collision::ToString(body.Shape().m_type) : "NoShape";
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

		void PrintEngineProfile(std::ofstream& log, int step, double time_s, EngineProfileAccumulator const& profile)
		{
			auto count = std::max(profile.m_sample_count, 1);
			Emit(log, std::format(
				"profile,{},{:.4f},{},{:.2f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}\n",
				step,
				time_s,
				profile.m_sample_count,
				static_cast<double>(profile.m_contact_count) / count,
				profile.m_scene_step_ms / count,
				profile.m_physics_ms / count,
				profile.m_engine.m_new_frame_ms / count,
				profile.m_engine.m_pack_ms / count,
				profile.m_engine.m_upload_ms / count,
				profile.m_engine.m_integrate_ms / count,
				profile.m_engine.m_sleepwake_ms / count,
				profile.m_engine.m_broadphase_ms / count,
				profile.m_engine.m_collide_ms / count,
				profile.m_engine.m_resolve_ms / count,
				profile.m_engine.m_selective_ms / count,
				profile.m_engine.m_sleepupdate_ms / count,
				profile.m_engine.m_readback_ms / count,
				profile.m_engine.m_gpu_run_ms / count,
				profile.m_engine.m_unpack_ms / count));
		}
		void PrintSleepScan(std::ofstream& log, int step, double time_s, Scene const& scene, bool non_spheres_only)
		{
			auto sample = MeasureSleep(scene, non_spheres_only);
			auto const& config = scene.m_physics.Config();
			Emit(log, std::format(
				"sleep step={:5d} t={:8.4f} sample={} dynamic={} sleeping={} low={} never={} lin(avg/p50/p90/p95/p99/max)=({:9.5f},{:9.5f},{:9.5f},{:9.5f},{:9.5f},{:4d}:{:9.5f}/{:.5f}) ang(avg/p50/p90/p95/p99/max)=({:9.5f},{:9.5f},{:9.5f},{:9.5f},{:9.5f},{:4d}:{:9.5f}/{:.5f})\n",
				step,
				time_s,
				non_spheres_only ? "non-spheres" : "all",
				sample.m_dynamic_count,
				sample.m_sleeping_count,
				sample.m_low_velocity_count,
				sample.m_never_sleep_count,
				sample.m_avg_lin_speed,
				sample.m_p50_lin_speed,
				sample.m_p90_lin_speed,
				sample.m_p95_lin_speed,
				sample.m_p99_lin_speed,
				sample.m_max_lin_body,
				sample.m_max_lin_speed,
				config.sleep_velocity_threshold_lin,
				sample.m_avg_ang_speed,
				sample.m_p50_ang_speed,
				sample.m_p90_ang_speed,
				sample.m_p95_ang_speed,
				sample.m_p99_ang_speed,
				sample.m_max_ang_body,
				sample.m_max_ang_speed,
				config.sleep_velocity_threshold_ang));
		}
		void PrintColumnMetric(std::ofstream& log, int step, double time_s, Scene const& scene, ColumnMetricState& metric)
		{
			auto const& top_body = scene.m_body[metric.m_top_body];
			auto const height = top_body.O2W().pos.z;
			auto const vel = top_body.VelocityWS();
			metric.m_min_top_height = std::min(metric.m_min_top_height, height);

			auto const sample_count = std::max(metric.m_sample_count, 1);
			auto const avg_physics_ms = metric.m_physics_ms / sample_count;
			auto const engine_fps = avg_physics_ms > 0.0 ? 1000.0 / avg_physics_ms : 0.0;
			auto const collision_stats = scene.m_physics.LastCollisionStats();
			Emit(log, std::format(
				"column step={:5d} t={:8.4f} top={} count={} z={:9.5f} min_z={:9.5f} ideal_z={:9.5f} err={:9.5f} min_err={:9.5f} vel=({:8.4f},{:8.4f},{:8.4f}) pairs={} contacts={} physics_ms={:8.3f} fps={:8.2f}\n",
				step,
				time_s,
				metric.m_top_body,
				metric.m_body_count,
				height,
				metric.m_min_top_height,
				metric.m_ideal_top_height,
				height - metric.m_ideal_top_height,
				metric.m_min_top_height - metric.m_ideal_top_height,
				vel.lin.x, vel.lin.y, vel.lin.z,
				collision_stats.m_pair_count,
				collision_stats.LastContactCount(),
				avg_physics_ms,
				engine_fps));

			metric.m_physics_ms = 0.0;
			metric.m_sample_count = 0;
		}
		void PrintPyramidMetric(std::ofstream& log, int step, double time_s, Scene const& scene, PyramidMetricState& metric)
		{
			auto pos_error_sq = 0.0;
			auto xy_error_sq = 0.0;
			auto kinetic_energy = 0.0;
			auto max_xy = 0.0f;
			auto max_drop = 0.0f;
			auto top_height = -std::numeric_limits<float>::max();
			auto fallen_count = 0;
			auto sleeping_count = 0;

			for (auto const& target : metric.m_bodies)
			{
				auto const& body = scene.m_body[target.m_body];
				auto const pos = body.O2W().pos;
				auto const dx = pos.x - target.m_initial_pos.x;
				auto const dy = pos.y - target.m_initial_pos.y;
				auto const dz = pos.z - target.m_initial_pos.z;
				auto const xy_error = std::sqrt(dx * dx + dy * dy);
				auto const z_drop = target.m_initial_pos.z - pos.z;

				pos_error_sq += dx * dx + dy * dy + dz * dz;
				xy_error_sq += xy_error * xy_error;
				kinetic_energy += body.KineticEnergy();
				max_xy = std::max(max_xy, xy_error);
				max_drop = std::max(max_drop, z_drop);
				top_height = std::max(top_height, pos.z);
				sleeping_count += body.Sleeping() ? 1 : 0;
				fallen_count += xy_error > target.m_fallen_radius || z_drop > 0.5f * target.m_height ? 1 : 0;
			}

			auto const body_count = static_cast<double>(metric.m_bodies.size());
			auto const rms_error = std::sqrt(pos_error_sq / body_count);
			auto const rms_xy_error = std::sqrt(xy_error_sq / body_count);
			auto const sample_count = std::max(metric.m_sample_count, 1);
			auto const avg_physics_ms = metric.m_physics_ms / sample_count;
			auto const engine_fps = avg_physics_ms > 0.0 ? 1000.0 / avg_physics_ms : 0.0;
			auto const collision_stats = scene.m_physics.LastCollisionStats();
			Emit(log, std::format(
				"pyramid step={:5d} t={:8.4f} count={} rms={:9.5f} rms_xy={:9.5f} max_xy={:9.5f} max_drop={:9.5f} top_z={:9.5f} top_err={:9.5f} fallen={} sleeping={} ke={:12.6f} pairs={} contacts={} physics_ms={:8.3f} fps={:8.2f}\n",
				step,
				time_s,
				metric.m_bodies.size(),
				rms_error,
				rms_xy_error,
				max_xy,
				max_drop,
				top_height,
				top_height - metric.m_initial_top_height,
				fallen_count,
				sleeping_count,
				kinetic_energy,
				collision_stats.m_pair_count,
				collision_stats.LastContactCount(),
				avg_physics_ms,
				engine_fps));

			metric.m_physics_ms = 0.0;
			metric.m_sample_count = 0;
		}

		void PrintBodyTrace(
			std::ofstream& log,
			int step,
			double time_s,
			int body_index,
			BodyTraceState const& before,
			BodyTraceState const& after,
			float total_kinetic_energy,
			std::span<BodyTraceContact const> contacts)
		{
			auto dke = after.m_kinetic_energy - before.m_kinetic_energy;
			Emit(log, std::format(
				"trace step={:5d} t={:8.4f} body={} pos=({:8.3f},{:8.3f},{:8.3f}) vel=({:8.3f},{:8.3f},{:8.3f}) ang=({:8.3f},{:8.3f},{:8.3f}) KE={:12.4f} dKE={:12.4f} totalKE={:12.4f} contacts={} state={}\n",
				step,
				time_s,
				body_index,
				after.m_pos.x, after.m_pos.y, after.m_pos.z,
				after.m_lin_vel.x, after.m_lin_vel.y, after.m_lin_vel.z,
				after.m_ang_vel.x, after.m_ang_vel.y, after.m_ang_vel.z,
				after.m_kinetic_energy,
				dke,
				total_kinetic_energy,
				std::ssize(contacts),
				after.m_sleeping ? "sleep" : "awake"));

			Emit(log, std::format(
				"  o2w x=({:8.5f},{:8.5f},{:8.5f}) y=({:8.5f},{:8.5f},{:8.5f}) z=({:8.5f},{:8.5f},{:8.5f}) pos=({:8.5f},{:8.5f},{:8.5f})\n",
				after.m_o2w.x.x, after.m_o2w.x.y, after.m_o2w.x.z,
				after.m_o2w.y.x, after.m_o2w.y.y, after.m_o2w.y.z,
				after.m_o2w.z.x, after.m_o2w.z.y, after.m_o2w.z.z,
				after.m_o2w.pos.x, after.m_o2w.pos.y, after.m_o2w.pos.z));

			for (auto const& contact : contacts)
			{
				Emit(log, std::format(
					"  contact#{:4d} pair=({:4d},{:4d}) depth={:10.6f} feature={} axis_a=({:8.4f},{:8.4f},{:8.4f}) point_a=({:8.4f},{:8.4f},{:8.4f})\n",
					contact.m_contact_index,
					contact.m_body_a,
					contact.m_body_b,
					contact.m_depth,
					contact.m_feature_count,
					contact.m_axis_a.x, contact.m_axis_a.y, contact.m_axis_a.z,
					contact.m_point_a.x, contact.m_point_a.y, contact.m_point_a.z));
			}
		}

		void PrintBodyContacts(std::ofstream& log, int body_index, std::span<BodyTraceContact const> contacts)
		{
			auto contact_count = 0;
			for (auto const& contact : contacts)
			{
				if (contact.m_body_a != body_index && contact.m_body_b != body_index)
					continue;

				++contact_count;
				Emit(log, std::format(
					"  body {} contact#{:4d} pair=({:4d},{:4d}) depth={:10.6f} feature={} axis_a=({:8.4f},{:8.4f},{:8.4f}) point_a=({:8.4f},{:8.4f},{:8.4f})\n",
					body_index,
					contact.m_contact_index,
					contact.m_body_a,
					contact.m_body_b,
					contact.m_depth,
					contact.m_feature_count,
					contact.m_axis_a.x, contact.m_axis_a.y, contact.m_axis_a.z,
					contact.m_point_a.x, contact.m_point_a.y, contact.m_point_a.z));
			}

			if (contact_count == 0)
				Emit(log, std::format("  body {} contacts: none\n", body_index));
		}

		void PrintCpuGroundContact(std::ofstream& log, Scene const& scene, int body_index, int ground_body_index)
		{
			if (body_index < 0 || body_index >= std::ssize(scene.m_body))
				return;
			if (ground_body_index < 0 || ground_body_index >= std::ssize(scene.m_body) || ground_body_index == body_index)
				return;

			auto const& body = scene.m_body[body_index];
			auto const& ground = scene.m_body[ground_body_index];
			if (!body.HasShape() || !ground.HasShape())
				return;

			auto const bbox_overlap = IsIntersection(body.BBoxWS(), ground.BBoxWS());
			auto contact = collision::Contact{};
			auto const hit = collision::Collide(body.Shape(), body.O2W(), ground.Shape(), ground.O2W(), contact);
			Emit(log, std::format(
				"  cpu_ground pair=({:4d},{:4d}) bbox_overlap={} hit={} depth={:10.6f} axis_a=({:8.4f},{:8.4f},{:8.4f}) point_a=({:8.4f},{:8.4f},{:8.4f})\n",
				body_index,
				ground_body_index,
				bbox_overlap ? "true" : "false",
				hit ? "true" : "false",
				hit ? contact.m_depth : 0.0f,
				hit ? contact.m_axis.x : 0.0f, hit ? contact.m_axis.y : 0.0f, hit ? contact.m_axis.z : 0.0f,
				hit ? contact.Point().x : 0.0f, hit ? contact.Point().y : 0.0f, hit ? contact.Point().z : 0.0f));
		}
	}

	SceneDiagnosticResult RunSceneDiagnostic(SceneDiagnosticOptions const& options)
	{
		auto log_path = AppDataPath() / "scene_diagnostic.log";
		auto log = std::ofstream(log_path, std::ios::out | std::ios::trunc);
		auto result = SceneDiagnosticResult{};

		if (options.m_column_metric && options.m_pyramid_metric)
			throw std::runtime_error("Scene diagnostic metrics are mutually exclusive: use either -column_metric or -pyramid_metric");

		Emit(log, std::format("Scene diagnostic log: {}\n", log_path.string()));
		Emit(log, std::format("Scene diagnostic scene: {}\n", options.m_scene_filepath.string()));
		Emit(log, std::format("steps={} dt={:.8f} report_interval={}\n", options.m_steps, options.m_dt, options.m_report_interval));
		if (options.m_engine_profile)
			Emit(log, "profile,step,time_s,samples,contacts,scene_step_ms,physics_ms,new_frame_ms,pack_ms,upload_ms,integrate_ms,sleepwake_ms,broadphase_ms,collide_ms,resolve_ms,selective_ms,sleepupdate_ms,readback_ms,gpu_run_ms,unpack_ms\n");
		else if (options.m_column_metric)
			Emit(log, "column_metric=true\n");
		else if (options.m_pyramid_metric)
			Emit(log, "pyramid_metric=true\n");
		else if (options.m_scan_bodies)
			Emit(log, std::format("scan_bodies=true scan_non_spheres={} ke_jump={:.3f}\n", options.m_scan_non_spheres, options.m_trace_ke_jump));
		else if (options.m_trace_body == -1)
			Emit(log, " step    time_s  hits   max_depth    a    b            KE\n");
		else
			Emit(log, std::format("trace_body={} trace_start={} trace_end={} ke_jump={:.3f}\n", options.m_trace_body, options.m_trace_start, options.m_trace_end, options.m_trace_ke_jump));

		if (!std::filesystem::exists(options.m_scene_filepath))
			throw std::runtime_error(std::format("Scene file not found: {}", options.m_scene_filepath.string()));

		auto scene_desc = scene_loader::LoadFromFile(options.m_scene_filepath);
		if (options.m_physics_substeps)
		{
			if (*options.m_physics_substeps < 1)
				throw std::runtime_error("Scene diagnostic -substeps must be at least 1");
			scene_desc.physics_substeps = *options.m_physics_substeps;
		}
		if (options.m_physics_solver_iterations)
		{
			if (*options.m_physics_solver_iterations < 0)
				throw std::runtime_error("Scene diagnostic -solver_iterations must be non-negative");
			scene_desc.physics_solver_iterations = *options.m_physics_solver_iterations;
		}
		if (options.m_physics_position_iterations)
		{
			if (*options.m_physics_position_iterations < 0)
				throw std::runtime_error("Scene diagnostic -position_iterations must be non-negative");
			scene_desc.physics_position_iterations = *options.m_physics_position_iterations;
		}
		if (options.m_physics_selective_refresh_passes)
		{
			if (*options.m_physics_selective_refresh_passes < 0)
				throw std::runtime_error("Scene diagnostic -selective_refresh_passes must be non-negative");
			scene_desc.physics_selective_refresh_passes = *options.m_physics_selective_refresh_passes;
		}
		if (options.m_physics_selective_refresh_max_pairs)
		{
			if (*options.m_physics_selective_refresh_max_pairs < 1)
				throw std::runtime_error("Scene diagnostic -selective_refresh_max_pairs must be at least 1");
			scene_desc.physics_selective_refresh_max_pairs = *options.m_physics_selective_refresh_max_pairs;
		}
		if (options.m_physics_selective_refresh_solver_iterations)
		{
			if (*options.m_physics_selective_refresh_solver_iterations < 0)
				throw std::runtime_error("Scene diagnostic -selective_refresh_solver_iterations must be non-negative");
			scene_desc.physics_selective_refresh_solver_iterations = *options.m_physics_selective_refresh_solver_iterations;
		}
		if (options.m_physics_selective_refresh_position_iterations)
		{
			if (*options.m_physics_selective_refresh_position_iterations < 0)
				throw std::runtime_error("Scene diagnostic -selective_refresh_position_iterations must be non-negative");
			scene_desc.physics_selective_refresh_position_iterations = *options.m_physics_selective_refresh_position_iterations;
		}
		if (options.m_physics_selective_refresh_bias_scale)
		{
			if (*options.m_physics_selective_refresh_bias_scale < 0.0f)
				throw std::runtime_error("Scene diagnostic -selective_refresh_bias_scale must be non-negative");
			scene_desc.physics_selective_refresh_bias_scale = *options.m_physics_selective_refresh_bias_scale;
		}
		if (options.m_physics_selective_refresh_restitution_scale)
		{
			if (*options.m_physics_selective_refresh_restitution_scale < 0.0f)
				throw std::runtime_error("Scene diagnostic -selective_refresh_restitution_scale must be non-negative");
			scene_desc.physics_selective_refresh_restitution_scale = *options.m_physics_selective_refresh_restitution_scale;
		}
		if (options.m_physics_selective_refresh_support_only)
		{
			if (*options.m_physics_selective_refresh_support_only != 0 && *options.m_physics_selective_refresh_support_only != 1)
				throw std::runtime_error("Scene diagnostic -selective_refresh_support_only must be 0 or 1");
			scene_desc.physics_selective_refresh_support_only = *options.m_physics_selective_refresh_support_only != 0;
		}
		if (options.m_physics_selective_refresh_resolve_support_only)
		{
			if (*options.m_physics_selective_refresh_resolve_support_only != 0 && *options.m_physics_selective_refresh_resolve_support_only != 1)
				throw std::runtime_error("Scene diagnostic -selective_refresh_resolve_support_only must be 0 or 1");
			scene_desc.physics_selective_refresh_resolve_support_only = *options.m_physics_selective_refresh_resolve_support_only != 0;
		}
		auto const ground_body_index = scene_desc.ground ? static_cast<int>(scene_desc.bodies.size()) : -1;
		auto column_metric = options.m_column_metric ? CreateColumnMetric(scene_desc) : ColumnMetricState{};
		auto pyramid_metric = options.m_pyramid_metric ? CreatePyramidMetric(scene_desc) : PyramidMetricState{};
		auto scene = Scene(nullptr);
		scene.LoadScene(std::move(scene_desc));
		if (column_metric.Enabled())
		{
			Emit(log, std::format(
				"column setup top={} count={} initial_z={:.5f} ideal_z={:.5f} drop={:.5f} ground={:.5f}\n",
				column_metric.m_top_body,
				column_metric.m_body_count,
				column_metric.m_initial_top_height,
				column_metric.m_ideal_top_height,
				column_metric.m_initial_top_height - column_metric.m_ideal_top_height,
				column_metric.m_ground_height));
		}
		if (pyramid_metric.Enabled())
		{
			Emit(log, std::format(
				"pyramid setup count={} initial_top={:.5f} initial_spread={:.5f}\n",
				pyramid_metric.m_bodies.size(),
				pyramid_metric.m_initial_top_height,
				pyramid_metric.m_initial_spread));
		}

		auto trace_contacts = std::vector<BodyTraceContact>{};
		if (options.m_trace_body != -1 || options.m_scan_bodies)
		{
			if (options.m_trace_body != -1 && (options.m_trace_body < 0 || options.m_trace_body >= std::ssize(scene.m_body)))
				throw std::runtime_error(std::format("Trace body index {} is out of range [0,{})", options.m_trace_body, std::ssize(scene.m_body)));

			if (options.m_trace_body != -1)
			{
				auto const& body = scene.m_body[options.m_trace_body];
				Emit(log, std::format("trace target: body={} shape={} mass={:.4f}\n", options.m_trace_body, ShapeName(body), body.Mass()));
			}

			scene.m_physics.Collisions += [&](auto&, std::span<physics::RbContact const> contacts)
			{
				trace_contacts.resize(0);
				for (int i = 0; i != std::ssize(contacts); ++i)
				{
					auto const& contact = contacts[i];
					auto body_a = BodyIndex(scene, contact.m_objA);
					auto body_b = BodyIndex(scene, contact.m_objB);
					if (!options.m_scan_bodies && body_a != options.m_trace_body && body_b != options.m_trace_body)
						continue;

					trace_contacts.push_back(BodyTraceContact{
						.m_contact_index = i,
						.m_body_a = body_a,
						.m_body_b = body_b,
						.m_depth = contact.m_depth,
						.m_axis_a = contact.m_axis,
						.m_point_a = contact.Point(),
						.m_feature_count = contact.Count(),
					});
				}
			};
		}

		auto prev_kinetic_energy = std::vector<float>(scene.m_body.size(), 0.0f);
		for (int i = 0; i != std::ssize(scene.m_body); ++i)
			prev_kinetic_energy[i] = scene.m_body[i].KineticEnergy();

		auto profile = EngineProfileAccumulator{};
		for (int step = 0; step != options.m_steps; ++step)
		{
			auto before = BodyTraceState{};
			if (options.m_trace_body != -1)
				before = CaptureBodyState(scene.m_body[options.m_trace_body]);
			if (options.m_trace_body != -1 || options.m_scan_bodies)
				trace_contacts.resize(0);

			scene.Step(options.m_dt);
			if (column_metric.Enabled())
			{
				column_metric.m_physics_ms += scene.m_last_step_profile.m_physics_ms;
				++column_metric.m_sample_count;
			}
			if (pyramid_metric.Enabled())
			{
				pyramid_metric.m_physics_ms += scene.m_last_step_profile.m_physics_ms;
				++pyramid_metric.m_sample_count;
			}

			if (options.m_engine_profile)
			{
				profile.Add(scene.m_last_step_profile, scene.m_physics.LastCollisionStats().LastContactCount());
				auto report_interval = std::max(options.m_report_interval, 1);
				if ((step + 1) % report_interval == 0 || step + 1 == options.m_steps)
				{
					PrintEngineProfile(log, step + 1, scene.m_clock, profile);
					profile.Reset();
				}

				continue;
			}

			if (options.m_column_metric)
			{
				auto report_interval = std::max(options.m_report_interval, 1);
				if ((step + 1) % report_interval == 0 || step + 1 == options.m_steps)
					PrintColumnMetric(log, step + 1, scene.m_clock, scene, column_metric);

				continue;
			}
			if (options.m_pyramid_metric)
			{
				auto report_interval = std::max(options.m_report_interval, 1);
				if ((step + 1) % report_interval == 0 || step + 1 == options.m_steps)
					PrintPyramidMetric(log, step + 1, scene.m_clock, scene, pyramid_metric);

				continue;
			}

			if (options.m_scan_bodies)
			{
				auto max_height = -std::numeric_limits<float>::max();
				auto max_kinetic_energy = -std::numeric_limits<float>::max();
				auto max_delta_kinetic_energy = -std::numeric_limits<float>::max();
				auto max_height_body = -1;
				auto max_kinetic_energy_body = -1;
				auto max_delta_kinetic_energy_body = -1;
				auto total_kinetic_energy = 0.0f;

				for (int i = 0; i != std::ssize(scene.m_body); ++i)
				{
					auto const& body = scene.m_body[i];
					auto kinetic_energy = body.KineticEnergy();
					auto delta_kinetic_energy = kinetic_energy - prev_kinetic_energy[i];
					auto height = body.O2W().pos.z;
					total_kinetic_energy += kinetic_energy;

					if (height > max_height)
					{
						max_height = height;
						max_height_body = i;
					}
					if (kinetic_energy > max_kinetic_energy)
					{
						max_kinetic_energy = kinetic_energy;
						max_kinetic_energy_body = i;
					}
					if (delta_kinetic_energy > max_delta_kinetic_energy)
					{
						max_delta_kinetic_energy = delta_kinetic_energy;
						max_delta_kinetic_energy_body = i;
					}

					prev_kinetic_energy[i] = kinetic_energy;
				}

				auto report_interval = std::max(options.m_report_interval, 1);
				auto report_frame = (step + 1) % report_interval == 0 || step + 1 == options.m_steps;
				auto report_jump = max_delta_kinetic_energy >= options.m_trace_ke_jump;
				if (report_frame || report_jump)
				{
					auto const& height_body = scene.m_body[max_height_body];
					auto const& kinetic_body = scene.m_body[max_kinetic_energy_body];
					auto const& delta_body = scene.m_body[max_delta_kinetic_energy_body];
					auto height_vel = height_body.VelocityWS();
					auto kinetic_vel = kinetic_body.VelocityWS();
					auto delta_vel = delta_body.VelocityWS();
					Emit(log, std::format(
						"scan step={:5d} t={:8.4f} totalKE={:12.4f} max_z=({:4d},{:8.3f},shape={},vel=({:7.2f},{:7.2f},{:7.2f})) maxKE=({:4d},{:12.4f},shape={},vel=({:7.2f},{:7.2f},{:7.2f})) maxDKE=({:4d},{:12.4f},shape={},vel=({:7.2f},{:7.2f},{:7.2f})) contacts={}\n",
						step + 1,
						scene.m_clock,
						total_kinetic_energy,
						max_height_body, max_height, ShapeName(height_body), height_vel.lin.x, height_vel.lin.y, height_vel.lin.z,
						max_kinetic_energy_body, max_kinetic_energy, ShapeName(kinetic_body), kinetic_vel.lin.x, kinetic_vel.lin.y, kinetic_vel.lin.z,
						max_delta_kinetic_energy_body, max_delta_kinetic_energy, ShapeName(delta_body), delta_vel.lin.x, delta_vel.lin.y, delta_vel.lin.z,
						std::ssize(trace_contacts)));

					if (report_jump)
						PrintBodyContacts(log, max_delta_kinetic_energy_body, trace_contacts);
					PrintSleepScan(log, step + 1, scene.m_clock, scene, options.m_scan_non_spheres);
				}

				continue;
			}

			if (options.m_trace_body != -1)
			{
				auto after = CaptureBodyState(scene.m_body[options.m_trace_body]);
				auto dke = after.m_kinetic_energy - before.m_kinetic_energy;
				auto in_window = step + 1 >= options.m_trace_start && step + 1 <= options.m_trace_end;
				auto report_jump = dke >= options.m_trace_ke_jump || !trace_contacts.empty();
				if (in_window || report_jump || step + 1 == options.m_steps)
				{
					PrintBodyTrace(log, step + 1, scene.m_clock, options.m_trace_body, before, after, TotalKineticEnergy(scene), trace_contacts);
					PrintCpuGroundContact(log, scene, options.m_trace_body, ground_body_index);
				}

				continue;
			}

			{
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
		}

		if (options.m_trace_body == -1 && !options.m_engine_profile && !options.m_scan_bodies)
		{
			Emit(log, std::format("worst: step={} max_depth={:.6f} pair=({},{})\n",
				result.m_max_depth_step,
				result.m_max_depth,
				result.m_body_a,
				result.m_body_b));
		}

		return result;
	}
}
