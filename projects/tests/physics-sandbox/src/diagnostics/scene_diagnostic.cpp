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
		struct CradleMetricState
		{
			std::vector<int> m_chain_bodies = {};
			int m_impactor_body = -1;
			int m_left_body = -1;
			int m_right_body = -1;
			int m_first_impact_step = -1;
			int m_right_move_step = -1;
			float m_velocity_threshold = 0.01f;
			float m_direction = 1.0f;
			float m_peak_right_vx = 0.0f;
			float m_peak_intermediate_vx = 0.0f;
			double m_physics_ms = 0.0;
			int m_sample_count = 0;

			bool Enabled() const
			{
				return m_impactor_body != -1 && m_chain_bodies.size() >= 2;
			}
		};
		struct DzhanibekovMetricState
		{
			int m_body = -1;
			int m_axis = -1;
			int m_required_periods = 10;
			float m_max_period_spread = 0.05f;
			double m_prev_time_s = 0.0;
			float m_prev_axis_omega = 0.0f;
			double m_last_flip_time_s = -1.0;
			std::vector<double> m_flip_times_s = {};
			std::vector<double> m_periods_s = {};
			float m_initial_kinetic_energy = 0.0f;
			float m_min_kinetic_energy = std::numeric_limits<float>::max();
			float m_max_kinetic_energy = 0.0f;
			double m_physics_ms = 0.0;
			int m_sample_count = 0;

			bool Enabled() const
			{
				return m_body != -1;
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
				m_engine.m_external_forces_ms += profile.m_engine.m_external_forces_ms;
				m_engine.m_integrate_ms += profile.m_engine.m_integrate_ms;
				m_engine.m_sleepwake_ms += profile.m_engine.m_sleepwake_ms;
				m_engine.m_broadphase_ms += profile.m_engine.m_broadphase_ms;
				m_engine.m_collide_ms += profile.m_engine.m_collide_ms;
				m_engine.m_resolve_ms += profile.m_engine.m_resolve_ms;
				m_engine.m_selective_ms += profile.m_engine.m_selective_ms;
				m_engine.m_sleepupdate_ms += profile.m_engine.m_sleepupdate_ms;
				m_engine.m_readback_ms += profile.m_engine.m_readback_ms;
				m_engine.m_gpu_run_ms += profile.m_engine.m_gpu_run_ms;
				m_engine.m_gpu_prepare_ms += profile.m_engine.m_gpu_prepare_ms;
				m_engine.m_gpu_execute_ms += profile.m_engine.m_gpu_execute_ms;
				m_engine.m_gpu_wait_ms += profile.m_engine.m_gpu_wait_ms;
				m_engine.m_gpu_reset_ms += profile.m_engine.m_gpu_reset_ms;
				m_engine.m_unpack_ms += profile.m_engine.m_unpack_ms;
				m_engine.m_readback_access_ms += profile.m_engine.m_readback_access_ms;
				m_engine.m_body_readback_copy_ms += profile.m_engine.m_body_readback_copy_ms;
				m_engine.m_contact_readback_copy_ms += profile.m_engine.m_contact_readback_copy_ms;
				m_engine.m_collision_events_ms += profile.m_engine.m_collision_events_ms;
				m_engine.m_sleep_island_unpack_ms += profile.m_engine.m_sleep_island_unpack_ms;
				m_engine.m_body_unpack_ms += profile.m_engine.m_body_unpack_ms;
				m_engine.m_unpack_diagnostics_ms += profile.m_engine.m_unpack_diagnostics_ms;
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

		// Return the component of 'vec' selected by a Dzhanibekov metric axis index.
		float AxisComponent(v4 const& vec, int axis)
		{
			switch (axis)
			{
			case 0:
			{
				return vec.x;
			}
			case 1:
			{
				return vec.y;
			}
			case 2:
			{
				return vec.z;
			}
			default:
			{
				throw std::runtime_error(std::format("Invalid Dzhanibekov axis {}", axis));
			}
			}
		}

		// Return a compact axis name for Dzhanibekov metric logging.
		char AxisName(int axis)
		{
			switch (axis)
			{
			case 0:
			{
				return 'x';
			}
			case 1:
			{
				return 'y';
			}
			case 2:
			{
				return 'z';
			}
			default:
			{
				throw std::runtime_error(std::format("Invalid Dzhanibekov axis {}", axis));
			}
			}
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
		CradleMetricState CreateCradleMetric(scene_loader::SceneDesc const& scene_desc)
		{
			struct Candidate
			{
				int m_body = -1;
				float m_x = 0.0f;
				float m_vx = 0.0f;
			};

			auto candidates = std::vector<Candidate>{};
			for (int i = 0; i != std::ssize(scene_desc.bodies); ++i)
			{
				auto const& body = scene_desc.bodies[i];
				if (body.mass <= 0.0f || body.shape_type != scene_loader::BodyDesc::EShape::Box)
					continue;

				candidates.push_back(Candidate{
					.m_body = i,
					.m_x = body.position.x,
					.m_vx = body.velocity.x,
				});
			}

			auto metric = CradleMetricState{};
			if (candidates.size() < 3)
				throw std::runtime_error("Cradle metric requires an impactor and at least two dynamic box bodies");

			auto impactor = std::ranges::max_element(candidates, [](Candidate const& lhs, Candidate const& rhs)
			{
				return std::abs(lhs.m_vx) < std::abs(rhs.m_vx);
			});
			if (impactor == std::end(candidates) || std::abs(impactor->m_vx) <= metric.m_velocity_threshold)
				throw std::runtime_error("Cradle metric requires a dynamic box impactor with non-zero X velocity");

			metric.m_impactor_body = impactor->m_body;
			metric.m_direction = impactor->m_vx < 0.0f ? -1.0f : 1.0f;
			for (auto const& candidate : candidates)
			{
				if (candidate.m_body == metric.m_impactor_body)
					continue;

				metric.m_chain_bodies.push_back(candidate.m_body);
			}
			std::ranges::sort(metric.m_chain_bodies, [&](int lhs, int rhs)
			{
				return metric.m_direction * scene_desc.bodies[lhs].position.x < metric.m_direction * scene_desc.bodies[rhs].position.x;
			});

			metric.m_left_body = metric.m_chain_bodies.front();
			metric.m_right_body = metric.m_chain_bodies.back();
			return metric;
		}

		// Create the Dzhanibekov metric state by selecting the fastest spinning body and its dominant body-space axis.
		DzhanibekovMetricState CreateDzhanibekovMetric(Scene const& scene)
		{
			auto metric = DzhanibekovMetricState{};
			auto best_speed_sq = 0.0f;
			auto best_velocity = v4{};
			for (int i = 0; i != std::ssize(scene.m_body); ++i)
			{
				auto const velocity = scene.m_body[i].VelocityOS().ang;
				auto const speed_sq = LengthSq(velocity);
				if (speed_sq <= best_speed_sq)
					continue;

				best_speed_sq = speed_sq;
				best_velocity = velocity;
				metric.m_body = i;
			}
			if (metric.m_body == -1 || best_speed_sq < 1e-10f)
				throw std::runtime_error("Dzhanibekov metric requires at least one spinning dynamic body");

			metric.m_axis = 0;
			if (std::abs(best_velocity.y) > std::abs(AxisComponent(best_velocity, metric.m_axis)))
				metric.m_axis = 1;
			if (std::abs(best_velocity.z) > std::abs(AxisComponent(best_velocity, metric.m_axis)))
				metric.m_axis = 2;

			auto const& body = scene.m_body[metric.m_body];
			metric.m_prev_time_s = scene.m_clock;
			metric.m_prev_axis_omega = AxisComponent(best_velocity, metric.m_axis);
			metric.m_initial_kinetic_energy = body.KineticEnergy();
			metric.m_min_kinetic_energy = metric.m_initial_kinetic_energy;
			metric.m_max_kinetic_energy = metric.m_initial_kinetic_energy;
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

		// Log GPU-vs-analytic buoyancy values captured after the physics step.
		void PrintBuoyancyDiagnostics(std::ofstream& log, int step, double time_s, Scene const& scene)
		{
			auto diagnostics = scene.BuoyancyDiagnostics();
			for (auto const& diag : diagnostics)
			{
				if (!diag.m_analytic_valid)
				{
					Emit(log, std::format(
						"buoyancy step={:5d} t={:8.4f} body={:4d} valid={} analytic=false volume={:10.6f} force=({:10.3f},{:10.3f},{:10.3f}) cob=({:9.4f},{:9.4f},{:9.4f}) torque=({:10.3f},{:10.3f},{:10.3f})\n",
						step,
						time_s,
						diag.m_body_index,
						diag.m_valid ? "true" : "false",
						diag.m_volume_m3,
						diag.m_force_ws.x, diag.m_force_ws.y, diag.m_force_ws.z,
						diag.m_centre_buoyancy_ws.x, diag.m_centre_buoyancy_ws.y, diag.m_centre_buoyancy_ws.z,
						diag.m_torque_ws.x, diag.m_torque_ws.y, diag.m_torque_ws.z));
					continue;
				}

				Emit(log, std::format(
					"buoyancy step={:5d} t={:8.4f} body={:4d} valid={} analytic={} volume={:10.6f} analytic_volume={:10.6f} volume_err={:10.6f} force=({:10.3f},{:10.3f},{:10.3f}) force_err=({:10.3f},{:10.3f},{:10.3f}) cob=({:9.4f},{:9.4f},{:9.4f}) cob_err=({:9.4f},{:9.4f},{:9.4f}) torque=({:10.3f},{:10.3f},{:10.3f}) torque_err=({:10.3f},{:10.3f},{:10.3f})\n",
					step,
					time_s,
					diag.m_body_index,
					diag.m_valid ? "true" : "false",
					diag.m_analytic_valid ? "true" : "false",
					diag.m_volume_m3,
					diag.m_analytic_volume_m3,
					diag.m_volume_error_m3,
					diag.m_force_ws.x, diag.m_force_ws.y, diag.m_force_ws.z,
					diag.m_force_error_ws.x, diag.m_force_error_ws.y, diag.m_force_error_ws.z,
					diag.m_centre_buoyancy_ws.x, diag.m_centre_buoyancy_ws.y, diag.m_centre_buoyancy_ws.z,
					diag.m_centre_buoyancy_error_ws.x, diag.m_centre_buoyancy_error_ws.y, diag.m_centre_buoyancy_error_ws.z,
					diag.m_torque_ws.x, diag.m_torque_ws.y, diag.m_torque_ws.z,
					diag.m_torque_error_ws.x, diag.m_torque_error_ws.y, diag.m_torque_error_ws.z));
			}
		}

		void PrintEngineProfile(std::ofstream& log, int step, double time_s, EngineProfileAccumulator const& profile)
		{
			auto count = std::max(profile.m_sample_count, 1);
			Emit(log, std::format(
				"profile,{},{:.4f},{},{:.2f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}\n",
				step,
				time_s,
				profile.m_sample_count,
				static_cast<double>(profile.m_contact_count) / count,
				profile.m_scene_step_ms / count,
				profile.m_physics_ms / count,
				profile.m_engine.m_new_frame_ms / count,
				profile.m_engine.m_pack_ms / count,
				profile.m_engine.m_upload_ms / count,
				profile.m_engine.m_external_forces_ms / count,
				profile.m_engine.m_integrate_ms / count,
				profile.m_engine.m_sleepwake_ms / count,
				profile.m_engine.m_broadphase_ms / count,
				profile.m_engine.m_collide_ms / count,
				profile.m_engine.m_resolve_ms / count,
				profile.m_engine.m_selective_ms / count,
				profile.m_engine.m_sleepupdate_ms / count,
				profile.m_engine.m_readback_ms / count,
				profile.m_engine.m_gpu_run_ms / count,
				profile.m_engine.m_unpack_ms / count,
				profile.m_engine.m_gpu_prepare_ms / count,
				profile.m_engine.m_gpu_execute_ms / count,
				profile.m_engine.m_gpu_wait_ms / count,
				profile.m_engine.m_gpu_reset_ms / count,
				profile.m_engine.m_readback_access_ms / count,
				profile.m_engine.m_body_readback_copy_ms / count,
				profile.m_engine.m_contact_readback_copy_ms / count,
				profile.m_engine.m_collision_events_ms / count,
				profile.m_engine.m_sleep_island_unpack_ms / count,
				profile.m_engine.m_body_unpack_ms / count,
				profile.m_engine.m_unpack_diagnostics_ms / count));
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
		// Print the sleep regression metric for the selected dynamic-body sample.
		void PrintSleepMetric(std::ofstream& log, int step, double time_s, Scene const& scene, bool non_spheres_only)
		{
			auto sample = MeasureSleep(scene, non_spheres_only);
			auto asleep = sample.m_dynamic_count != 0 && sample.m_sleeping_count == sample.m_dynamic_count;
			Emit(log, std::format(
				"sleep_metric step={:5d} t={:8.4f} sample={} dynamic={} sleeping={} low={} asleep={}\n",
				step,
				time_s,
				non_spheres_only ? "non-spheres" : "all",
				sample.m_dynamic_count,
				sample.m_sleeping_count,
				sample.m_low_velocity_count,
				asleep ? "true" : "false"));
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
		void PrintCradleMetric(std::ofstream& log, int step, double time_s, Scene const& scene, CradleMetricState& metric)
		{
			auto const impactor_vx = scene.m_body[metric.m_impactor_body].VelocityWS().lin.x;
			auto const left_vx = scene.m_body[metric.m_left_body].VelocityWS().lin.x;
			auto const right_vx = scene.m_body[metric.m_right_body].VelocityWS().lin.x;
			auto max_intermediate_vx = 0.0f;
			auto sum_intermediate_vx = 0.0f;
			auto total_kinetic_energy = 0.0f;

			for (auto const& body : scene.m_body)
				total_kinetic_energy += body.KineticEnergy();

			for (int i = 1; i + 1 < std::ssize(metric.m_chain_bodies); ++i)
			{
				auto const vx = std::abs(scene.m_body[metric.m_chain_bodies[i]].VelocityWS().lin.x);
				max_intermediate_vx = std::max(max_intermediate_vx, vx);
				sum_intermediate_vx += vx;
			}

			if (metric.m_first_impact_step == -1 && std::abs(left_vx) > metric.m_velocity_threshold)
				metric.m_first_impact_step = step;
			if (metric.m_right_move_step == -1 && std::abs(right_vx) > metric.m_velocity_threshold)
				metric.m_right_move_step = step;

			metric.m_peak_right_vx = std::max(metric.m_peak_right_vx, std::abs(right_vx));
			metric.m_peak_intermediate_vx = std::max(metric.m_peak_intermediate_vx, max_intermediate_vx);

			auto const delay = metric.m_first_impact_step != -1 && metric.m_right_move_step != -1
				? metric.m_right_move_step - metric.m_first_impact_step
				: -1;
			auto const sample_count = std::max(metric.m_sample_count, 1);
			auto const avg_physics_ms = metric.m_physics_ms / sample_count;
			auto const engine_fps = avg_physics_ms > 0.0 ? 1000.0 / avg_physics_ms : 0.0;
			auto const collision_stats = scene.m_physics.LastCollisionStats();
			Emit(log, std::format(
				"cradle step={:5d} t={:8.4f} impact_step={} right_step={} delay={} impactor_vx={:9.4f} left_vx={:9.4f} right_vx={:9.4f} max_mid_vx={:9.4f} sum_mid_vx={:9.4f} peak_right_vx={:9.4f} peak_mid_vx={:9.4f} ke={:12.6f} pairs={} contacts={} physics_ms={:8.3f} fps={:8.2f}\n",
				step,
				time_s,
				metric.m_first_impact_step,
				metric.m_right_move_step,
				delay,
				impactor_vx,
				left_vx,
				right_vx,
				max_intermediate_vx,
				sum_intermediate_vx,
				metric.m_peak_right_vx,
				metric.m_peak_intermediate_vx,
				total_kinetic_energy,
				collision_stats.m_pair_count,
				collision_stats.LastContactCount(),
				avg_physics_ms,
				engine_fps));

			metric.m_physics_ms = 0.0;
			metric.m_sample_count = 0;
		}

		// Update flip timing and energy bounds for the Dzhanibekov metric.
		void UpdateDzhanibekovMetric(Scene const& scene, double time_s, DzhanibekovMetricState& metric)
		{
			auto const& body = scene.m_body[metric.m_body];
			auto const axis_omega = AxisComponent(body.VelocityOS().ang, metric.m_axis);
			auto const kinetic_energy = body.KineticEnergy();
			metric.m_min_kinetic_energy = std::min(metric.m_min_kinetic_energy, kinetic_energy);
			metric.m_max_kinetic_energy = std::max(metric.m_max_kinetic_energy, kinetic_energy);

			if (metric.m_prev_axis_omega * axis_omega < 0.0f)
			{
				auto const denom = std::abs(metric.m_prev_axis_omega) + std::abs(axis_omega);
				auto const alpha = denom > 0.0f ? std::abs(metric.m_prev_axis_omega) / denom : 0.0f;
				auto const flip_time = metric.m_prev_time_s + alpha * (time_s - metric.m_prev_time_s);
				if (metric.m_last_flip_time_s >= 0.0)
					metric.m_periods_s.push_back(flip_time - metric.m_last_flip_time_s);

				metric.m_flip_times_s.push_back(flip_time);
				metric.m_last_flip_time_s = flip_time;
			}

			metric.m_prev_axis_omega = axis_omega;
			metric.m_prev_time_s = time_s;
		}

		// Calculate period statistics from the collected Dzhanibekov flip intervals.
		void DzhanibekovPeriodStats(DzhanibekovMetricState const& metric, double& mean_period, double& min_period, double& max_period, double& period_spread)
		{
			mean_period = 0.0;
			min_period = std::numeric_limits<double>::max();
			max_period = 0.0;
			period_spread = 0.0;
			if (metric.m_periods_s.empty())
				return;

			for (auto period : metric.m_periods_s)
			{
				mean_period += period;
				min_period = std::min(min_period, period);
				max_period = std::max(max_period, period);
			}
			mean_period /= static_cast<double>(metric.m_periods_s.size());
			period_spread = mean_period > 0.0 ? (max_period - min_period) / mean_period : 0.0;
		}

		// Print the current Dzhanibekov period-stability metric.
		void PrintDzhanibekovMetric(std::ofstream& log, int step, double time_s, Scene const& scene, DzhanibekovMetricState& metric)
		{
			auto const& body = scene.m_body[metric.m_body];
			auto const velocity_os = body.VelocityOS().ang;
			auto mean_period = 0.0;
			auto min_period = 0.0;
			auto max_period = 0.0;
			auto period_spread = 0.0;
			DzhanibekovPeriodStats(metric, mean_period, min_period, max_period, period_spread);

			auto const sample_count = std::max(metric.m_sample_count, 1);
			auto const avg_physics_ms = metric.m_physics_ms / sample_count;
			auto const engine_fps = avg_physics_ms > 0.0 ? 1000.0 / avg_physics_ms : 0.0;
			auto const kinetic_energy = body.KineticEnergy();
			auto const energy_drift = metric.m_initial_kinetic_energy != 0.0f
				? (kinetic_energy - metric.m_initial_kinetic_energy) / metric.m_initial_kinetic_energy
				: 0.0f;
			Emit(log, std::format(
				"dzhanibekov step={:5d} t={:8.4f} body={} axis={} flips={} periods={} mean_period={:9.5f} min_period={:9.5f} max_period={:9.5f} period_spread={:8.5f} omega_os=({:9.4f},{:9.4f},{:9.4f}) ke={:12.6f} ke_drift={:9.5f} physics_ms={:8.3f} fps={:8.2f}\n",
				step,
				time_s,
				metric.m_body,
				AxisName(metric.m_axis),
				metric.m_flip_times_s.size(),
				metric.m_periods_s.size(),
				mean_period,
				min_period,
				max_period,
				period_spread,
				velocity_os.x, velocity_os.y, velocity_os.z,
				kinetic_energy,
				energy_drift,
				avg_physics_ms,
				engine_fps));

			metric.m_physics_ms = 0.0;
			metric.m_sample_count = 0;
		}

		// Fail the scene diagnostic if the Dzhanibekov flip period drifts beyond the regression threshold.
		void ValidateDzhanibekovMetric(DzhanibekovMetricState const& metric)
		{
			if (std::ssize(metric.m_periods_s) < metric.m_required_periods)
				throw std::runtime_error(std::format("Dzhanibekov metric failed: expected at least {} periods, measured {}", metric.m_required_periods, metric.m_periods_s.size()));

			auto mean_period = 0.0;
			auto min_period = 0.0;
			auto max_period = 0.0;
			auto period_spread = 0.0;
			DzhanibekovPeriodStats(metric, mean_period, min_period, max_period, period_spread);
			if (period_spread > metric.m_max_period_spread)
			{
				throw std::runtime_error(std::format(
					"Dzhanibekov metric failed: period spread {:.5f} exceeded {:.5f} (mean={:.5f}, min={:.5f}, max={:.5f})",
					period_spread,
					metric.m_max_period_spread,
					mean_period,
					min_period,
					max_period));
			}
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

		auto metric_count = 0;
		metric_count += options.m_column_metric ? 1 : 0;
		metric_count += options.m_pyramid_metric ? 1 : 0;
		metric_count += options.m_cradle_metric ? 1 : 0;
		metric_count += options.m_dzhanibekov_metric ? 1 : 0;
		metric_count += options.m_sleep_metric ? 1 : 0;
		if (metric_count > 1)
			throw std::runtime_error("Scene diagnostic metrics are mutually exclusive: use one of -column_metric, -pyramid_metric, -cradle_metric, -dzhanibekov_metric, or -sleep_metric");

		Emit(log, std::format("Scene diagnostic log: {}\n", log_path.string()));
		Emit(log, std::format("Scene diagnostic scene: {}\n", options.m_scene_filepath.string()));
		Emit(log, std::format("steps={} dt={:.8f} report_interval={}\n", options.m_steps, options.m_dt, options.m_report_interval));
		if (options.m_engine_profile)
			Emit(log, "profile,step,time_s,samples,contacts,scene_step_ms,physics_ms,new_frame_ms,pack_ms,upload_ms,external_forces_ms,integrate_ms,sleepwake_ms,broadphase_ms,collide_ms,resolve_ms,selective_ms,sleepupdate_ms,readback_ms,gpu_run_ms,unpack_ms,gpu_prepare_ms,gpu_execute_ms,gpu_wait_ms,gpu_reset_ms,readback_access_ms,body_readback_copy_ms,contact_readback_copy_ms,collision_events_ms,sleep_island_unpack_ms,body_unpack_ms,unpack_diagnostics_ms\n");
		else if (options.m_column_metric)
			Emit(log, "column_metric=true\n");
		else if (options.m_pyramid_metric)
			Emit(log, "pyramid_metric=true\n");
		else if (options.m_cradle_metric)
			Emit(log, "cradle_metric=true\n");
		else if (options.m_dzhanibekov_metric)
			Emit(log, "dzhanibekov_metric=true\n");
		else if (options.m_sleep_metric)
			Emit(log, std::format("sleep_metric=true sleep_metric_non_spheres={}\n", options.m_sleep_metric_non_spheres));
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
		if (options.m_physics_broadphase_aabb_margin)
		{
			if (*options.m_physics_broadphase_aabb_margin < 0.0f)
				throw std::runtime_error("Scene diagnostic -broadphase_aabb_margin must be non-negative");
			scene_desc.physics_broadphase_aabb_margin = *options.m_physics_broadphase_aabb_margin;
		}
		if (options.m_physics_contact_sort_propagation_scale)
		{
			if (*options.m_physics_contact_sort_propagation_scale < 0.0f)
				throw std::runtime_error("Scene diagnostic -contact_sort_propagation_scale must be non-negative");
			scene_desc.physics_contact_sort_propagation_scale = *options.m_physics_contact_sort_propagation_scale;
		}
		if (options.m_physics_contact_sort_shock_iterations)
		{
			if (*options.m_physics_contact_sort_shock_iterations < 0)
				throw std::runtime_error("Scene diagnostic -contact_sort_shock_iterations must be non-negative");
			scene_desc.physics_contact_sort_shock_iterations = *options.m_physics_contact_sort_shock_iterations;
		}
		if (options.m_physics_contact_slop_scale)
		{
			if (*options.m_physics_contact_slop_scale < 0.0f)
				throw std::runtime_error("Scene diagnostic -contact_slop_scale must be non-negative");
			scene_desc.physics_contact_slop_scale = *options.m_physics_contact_slop_scale;
		}
		if (options.m_physics_support_contact_slop_scale)
		{
			if (*options.m_physics_support_contact_slop_scale < 0.0f)
				throw std::runtime_error("Scene diagnostic -support_contact_slop_scale must be non-negative");
			scene_desc.physics_support_contact_slop_scale = *options.m_physics_support_contact_slop_scale;
		}
		if (options.m_physics_warm_start_scale)
		{
			if (*options.m_physics_warm_start_scale < 0.0f)
				throw std::runtime_error("Scene diagnostic -warm_start_scale must be non-negative");
			scene_desc.physics_warm_start_scale = *options.m_physics_warm_start_scale;
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
		if (options.m_physics_selective_refresh_body_limit)
		{
			if (*options.m_physics_selective_refresh_body_limit < 0)
				throw std::runtime_error("Scene diagnostic -selective_refresh_body_limit must be non-negative");
			scene_desc.physics_selective_refresh_body_limit = *options.m_physics_selective_refresh_body_limit;
		}
		if (options.m_physics_selective_refresh_contact_limit)
		{
			if (*options.m_physics_selective_refresh_contact_limit < 0)
				throw std::runtime_error("Scene diagnostic -selective_refresh_contact_limit must be non-negative");
			scene_desc.physics_selective_refresh_contact_limit = *options.m_physics_selective_refresh_contact_limit;
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
		if (options.m_physics_selective_refresh_adaptive_body_limit)
		{
			if (*options.m_physics_selective_refresh_adaptive_body_limit < 0)
				throw std::runtime_error("Scene diagnostic -selective_refresh_adaptive_body_limit must be non-negative");
			scene_desc.physics_selective_refresh_adaptive_body_limit = *options.m_physics_selective_refresh_adaptive_body_limit;
		}
		if (options.m_physics_selective_refresh_adaptive_solver_iterations)
		{
			if (*options.m_physics_selective_refresh_adaptive_solver_iterations < 0)
				throw std::runtime_error("Scene diagnostic -selective_refresh_adaptive_solver_iterations must be non-negative");
			scene_desc.physics_selective_refresh_adaptive_solver_iterations = *options.m_physics_selective_refresh_adaptive_solver_iterations;
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
		auto cradle_metric = options.m_cradle_metric ? CreateCradleMetric(scene_desc) : CradleMetricState{};
		auto scene = Scene(nullptr);
		scene.LoadScene(std::move(scene_desc));
		auto dzhanibekov_metric = options.m_dzhanibekov_metric ? CreateDzhanibekovMetric(scene) : DzhanibekovMetricState{};
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
		if (cradle_metric.Enabled())
		{
			Emit(log, std::format(
				"cradle setup impactor={} source={} target={} chain_count={} direction={:.0f} velocity_threshold={:.5f}\n",
				cradle_metric.m_impactor_body,
				cradle_metric.m_left_body,
				cradle_metric.m_right_body,
				cradle_metric.m_chain_bodies.size(),
				cradle_metric.m_direction,
				cradle_metric.m_velocity_threshold));
		}
		if (dzhanibekov_metric.Enabled())
		{
			Emit(log, std::format(
				"dzhanibekov setup body={} axis={} required_periods={} max_period_spread={:.5f} initial_ke={:.6f}\n",
				dzhanibekov_metric.m_body,
				AxisName(dzhanibekov_metric.m_axis),
				dzhanibekov_metric.m_required_periods,
				dzhanibekov_metric.m_max_period_spread,
				dzhanibekov_metric.m_initial_kinetic_energy));
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
			if (cradle_metric.Enabled())
			{
				cradle_metric.m_physics_ms += scene.m_last_step_profile.m_physics_ms;
				++cradle_metric.m_sample_count;
			}
			if (dzhanibekov_metric.Enabled())
			{
				dzhanibekov_metric.m_physics_ms += scene.m_last_step_profile.m_physics_ms;
				++dzhanibekov_metric.m_sample_count;
				UpdateDzhanibekovMetric(scene, scene.m_clock, dzhanibekov_metric);
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
			if (options.m_cradle_metric)
			{
				auto report_interval = std::max(options.m_report_interval, 1);
				if ((step + 1) % report_interval == 0 || step + 1 == options.m_steps)
					PrintCradleMetric(log, step + 1, scene.m_clock, scene, cradle_metric);

				continue;
			}
			if (options.m_dzhanibekov_metric)
			{
				auto report_interval = std::max(options.m_report_interval, 1);
				if ((step + 1) % report_interval == 0 || step + 1 == options.m_steps)
					PrintDzhanibekovMetric(log, step + 1, scene.m_clock, scene, dzhanibekov_metric);

				continue;
			}
			if (options.m_sleep_metric)
			{
				auto report_interval = std::max(options.m_report_interval, 1);
				auto report_frame = (step + 1) % report_interval == 0 || step + 1 == options.m_steps;
				if (report_frame)
					PrintSleepMetric(log, step + 1, scene.m_clock, scene, options.m_sleep_metric_non_spheres);

				if (step + 1 == options.m_steps)
				{
					auto sample = MeasureSleep(scene, options.m_sleep_metric_non_spheres);
					if (sample.m_dynamic_count == 0 || sample.m_sleeping_count != sample.m_dynamic_count)
					{
						throw std::runtime_error(std::format(
							"Sleep metric failed: sleeping {} of {} dynamic bodies at t={:.4f}",
							sample.m_sleeping_count,
							sample.m_dynamic_count,
							scene.m_clock));
					}
				}

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
				{
					PrintSample(log, step + 1, scene.m_clock, sample);
					PrintBuoyancyDiagnostics(log, step + 1, scene.m_clock, scene);
				}
			}
		}

		if (dzhanibekov_metric.Enabled())
			ValidateDzhanibekovMetric(dzhanibekov_metric);

		if (options.m_trace_body == -1 && !options.m_engine_profile && !options.m_scan_bodies && !options.m_cradle_metric && !options.m_dzhanibekov_metric)
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
