#include "src/forward.h"
#include "src/diagnostics/scene_diagnostic.h"
#include "src/scene/scene.h"
#include "src/utils/scene_loader.h"

namespace physics_sandbox::diag
{
	namespace
	{
		// Unwrapped coordinates of one-axis revolute joints, independent of principal quaternion rotation vectors.
		struct HingeDomainSample
		{
			float m_max_abs_q = 0.0f;
			int m_count = 0;
			int m_outside_principal_count = 0;
		};

		// Check the scalar |q| < pi prerequisite for identifying a hinge coordinate with a principal rotation angle.
		HingeDomainSample MeasureHingeDomain(std::span<physics::Articulation const> articulations)
		{
			// Read the owned coordinates without reconstructing them from periodic link orientations.
			auto sample = HingeDomainSample{};
			for (auto const& tree : articulations)
			{
				// Roots have no joint, and multi-axis joints do not define a single hinge angle.
				for (int index = 1; index != tree.LinkCount(); ++index)
				{
					// Spherical quaternion components are not scalar angles.
					auto const link = tree.LinkAt(index);
					auto const& joint = tree.JointDescription(link);
					switch (joint.m_type)
					{
						case physics::EArticulationJointType::Axes: { break; }
						case physics::EArticulationJointType::Spherical: { continue; }
						default: { throw std::runtime_error("Unknown articulation joint type in hinge-domain screen"); }
					}
					if (joint.m_dof_count != 1)
						continue;

					// Prismatic coordinates are distances and have no principal-angle domain.
					switch (joint.m_axes[0].m_type)
					{
						case physics::EArticulationAxisType::Revolute: { break; }
						case physics::EArticulationAxisType::Prismatic: { continue; }
						default: { throw std::runtime_error("Unknown articulation axis type in hinge-domain screen"); }
					}
					auto const q = tree.JointPosition(link)[0];
					if (!std::isfinite(q))
						throw std::runtime_error("Nonfinite committed hinge coordinate");

					// The endpoints at +/-pi are excluded because the principal chart is not unique there.
					++sample.m_count;
					sample.m_max_abs_q = std::max(sample.m_max_abs_q, std::abs(q));
					sample.m_outside_principal_count += std::abs(q) < constants<float>::tau_by_2 ? 0 : 1;
				}
			}
			return sample;
		}

		struct PenetrationSample
		{
			float m_max_depth = 0.0f;
			int m_contact_count = 0;
			int m_body_a = -1;
			int m_body_b = -1;
			int m_articulation = -1;
			int m_link = -1;
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
			float m_avg_horizontal_speed = 0.0f;
			float m_avg_vertical_speed = 0.0f;
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
				m_engine.m_constraint_pack_ms += profile.m_engine.m_constraint_pack_ms;
				m_engine.m_articulation_pack_ms += profile.m_engine.m_articulation_pack_ms;
				m_engine.m_upload_ms += profile.m_engine.m_upload_ms;
				m_engine.m_constraint_upload_ms += profile.m_engine.m_constraint_upload_ms;
				m_engine.m_articulation_upload_ms += profile.m_engine.m_articulation_upload_ms;
				m_engine.m_external_forces_ms += profile.m_engine.m_external_forces_ms;
				m_engine.m_integrate_ms += profile.m_engine.m_integrate_ms;
				m_engine.m_articulation_integrate_ms += profile.m_engine.m_articulation_integrate_ms;
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
				m_engine.m_articulation_unpack_ms += profile.m_engine.m_articulation_unpack_ms;
				m_engine.m_substep_count += profile.m_engine.m_substep_count;
				m_engine.m_submission_count += profile.m_engine.m_submission_count;
				m_engine.m_wait_count += profile.m_engine.m_wait_count;
				m_engine.m_readback_copy_count += profile.m_engine.m_readback_copy_count;
				m_engine.m_terrain_gpu_ms += profile.m_engine.m_terrain_gpu_ms;
				m_engine.m_water_gpu_ms += profile.m_engine.m_water_gpu_ms;
			}

			void Reset()
			{
				*this = {};
			}
		};

		// Collide two shapes on the CPU, expanding compound shapes into their leaf children. Returns the deepest leaf contact.
		bool CollideLeaves(collision::Shape const& lhs, m4x4 const& l2w, collision::Shape const& rhs, m4x4 const& r2w, collision::Contact& contact)
		{
			// The CPU narrow phase has no compound pairs, so test each child placed by its shape-to-root transform.
			if (lhs.m_type == collision::EShape::Array || rhs.m_type == collision::EShape::Array)
			{
				auto const& array = collision::shape_cast<collision::ShapeArray>(lhs.m_type == collision::EShape::Array ? lhs : rhs);
				auto const& other = lhs.m_type == collision::EShape::Array ? rhs : lhs;
				auto const& a2w = lhs.m_type == collision::EShape::Array ? l2w : r2w;
				auto const& o2w = lhs.m_type == collision::EShape::Array ? r2w : l2w;
				auto const array_is_lhs = lhs.m_type == collision::EShape::Array;

				auto hit = false;
				for (auto child = array.begin(); child != array.end(); child = collision::next(child))
				{
					// Keep the argument order so the contact normal convention matches the caller's order.
					// Children already carry shape-to-root transforms, including nested arrays. Each leaf applies its transform exactly once.
					auto leaf = collision::Contact{};
					auto const leaf_hit = array_is_lhs
						? CollideLeaves(*child, a2w, other, o2w, leaf)
						: CollideLeaves(other, o2w, *child, a2w, leaf);
					if (!leaf_hit || (hit && leaf.m_depth <= contact.m_depth))
						continue;

					contact = leaf;
					hit = true;
				}
				return hit;
			}

			// Leaf pairs use the standard narrow phase.
			return collision::Collide(lhs, l2w, rhs, r2w, contact);
		}

		// A shaped endpoint in committed scene state; full-scene indices include sleeping trees omitted from a submitted GPU frame.
		struct CollisionPlacement
		{
			physics::BodyRef m_identity;
			physics::BodyRef m_parent_identity;
			collision::Shape const* m_shape = nullptr;
			m4x4 m_o2w = m4x4::Identity();
			BBox m_bounds = {};
			int m_body_index = -1;
			int m_articulation = -1;
			int m_link = -1;
			int m_parent_index = -1;
			bool m_collide_parent = false;
			bool m_collide_self = true;
			uint32_t m_layer = physics::DefaultCollisionLayer;
			uint32_t m_mask = physics::DefaultCollisionMask;
		};

		// Identify geometry independently of sort order, transient GPU slots, and active solver rows.
		std::string GeometryKey(physics::BodyRef const& identity)
		{
			switch (identity.m_type)
			{
				case physics::EConstraintBodyType::World: { return "world"; }
				case physics::EConstraintBodyType::Rigid: { return std::format("r:{}", identity.m_body_id.m_value); }
				case physics::EConstraintBodyType::ArticulationLink: { return std::format("a:{}:{}:{}", identity.m_articulation_id.m_value, identity.m_link.m_index, identity.m_link.m_generation); }
				default: { throw std::runtime_error("Unsupported geometry endpoint identity"); }
			}
		}

		// Use transient full-scene indices only for the native exclusion table, never history identity.
		uint64_t CollisionPairKey(int a, int b)
		{
			return (uint64_t{static_cast<uint32_t>(std::min(a, b)) + 1U} << 32) | (static_cast<uint32_t>(std::max(a, b)) + 1U);
		}

		// Share the scan's exact eligibility law with outstanding-pair separation accounting.
		bool GeometryPairEligible(CollisionPlacement const& a, CollisionPlacement const& b, std::span<uint64_t const> excluded)
		{
			if (!physics::CollisionFilterPasses(a.m_layer, a.m_mask, b.m_layer, b.m_mask) ||
				std::ranges::binary_search(excluded, CollisionPairKey(a.m_body_index, b.m_body_index)))
				return false;

			if (a.m_articulation >= 0 && a.m_articulation == b.m_articulation)
			{
				// Parent contact policy is independent of each link's non-adjacent self-contact policy.
				auto const a_is_child = a.m_parent_index == b.m_body_index;
				auto const b_is_child = b.m_parent_index == a.m_body_index;
				if ((a_is_child && !a.m_collide_parent) || (b_is_child && !b.m_collide_parent))
					return false;

				if (!a_is_child && !b_is_child && (!a.m_collide_self || !b.m_collide_self))
					return false;
			}
			return true;
		}

		// Measure committed descriptor geometry, not solver row activation or articulation coordinate shortcuts.
		struct GeometryLimitSample
		{
			m4x4 m_frame_a;
			m4x4 m_frame_b;
			quat m_relative;
			std::array<float, 6> m_coordinates = {};
			std::array<float, 6> m_excess = {};
			float m_angular_excess = 0;
			bool m_has_angular_limit = false;
			bool m_regular = true;
		};

		// Return hard interval/lock excess; a drive target is not a hard positional constraint.
		float GeometryAxisExcess(physics::ConstraintAxisDesc const& axis, float position)
		{
			switch (axis.m_mode)
			{
				case physics::EConstraintAxisMode::Free:
				case physics::EConstraintAxisMode::Driven: { return 0; }
				case physics::EConstraintAxisMode::Locked: { return std::abs(position - axis.m_target_position); }
				case physics::EConstraintAxisMode::Limited: { return std::max({axis.m_limits.m_beg - position, 0.0f, position - axis.m_limits.m_end}); }
				default: { throw std::runtime_error("Unsupported geometry axis mode"); }
			}
		}

		// Report whether an axis declares a hard positional obligation.
		bool GeometryHardAxis(physics::ConstraintAxisDesc const& axis)
		{
			switch (axis.m_mode)
			{
				case physics::EConstraintAxisMode::Free:
				case physics::EConstraintAxisMode::Driven: { return false; }
				case physics::EConstraintAxisMode::Locked:
				case physics::EConstraintAxisMode::Limited: { return true; }
				default: { throw std::runtime_error("Unsupported geometry axis mode"); }
			}
		}

		// Compose the endpoint-local frames and reuse the production coordinate/boundary law unchanged.
		GeometryLimitSample MeasureGeometryLimit(physics::ConstraintDesc const& desc, m4x4 const& a2w, m4x4 const& b2w)
		{
			auto sample = GeometryLimitSample{
				.m_frame_a = a2w * desc.m_frame_a.m_constraint_to_body,
				.m_frame_b = b2w * desc.m_frame_b.m_constraint_to_body,
			};
			sample.m_relative = ~quat(sample.m_frame_a.rot) * quat(sample.m_frame_b.rot);
			switch (desc.Type())
			{
				case physics::EConstraintType::SphericalAngular:
				{
					auto const& parameters = std::get<physics::SphericalAngularConstraintParameters>(desc.m_parameters);
					auto const pi = constants<float>::tau_by_2;
					auto const swing_limited = parameters.m_swing_x.m_beg != -pi || parameters.m_swing_x.m_end != pi || parameters.m_swing_z.m_beg != -pi || parameters.m_swing_z.m_end != pi;
					auto const twist_limited = parameters.m_twist.m_beg != -pi || parameters.m_twist.m_end != pi;
					auto const coordinates = physics::MeasureSphericalCoordinates(sample.m_relative.xyzw);
					sample.m_coordinates[3] = coordinates.m_swing.x;
					sample.m_coordinates[4] = coordinates.m_twist;
					sample.m_coordinates[5] = coordinates.m_swing.y;
					sample.m_has_angular_limit = swing_limited || twist_limited;
					sample.m_regular = !sample.m_has_angular_limit || coordinates.m_regular;
					if (swing_limited)
					{
						auto const boundary = physics::MeasureSwingBoundary(coordinates.m_swing, v2{parameters.m_swing_x.m_beg, parameters.m_swing_x.m_end}, v2{parameters.m_swing_z.m_beg, parameters.m_swing_z.m_end});
						sample.m_excess[3] = std::max(boundary.m_distance, 0.0f);
					}
					if (twist_limited)
						sample.m_excess[4] = std::max({parameters.m_twist.m_beg - coordinates.m_twist, 0.0f, coordinates.m_twist - parameters.m_twist.m_end});

					sample.m_angular_excess = std::max(sample.m_excess[3], sample.m_excess[4]);
					break;
				}
				case physics::EConstraintType::D6:
				{
					auto const delta = (sample.m_frame_b.pos - sample.m_frame_a.pos).w0();
					auto const [axis, angle] = AxisAngle(sample.m_frame_b.rot * Transpose(sample.m_frame_a.rot));
					for (int i = 0; i != 3; ++i)
					{
						sample.m_coordinates[i] = Dot3(delta, sample.m_frame_a[i]);
						sample.m_coordinates[3 + i] = Dot3(angle * axis.w0(), sample.m_frame_a[i]);
						sample.m_excess[i] = GeometryAxisExcess(desc.D6().m_linear[i], sample.m_coordinates[i]);
						sample.m_excess[3 + i] = GeometryAxisExcess(desc.D6().m_angular[i], sample.m_coordinates[3 + i]);
						sample.m_has_angular_limit |= GeometryHardAxis(desc.D6().m_angular[i]);
						sample.m_angular_excess = std::max(sample.m_angular_excess, sample.m_excess[3 + i]);
					}
					sample.m_regular = !sample.m_has_angular_limit || angle < constants<float>::tau_by_2;
					break;
				}
				default: { throw std::runtime_error("Unsupported geometry constraint"); }
			}
			return sample;
		}

		// Track sampled episodes only; intervening recovery/reimpact is not excluded by consecutive samples.
		struct GeometryEpisodes
		{
			struct Episode
			{
				int m_first;
				int m_seen;
			};
			std::map<std::string, Episode> m_open;
			std::ostream& m_log;
			int m_frame = -1;
			bool m_candidate = false;
			bool m_lifecycle_censored = false;

			// Report unresolved sampled coverage, including episodes terminated earlier in the history.
			bool NeedsReview() const
			{
				// Removing an ineligible episode cannot supply its missing geometric recovery.
				return m_candidate || m_lifecycle_censored || !m_open.empty();
			}

			// Distinguish historical lifecycle termination from obligations still open at run end.
			std::string_view TerminalReason() const
			{
				// Candidate and currently open episodes take precedence; historical censor events retain their own reasons.
				if (m_candidate)
					return "investigation_candidate";

				if (!m_open.empty())
					return "end_censored";

				return NeedsReview() ? "lifecycle_censored" : "sampled_complete";
			}

			// Start the next complete-scan attempt; gaps must not erase a deadline.
			void Begin(int frame)
			{
				if (frame != m_frame + 1)
					throw std::runtime_error("Geometry history requires consecutive frames");

				m_frame = frame;
			}

			// Observe one eligible whole-pair or whole-angular-constraint metric.
			void Sample(std::string const& key, float value, double level)
			{
				if (!std::isfinite(value))
					throw std::runtime_error("Nonfinite geometry metric: " + key);

				auto iter = m_open.find(key);
				if (value <= level)
				{
					if (iter != m_open.end())
					{
						m_log << "recovered," << m_frame << ',' << key << ',' << iter->second.m_first << ',' << iter->second.m_first + 2 << '\n';
						m_open.erase(iter);
					}
					return;
				}
				if (iter == m_open.end())
				{
					iter = m_open.emplace(key, Episode{m_frame, m_frame}).first;
					m_log << "started," << m_frame << ',' << key << ',' << m_frame << ',' << m_frame + 2 << '\n';
				}
				iter->second.m_seen = m_frame;
				if (m_frame >= iter->second.m_first + 2)
				{
					m_candidate = true;
					m_log << "investigation_candidate," << m_frame << ',' << key << ',' << iter->second.m_first << ',' << iter->second.m_first + 2 << '\n';
				}
			}

			// Lifecycle termination is explicitly not a below-threshold geometric observation.
			void Censor(std::string const& key, std::string_view reason)
			{
				// Only an unresolved episode can lose its opportunity for observed recovery.
				auto const iter = m_open.find(key);
				if (iter == m_open.end())
					return;

				// Preserve the lost coverage after removing this episode from future scan obligations.
				m_lifecycle_censored = true;
				m_log << reason << ',' << m_frame << ',' << key << ',' << iter->second.m_first << ',' << iter->second.m_first + 2 << '\n';
				m_open.erase(iter);
			}

			// A complete frame must account for every still-open obligation.
			void Complete() const
			{
				for (auto const& [key, episode] : m_open)
				{
					if (episode.m_seen != m_frame)
						throw std::runtime_error("Incomplete geometry scan for " + key);
				}
			}
		};

		// Write explicit round-trip coefficients, never native padding or pointers.
		void GeometryMatrix(std::ostream& out, m4x4 const& matrix)
		{
			for (int c = 0; c != 4; ++c)
			{
				for (int r = 0; r != 4; ++r)
					out << ',' << matrix[c][r];
			}
		}

		// Keep representation diagnostics separate from the physical geometry levels.
		double GeometryRotationError(m4x4 const& matrix)
		{
			auto error = 0.0;
			for (int c = 0; c != 4; ++c)
			{
				for (int r = 0; r != 4; ++r)
				{
					if (!std::isfinite(matrix[c][r]))
						throw std::runtime_error("Nonfinite committed geometry transform");
				}
			}
			for (int a = 0; a != 3; ++a)
			{
				for (int b = 0; b != 3; ++b)
				{
					auto value = 0.0;
					for (int r = 0; r != 3; ++r)
						value += static_cast<double>(matrix[a][r]) * matrix[b][r];

					error = std::max(error, std::abs(value - (a == b ? 1.0 : 0.0)));
				}
			}
			return error;
		}

		// Export only the native leaf kinds used by these workloads; other input is an explicit unsupported case.
		void GeometryShape(std::ostream& out, collision::Shape const& shape)
		{
			out << ',' << static_cast<int>(shape.m_type) << ',' << shape.m_material_id << ',' << static_cast<int>(shape.m_flags);
			GeometryMatrix(out, shape.m_s2r);
			for (auto value : {shape.m_bbox.m_centre.x, shape.m_bbox.m_centre.y, shape.m_bbox.m_centre.z, shape.m_bbox.m_radius.x, shape.m_bbox.m_radius.y, shape.m_bbox.m_radius.z})
				out << ',' << value;

			switch (shape.m_type)
			{
				case collision::EShape::Box:
				{
					auto const radius = collision::shape_cast<collision::ShapeBox>(shape).m_radius;
					out << ',' << radius.x << ',' << radius.y << ',' << radius.z;
					break;
				}
				case collision::EShape::Sphere: { out << ',' << collision::shape_cast<collision::ShapeSphere>(shape).m_radius; break; }
				default: { throw std::runtime_error("Geometry recording supports only native Box and Sphere leaves"); }
			}
		}

		// Retain committed evidence in a fresh directory, outside the physics and timing interfaces.
		struct GeometryHistory
		{
			std::ofstream m_primary, m_catalog, m_geometry, m_events;
			GeometryEpisodes m_episodes;
			std::map<std::string, std::string> m_catalog_rows;
			std::map<std::string, std::pair<std::string, std::string>> m_pairs;
			std::map<std::string, m4x4> m_endpoint_poses;
			int m_frame = -1;
			int m_pair_tests = 0;
			int m_constraint_samples = 0;
			bool m_finite = true;

			// Refuse overwriting a prior run and make failed writes explicit.
			explicit GeometryHistory(std::filesystem::path const& directory)
				: m_episodes{{}, m_events}
			{
				if (!std::filesystem::create_directory(directory))
					throw std::runtime_error("Geometry history requires a new output directory");

				for (auto const& [stream, name] : {std::pair{&m_primary, "primary.bin"}, {&m_catalog, "catalog.csv"}, {&m_geometry, "geometry.csv"}, {&m_events, "events.csv"}})
				{
					stream->exceptions(std::ios::failbit | std::ios::badbit);
					stream->open(directory / name, std::ios::binary);
					*stream << std::setprecision(17);
				}
			}

			// Flush all streams before a metric, dispatch, or terminal decision can fail.
			void Flush()
			{
				m_primary.flush();
				m_catalog.flush();
				m_geometry.flush();
				m_events.flush();
			}

			// Write a static catalog entry only when its exact resolved coefficients or lifecycle change.
			bool Catalog(std::string const& key, std::string const& row)
			{
				auto const iter = m_catalog_rows.find(key);
				if (iter != m_catalog_rows.end() && iter->second == row)
					return false;

				m_catalog << "catalog," << m_frame << ',' << key << ',' << row << '\n';
				m_catalog_rows[key] = row;
				return true;
			}

			// Write raw binary32 coefficients with an explicit keyed offset/count record.
			void Primary(std::string const& key, std::span<float const> values)
			{
				m_geometry << "primary," << m_frame << ',' << key << ',' << m_primary.tellp() << ',' << values.size() << '\n';
				m_primary.write(reinterpret_cast<char const*>(values.data()), static_cast<std::streamsize>(values.size_bytes()));
				for (auto value : values)
					m_finite &= std::isfinite(value);
			}

			// Record immutable link mass/frame coefficients without using a derived kinematics query.
			void LinkCatalog(physics::Articulation const& tree, physics::LinkHandle link, int tree_index, int link_index)
			{
				auto const key = GeometryKey(physics::BodyRef::Link(tree, link));
				auto const& desc = tree.LinkDescription(link);
				auto const parent = tree.Parent(link);
				auto row = std::ostringstream{};
				row << std::setprecision(17) << "link," << tree_index << ',' << link_index << ',' << parent.m_index << ',' << parent.m_generation << ',' << static_cast<int>(tree.RootType());
				for (auto vector : {desc.m_inertia.m_diagonal, desc.m_inertia.m_products, desc.m_inertia.m_com_and_mass})
				{
					for (int i = 0; i != 4; ++i)
						row << ',' << vector[i];
				}
				GeometryMatrix(row, desc.m_shape_to_link);
				if (parent)
				{
					auto const& joint = tree.JointDescription(link);
					row << ',' << static_cast<int>(joint.m_type) << ',' << joint.PositionCount() << ',' << joint.m_dof_count;
					GeometryMatrix(row, joint.m_joint_to_parent);
					GeometryMatrix(row, joint.m_joint_to_child);
					for (int axis = 0; axis != joint.m_dof_count; ++axis)
					{
						row << ',' << static_cast<int>(joint.m_axes[axis].m_type);
						for (int i = 0; i != 4; ++i)
							row << ',' << joint.m_axes[axis].m_axis[i];
					}
				}
				Catalog(key, row.str());
			}

			// Retain every primary coefficient before any geometric or finite-state judgment.
			void Capture(Scene const& scene, int frame, double dt)
			{
				m_frame = frame;
				m_finite = true;
				m_pair_tests = 0;
				m_constraint_samples = 0;
				m_endpoint_poses.clear();
				m_episodes.Begin(frame);
				m_geometry << "scan_begin," << frame << ',' << scene.m_clock << ',' << dt << ',' << static_cast<float>(dt) / scene.m_physics_substeps << ','
					<< (frame == 0 ? 0 : scene.m_physics.LastStepProfile().m_substep_count) << '\n';
				for (auto const& body : scene.m_body)
				{
					auto const key = GeometryKey(physics::BodyRef::Rigid(body));
					auto const pose = body.O2W();
					auto const momentum = body.MomentumWS();
					auto values = std::vector<float>{};
					for (int c = 0; c != 4; ++c)
					{
						for (int r = 0; r != 4; ++r)
							values.push_back(pose[c][r]);
					}
					for (auto vector : {momentum.ang, momentum.lin})
					{
						for (int i = 0; i != 3; ++i)
							values.push_back(vector[i]);
					}
					Primary(key, values);
					m_geometry << "sleep," << frame << ',' << key << ',' << body.Sleeping() << '\n';
				}
				for (auto const& tree : scene.m_articulation)
				{
					auto const key = GeometryKey(physics::BodyRef::Link(tree, tree.Root()));
					auto const pose = tree.RootToWorld();
					auto const velocity = tree.RootVelocity();
					auto values = std::vector<float>{};
					for (int c = 0; c != 4; ++c)
					{
						for (int r = 0; r != 4; ++r)
							values.push_back(pose[c][r]);
					}
					for (auto vector : {velocity.ang, velocity.lin})
					{
						for (int i = 0; i != 3; ++i)
							values.push_back(vector[i]);
					}
					Primary(key, values);
					m_geometry << "sleep," << frame << ',' << key << ',' << tree.Sleeping() << ',' << tree.NeverSleep() << '\n';
					for (int i = 1; i != tree.LinkCount(); ++i)
					{
						auto const link = tree.LinkAt(i);
						auto const joint_key = GeometryKey(physics::BodyRef::Link(tree, link));
						Primary(joint_key + ":q", tree.JointPosition(link));
						Primary(joint_key + ":v", tree.JointVelocity(link));
					}
				}
				Flush();
				if (!m_finite)
					throw std::runtime_error("Nonfinite primary geometry state; raw record retained");

				// Demo kill-zone freezing is a lifecycle stop, not physical geometric recovery.
				for (auto const& body : scene.m_body)
				{
					if (body.Mass() < physics::InfiniteMass * 0.5f && body.O2W().pos.z < scene.m_kill_zone_height)
						throw std::runtime_error("Rigid endpoint reached the scene kill zone");
				}
				for (auto const& tree : scene.m_articulation)
				{
					switch (tree.RootType())
					{
						case physics::EArticulationRootType::Fixed: { break; }
						case physics::EArticulationRootType::Floating:
						{
							if (tree.RootToWorld().pos.z < scene.m_kill_zone_height)
								throw std::runtime_error("Articulation reached the scene kill zone");

							break;
						}
						default: { throw std::runtime_error("Unsupported articulation root in geometry history"); }
					}
				}

				// Catalogs and representation diagnostics follow the already durable raw snapshot.
				for (int t = 0; t != isize(scene.m_articulation); ++t)
				{
					auto const& tree = scene.m_articulation[t];
					for (int i = 0; i != tree.LinkCount(); ++i)
					{
						auto const link = tree.LinkAt(i);
						auto const key = GeometryKey(physics::BodyRef::Link(tree, link));
						LinkCatalog(tree, link, t, i);
						auto const pose = tree.LinkToWorld(link);
						m_endpoint_poses[key] = pose;
						m_geometry << "endpoint," << frame << ',' << key;
						GeometryMatrix(m_geometry, pose);
						m_geometry << ',' << GeometryRotationError(pose) << '\n';
						if (i == 0)
							continue;

						auto const& joint = tree.JointDescription(link);
						auto const q = tree.JointPosition(link);
						switch (joint.m_type)
						{
							case physics::EArticulationJointType::Spherical:
							{
								auto norm_sq = 0.0;
								for (auto value : q)
									norm_sq += static_cast<double>(value) * value;

								m_geometry << "quaternion_norm_sq," << frame << ',' << key << ',' << norm_sq << '\n';
								break;
							}
							case physics::EArticulationJointType::Axes:
							{
								if (joint.m_dof_count == 1)
								{
									switch (joint.m_axes[0].m_type)
									{
										case physics::EArticulationAxisType::Revolute:
										{
											auto const regular = std::abs(q[0]) < constants<float>::tau_by_2;
											m_geometry << "hinge_domain," << frame << ',' << key << ',' << q[0] << ',' << regular << '\n';
											if (!regular)
												throw std::runtime_error("Committed hinge left the principal domain: " + key);

											break;
										}
										case physics::EArticulationAxisType::Prismatic: { break; }
										default: { throw std::runtime_error("Unsupported recorded articulation axis"); }
									}
								}
								break;
							}
							default: { throw std::runtime_error("Unsupported recorded articulation joint"); }
						}
					}
				}
				for (int i = 0; i != isize(scene.m_body); ++i)
				{
					auto const& body = scene.m_body[i];
					auto const key = GeometryKey(physics::BodyRef::Rigid(body));
					m_endpoint_poses[key] = body.O2W();
					Catalog(key, std::format("rigid,{},{}", i, body.Mass()));
				}
			}

			// Retain placed leaf input and collision policies before the existing pair scan.
			void Placements(std::span<CollisionPlacement const> placements)
			{
				for (auto const& placement : placements)
				{
					auto const key = GeometryKey(placement.m_identity);
					auto row = std::ostringstream{};
					row << std::setprecision(17) << "shape," << GeometryKey(placement.m_parent_identity) << ','
						<< placement.m_collide_parent << ',' << placement.m_collide_self << ',' << placement.m_layer << ',' << placement.m_mask;
					GeometryShape(row, *placement.m_shape);
					if (Catalog("shape:" + key, row.str()))
					{
						for (auto const& [pair_key, endpoints] : m_pairs)
						{
							if (endpoints.first == key || endpoints.second == key)
								m_episodes.Censor(pair_key, "endpoint_edit");
						}
					}
					m_geometry << "placement," << m_frame << ',' << key << ',' << placement.m_body_index << ',' << placement.m_articulation << ',' << placement.m_link << ',' << placement.m_parent_index;
					GeometryMatrix(m_geometry, placement.m_o2w);
					for (auto value : {placement.m_bounds.Lower().x, placement.m_bounds.Lower().y, placement.m_bounds.Lower().z, placement.m_bounds.Upper().x, placement.m_bounds.Upper().y, placement.m_bounds.Upper().z})
					{
						m_geometry << ',' << value;
						if (!std::isfinite(value))
							throw std::runtime_error("Nonfinite committed shape bounds: " + key);
					}
					m_geometry << '\n';
				}
			}

			// Keep every colliding pair, and explicit narrow-phase separation for an outstanding pair.
			void Pair(CollisionPlacement const& a, CollisionPlacement const& b, bool hit, collision::Contact const& contact)
			{
				++m_pair_tests;
				auto key_a = GeometryKey(a.m_identity);
				auto key_b = GeometryKey(b.m_identity);
				if (key_b < key_a)
					std::swap(key_a, key_b);

				auto const key = "pair:" + key_a + "|" + key_b;
				if (!hit && !m_episodes.m_open.contains(key))
					return;

				if (hit)
				{
					// The query already returns world-space witnesses in its original A/B order, not the sorted episode order.
					auto const count = contact.Count();
					auto const feature = static_cast<int>(contact.m_feature);
					m_geometry << "contact_witness," << m_frame << ',' << key << ',' << GeometryKey(a.m_identity) << ',' << GeometryKey(b.m_identity)
						<< ",root,root,world," << contact.m_axis.x << ',' << contact.m_axis.y << ',' << contact.m_axis.z << ',' << contact.m_axis.w
						<< ',' << contact.m_depth << ',' << feature << ',' << contact.m_mat_idA << ',' << contact.m_mat_idB << ',' << count;
					for (auto const& point : contact.Points())
						m_geometry << ',' << point.x << ',' << point.y << ',' << point.z << ',' << point.w;

					m_geometry << '\n';
					if (!std::isfinite(contact.m_depth))
						throw std::runtime_error("Nonfinite native contact depth: " + key);

					if (feature < 1 || feature > collision::Contact::MaxManifoldPoints || !IsFinite(contact.m_axis))
						throw std::runtime_error("Invalid native contact witness: " + key);

					for (auto const& point : contact.Points())
					{
						if (!IsFinite(point))
							throw std::runtime_error("Nonfinite native contact point: " + key);
					}
				}
				auto const depth = hit ? std::max(contact.m_depth, 0.0f) : 0.0f;
				m_geometry << "pair," << m_frame << ',' << key << ',' << (hit ? "contact" : "narrow_separated") << ',' << depth << '\n';
				m_pairs[key] = {key_a, key_b};
				m_episodes.Sample(key, depth, 0.005);
			}

			// Missing pairs need conservative separation or an explicit eligibility change, not an invented zero.
			void EndPairs(std::span<CollisionPlacement const> placements, std::span<uint64_t const> excluded)
			{
				auto current = std::map<std::string, CollisionPlacement const*>{};
				for (auto const& placement : placements)
					current.emplace(GeometryKey(placement.m_identity), &placement);

				for (auto const& [key, endpoints] : m_pairs)
				{
					auto const episode = m_episodes.m_open.find(key);
					if (episode == m_episodes.m_open.end() || episode->second.m_seen == m_frame)
						continue;

					if (!current.contains(endpoints.first) || !current.contains(endpoints.second))
					{
						m_episodes.Censor(key, "endpoint_removed");
						continue;
					}
					auto const& a = *current.at(endpoints.first);
					auto const& b = *current.at(endpoints.second);
					if (!GeometryPairEligible(a, b, excluded))
					{
						m_episodes.Censor(key, "pair_ineligible");
						continue;
					}
					if (IsIntersection(a.m_bounds, b.m_bounds))
						throw std::runtime_error("Eligible overlapping pair missing from complete geometry scan: " + key);

					m_geometry << "pair," << m_frame << ',' << key << ",bounds_separated,0\n";
					m_episodes.Sample(key, 0.0f, 0.005);
				}

				// Only outstanding episodes need endpoint history for the next sparse scan.
				std::erase_if(m_pairs, [&](auto const& pair)
				{
					return !m_episodes.m_open.contains(pair.first);
				});
			}

			// Record native and packed descriptors, then measure eligible whole-angular-constraint errors.
			void Limits(Scene const& scene)
			{
				auto const packed = physics::PackGpuConstraints(scene.m_constraints, physics::BodyRemap{scene.m_body_ptrs, scene.m_articulation_ptrs});
				m_geometry << "revisions," << m_frame << ',' << packed.m_topology_revision << ',' << packed.m_parameter_revision << '\n';
				auto active = std::map<std::string, bool>{};
				for (uint32_t index = 0; index != packed.m_endpoints.size(); ++index)
				{
					auto const handle = physics::ConstraintHandle{index, packed.m_endpoints[index].generation};
					auto const key = std::format("constraint:{}:{}", index, handle.m_generation);
					auto const occupied = scene.m_constraints.Contains(handle);
					if (!occupied)
					{
						Catalog(key, "vacant");
						m_geometry << "eligibility," << m_frame << ',' << key << ",0,0,0,0\n";
						continue;
					}
					auto const& desc = scene.m_constraints.Get(handle);
					auto const broken = scene.m_constraints.IsBroken(handle);
					auto const eligible = desc.m_enabled && !broken;
					auto row = std::ostringstream{};
					row << std::setprecision(17) << "constraint," << static_cast<int>(desc.Type()) << ',' << GeometryKey(desc.m_frame_a.m_body) << ',' << GeometryKey(desc.m_frame_b.m_body)
						<< ',' << desc.m_enabled << ',' << broken << ',' << desc.m_collide_connected << ',' << desc.m_break_force << ',' << desc.m_break_torque;
					GeometryMatrix(row, desc.m_frame_a.m_constraint_to_body);
					GeometryMatrix(row, desc.m_frame_b.m_constraint_to_body);
					switch (desc.Type())
					{
						case physics::EConstraintType::D6:
						{
							for (auto const& axes : {desc.D6().m_linear, desc.D6().m_angular})
							{
								for (auto const& axis : axes)
									row << ',' << static_cast<int>(axis.m_mode) << ',' << axis.m_limits.m_beg << ',' << axis.m_limits.m_end << ',' << axis.m_target_position << ',' << axis.m_target_velocity << ',' << axis.m_stiffness << ',' << axis.m_damping << ',' << axis.m_max_force;
							}
							break;
						}
						case physics::EConstraintType::SphericalAngular:
						{
							auto const& parameters = std::get<physics::SphericalAngularConstraintParameters>(desc.m_parameters);
							row << ',' << parameters.m_swing_x.m_beg << ',' << parameters.m_swing_x.m_end << ',' << parameters.m_swing_z.m_beg << ',' << parameters.m_swing_z.m_end << ','
								<< parameters.m_twist.m_beg << ',' << parameters.m_twist.m_end << ',' << parameters.m_damping;
							break;
						}
						default: { throw std::runtime_error("Unsupported recorded constraint"); }
					}
					if (Catalog(key, row.str()))
					{
						// Native named coefficients decide lifecycle equality; packed padding never resets an episode.
						auto const& gpu = packed.m_descriptors[index];
						auto const bytes = std::as_bytes(std::span{&gpu, 1});
						auto hex = std::ostringstream{};
						for (auto value : bytes)
							hex << std::hex << std::setw(2) << std::setfill('0') << std::to_integer<unsigned>(value);

						m_catalog << "packed_constraint," << m_frame << ',' << key << ',' << hex.str() << '\n';
						m_episodes.Censor(key, !desc.m_enabled ? "disabled" : broken ? "broken" : "descriptor_edit");
					}

					active[key] = eligible;
					m_geometry << "eligibility," << m_frame << ',' << key << ',' << occupied << ',' << desc.m_enabled << ',' << broken << ',' << eligible << '\n';
					if (!eligible)
						continue;

					auto endpoint = [&](physics::BodyRef const& body)
					{
						return body.IsWorld() ? m4x4::Identity() : m_endpoint_poses.at(GeometryKey(body));
					};
					auto const sample = MeasureGeometryLimit(desc, endpoint(desc.m_frame_a.m_body), endpoint(desc.m_frame_b.m_body));
					++m_constraint_samples;
					m_geometry << "limit," << m_frame << ',' << key << ',' << sample.m_regular << ',' << sample.m_has_angular_limit << ',' << sample.m_angular_excess;
					for (auto value : sample.m_coordinates)
						m_geometry << ',' << value;

					for (auto value : sample.m_excess)
						m_geometry << ',' << value;

					GeometryMatrix(m_geometry, sample.m_frame_a);
					GeometryMatrix(m_geometry, sample.m_frame_b);
					for (int i = 0; i != 4; ++i)
						m_geometry << ',' << sample.m_relative.xyzw[i];

					m_geometry << '\n';
					for (auto const& values : {sample.m_coordinates, sample.m_excess})
					{
						for (auto value : values)
						{
							if (!std::isfinite(value))
								throw std::runtime_error("Nonfinite committed constraint coordinate: " + key);
						}
					}
					if (!sample.m_regular)
						throw std::runtime_error("Irregular committed constraint chart: " + key);

					if (sample.m_has_angular_limit)
						m_episodes.Sample(key, sample.m_angular_excess, 0.02);
				}
				auto removed = std::vector<std::string>{};
				for (auto const& [key, episode] : m_episodes.m_open)
				{
					if (key.starts_with("constraint:") && !active.contains(key))
						removed.push_back(key);
				}
				for (auto const& key : removed)
					m_episodes.Censor(key, "constraint_removed");
			}
		};

		// Measure all collision-enabled shaped endpoint pairs after commit, including sleeping bodies and links.
		PenetrationSample MeasurePenetration(Scene const& scene, GeometryHistory* history = nullptr)
		{
			auto sample = PenetrationSample{};

			for (auto const& body : scene.m_body)
				sample.m_kinetic_energy += body.KineticEnergy();
			for (auto const& articulation : scene.m_articulation)
				sample.m_kinetic_energy += articulation.KineticEnergy();

			// Use the native persistent-constraint owner to preserve enabled/broken and all-free exclusion semantics.
			auto const remap = physics::BodyRemap{scene.m_body_ptrs, scene.m_articulation_ptrs};
			auto const constraints = physics::PackGpuConstraints(scene.m_constraints, remap);
			auto excluded = std::vector<uint64_t>{};
			for (auto const& pair : constraints.m_collision_exclusions.m_slots)
			{
				if (pair.body_idx_a_plus_one != 0)
					excluded.push_back((uint64_t{pair.body_idx_a_plus_one} << 32) | pair.body_idx_b_plus_one);
			}
			for (auto const& [a, b] : scene.m_collision_ignore_indices)
				excluded.push_back(CollisionPairKey(a, b));

			std::ranges::sort(excluded);

			// Keep native root-space shape transforms and full-scene slots even when intervening links have no shape.
			auto placements = std::vector<CollisionPlacement>{};
			for (int i = 0; i != std::ssize(scene.m_body); ++i)
			{
				auto const& body = scene.m_body[i];
				if (!body.HasShape())
					continue;

				placements.push_back(CollisionPlacement{
					.m_identity = physics::BodyRef::Rigid(body),
					.m_shape = &body.Shape(), .m_o2w = body.O2W(), .m_bounds = body.BBoxWS(), .m_body_index = i,
					.m_layer = body.CollisionLayer(), .m_mask = body.CollisionMask(),
				});
			}
			for (int articulation_index = 0; articulation_index != isize(scene.m_articulation); ++articulation_index)
			{
				auto const& articulation = scene.m_articulation[articulation_index];
				for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
				{
					auto const link = articulation.LinkAt(link_index);
					auto const& desc = articulation.LinkDescription(link);
					if (desc.m_shape == nullptr)
						continue;

					auto const parent = articulation.Parent(link);
					auto const shape_to_world = articulation.LinkToWorld(link) * desc.m_shape_to_link;
					placements.push_back(CollisionPlacement{
						.m_identity = physics::BodyRef::Link(articulation, link),
						.m_parent_identity = parent ? physics::BodyRef::Link(articulation, parent) : physics::BodyRef::World(),
						.m_shape = desc.m_shape, .m_o2w = shape_to_world,
						.m_bounds = shape_to_world * (desc.m_shape->m_s2r * desc.m_shape->m_bbox),
						.m_body_index = remap.Resolve(physics::BodyRef::Link(articulation, link)),
						.m_articulation = articulation_index, .m_link = link_index,
						.m_parent_index = parent ? remap.Resolve(physics::BodyRef::Link(articulation, parent)) : -1,
						.m_collide_parent = desc.m_collide_parent, .m_collide_self = desc.m_collide_self,
					});
				}
			}

			// Sweep conservative X intervals before narrow phase; physical screening is separate from engine timing.
			if (history)
			{
				history->Placements(placements);
				auto ignores = std::ostringstream{};
				ignores << "excluded";
				for (auto key : excluded)
					ignores << ',' << key;

				history->Catalog("collision_exclusions", ignores.str());
			}
			std::ranges::sort(placements, {}, [](auto const& placement) { return placement.m_bounds.Lower().x; });
			for (int i = 0; i != isize(placements); ++i)
			{
				auto const& a = placements[i];
				for (int j = i + 1; j != isize(placements); ++j)
				{
					auto const& b = placements[j];
					if (b.m_bounds.Lower().x > a.m_bounds.Upper().x)
						break;

					if (!IsIntersection(a.m_bounds, b.m_bounds) || !GeometryPairEligible(a, b, excluded))
						continue;

					auto contact = collision::Contact{};
					auto const hit = CollideLeaves(*a.m_shape, a.m_o2w, *b.m_shape, b.m_o2w, contact);
					if (history)
						history->Pair(a, b, hit, contact);

					if (!hit)
						continue;

					++sample.m_contact_count;
					if (contact.m_depth <= sample.m_max_depth)
						continue;

					sample.m_max_depth = contact.m_depth;
					sample.m_body_a = a.m_body_index;
					sample.m_body_b = b.m_body_index;
					auto const& link = a.m_articulation >= 0 ? a : b;
					sample.m_articulation = link.m_articulation;
					sample.m_link = link.m_link;
				}
			}
			if (history)
				history->EndPairs(placements, excluded);

			return sample;
		}

		// Run ordinary scene visits with durable committed-state evidence and a sampled, not continuous, stop rule.
		SceneDiagnosticResult RunGeometryHistory(Scene& scene, SceneDiagnosticOptions const& options)
		{
			auto history = GeometryHistory(options.m_geometry_history);
			try
			{
				// Preserve inputs/settings; recording cannot silently omit non-leaf world geometry.
				std::filesystem::copy_file(options.m_scene_filepath, options.m_geometry_history / "scene.json");
				if (scene.m_terrain_surface || scene.m_water || scene.m_atmosphere_gfx)
					throw std::runtime_error("Geometry history does not record terrain, water, or atmosphere inputs");

				auto const& config = scene.m_physics.Config();
				#define GEOMETRY_SETTING(name) history.m_catalog << "setting," #name "," << config.name << '\n'
				GEOMETRY_SETTING(max_collision_pairs); GEOMETRY_SETTING(max_collision_events); GEOMETRY_SETTING(max_internal_substeps);
				GEOMETRY_SETTING(sleeping_enabled); GEOMETRY_SETTING(sleep_velocity_threshold_lin); GEOMETRY_SETTING(sleep_velocity_threshold_ang); GEOMETRY_SETTING(sleep_delay_s);
				GEOMETRY_SETTING(solver_iterations); GEOMETRY_SETTING(contact_velocity_tolerance); GEOMETRY_SETTING(fused_colour_sweep_limit); GEOMETRY_SETTING(push_out_iterations);
				GEOMETRY_SETTING(broadphase_aabb_margin); GEOMETRY_SETTING(contact_sort_propagation_scale); GEOMETRY_SETTING(contact_sort_shock_iterations);
				GEOMETRY_SETTING(contact_sort_shock_alignment); GEOMETRY_SETTING(contact_sort_shock_min_strength); GEOMETRY_SETTING(contact_sort_shock_decay);
				GEOMETRY_SETTING(penetration_slop); GEOMETRY_SETTING(velocity_baumgarte); GEOMETRY_SETTING(position_slop); GEOMETRY_SETTING(position_baumgarte);
				GEOMETRY_SETTING(contact_slop_scale); GEOMETRY_SETTING(support_contact_slop_scale); GEOMETRY_SETTING(warm_start_scale);
				GEOMETRY_SETTING(constraint_relaxation); GEOMETRY_SETTING(constraint_coupled_relaxation); GEOMETRY_SETTING(constraint_coupled_backtrack_limit);
				GEOMETRY_SETTING(constraint_position_relaxation); GEOMETRY_SETTING(constraint_position_beta); GEOMETRY_SETTING(constraint_max_position_speed);
				GEOMETRY_SETTING(constraint_regularization); GEOMETRY_SETTING(constraint_warm_start_factor);
				GEOMETRY_SETTING(deep_penetration_threshold); GEOMETRY_SETTING(deep_penetration_range); GEOMETRY_SETTING(deep_penetration_baumgarte_min); GEOMETRY_SETTING(deep_penetration_baumgarte_max);
				GEOMETRY_SETTING(selective_refresh_passes); GEOMETRY_SETTING(selective_refresh_max_pairs); GEOMETRY_SETTING(selective_refresh_body_limit); GEOMETRY_SETTING(selective_refresh_contact_limit);
				GEOMETRY_SETTING(selective_refresh_solver_iterations); GEOMETRY_SETTING(selective_refresh_position_iterations); GEOMETRY_SETTING(selective_refresh_bias_scale);
				GEOMETRY_SETTING(selective_refresh_restitution_scale); GEOMETRY_SETTING(selective_refresh_adaptive_body_limit); GEOMETRY_SETTING(selective_refresh_adaptive_solver_iterations);
				GEOMETRY_SETTING(selective_refresh_support_only); GEOMETRY_SETTING(selective_refresh_resolve_support_only); GEOMETRY_SETTING(selective_refresh_depth_slop);
				GEOMETRY_SETTING(selective_refresh_support_depth_slop); GEOMETRY_SETTING(selective_refresh_closing_speed_slop); GEOMETRY_SETTING(selective_refresh_support_alignment);
				GEOMETRY_SETTING(selective_refresh_aabb_margin); GEOMETRY_SETTING(world_collision_layer); GEOMETRY_SETTING(world_collision_mask);
				#undef GEOMETRY_SETTING
				history.m_catalog << "scene," << scene.m_body.size() << ',' << scene.m_articulation.size() << ',' << scene.m_constraints.Count() << ','
					<< scene.m_physics_substeps << ',' << options.m_dt << ',' << scene.m_gravity.x << ',' << scene.m_gravity.y << ',' << scene.m_gravity.z << ',' << scene.m_kill_zone_height << '\n';
				history.m_catalog << "build," << compute::HasDebugLayer(scene.m_physics.Device()) << ',' << PR_PHYSICS_DIAGNOSTICS << ',' << PR_PHYSICS_PROFILE << '\n';
				history.m_catalog << "levels," << 0.005 << ',' << 0.02 << ",2\n";
				history.m_catalog << "packed_descriptor_abi," << sizeof(physics::GpuConstraintDesc) << ',' << offsetof(physics::GpuConstraintDesc, frame_a) << ','
					<< offsetof(physics::GpuConstraintDesc, frame_b) << ',' << offsetof(physics::GpuConstraintDesc, axes) << ',' << offsetof(physics::GpuConstraintDesc, type) << ','
					<< offsetof(physics::GpuConstraintDesc, swing_limits) << ',' << offsetof(physics::GpuConstraintDesc, twist_limits) << ',' << offsetof(physics::GpuConstraintDesc, angular_damping) << '\n';
				for (int i = 0; i != isize(scene.m_articulation_actuation); ++i)
				{
					auto const& actuation = scene.m_articulation_actuation[i];
					history.m_catalog << "actuation," << i << ',' << actuation.m_torque_per_kg << ',' << actuation.m_period_s << ',' << actuation.m_phase << '\n';
				}

				// A candidate finishes its committed scan, then stops before any subsequent Scene::Step.
				for (int frame = 0; frame <= options.m_steps; ++frame)
				{
					if (frame != 0)
					{
						history.m_events << "step_attempt," << frame << '\n';
						history.Flush();
						scene.Step(options.m_dt);
					}
					history.Capture(scene, frame, options.m_dt);
					history.Limits(scene);
					auto const penetration = MeasurePenetration(scene, &history);
					history.m_episodes.Complete();
					history.m_geometry << "scan_complete," << frame << ',' << history.m_pair_tests << ',' << penetration.m_contact_count << ',' << penetration.m_max_depth << ','
						<< history.m_episodes.m_open.size() << ',' << history.m_episodes.m_candidate << ',' << history.m_constraint_samples << '\n';
					history.Flush();
					if (history.m_episodes.m_candidate)
						break;
				}

				// Record run-end censoring separately from historical lifecycle censoring and successful sampled recovery.
				auto const needs_review = history.m_episodes.NeedsReview();
				auto const reason = history.m_episodes.TerminalReason();
				history.m_events << "terminal," << history.m_frame << ',' << reason << '\n';
				for (auto const& [key, episode] : history.m_episodes.m_open)
					history.m_events << "end_censored," << history.m_frame << ',' << key << ',' << episode.m_first << ',' << episode.m_first + 2 << '\n';

				history.Flush();
				printf("Geometry history: %s at frame %d; impact-travel allowance unmeasured.\n", reason.data(), history.m_frame);
				return SceneDiagnosticResult{.m_geometry_needs_review = needs_review};
			}
			catch (std::exception const& error)
			{
				// A native/observer failure leaves the durable primary and an explicitly incomplete scan.
				history.m_events << "terminal," << history.m_frame << ",failure," << std::quoted(error.what()) << '\n';
				history.Flush();
				throw;
			}
		}

#if PR_UNITTESTS
		// Isolate terminal-decision coverage for the class-filtered CPU runner; no scene or Engine is required.
		PRUnitTestClass(SceneGeometryTerminalTests)
		{
			// Check lifecycle-censored review and recovery-only success through the recorder's actual terminal decision.
			PRUnitTestMethod(LifecycleCensoringRequiresReview, Quick)
			{
				// End an unresolved episode before its deadline, leaving neither a candidate nor an open obligation.
				auto log = std::ostringstream{};
				auto episodes = GeometryEpisodes{{}, log};
				episodes.Begin(0);
				episodes.Sample("pair:a|b", 0.006f, 0.005f);
				episodes.Complete();
				PR_EXPECT(episodes.NeedsReview() && episodes.TerminalReason() == "end_censored");
				episodes.Begin(1);
				episodes.Censor("pair:a|b", "pair_ineligible");
				episodes.Complete();
				PR_EXPECT(episodes.m_open.empty() && !episodes.m_candidate);
				PR_EXPECT(episodes.NeedsReview());
				PR_EXPECT(episodes.TerminalReason() == "lifecycle_censored");
				PR_EXPECT(log.str().find("pair_ineligible,1,pair:a|b,0,2") != std::string::npos);
				PR_EXPECT(log.str().find("recovered,") == std::string::npos && log.str().find("end_censored,") == std::string::npos);

				// Recovery of a later episode with the same identity does not recover the earlier censored episode.
				episodes.Begin(2);
				episodes.Sample("pair:a|b", 0.006f, 0.005f);
				episodes.Complete();
				episodes.Begin(3);
				episodes.Sample("pair:a|b", 0.0f, 0.005f);
				episodes.Complete();
				PR_EXPECT(episodes.m_open.empty() && !episodes.m_candidate && episodes.NeedsReview());
				PR_EXPECT(episodes.TerminalReason() == "lifecycle_censored");

				// A separate recovery-only history succeeds; censoring a key with no open episode is harmless.
				auto recovered_log = std::ostringstream{};
				auto recovered = GeometryEpisodes{{}, recovered_log};
				recovered.Begin(0);
				recovered.Sample("pair:a|b", 0.006f, 0.005f);
				recovered.Complete();
				recovered.Begin(1);
				recovered.Sample("pair:a|b", 0.0f, 0.005f);
				recovered.Complete();
				PR_EXPECT(!recovered.NeedsReview() && recovered.TerminalReason() == "sampled_complete");
				recovered.Censor("pair:a|b", "pair_ineligible");
				PR_EXPECT(!recovered.NeedsReview() && recovered.TerminalReason() == "sampled_complete");
			}
		};

		// Portable checks of new observation adapters; no Engine construction or native trajectory.
		PRUnitTestClass(SceneGeometryRecorderTests)
		{
			// Check newly composed descriptor frames and whole-angular aggregation, not a duplicate coordinate implementation.
			PRUnitTestMethod(DescriptorGeometryAdapter, Quick)
			{
				auto const frame_a = m4x4::Transform(v4::XAxis(), 0.5f, v4{1, 2, 3, 1});
				auto const frame_b = m4x4::Transform(v4::YAxis(), -0.25f, v4{-2, 1, 4, 1});
				auto const endpoint_a = m4x4::Transform(v4::ZAxis(), 0.4f, v4{3, -2, 1, 1});
				auto const desired = m4x4::Transform(v4::XAxis(), 0.75f, v4::Origin());
				auto const endpoint_b = endpoint_a * frame_a * desired * InvertOrthonormal(frame_b);
				auto desc = physics::ConstraintDesc{};
				desc.m_frame_a.m_constraint_to_body = frame_a;
				desc.m_frame_b.m_constraint_to_body = frame_b;
				desc.D6().m_angular[0] = {.m_mode = physics::EConstraintAxisMode::Limited, .m_limits = {-0.1f, 0.2f}};
				auto const limited = MeasureGeometryLimit(desc, endpoint_a, endpoint_b);
				PR_EXPECT(limited.m_regular && limited.m_has_angular_limit);
				PR_EXPECT(std::abs(limited.m_coordinates[3] - 0.75f) < 2.0e-6f);
				PR_EXPECT(std::abs(limited.m_angular_excess - 0.55f) < 2.0e-6f);
				desc.D6().m_angular[0].m_mode = physics::EConstraintAxisMode::Driven;
				auto const driven = MeasureGeometryLimit(desc, endpoint_a, endpoint_b);
				PR_EXPECT(!driven.m_has_angular_limit && driven.m_angular_excess == 0);

				// A point displaced along an asymmetric ellipse's outward normal has known Euclidean distance.
				auto const query = v2{0.6f, 0.4f} + 0.2f * Normalise(v2{0.6f, 1.6f});
				auto const orientation = ExpMap<quat>(0.5f * v3{query.x, 0, query.y});
				desc.m_parameters = physics::SphericalAngularConstraintParameters{.m_swing_x = {-1, 1}, .m_swing_z = {-0.5f, 0.5f}, .m_twist = {-0.1f, 0.1f}};
				auto const swing = MeasureGeometryLimit(desc, endpoint_a, endpoint_a * frame_a * m4x4(ToMatrix<m3x3>(orientation), v4::Origin()) * InvertOrthonormal(frame_b));
				PR_EXPECT(swing.m_regular && std::abs(swing.m_angular_excess - 0.2f) < 3.0e-6f);
				auto const both = MeasureGeometryLimit(desc, endpoint_a, endpoint_a * frame_a * m4x4(ToMatrix<m3x3>(orientation * ExpMap<quat>(v3{0, 0.3f, 0})), v4::Origin()) * InvertOrthonormal(frame_b));
				PR_EXPECT(std::abs(both.m_excess[3] - 0.2f) < 3.0e-6f);
				PR_EXPECT(std::abs(both.m_angular_excess - 0.5f) < 3.0e-6f);
			}

			// Verify deadlines survive competing identities and that lifecycle changes never report recovery.
			PRUnitTestMethod(SampledGeometryEpisodes, Quick)
			{
				auto log = std::ostringstream{};
				auto episodes = GeometryEpisodes{{}, log};
				episodes.Begin(0);
				episodes.Sample("pair:a|b", 0.006f, 0.005f);
				episodes.Sample("constraint:1", 0.03f, 0.02f);
				episodes.Complete();
				episodes.Begin(1);
				episodes.Sample("pair:a|b", 0.007f, 0.005f);
				episodes.Sample("pair:a|c", 0.01f, 0.005f);
				episodes.Censor("constraint:1", "descriptor_edit");
				episodes.Sample("constraint:1", 0.03f, 0.02f);
				episodes.Complete();
				PR_EXPECT(!episodes.m_candidate && episodes.m_open.at("constraint:1").m_first == 1);
				episodes.Begin(2);
				episodes.Sample("pair:a|b", 0.006f, 0.005f);
				episodes.Sample("pair:a|c", 0.0f, 0.005f);
				episodes.Censor("constraint:1", "broken");
				episodes.Complete();
				PR_EXPECT(episodes.m_candidate && episodes.m_open.size() == 1);
				PR_EXPECT(log.str().find("recovered,2,pair:a|c") != std::string::npos);
				PR_EXPECT(log.str().find("recovered,2,constraint:1") == std::string::npos);
				PR_EXPECT(log.str().find("investigation_candidate,2,pair:a|b,0,2") != std::string::npos);
				PR_THROWS(episodes.Begin(4), std::runtime_error);

				// Whole-joint identity stays constant when the maximum switches from swing to twist.
				auto joint_log = std::ostringstream{};
				auto joint = GeometryEpisodes{{}, joint_log};
				for (int frame = 0; frame != 3; ++frame)
				{
					joint.Begin(frame);
					auto const swing = frame == 1 ? 0.0f : 0.03f;
					auto const twist = frame == 1 ? 0.04f : 0.0f;
					joint.Sample("joint", std::max(swing, twist), 0.02f);
					joint.Complete();
				}
				PR_EXPECT(joint.m_candidate);
			}

			// Check the recorder's eligibility adapter and explicit unsupported-shape handling without a GPU scene.
			PRUnitTestMethod(PairAndShapeRecordingAdapter, Quick)
			{
				auto const shape = collision::ShapeSphere(1);
				auto a = CollisionPlacement{.m_shape = &shape.m_base, .m_body_index = 0, .m_articulation = 0, .m_link = 0};
				auto b = CollisionPlacement{.m_shape = &shape.m_base, .m_body_index = 1, .m_articulation = 0, .m_link = 1, .m_parent_index = 0};
				PR_EXPECT(!GeometryPairEligible(a, b, {}));
				b.m_collide_parent = true;
				PR_EXPECT(GeometryPairEligible(a, b, {}));
				auto const exclusions = std::array{CollisionPairKey(0, 1)};
				PR_EXPECT(!GeometryPairEligible(a, b, exclusions));
				auto output = std::ostringstream{};
				GeometryShape(output, shape.m_base);
				PR_EXPECT(!output.str().empty());
				auto unsupported = shape.m_base;
				unsupported.m_type = collision::EShape::Array;
				PR_THROWS(GeometryShape(output, unsupported), std::runtime_error);
			}

			// Exercise the actual recorder's durable raw writer and sparse-pair completion adapter without constructing an Engine.
			PRUnitTestMethod(SparsePairRecording, Quick)
			{
				auto const directory = std::filesystem::temp_directory_path() / std::format("rylogic-geometry-{}", std::chrono::steady_clock::now().time_since_epoch().count());
				{
					auto history = GeometryHistory(directory);
					auto const shape = collision::ShapeSphere(1);
					auto placements = std::array{
						CollisionPlacement{.m_identity = {.m_type = physics::EConstraintBodyType::Rigid, .m_body_id = {17}}, .m_shape = &shape.m_base, .m_bounds = BBox(v4::Origin(), v4{1, 1, 1, 0}), .m_body_index = 0},
						CollisionPlacement{.m_identity = {.m_type = physics::EConstraintBodyType::Rigid, .m_body_id = {19}}, .m_shape = &shape.m_base, .m_bounds = BBox(v4::Origin(), v4{1, 1, 1, 0}), .m_body_index = 1},
					};
					auto contact = collision::Contact{};
					contact.m_depth = 0.006f;
					contact.m_axis = v4{0, 1, 0, 0};
					contact.m_feature = collision::EFeature::Edge;
					contact.m_mat_idA = 7;
					contact.m_mat_idB = 11;
					contact.m_manifold[0] = v4{1, 2, 3, 1};
					contact.m_manifold[1] = v4{4, 5, 6, 1};
					history.m_frame = 0;
					history.m_episodes.Begin(0);
					auto const raw = std::array{0.125f, -0.25f, 0.5f, 0.75f};
					history.Primary("raw", raw);
					history.Placements(placements);
					history.Pair(placements[1], placements[0], true, contact);
					history.EndPairs(placements, {});
					history.m_episodes.Complete();
					history.Flush();
					auto input = std::ifstream(directory / "primary.bin", std::ios::binary);
					auto actual = std::array<float, 4>{};
					input.read(reinterpret_cast<char*>(actual.data()), sizeof(actual));
					PR_EXPECT(actual == raw && std::filesystem::file_size(directory / "primary.bin") == sizeof(raw));

					// A conservative bounds proof closes a missing outstanding pair, without reporting every separated pair.
					history.m_frame = 1;
					history.m_episodes.Begin(1);
					placements[1].m_bounds = BBox(v4{5, 0, 0, 1}, v4{1, 1, 1, 0});
					history.Placements(placements);
					history.EndPairs(placements, {});
					history.m_episodes.Complete();
					PR_EXPECT(history.m_episodes.m_open.empty() && history.m_pairs.empty());

					// Exclusion terminates a new episode without fabricating below-threshold geometry.
					history.m_frame = 2;
					history.m_episodes.Begin(2);
					placements[1].m_bounds = placements[0].m_bounds;
					history.Placements(placements);
					history.Pair(placements[0], placements[1], true, contact);
					history.m_episodes.Complete();

					// A nonfinite signed depth must remain raw failure evidence, never a clamped recovery.
					contact.m_depth = -std::numeric_limits<float>::infinity();
					PR_THROWS(history.Pair(placements[0], placements[1], true, contact), std::runtime_error);
					PR_EXPECT(history.m_episodes.m_open.contains("pair:r:17|r:19"));
					history.m_frame = 3;
					history.m_episodes.Begin(3);
					auto const excluded = std::array{CollisionPairKey(0, 1)};
					history.EndPairs(placements, excluded);
					history.m_episodes.Complete();
					PR_EXPECT(history.m_episodes.m_open.empty() && history.m_pairs.empty() && !history.m_episodes.m_candidate);
					history.Flush();
				}
				auto events_file = std::ifstream(directory / "events.csv");
				auto const events = std::string(std::istreambuf_iterator<char>(events_file), {});
				PR_EXPECT(events.find("recovered,1,pair:r:17|r:19") != std::string::npos);
				PR_EXPECT(events.find("pair_ineligible,3,pair:r:17|r:19") != std::string::npos);
				PR_EXPECT(events.find("recovered,2,") == std::string::npos);
				events_file.close();

				// Canonical pair identity must not change the native witness's argument order, points or material order.
				auto geometry_file = std::ifstream(directory / "geometry.csv");
				auto const geometry = std::string(std::istreambuf_iterator<char>(geometry_file), {});
				PR_EXPECT(geometry.find("contact_witness,0,pair:r:17|r:19,r:19,r:17,root,root,world,0,1,0,0,") != std::string::npos);
				PR_EXPECT(geometry.find(",2,7,11,2,1,2,3,1,4,5,6,1\n") != std::string::npos);
				PR_EXPECT(geometry.find(",-inf,2,7,11,2,1,2,3,1,4,5,6,1\n") != std::string::npos);
				geometry_file.close();

				// Only this test's four named files and now-empty directory are removed.
				for (auto const* name : {"primary.bin", "catalog.csv", "geometry.csv", "events.csv"})
					std::filesystem::remove(directory / name);

				std::filesystem::remove(directory);
			}
		};

		// A periodic orientation must not hide scalar hinge coordinates outside the workload's principal-angle domain.
		PRUnitTestClass(SceneHingeDomainTests)
		{
			// A full turn plus a legal principal angle remains outside the scalar domain.
			PRUnitTestMethod(UnwrappedHingesCannotAliasLegalPrincipalAngles, Quick)
			{
				// Include non-angular and quaternion joints so only the genuine scalar hinge contributes.
				auto builder = physics::ArticulationBuilder{};
				auto const inertia = physics::ArticulationLinkDesc{.m_inertia = physics::Inertia::Sphere(0.25f, 1)};
				auto const root = builder.AddFixedRoot(inertia);
				auto const hinge = builder.AddLink(root, physics::ArticulationJointDesc::Revolute(v4::XAxis()), inertia);
				auto slider = physics::ArticulationJointDesc::Prismatic();
				slider.m_initial_position[0] = 100;
				builder.AddLink(root, slider, inertia);
				builder.AddLink(root, physics::ArticulationJointDesc::Spherical(), inertia);
				auto trees = std::vector<physics::Articulation>{};
				trees.push_back(builder.Build());
				auto tree_ptrs = std::array{&trees[0]};
				auto constraints = physics::ConstraintSet{};
				auto desc = physics::ConstraintDesc{
					.m_frame_a = {.m_body = physics::BodyRef::Link(trees[0], root)},
					.m_frame_b = {.m_body = physics::BodyRef::Link(trees[0], hinge)},
				};
				desc.D6().m_angular[0].m_mode = physics::EConstraintAxisMode::Limited;
				desc.D6().m_angular[0].m_limits = {-0.5f, 0.5f};
				constraints.Add(desc);
				auto const remap = physics::BodyRemap{{}, tree_ptrs};
				for (auto const q : {0.2f, constants<float>::tau + 0.2f, -constants<float>::tau - 0.2f})
				{
					// The actual CPU D6 compiler sees a legal principal angle for all three scalar coordinates.
					trees[0].JointPosition(hinge, std::span{&q, 1});
					auto const rows = physics::CompileConstraints(constraints, remap).m_rows;
					PR_EXPECT(rows.size() == 1);
					PR_EXPECT(FEqlAbsolute(std::abs(rows[0].m_position), 0.2f, 1.0e-5f));
					auto const sample = MeasureHingeDomain(trees);
					PR_EXPECT(sample.m_count == 1 && sample.m_max_abs_q == std::abs(q));
					PR_EXPECT(sample.m_outside_principal_count == (q == 0.2f ? 0 : 1));
					log_stream() << "q=" << q << " principal=" << rows[0].m_position << " max_abs_hinge_q=" << sample.m_max_abs_q
						<< " outside_principal=" << sample.m_outside_principal_count << '\n';
				}

				// Both cuts of the principal chart are outside the strict domain.
				for (auto const q : {-constants<float>::tau_by_2, constants<float>::tau_by_2})
				{
					trees[0].JointPosition(hinge, std::span{&q, 1});
					PR_EXPECT(MeasureHingeDomain(trees).m_outside_principal_count == 1);
				}
			}
		};

		// Geometry screens must see every articulated pair category and honor the same explicit exclusions as native collision.
		PRUnitTestClass(SceneCommittedGeometryTests)
		{
			// Nonidentity and nested array frames must not move already root-space leaf transforms a second time.
			PRUnitTestMethod(ArrayLeavesKeepTheirRootSpaceTransforms, Quick)
			{
				// ShapeArray stores contiguous variable-size children, not owning pointers.
				struct SphereArray
				{
					collision::ShapeArray m_array{m4x4::Translation(10, 0, 0)};
					collision::ShapeSphere m_leaf{1.0f, m4x4::Translation(1, 0, 0)};
				};
				struct NestedArray
				{
					collision::ShapeArray m_array{m4x4::Translation(-7, 0, 0)};
					SphereArray m_child;
				};
				static_assert(offsetof(SphereArray, m_leaf) == sizeof(collision::ShapeArray));
				static_assert(offsetof(NestedArray, m_child) == sizeof(collision::ShapeArray));
				auto nested = NestedArray{};
				nested.m_child.m_array.Complete(1);
				nested.m_array.Complete(1);
				auto sphere = collision::ShapeSphere{1.0f};
				auto const other_to_world = m4x4::Translation(1.5f, 0, 0);
				for (auto shape : {&nested.m_child.m_array.m_base, &nested.m_array.m_base})
				{
					PR_EXPECT(IsIntersection(shape->m_s2r * shape->m_bbox, other_to_world * sphere.m_base.m_bbox));
					for (auto swap : {false, true})
					{
						auto contact = collision::Contact{};
						auto const hit = swap
							? CollideLeaves(sphere.m_base, other_to_world, *shape, m4x4::Identity(), contact)
							: CollideLeaves(*shape, m4x4::Identity(), sphere.m_base, other_to_world, contact);
						PR_EXPECT(hit);
						PR_EXPECT(FEqlAbsolute(contact.m_depth, 1.5f, 1.0e-5f));
					}
				}
			}

			// Floating root overlaps need no ground or ordinary body to be observable.
			PRUnitTestMethod(FloatingRootsRigidLinksAndExplicitExclusions, Extended)
			{
				auto sphere = collision::ShapeSphere{1.0f};
				auto scene = Scene(nullptr);
				for (int index = 0; index != 2; ++index)
				{
					auto builder = physics::ArticulationBuilder{};
					builder.AddFloatingRoot(physics::ArticulationLinkDesc{.m_inertia = physics::Inertia::Sphere(1, 1), .m_shape = &sphere.m_base}, m4x4::Translation(static_cast<float>(index), 0, 10));
					scene.m_articulation.push_back(builder.Build());
				}
				scene.m_articulation_actuation.resize(2);
				scene.RebuildStepInputs();
				auto sample = MeasurePenetration(scene);
				PR_EXPECT(sample.m_contact_count == 1);
				PR_EXPECT(FEqlAbsolute(sample.m_max_depth, 1.0f, 1.0e-5f));

				// Even an all-free enabled descriptor may suppress collision; disabling it must release the pair.
				auto const handle = scene.m_constraints.Add(physics::ConstraintDesc{
					.m_frame_a = {.m_body = physics::BodyRef::Link(scene.m_articulation[0], scene.m_articulation[0].LinkAt(0))},
					.m_frame_b = {.m_body = physics::BodyRef::Link(scene.m_articulation[1], scene.m_articulation[1].LinkAt(0))},
				});
				PR_EXPECT(MeasurePenetration(scene).m_contact_count == 0);
				scene.m_constraints.SetEnabled(handle, false);
				PR_EXPECT(MeasurePenetration(scene).m_contact_count == 1);

				// A non-ground rigid body can be the deepest overlap, and its layer mask must still apply to links.
				scene.m_body.emplace_back(nullptr, &sphere.m_base, m4x4::Translation(0.25f, 0, 10), physics::Inertia::Sphere(1, 1));
				scene.RebuildStepInputs();
				sample = MeasurePenetration(scene);
				PR_EXPECT(FEqlAbsolute(sample.m_max_depth, 1.75f, 1.0e-5f));
				PR_EXPECT(sample.m_body_a == 0 || sample.m_body_b == 0);
				scene.m_body[0].CollisionFilter(2U, 2U);
				PR_EXPECT(MeasurePenetration(scene).m_contact_count == 1);

				// Explicit ordinary-body ignore pairs remain effective without any persistent constraint.
				scene.ClearSimulationObjects();
				scene.m_body.emplace_back(nullptr, &sphere.m_base, m4x4::Translation(0, 0, 10), physics::Inertia::Sphere(1, 1));
				scene.m_body.emplace_back(nullptr, &sphere.m_base, m4x4::Translation(1, 0, 10), physics::Inertia::Sphere(1, 1));
				scene.m_collision_ignore_indices.emplace_back(0, 1);
				scene.RebuildStepInputs();
				PR_EXPECT(MeasurePenetration(scene).m_contact_count == 0);
				scene.m_collision_ignore_indices.clear();
				scene.RebuildStepInputs();
				PR_EXPECT(MeasurePenetration(scene).m_contact_count == 1);
			}

			// Parent overlap and enabled nonparent overlap are separate policies, not a whole-tree collision toggle.
			PRUnitTestMethod(ParentAndNonparentPoliciesRemainIndependent, Extended)
			{
				auto sphere = collision::ShapeSphere{1.0f};
				auto scene = Scene(nullptr);
				for (auto const& [self, parent, expected] : {std::tuple{false, false, 0}, std::tuple{true, false, 1}, std::tuple{false, true, 2}})
				{
					scene.ClearSimulationObjects();
					auto builder = physics::ArticulationBuilder{};
					auto const desc = physics::ArticulationLinkDesc{
						.m_inertia = physics::Inertia::Sphere(1, 1), .m_shape = &sphere.m_base, .m_collide_parent = parent, .m_collide_self = self,
					};
					auto const root = builder.AddFloatingRoot(desc, m4x4::Translation(0, 0, 10));
					auto const joint = physics::ArticulationJointDesc::Fixed(m4x4::Translation(0.25f, 0, 0));
					auto const child = builder.AddLink(root, joint, desc);
					builder.AddLink(child, joint, desc);
					scene.m_articulation.push_back(builder.Build());
					scene.m_articulation_actuation.resize(1);
					scene.RebuildStepInputs();
					PR_EXPECT(MeasurePenetration(scene).m_contact_count == expected);
				}
			}

			// The converted authored workloads must reach native typed storage without losing forces, limits, or damping.
			PRUnitTestMethod(ConvertedRagdollsRetainNativeLawsAndInitialSeparation, Extended)
			{
				auto scene = Scene(nullptr);
				for (auto const& [filename, torque_per_kg] : {std::pair{"articulation_ragdoll_dispersed.json", 3.0f}, std::pair{"articulation_ragdoll_pile.json", 1.5f}})
				{
					auto const source = scene_loader::LoadFromFile(std::filesystem::path("projects\\tests\\physics-sandbox\\scenes") / filename);
					scene.LoadScene(source);
					PR_EXPECT(scene.m_articulation.size() == 100);
					PR_EXPECT(scene.m_constraints.Count() == 1300);
					PR_EXPECT(scene.m_articulation_actuation.size() == 100);
					PR_EXPECT(scene.m_physics_substeps == 3);
					auto const& config = scene.m_physics.Config();
					PR_EXPECT(config.solver_iterations == 10 && config.push_out_iterations == 1);
					PR_EXPECT(config.constraint_position_beta == 0.2f && config.constraint_max_position_speed == 2.0f);
					auto const native = physics::PackGpuConstraints(scene.m_constraints, physics::BodyRemap{scene.m_body_ptrs, scene.m_articulation_ptrs});
					PR_EXPECT(native.m_descriptors.size() == 1300 && native.m_coupled_active_count == 1300);
					auto spherical_count = 0;
					auto limit_count = 0;
					auto damper_count = 0;
					for (int index = 0; index != isize(source.constraints); ++index)
					{
						auto const& expected = source.constraints[index];
						auto const& actual = native.m_descriptors[index];
						switch (expected.m_type)
						{
							case scene_loader::ConstraintDesc::EType::SphericalAngular:
							{
								++spherical_count;
								PR_EXPECT(actual.type == physics::GpuConstraintType_SphericalAngular);
								PR_EXPECT(All(actual.swing_limits == v4(expected.m_spherical.m_swing_x.m_beg, expected.m_spherical.m_swing_x.m_end, expected.m_spherical.m_swing_z.m_beg, expected.m_spherical.m_swing_z.m_end)));
								PR_EXPECT(All(actual.twist_limits == v2(expected.m_spherical.m_twist.m_beg, expected.m_spherical.m_twist.m_end)));
								PR_EXPECT(actual.angular_damping == expected.m_spherical.m_damping);
								break;
							}
							case scene_loader::ConstraintDesc::EType::D6:
							{
								PR_EXPECT(actual.type == physics::GpuConstraintType_D6);
								limit_count += actual.axes[3].mode == static_cast<int>(physics::EConstraintAxisMode::Limited);
								damper_count += actual.axes[3].mode == static_cast<int>(physics::EConstraintAxisMode::Driven);
								for (int axis = 0; axis != 6; ++axis)
								{
									auto const& wanted = axis < 3 ? expected.m_linear[axis] : expected.m_angular[axis - 3];
									auto const& packed = actual.axes[axis];
									PR_EXPECT(packed.mode == static_cast<int>(wanted.m_mode));
									PR_EXPECT(packed.lower_limit == wanted.m_limits.m_beg && packed.upper_limit == wanted.m_limits.m_end);
									PR_EXPECT(packed.damping == wanted.m_damping && packed.stiffness == wanted.m_stiffness);
									PR_EXPECT(packed.target_velocity == wanted.m_target_velocity && packed.max_force == wanted.m_max_force);
								}
								break;
							}
							default: { throw std::runtime_error("Unexpected constraint in converted ragdoll workload"); }
						}
					}
					PR_EXPECT(spherical_count == 500 && limit_count == 400 && damper_count == 400);

					// Check per-tree authored phase/period and the three-velocity spherical force domain.
					scene.ApplyArticulationActuation(0.125);
					auto links = 0;
					auto joint_velocities = 0;
					auto max_force_error = 0.0f;
					for (int index = 0; index != 100; ++index)
					{
						auto const& tree = scene.m_articulation[index];
						auto const& actuation = source.articulations[index].m_actuation;
						links += tree.LinkCount();
						PR_EXPECT(tree.LinkCount() == 10 && actuation.m_torque_per_kg == torque_per_kg);
						auto coordinate = 0;
						for (int link_index = 1; link_index != tree.LinkCount(); ++link_index)
						{
							auto const link = tree.LinkAt(link_index);
							for (auto force : tree.JointForce(link))
							{
								auto const phase = static_cast<float>(0.125 / actuation.m_period_s) * constants<float>::tau + actuation.m_phase + 2.39996323f * static_cast<float>(coordinate++);
								auto const expected = actuation.m_torque_per_kg * tree.LinkDescription(link).m_inertia.Mass() * std::sin(phase);
								PR_EXPECT(std::isfinite(force) && std::isfinite(expected));
								max_force_error = std::max(max_force_error, std::abs(force - expected));
							}
						}
						joint_velocities += coordinate;
					}
					PR_EXPECT(links == 1000 && joint_velocities == 1900 && max_force_error <= 1.0e-5f);
					auto const geometry = MeasurePenetration(scene);
					PR_EXPECT(geometry.m_contact_count == 0 && geometry.m_max_depth == 0.0f);
					log_stream() << filename << " trees=100 links=" << links << " spherical=" << spherical_count << " hinge_limits=" << limit_count << " hinge_dampers=" << damper_count
						<< " joint_velocities=" << joint_velocities << " torque_per_kg=" << torque_per_kg << " force_error=" << max_force_error << " initial_contacts=" << geometry.m_contact_count
						<< " stepping=3/10/1 beta=" << config.constraint_position_beta << " max_position_speed=" << config.constraint_max_position_speed
						<< " has_debug_layer=" << compute::HasDebugLayer(scene.m_physics.Device()) << '\n';
				}
			}
		};
#endif

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

		// Report the actual convex-hull complexity and volume range produced by procedural polytope inputs.
		void PrintPolytopeProfile(std::ofstream& log, Scene const& scene)
		{
			auto count = 0;
			auto min_vert_count = std::numeric_limits<int>::max();
			auto max_vert_count = 0;
			auto min_face_count = std::numeric_limits<int>::max();
			auto max_face_count = 0;
			auto min_volume = std::numeric_limits<float>::max();
			auto max_volume = 0.0f;
			auto total_vert_count = 0;
			auto total_face_count = 0;
			auto total_volume = 0.0;
			for (auto const& body : scene.m_body)
			{
				if (!body.HasShape() || body.Shape().m_type != collision::EShape::Polytope)
					continue;

				auto const& poly = collision::shape_cast<collision::ShapePolytope>(body.Shape());
				auto const volume = collision::CalcVolume(poly);
				++count;
				min_vert_count = std::min(min_vert_count, poly.m_vert_count);
				max_vert_count = std::max(max_vert_count, poly.m_vert_count);
				min_face_count = std::min(min_face_count, poly.m_face_count);
				max_face_count = std::max(max_face_count, poly.m_face_count);
				min_volume = std::min(min_volume, volume);
				max_volume = std::max(max_volume, volume);
				total_vert_count += poly.m_vert_count;
				total_face_count += poly.m_face_count;
				total_volume += volume;
			}
			if (count == 0)
				return;

			Emit(log, std::format(
				"polytope_profile,count={},verts_min={},verts_avg={:.2f},verts_max={},faces_min={},faces_avg={:.2f},faces_max={},volume_min={:.5f},volume_avg={:.5f},volume_max={:.5f}\n",
				count,
				min_vert_count,
				static_cast<double>(total_vert_count) / count,
				max_vert_count,
				min_face_count,
				static_cast<double>(total_face_count) / count,
				max_face_count,
				min_volume,
				total_volume / count,
				max_volume));
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
				sample.m_avg_horizontal_speed += Length(v2{vel.lin.x, vel.lin.y});
				sample.m_avg_vertical_speed += std::abs(vel.lin.z);
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
				sample.m_avg_horizontal_speed /= sample.m_dynamic_count;
				sample.m_avg_vertical_speed /= sample.m_dynamic_count;
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

		// Fingerprint rigid transforms, world velocities and sleep state in scene order, excluding pointers and structure padding.
		uint64_t RigidStateHash(Scene const& scene)
		{
			// Keep diagnostics outside Scene::Step so their cost is not reported as physics computation.
			auto hash = pr::hash::FNV_offset_basis64;
			for (auto const& body : scene.m_body)
			{
				// Serialize only physical scalar values; this is not a hash of articulation state or hidden solver caches.
				auto const& o2w = body.O2W();
				auto const velocity = body.VelocityWS();
				auto const state = std::array{
					o2w.x.x, o2w.x.y, o2w.x.z, o2w.y.x, o2w.y.y, o2w.y.z, o2w.z.x, o2w.z.y, o2w.z.z,
					o2w.pos.x, o2w.pos.y, o2w.pos.z,
					velocity.lin.x, velocity.lin.y, velocity.lin.z, velocity.ang.x, velocity.ang.y, velocity.ang.z,
					body.Sleeping() ? 1.0f : 0.0f,
				};
				auto const* bytes = reinterpret_cast<char const*>(state.data());
				hash = pr::hash::Hash64CT(bytes, bytes + sizeof(state), hash);
			}
			return hash;
		}

		// Retain committed primary articulation state without warming derived link-kinematics caches.
		void PrintArticulationProfileState(std::ofstream& log, int step, Scene const& scene)
		{
			auto output = std::ostringstream{};
			output << std::setprecision(std::numeric_limits<float>::max_digits10);
			for (int tree = 0; tree != isize(scene.m_articulation); ++tree)
			{
				auto const& articulation = scene.m_articulation[tree];
				auto const& pose = articulation.RootToWorld();
				auto const velocity = articulation.RootVelocity();
				output << "articulation_root," << step << ',' << tree;
				for (auto value : {pose.x.x, pose.x.y, pose.x.z, pose.y.x, pose.y.y, pose.y.z, pose.z.x, pose.z.y, pose.z.z,
					pose.pos.x, pose.pos.y, pose.pos.z, velocity.ang.x, velocity.ang.y, velocity.ang.z, velocity.lin.x, velocity.lin.y, velocity.lin.z})
					output << ',' << value;

				output << '\n';
				for (int link_index = 1; link_index != articulation.LinkCount(); ++link_index)
				{
					auto const link = articulation.LinkAt(link_index);
					auto const q = articulation.JointPosition(link);
					auto const v = articulation.JointVelocity(link);
					output << "articulation_joint," << step << ',' << tree << ',' << link_index << ',' << q.size() << ',' << v.size();
					for (auto value : q)
						output << ',' << value;

					for (auto value : v)
						output << ',' << value;

					output << '\n';
				}
			}
			Emit(log, output.str());
		}

		// Report phase averages and the final rigid state of this reporting interval.
		void PrintEngineProfile(std::ofstream& log, int step, double time_s, EngineProfileAccumulator const& profile, Scene const& scene)
		{
			auto count = std::max(profile.m_sample_count, 1);
			auto const average = [count](auto value)
			{
				return static_cast<double>(value) / count;
			};

			// Keep the row assembled in field order so benchmark readers can map it directly from the emitted header.
			auto row = std::ostringstream{};
			row << std::fixed << std::setprecision(4);
			row
				<< "profile," << step
				<< ',' << time_s
				<< ',' << profile.m_sample_count
				<< ',' << std::setprecision(2) << average(profile.m_contact_count)
				<< std::setprecision(4)
				<< ',' << average(profile.m_scene_step_ms)
				<< ',' << average(profile.m_physics_ms)
				<< ',' << average(profile.m_engine.m_new_frame_ms)
				<< ',' << average(profile.m_engine.m_pack_ms)
				<< ',' << average(profile.m_engine.m_constraint_pack_ms)
				<< ',' << average(profile.m_engine.m_articulation_pack_ms)
				<< ',' << average(profile.m_engine.m_upload_ms)
				<< ',' << average(profile.m_engine.m_constraint_upload_ms)
				<< ',' << average(profile.m_engine.m_articulation_upload_ms)
				<< ',' << average(profile.m_engine.m_external_forces_ms)
				<< ',' << average(profile.m_engine.m_integrate_ms)
				<< ',' << average(profile.m_engine.m_articulation_integrate_ms)
				<< ',' << average(profile.m_engine.m_sleepwake_ms)
				<< ',' << average(profile.m_engine.m_broadphase_ms)
				<< ',' << average(profile.m_engine.m_collide_ms)
				<< ',' << average(profile.m_engine.m_resolve_ms)
				<< ',' << average(profile.m_engine.m_selective_ms)
				<< ',' << average(profile.m_engine.m_sleepupdate_ms)
				<< ',' << average(profile.m_engine.m_readback_ms)
				<< ',' << average(profile.m_engine.m_gpu_run_ms)
				<< ',' << average(profile.m_engine.m_unpack_ms)
				<< ',' << average(profile.m_engine.m_gpu_prepare_ms)
				<< ',' << average(profile.m_engine.m_gpu_execute_ms)
				<< ',' << average(profile.m_engine.m_gpu_wait_ms)
				<< ',' << average(profile.m_engine.m_gpu_reset_ms)
				<< ',' << average(profile.m_engine.m_readback_access_ms)
				<< ',' << average(profile.m_engine.m_body_readback_copy_ms)
				<< ',' << average(profile.m_engine.m_contact_readback_copy_ms)
				<< ',' << average(profile.m_engine.m_collision_events_ms)
				<< ',' << average(profile.m_engine.m_sleep_island_unpack_ms)
				<< ',' << average(profile.m_engine.m_body_unpack_ms)
				<< ',' << average(profile.m_engine.m_articulation_unpack_ms)
				<< ',' << average(profile.m_engine.m_substep_count)
				<< ',' << average(profile.m_engine.m_submission_count)
				<< ',' << average(profile.m_engine.m_wait_count)
				<< ',' << average(profile.m_engine.m_readback_copy_count)
				<< ',' << average(profile.m_engine.m_terrain_gpu_ms)
				<< ',' << average(profile.m_engine.m_water_gpu_ms)
				<< ',' << RigidStateHash(scene)
				<< '\n';
			Emit(log, row.str());
		}
		void PrintSleepScan(std::ofstream& log, int step, double time_s, Scene const& scene, bool non_spheres_only)
		{
			auto sample = MeasureSleep(scene, non_spheres_only);
			auto const& config = scene.m_physics.Config();
			Emit(log, std::format(
				"sleep step={:5d} t={:8.4f} sample={} dynamic={} sleeping={} low={} never={} lin(avg/horiz/vert/p50/p90/p95/p99/max)=({:9.5f},{:9.5f},{:9.5f},{:9.5f},{:9.5f},{:9.5f},{:9.5f},{:4d}:{:9.5f}/{:.5f}) ang(avg/p50/p90/p95/p99/max)=({:9.5f},{:9.5f},{:9.5f},{:9.5f},{:9.5f},{:4d}:{:9.5f}/{:.5f})\n",
				step,
				time_s,
				non_spheres_only ? "non-spheres" : "all",
				sample.m_dynamic_count,
				sample.m_sleeping_count,
				sample.m_low_velocity_count,
				sample.m_never_sleep_count,
				sample.m_avg_lin_speed,
				sample.m_avg_horizontal_speed,
				sample.m_avg_vertical_speed,
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

		// Report how far a traced body's lowest point sits above the water surface under its centre. A positive gap means the
		// body is airborne. The surface height is taken at the centre, so for large tilted bodies the gap is approximate.
		void PrintWaterClearance(std::ofstream& log, Scene const& scene, int body_index)
		{
			// Only water scenes with a valid traced body have a clearance.
			if (!scene.m_water || body_index < 0 || body_index >= std::ssize(scene.m_body))
				return;

			auto const& body = scene.m_body[body_index];
			if (!body.HasShape())
				return;

			// The support vertex in the downward direction is the shape's true lowest point, unlike a bounding box,
			// which is loose for rotated shapes.
			auto const s2w = body.O2W() * body.Shape().m_s2r;
			auto sup_id = 0;
			auto const lowest_ws = s2w * collision::SupportVertex(body.Shape(), InvertAffine(s2w) * v4{0, 0, -1, 0}, 0, sup_id);
			auto const pos = body.O2W().pos;
			auto const surface = scene.m_water->forces.m_field.Height(physics::terrain::v2d{pos.x, pos.y}, scene.m_clock);
			auto const clearance = lowest_ws.z - surface;
			Emit(log, std::format("  water surface={:8.4f} clearance={:8.4f} {}\n", surface, clearance, clearance > 0 ? "AIRBORNE" : "wet"));
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
			auto const hit = CollideLeaves(body.Shape(), body.O2W(), ground.Shape(), ground.O2W(), contact);
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

	// CPU FP64 terrain measurements on a 0.05 m reference grid, evaluated outside the timed physics step.
	struct TerrainMetric
	{
		std::vector<v4> m_initial;
		std::unordered_map<collision::Shape const*, physics::surface::Plan> m_plans;

		// Capture initial body positions for later displacement and terrain-height comparisons.
		explicit TerrainMetric(Scene const& scene)
		{
			if (!scene.m_terrain_surface)
				return;

			// Retain scene order so each report compares a body with its original position.
			for (auto const& body : scene.m_body)
				m_initial.push_back(body.O2W().pos);
		}

		// Report finite dynamic-body state and sampled penetration; the scene must retain body identities and ordering.
		void Print(std::ofstream& log, int step, Scene const& scene)
		{
			if (!scene.m_terrain_surface)
				return;

			// Reports require the same positional baseline as construction.
			if (m_initial.size() != scene.m_body.size())
				throw std::runtime_error("Terrain diagnostic lost or replaced a body");

			// Accumulate dynamic-body motion and energy independently of the physics timing interval.
			auto max_depth = 0.0, energy = 0.0, displacement = 0.0, height_drop = 0.0;
			auto min_z = std::numeric_limits<float>::max(), max_z = -min_z;
			auto downhill = 0, sleeping = 0, dynamic = 0;
			auto const& source = *scene.m_terrain_surface;
			for (size_t i = 0; i != scene.m_body.size(); ++i)
			{
				auto const& body = scene.m_body[i];
				if (body.InvMass() == 0)
					continue;

				// Reject invalid state rather than allowing it to contaminate aggregate measurements.
				if (!IsFinite(body.O2W().pos) || !std::isfinite(body.KineticEnergy()))
					throw std::runtime_error("Terrain diagnostic encountered nonfinite body state");

				// Compare motion and terrain elevation against this body's captured starting position.
				++dynamic;
				sleeping += body.Sleeping() ? 1 : 0;
				auto delta = body.O2W().pos - m_initial[i];
				displacement += std::hypot(double(delta.x), double(delta.y));
				height_drop -= delta.z;
				min_z = std::min(min_z, body.O2W().pos.z);
				max_z = std::max(max_z, body.O2W().pos.z);
				energy += body.KineticEnergy() - body.Mass() * Dot3(scene.m_gravity, body.CentreOfMassPositionWS());
				auto height_now = source.Sample({body.O2W().pos.x, body.O2W().pos.y}).m_height;
				auto height_initial = source.Sample({m_initial[i].x, m_initial[i].y}).m_height;
				downhill += height_now < height_initial - 0.02 ? 1 : 0;

				// Measure each physical leaf against the canonical field using its own shape-to-root transform.
				auto sample_shape = [&](auto&& self, collision::Shape const& shape) -> void
				{
					switch (shape.m_type)
					{
						case collision::EShape::NoShape: { return; }
						case collision::EShape::Array:
						{
							auto const& array = collision::shape_cast<collision::ShapeArray>(shape);
							for (auto child = array.begin(); child != array.end(); child = collision::next(child))
								self(self, *child);
							return;
						}
						default: { break; }
					}

					// Reuse the fixed reference grid without rebuilding its geometry for every report.
					auto found = m_plans.find(&shape);
					if (found == m_plans.end())
						found = m_plans.emplace(&shape, physics::surface::BuildPlan(shape, 0.05f)).first;

					// Convert vertical gaps to local tangent-plane depths, matching the collision depth convention.
					auto const& plan = found->second;
					auto const s2w = body.O2W() * shape.m_s2r;
					for (uint32_t ordinal = 0; ordinal != plan.m_count; ++ordinal)
					{
						auto position = s2w * physics::surface::EmitSurfaceSample(plan, ordinal).m_pos_local;
						auto field = source.Sample({position.x, position.y});
						auto depth = (field.m_height - position.z) / std::sqrt(1 + LengthSq(field.m_gradient_xy));
						max_depth = std::max(max_depth, depth);
					}
				};
				sample_shape(sample_shape, body.Shape());
			}

			// Flush each report boundary so a failed later frame does not discard the last measured state.
			log << std::format("terrain_metric,step={},bodies={},sleeping={},downhill={},mean_horizontal_m={:.6f},mean_drop_m={:.6f},min_z={:.6f},max_z={:.6f},max_normal_depth_m={:.6f},mechanical_energy_j={:.6f}\n",
				step, dynamic, sleeping, downhill, displacement / std::max(1, dynamic), height_drop / std::max(1, dynamic), min_z, max_z, max_depth, energy);
			log.flush();
		}
	};

	SceneDiagnosticResult RunSceneDiagnostic(SceneDiagnosticOptions const& options)
	{
		// Install the optional GPU capture layer before scene loading can create any D3D12 objects.
		if (!options.m_pix_capture_path.empty() || options.m_pix_capture_step != 0)
		{
			if (options.m_pix_capture_path.empty() || options.m_pix_capture_step < 1 || options.m_pix_capture_step > options.m_steps)
				throw std::invalid_argument("PIX capture requires -pix_capture <path> and -pix_capture_step within [1, steps]");

			if (std::filesystem::exists(options.m_pix_capture_path))
				throw std::invalid_argument("PIX capture output already exists");

			#if PR_PIX_ENABLED
			compute::pix::LoadLatestWinPixGpuCapturer();
			#else
			throw std::runtime_error("This build does not support PIX capture");
			#endif
		}

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

		// Geometry histories require every ordinary committed frame, not a branch that skips its measurements.
		if (!options.m_geometry_history.empty() && (options.m_engine_profile || metric_count != 0 || options.m_trace_body != -1 || options.m_scan_bodies || !options.m_pix_capture_path.empty()))
			throw std::runtime_error("Geometry history is separate from profile, trace, other metrics, and GPU captures");

		if (!options.m_geometry_history.empty() && (options.m_steps < 0 || options.m_steps == std::numeric_limits<int>::max() || !(options.m_dt > 0) || !std::isfinite(options.m_dt)))
			throw std::runtime_error("Geometry history requires a finite positive interval and a representable nonnegative frame count");

		Emit(log, std::format("Scene diagnostic log: {}\n", log_path.string()));
		Emit(log, std::format("Scene diagnostic scene: {}\n", options.m_scene_filepath.string()));
		Emit(log, std::format("steps={} dt={:.8f} report_interval={}\n", options.m_steps, options.m_dt, options.m_report_interval));
		if (options.m_engine_profile)
			Emit(log, "profile,step,time_s,samples,contacts,scene_step_ms,physics_ms,new_frame_ms,pack_ms,constraint_pack_ms,articulation_pack_ms,upload_ms,constraint_upload_ms,articulation_upload_ms,external_forces_ms,integrate_ms,articulation_integrate_ms,sleepwake_ms,broadphase_ms,collide_ms,resolve_ms,selective_ms,sleepupdate_ms,readback_ms,gpu_run_ms,unpack_ms,gpu_prepare_ms,gpu_execute_ms,gpu_wait_ms,gpu_reset_ms,readback_access_ms,body_readback_copy_ms,contact_readback_copy_ms,collision_events_ms,sleep_island_unpack_ms,body_unpack_ms,articulation_unpack_ms,substeps,submissions,waits,readback_copies,terrain_gpu_ms,water_gpu_ms,rigid_state_hash\n");
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
		if (
			options.m_buoyancy_linear_drag_time_constant_s ||
			options.m_buoyancy_angular_drag_time_constant_s ||
			options.m_buoyancy_tangential_drag_coefficient ||
			options.m_buoyancy_wave_resistance)
		{
			if (scene.m_physics.Water() == nullptr)
				throw std::runtime_error("Scene diagnostic buoyancy overrides require a water scene");

			auto config = *scene.m_physics.Water();
			if (options.m_buoyancy_linear_drag_time_constant_s)
				config.m_linear_drag_time_constant_s = *options.m_buoyancy_linear_drag_time_constant_s;
			if (options.m_buoyancy_angular_drag_time_constant_s)
				config.m_angular_drag_time_constant_s = *options.m_buoyancy_angular_drag_time_constant_s;
			if (options.m_buoyancy_tangential_drag_coefficient)
				config.m_tangential_drag_coefficient = *options.m_buoyancy_tangential_drag_coefficient;
			if (options.m_buoyancy_wave_resistance)
				config.m_wave_resistance = *options.m_buoyancy_wave_resistance;
			scene.m_physics.Water(config);
		}
		if (options.m_air_density)
		{
			// Air forces are currently applied only by the engine's water pass, to the dry parts of floating bodies.
			if (scene.m_physics.Water() == nullptr)
				throw std::runtime_error("Scene diagnostic -air_density requires a water scene");

			auto config = *scene.m_physics.Water();
			config.m_air.m_density = *options.m_air_density;
			scene.m_physics.Water(config);
		}
		// The recorder owns its complete scan loop without adding observer work to timed or unrelated diagnostics.
		if (!options.m_geometry_history.empty())
			return RunGeometryHistory(scene, options);

		Emit(log, std::format(
			"load_profile,total_ms={:.4f},prepare_ms={:.4f},bbox_ms={:.4f},shapes_ms={:.4f},bodies_ms={:.4f},buoyancy_ms={:.4f},bodies={},articulations={},constraints={},shapes={}\n",
			scene.m_last_load_profile.m_total_ms,
			scene.m_last_load_profile.m_prepare_ms,
			scene.m_last_load_profile.m_bbox_ms,
			scene.m_last_load_profile.m_shapes_ms,
			scene.m_last_load_profile.m_bodies_ms,
			scene.m_last_load_profile.m_buoyancy_ms,
			scene.m_body.size(),
			scene.m_articulation.size(),
			scene.m_constraints.Count(),
			scene.m_last_load_profile.m_shape_count));
		if (auto const* config_ptr = scene.m_physics.Water())
		{
			auto const& config = *config_ptr;
			Emit(log, std::format(
				"buoyancy_config,linear_drag_time_constant_s={:.5f},angular_drag_time_constant_s={:.5f},tangential_drag_coefficient={:.5f},wave_resistance={:.5f},air_density={:.5f}\n",
				config.m_linear_drag_time_constant_s,
				config.m_angular_drag_time_constant_s,
				config.m_tangential_drag_coefficient,
				config.m_wave_resistance,
				config.m_air.m_density));
		}
		PrintPolytopeProfile(log, scene);
		auto const& engine_config = scene.m_physics.Config();
		Emit(log, std::format(
			"engine_settings,substeps={},velocity_sweeps={},position_sweeps={},beta={},max_position_speed={},has_debug_layer={},physics_diagnostics={},physics_profile={}\n",
			scene.m_physics_substeps, engine_config.solver_iterations, engine_config.push_out_iterations, engine_config.constraint_position_beta,
			engine_config.constraint_max_position_speed, compute::HasDebugLayer(scene.m_physics.Device()), PR_PHYSICS_DIAGNOSTICS, PR_PHYSICS_PROFILE));
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

		// The existing event reports the first rejection per owner in a frame, not every rejected trial or sweep.
		auto step_rejections = std::vector<physics::ConstraintRejectionEvent>{};
		if (options.m_engine_profile)
		{
			scene.m_physics.ConstraintRejections += [&](auto&, std::span<physics::ConstraintRejectionEvent const> events)
			{
				step_rejections.assign(events.begin(), events.end());
			};
			PrintArticulationProfileState(log, 0, scene);
		}

		// Capture terrain diagnostics before stepping and report them outside the timed physics interval.
		auto terrain_metric = TerrainMetric(scene);
		terrain_metric.Print(log, 0, scene);
		for (int step = 0; step != options.m_steps; ++step)
		{
			step_rejections.clear();
			auto before = BodyTraceState{};
			if (options.m_trace_body != -1)
				before = CaptureBodyState(scene.m_body[options.m_trace_body]);
			if (options.m_trace_body != -1 || options.m_scan_bodies)
				trace_contacts.resize(0);

			{
				// Scene::Step waits for its submitted GPU work, so capture closes after this tick's completion fence.
				auto capture = compute::pix::CaptureScope(options.m_pix_capture_path, step + 1 == options.m_pix_capture_step);
				scene.Step(options.m_dt);
			}
			if ((step + 1) % std::max(options.m_report_interval, 1) == 0 || step + 1 == options.m_steps)
				terrain_metric.Print(log, step + 1, scene);

			// Accumulate the scene's recorded physics duration rather than the additional terrain-reporting cost.
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
				// These counts are for this completed frame, separately from the interval-averaged host timing row.
				auto const& features = scene.m_physics.LastFeatureStats();
				auto const& work = features.m_coupled.m_velocity_work;
				Emit(log, std::format("coupled_work,step={},has_snapshots={},sweeps={},recorded_velocity_dispatches={},owner_sweeps={},started_trials={},accepted_owner_sweeps={},zero_attempt_accepted_owner_sweeps={},admission_rejected_owner_sweeps={},solve_rejected_owner_sweeps={},numerical_failure_owner_sweeps={},recorded_coupled_dispatches={},distinct_rejected_owners={}\n",
					step + 1, work.m_has_snapshots, work.m_sweeps, work.m_recorded_dispatches, work.m_owner_sweeps, work.m_started_trials,
					work.m_accepted_owner_sweeps, work.m_zero_attempt_accepted_owner_sweeps, work.m_admission_rejected_owner_sweeps, work.m_solve_rejected_owner_sweeps,
					work.m_numerical_failure_owner_sweeps, features.m_coupled.m_resources.m_dispatch_count, step_rejections.size()));
				for (auto const& rejection : step_rejections)
				{
					Emit(log, std::format("first_owner_rejection,step={},owner={},index={},generation={},phase={},outcome={},substep={},attempts={},flags={},relaxation={:.9g},merit_change={:.9g}\n",
						step + 1, static_cast<int>(rejection.m_owner), rejection.m_index, rejection.m_generation, static_cast<int>(rejection.m_phase),
						static_cast<int>(rejection.m_outcome), rejection.m_substep_index, rejection.m_iteration_count,
						rejection.m_failure_flags, rejection.m_relaxation, rejection.m_merit_change));
				}
				PrintArticulationProfileState(log, step + 1, scene);

				profile.Add(scene.m_last_step_profile, scene.m_physics.LastCollisionStats().LastContactCount());
				auto report_interval = std::max(options.m_report_interval, 1);
				if ((step + 1) % report_interval == 0 || step + 1 == options.m_steps)
				{
					PrintEngineProfile(log, step + 1, scene.m_clock, profile, scene);
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
					PrintWaterClearance(log, scene, options.m_trace_body);
				}

				continue;
			}

			{
				auto sample = MeasurePenetration(scene);
				auto const hinges = MeasureHingeDomain(scene.m_articulation);
				result.m_hinge_count = hinges.m_count;
				result.m_max_abs_hinge_q = std::max(result.m_max_abs_hinge_q, hinges.m_max_abs_q);
				result.m_hinge_outside_principal_steps += hinges.m_outside_principal_count != 0 ? 1 : 0;
				if (sample.m_max_depth > result.m_max_depth)
				{
					result.m_max_depth = sample.m_max_depth;
					result.m_max_depth_step = step + 1;
					result.m_body_a = sample.m_body_a;
					result.m_body_b = sample.m_body_b;
					result.m_articulation = sample.m_articulation;
					result.m_link = sample.m_link;
				}

				auto report_interval = std::max(options.m_report_interval, 1);
				if ((step + 1) % report_interval == 0 || step + 1 == options.m_steps)
				{
					PrintSample(log, step + 1, scene.m_clock, sample);
					if (hinges.m_count != 0)
					{
						// Report the prerequisite separately from geometric error; it is not a restriction on valid engine motion.
						Emit(log, std::format("hinge_domain,step={},hinges={},max_abs_hinge_q={:.9f},outside_principal={},principal_hinge_domain={}\n",
							step + 1, hinges.m_count, hinges.m_max_abs_q, hinges.m_outside_principal_count, hinges.m_outside_principal_count == 0));
					}
				}
			}
		}

		if (dzhanibekov_metric.Enabled())
			ValidateDzhanibekovMetric(dzhanibekov_metric);

		if (options.m_trace_body == -1 && !options.m_engine_profile && !options.m_scan_bodies && !options.m_cradle_metric && !options.m_dzhanibekov_metric)
		{
			Emit(log, std::format("worst: step={} max_depth={:.6f} pair=({},{}) articulation={} link={}\n",
				result.m_max_depth_step,
				result.m_max_depth,
				result.m_body_a,
				result.m_body_b,
				result.m_articulation,
				result.m_link));
			if (result.m_hinge_count != 0)
			{
				// Retain excursions from unreported steps so returning to the principal interval cannot hide them.
				Emit(log, std::format("worst_hinges,max_abs_hinge_q={:.9f},outside_principal_steps={},principal_hinge_domain={}\n",
					result.m_max_abs_hinge_q, result.m_hinge_outside_principal_steps, result.m_hinge_outside_principal_steps == 0));
			}
		}

		return result;
	}
}
