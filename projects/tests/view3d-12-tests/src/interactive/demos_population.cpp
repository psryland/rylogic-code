//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Demos of GPU-culled instanced population rendering.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <stdexcept>
#include <vector>
#include "pr/gui/wingui.h"
#include "pr/view3d-12/view3d-dll.h"
#include "demo.h"
#include "controls_panel.h"

namespace view3d_test
{
	namespace
	{
		namespace api = pr::view3d;

		// A large field of instances of one prototype, culled and LOD-selected on the GPU. Each LOD has its own colour, so LOD selection and the
		// dithered transitions between LODs are visible. The field is generated from a fixed seed, so every run shows the same scene.
		struct PopulationFieldDemo :IDemo
		{
			static constexpr int Side = 256;          // Instances per side of the field
			static constexpr int Tile = 8;            // Instances per side of one block
			static constexpr float Spacing = 1.5f;    // Distance between neighbouring instances
			static constexpr float DrawDistance = 250.0f;
			static constexpr int BlockSlots = Tile * Tile;
			static constexpr int BlockCount = (Side / Tile) * (Side / Tile);

			DemoContext m_ctx;
			demo::SceneObjects m_scene;
			std::vector<api::Object> m_lod_objects; // Supply the LOD models. They are not added to the window.
			api::PopulationPrototypePtr m_prototype;
			api::PopulationPtr m_population;
			std::vector<api::BBox> m_block_bounds;
			pr::gui::Label* m_stats_label;
			double m_stats_timer;
			float m_density;
			bool m_shown;

			explicit PopulationFieldDemo(DemoContext const& ctx)
				: m_ctx(ctx)
				, m_scene(ctx.m_window)
				, m_lod_objects()
				, m_prototype()
				, m_population()
				, m_block_bounds()
				, m_stats_label()
				, m_stats_timer()
				, m_density(1.0f)
				, m_shown(true)
			{
				// Three LODs, most detailed first. Green spheres fade into yellow boxes, which switch to small red boxes in the distance.
				for (auto script : {
					"*Sphere lod0 FF40C040 { *Data {0.5} *o2w {*pos {0 0 0.5}} }",
					"*Box lod1 FFE0E040 { *Data {0.8 0.8 1} *o2w {*pos {0 0 0.5}} }",
					"*Box lod2 FFE04040 { *Data {0.5 0.5 1} *o2w {*pos {0 0 0.5}} }" })
				{
					// View3D reports script errors through the error callback and returns null
					auto obj = View3D_ObjectCreateLdrA(script, FALSE, nullptr, nullptr);
					if (obj == nullptr)
						throw std::runtime_error(std::string("Failed to create population LOD: ") + script);

					m_lod_objects.push_back(obj);
				}
				api::PopulationLod lods[] =
				{
					{ .m_object = m_lod_objects[0], .m_min_size = 24.0f, .m_transition = 0.4f },
					{ .m_object = m_lod_objects[1], .m_min_size = 6.0f, .m_transition = 0.4f },
					{ .m_object = m_lod_objects[2], .m_min_size = 1.5f, .m_transition = 0.0f },
				};
				m_prototype.reset(View3D_PopulationPrototypeCreate(lods, 3, DrawDistance));
				m_population.reset(View3D_PopulationCreate(m_prototype.get(), Side * Side, BlockCount, 0));
				if (!m_prototype || !m_population)
					throw std::runtime_error("Failed to create the population");

				// Fill each 8x8 tile as one block, with jittered positions, random heading, and random scale from a fixed-seed generator
				auto rng = uint32_t{ 20260601 };
				auto random = [&](float lo, float hi)
				{
					rng = rng * 1664525u + 1013904223u;
					return lo + (hi - lo) * static_cast<float>(rng >> 8) / 16777216.0f;
				};
				auto const origin = -0.5f * Side * Spacing;
				auto instances = std::vector<api::PopulationInstance>(BlockSlots);
				for (int b = 0; b != BlockCount; ++b)
				{
					// Instances of one block are contiguous, so the block's live count can thin the tile without moving instances
					auto bx = (b % (Side / Tile)) * Tile;
					auto by = (b / (Side / Tile)) * Tile;
					auto max_scale = 0.0f;
					for (int i = 0; i != BlockSlots; ++i)
					{
						// Random placement within the instance's grid cell
						auto x = origin + (bx + i % Tile + random(0.1f, 0.9f)) * Spacing;
						auto y = origin + (by + i / Tile + random(0.1f, 0.9f)) * Spacing;
						auto angle = random(0, 6.2831853f);
						auto scale = random(0.6f, 1.4f);
						auto shade = static_cast<uint32_t>(random(160, 255));
						auto c = scale * std::cos(angle);
						auto s = scale * std::sin(angle);
						max_scale = std::max(max_scale, scale);

						// Rotation about z with uniform scale. The tint varies the brightness so neighbouring instances are distinguishable.
						auto& inst = instances[i];
						inst = api::PopulationInstance{};
						inst.m_o2w[0] = api::Vec4{ c, -s, 0, x };
						inst.m_o2w[1] = api::Vec4{ s, c, 0, y };
						inst.m_o2w[2] = api::Vec4{ 0, 0, scale, 0 };
						inst.m_tint = 0xFF000000 | (shade << 16) | (shade << 8) | shade;
						inst.m_flags = api::EPopulationInstanceFlags::Enabled;
					}
					View3D_PopulationInstancesUpdate(m_population.get(), b * BlockSlots, instances.data(), BlockSlots);

					// The block bound contains the tile plus the largest scaled prototype
					auto half = 0.5f * Tile * Spacing;
					m_block_bounds.push_back(api::BBox{
						.centre = { origin + bx * Spacing + half, origin + by * Spacing + half, 0.5f * max_scale, 1 },
						.radius = { half + max_scale, half + max_scale, 0.5f * max_scale + 0.5f, 0 },
					});
				}
				ApplyDensity();
				View3D_WindowAddPopulation(ctx.m_window, m_population.get());

				// A ground plane and a camera looking across the field
				m_scene.Add("*Plane ground FF406030 { *Data {400 400} *AxisId {+3} }");
				View3D_CameraClipPlanesSet(ctx.m_window, 0.1f, 1000.0f, api::EClipPlanes::Both);
				View3D_CameraPositionSet(ctx.m_window, { 0, -200, 12, 1 }, { 0, -150, 0, 1 }, { 0, 0, 1, 0 });

				// Controls
				auto& ui = ctx.m_controls;
				ui.AddLabel(std::format(L"{} instances in {} blocks. Green, yellow, and red are LODs 0, 1, and 2.", Side * Side, BlockCount));
				ui.AddSlider(L"Density", 0, 1, m_density, [this](float v)
				{
					m_density = v;
					ApplyDensity();
				});
				ui.AddCheckBox(L"Show population", m_shown, [this](bool v)
				{
					m_shown = v;
					if (v)
						View3D_WindowAddPopulation(m_ctx.m_window, m_population.get());
					else
						View3D_WindowRemovePopulation(m_ctx.m_window, m_population.get());
				});
				m_stats_label = ui.AddLabel(L"\n\n\n\n\n\n\n");
			}
			~PopulationFieldDemo()
			{
				// Stop drawing the population before its LOD objects are deleted
				View3D_WindowRemovePopulation(m_ctx.m_window, m_population.get());
				m_population.reset();
				m_prototype.reset();
				for (auto obj : m_lod_objects)
					View3D_ObjectDelete(obj);
			}

			// Set every block's live count from the density. Slots beyond the live count are not tested on the GPU.
			void ApplyDensity()
			{
				// Instances are randomly placed within their cells, so the first slots of each tile thin the tile evenly
				auto live = static_cast<int>(std::lround(m_density * BlockSlots));
				for (int b = 0; b != BlockCount; ++b)
					View3D_PopulationBlockSet(m_population.get(), b, m_block_bounds[b], b * BlockSlots, BlockSlots, live);
			}

			// Refresh the diagnostics twice a second. The counters lag the displayed frame by the frames in flight.
			void Step(double dt) override
			{
				// Throttle the label updates so the text is readable
				m_stats_timer += dt;
				if (m_stats_timer < 0.5)
					return;

				m_stats_timer = 0;
				auto stats = api::PopulationStats{};
				if (!View3D_PopulationStats(m_population.get(), stats))
					return;

				m_stats_label->Text(std::format(
					L"Views: {}\nBlocks: {} active, {} rejected\nInstances tested: {}\nEntries: {} (LOD0 {}, LOD1 {}, LOD2 {})\nIndirect draws: {}\nResident: {} KiB, scratch: {} KiB\nUploaded: {} KiB",
					stats.m_views, stats.m_active_blocks, stats.m_blocks_rejected, stats.m_instances_tested,
					stats.m_entries_emitted, stats.m_entries_per_lod[0], stats.m_entries_per_lod[1], stats.m_entries_per_lod[2],
					stats.m_indirect_draws, stats.m_resident_bytes / 1024, stats.m_scratch_bytes / 1024, stats.m_upload_bytes / 1024).c_str());
			}
		};
	}

	// Demo factories
	std::unique_ptr<IDemo> CreatePopulationFieldDemo(DemoContext const& ctx)
	{
		return std::make_unique<PopulationFieldDemo>(ctx);
	}
}
