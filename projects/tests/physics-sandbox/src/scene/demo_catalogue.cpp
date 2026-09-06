#include "src/forward.h"
#include "src/scene/demo.h"
#include "src/utils/scene_loader.h"

namespace physics_sandbox
{
	namespace
	{
		constexpr auto Categories = std::array
		{
			DemoCategoryInfo{EDemoCategory::DynamicsAndConservation, "Dynamics and Conservation"},
			DemoCategoryInfo{EDemoCategory::CollisionAndContacts, "Collision and Contacts"},
			DemoCategoryInfo{EDemoCategory::StacksAndStability, "Stacks and Stability"},
			DemoCategoryInfo{EDemoCategory::ConstraintsAndMechanisms, "Constraints and Mechanisms"},
			DemoCategoryInfo{EDemoCategory::ArticulationsAndRobotics, "Articulations and Robotics"},
			DemoCategoryInfo{EDemoCategory::Buoyancy, "Buoyancy"},
			DemoCategoryInfo{EDemoCategory::StressAndScaling, "Stress and Scaling"},
			DemoCategoryInfo{EDemoCategory::FeatureShowcases, "Feature Showcases"},
		};

		// Existing general-purpose scenes predate embedded demo metadata, so retain their catalogue presentation while still loading all content from JSON.
		struct LegacyDemoInfo
		{
			std::string_view m_filename;
			EDemoCategory m_category;
			std::string_view m_name;
			std::string_view m_display_name;
			int m_order;
		};

		constexpr auto LegacyDemos = std::array
		{
			LegacyDemoInfo{"all_shapes_drop.json", EDemoCategory::CollisionAndContacts, "all-shapes-drop", "All Shape Types", 0},
			LegacyDemoInfo{"angular_integration_stress.json", EDemoCategory::DynamicsAndConservation, "angular-integration-stress", "Angular Integration", 0},
			LegacyDemoInfo{"box_vs_sphere.json", EDemoCategory::CollisionAndContacts, "box-vs-sphere", "Box vs Sphere", 1},
			LegacyDemoInfo{"brick_pyramid.json", EDemoCategory::StacksAndStability, "brick-pyramid", "Brick Pyramid", 0},
			LegacyDemoInfo{"brick_wall.json", EDemoCategory::StressAndScaling, "brick-wall", "Brick Wall", 0},
			LegacyDemoInfo{"brick_wall_2000.json", EDemoCategory::StressAndScaling, "brick-wall-2000", "Brick Wall (2,000 Bricks)", 1},
			LegacyDemoInfo{"buoyancy_stress_1000.json", EDemoCategory::Buoyancy, "buoyancy-stress-1000", "Buoyancy Stress (1,000 Bodies)", 0},
			LegacyDemoInfo{"drop_test.json", EDemoCategory::CollisionAndContacts, "drop-test", "Single Box Drop", 2},
			LegacyDemoInfo{"dzhanibekov_effect.json", EDemoCategory::DynamicsAndConservation, "dzhanibekov-effect", "Dzhanibekov Effect (Rigid Body)", 1},
			LegacyDemoInfo{"floater.json", EDemoCategory::Buoyancy, "floater", "Floating Shapes", 1},
			LegacyDemoInfo{"generated_stress.json", EDemoCategory::StressAndScaling, "generated-stress", "Generated Body Stress", 2},
			LegacyDemoInfo{"gravity_playground.json", EDemoCategory::CollisionAndContacts, "gravity-playground", "Gravity and Shape Playground", 3},
			LegacyDemoInfo{"newtons_cradle.json", EDemoCategory::StacksAndStability, "newtons-cradle", "Shock Propagation", 1},
			LegacyDemoInfo{"oblique_spheres.json", EDemoCategory::CollisionAndContacts, "oblique-spheres", "Oblique Sphere Collision", 4},
			LegacyDemoInfo{"pancake.json", EDemoCategory::StressAndScaling, "pancake", "Large Column Stack", 3},
			LegacyDemoInfo{"plate_drop.json", EDemoCategory::StacksAndStability, "plate-drop", "Thin Plate Settling", 2},
			LegacyDemoInfo{"polytope_drop.json", EDemoCategory::CollisionAndContacts, "polytope-drop", "Polytope Drop", 5},
			LegacyDemoInfo{"simultaneous_impact_1000.json", EDemoCategory::StressAndScaling, "simultaneous-impact-1000", "Simultaneous Impact (1,000 Bodies)", 4},
			LegacyDemoInfo{"spinning_tops.json", EDemoCategory::DynamicsAndConservation, "spinning-tops", "Spinning Tops", 2},
			LegacyDemoInfo{"stacked_column.json", EDemoCategory::StacksAndStability, "stacked-column", "Stacked Column", 3},
			LegacyDemoInfo{"stress_test_1000.json", EDemoCategory::StressAndScaling, "stress-test-1000", "Mixed Shape Stress (1,000 Bodies)", 5},
			LegacyDemoInfo{"three_body_chain.json", EDemoCategory::CollisionAndContacts, "three-body-chain", "Three-Body Collision Chain", 6},
		};

		// Compare command names without making CLI spelling depend on punctuation or letter case.
		bool EquivalentName(std::string_view lhs, std::string_view rhs)
		{
			if (lhs.size() != rhs.size())
				return false;

			for (auto i = size_t{}; i != lhs.size(); ++i)
			{
				auto const left = lhs[i] == '_' ? '-' : static_cast<char>(std::tolower(static_cast<unsigned char>(lhs[i])));
				auto const right = rhs[i] == '_' ? '-' : static_cast<char>(std::tolower(static_cast<unsigned char>(rhs[i])));
				if (left != right)
					return false;
			}
			return true;
		}

		// Scan for the optional metadata key without materialising large non-demo scene documents during menu construction.
		bool ContainsDemoToken(std::filesystem::path const& filepath)
		{
			auto stream = std::ifstream(filepath);
			if (!stream)
				throw std::runtime_error(std::format("Failed to read physics scene catalogue candidate: {}", filepath.string()));

			constexpr auto token = std::string_view{"\"demo\""};
			auto matched = size_t{};
			for (auto ch = char{}; stream.get(ch);)
			{
				matched = ch == token[matched] ? matched + 1 : ch == token[0] ? 1 : 0;
				if (matched == token.size())
					return true;
			}
			return false;
		}

		// Resolve metadata group text to the stable category used by menus and ordering.
		EDemoCategory ParseCategory(std::string_view group)
		{
			auto const iter = std::ranges::find_if(Categories, [=](DemoCategoryInfo const& category)
			{
				return EquivalentName(group, category.m_display_name);
			});
			if (iter == Categories.end())
				throw std::runtime_error(std::format("Unknown physics demo group '{}'", group));

			return iter->m_category;
		}

		// Convert one legacy catalogue entry into the same runtime representation as metadata-backed scenes.
		DemoInfo MakeDemoInfo(LegacyDemoInfo const& legacy, std::filesystem::path const& filepath)
		{
			return DemoInfo
			{
				.m_category = legacy.m_category,
				.m_name = std::string(legacy.m_name),
				.m_display_name = std::string(legacy.m_display_name),
				.m_description = {},
				.m_order = legacy.m_order,
				.m_filepath = filepath,
			};
		}

		// Discover classified JSON files from the deployed scene directory so demo content and metadata remain runtime-editable.
		std::vector<DemoInfo> BuildDemoCatalogue()
		{
			auto const scene_dir = win32::ExeDir() / "scenes";
			if (!std::filesystem::is_directory(scene_dir))
				throw std::runtime_error(std::format("Physics demo directory not found: {}", scene_dir.string()));

			auto demos = std::vector<DemoInfo>{};
			for (auto const& entry : std::filesystem::directory_iterator(scene_dir))
			{
				if (!entry.is_regular_file() || !EquivalentName(entry.path().extension().string(), ".json"))
					continue;

				// Embedded metadata owns newly-authored presentation; unchanged legacy files use the stable fallback without constructing a JSON DOM.
				auto const filename = entry.path().filename().string();
				auto const legacy = std::ranges::find_if(LegacyDemos, [&](LegacyDemoInfo const& info)
				{
					return EquivalentName(filename, info.m_filename);
				});
				auto const metadata = ContainsDemoToken(entry.path())
					? scene_loader::LoadMetadataFromFile(entry.path())
					: std::optional<scene_loader::SceneMetadata>{};
				if (metadata.has_value())
				{
					demos.push_back(DemoInfo
					{
						.m_category = ParseCategory(metadata->m_group),
						.m_name = metadata->m_command.empty() ? entry.path().stem().string() : metadata->m_command,
						.m_display_name = metadata->m_name,
						.m_description = metadata->m_description,
						.m_order = metadata->m_order,
						.m_filepath = entry.path(),
					});
					continue;
				}

				if (legacy != LegacyDemos.end())
					demos.push_back(MakeDemoInfo(*legacy, entry.path()));
			}

			// Category declaration order and per-file order make menu placement deterministic across filesystems.
			std::ranges::sort(demos, {}, [](DemoInfo const& demo)
			{
				return std::tuple(static_cast<int>(demo.m_category), demo.m_order, demo.m_display_name);
			});
			for (auto lhs = size_t{}; lhs != demos.size(); ++lhs)
			{
				for (auto rhs = lhs + 1; rhs != demos.size(); ++rhs)
				{
					if (EquivalentName(demos[lhs].m_name, demos[rhs].m_name))
						throw std::runtime_error(std::format("Duplicate physics demo command '{}'", demos[lhs].m_name));
				}
			}
			return demos;
		}
	}

	// Return demonstration categories in stable menu order.
	std::span<DemoCategoryInfo const> DemoCategoryCatalogue()
	{
		return Categories;
	}

	// Discover deployed JSON demonstrations and return them in stable grouped order.
	std::span<DemoInfo const> DemoCatalogue()
	{
		static auto const demos = BuildDemoCatalogue();
		return demos;
	}

	// Resolve a case-insensitive command-line demonstration name.
	DemoInfo const* FindDemo(std::string_view name)
	{
		auto const demos = DemoCatalogue();
		auto const iter = std::ranges::find_if(demos, [=](DemoInfo const& info)
		{
			return EquivalentName(name, info.m_name);
		});
		return iter != demos.end() ? &*iter : nullptr;
	}
}
