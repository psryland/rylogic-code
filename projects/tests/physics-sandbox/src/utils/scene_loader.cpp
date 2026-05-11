#include "src/forward.h"
#include "src/utils/scene_loader.h"

namespace physics_sandbox::scene_loader
{
	namespace
	{
		enum class EGeneratorSelector
		{
			Random,
			Linear,
		};
		using NamedShapeMap = std::vector<std::pair<std::string, pr::json::Value const*>>;

		bool IsNumber(pr::json::Value const& jv)
		{
			return jv.as<double>() != nullptr;
		}
		bool IsVec3(pr::json::Value const& jv)
		{
			if (auto const* arr = jv.as<pr::json::Array>())
				return arr->size() >= 3 && IsNumber((*arr)[0]) && IsNumber((*arr)[1]) && IsNumber((*arr)[2]);

			return false;
		}
		float LinearT(int index, int count)
		{
			return count <= 1 ? 0.0f : float(index) / float(count - 1);
		}
		float SelectFloat(float min_value, float max_value, EGeneratorSelector selector, int index, int count, std::default_random_engine& rng)
		{
			switch (selector)
			{
				case EGeneratorSelector::Random:
				{
					auto range = std::uniform_real_distribution<float>(std::min(min_value, max_value), std::max(min_value, max_value));
					return range(rng);
				}
				case EGeneratorSelector::Linear:
				{
					auto t = LinearT(index, count);
					return Lerp(min_value, max_value, t);
				}
				default:
				{
					throw std::runtime_error("Unknown body generator selector");
				}
			}
		}
		v4 SelectVec3(v4 const& min_value, v4 const& max_value, EGeneratorSelector selector, int index, int count, std::default_random_engine& rng)
		{
			return v4{
				SelectFloat(min_value.x, max_value.x, selector, index, count, rng),
				SelectFloat(min_value.y, max_value.y, selector, index, count, rng),
				SelectFloat(min_value.z, max_value.z, selector, index, count, rng),
				min_value.w,
			};
		}
		float ReadFloatRange(pr::json::Value const& jv, EGeneratorSelector selector, int index, int count, std::default_random_engine& rng)
		{
			if (auto const* arr = jv.as<pr::json::Array>(); arr != nullptr && arr->size() == 2 && IsNumber((*arr)[0]) && IsNumber((*arr)[1]))
				return SelectFloat((*arr)[0].to<float>(), (*arr)[1].to<float>(), selector, index, count, rng);

			return jv.to<float>();
		}
		v4 ReadVec3Range(pr::json::Value const& jv, float w, EGeneratorSelector selector, int index, int count, std::default_random_engine& rng)
		{
			if (auto const* arr = jv.as<pr::json::Array>(); arr != nullptr && arr->size() == 2 && IsVec3((*arr)[0]) && IsVec3((*arr)[1]))
				return SelectVec3(ReadVec3((*arr)[0], w), ReadVec3((*arr)[1], w), selector, index, count, rng);

			return ReadVec3(jv, w);
		}
		Colour32 ReadColour(pr::json::Value const& jv)
		{
			if (auto const* str = jv.as<std::string>())
				return To<Colour32>(*str);
			if (auto const* num = jv.as<double>())
				return Colour32(static_cast<uint32_t>(std::llround(*num)));

			throw std::runtime_error("Expected a colour string or integer value");
		}
		Colour32 SelectColour(Colour32 min_value, Colour32 max_value, EGeneratorSelector selector, int index, int count, std::default_random_engine& rng)
		{
			switch (selector)
			{
				case EGeneratorSelector::Random:
				{
					auto channel = [&](int min_channel, int max_channel)
					{
						auto range = std::uniform_int_distribution<int>(std::min(min_channel, max_channel), std::max(min_channel, max_channel));
						return range(rng);
					};
					return Colour32(
						channel(min_value.r, max_value.r),
						channel(min_value.g, max_value.g),
						channel(min_value.b, max_value.b),
						channel(min_value.a, max_value.a));
				}
				case EGeneratorSelector::Linear:
				{
					auto t = LinearT(index, count);
					auto channel = [&](int min_channel, int max_channel)
					{
						return static_cast<int>(std::lround(Lerp(float(min_channel), float(max_channel), t)));
					};
					return Colour32(
						channel(min_value.r, max_value.r),
						channel(min_value.g, max_value.g),
						channel(min_value.b, max_value.b),
						channel(min_value.a, max_value.a));
				}
				default:
				{
					throw std::runtime_error("Unknown body generator selector");
				}
			}
		}
		Colour32 ReadColourRange(pr::json::Value const& jv, EGeneratorSelector selector, int index, int count, std::default_random_engine& rng)
		{
			if (auto const* arr = jv.as<pr::json::Array>(); arr != nullptr && arr->size() == 2)
				return SelectColour(ReadColour((*arr)[0]), ReadColour((*arr)[1]), selector, index, count, rng);

			return ReadColour(jv);
		}
		EGeneratorSelector ReadGeneratorSelector(pr::json::Object const& jgen)
		{
			auto selector = std::string("random");
			if (auto const* jselector = jgen.find("selector"))
				selector = jselector->to<std::string>();

			if (selector == "random")
				return EGeneratorSelector::Random;
			if (selector == "linear")
				return EGeneratorSelector::Linear;

			throw std::runtime_error(pr::FmtS("Unknown body generator selector: '%s'", selector.c_str()));
		}
		std::string GeneratedName(std::string name, int index, int count)
		{
			if (name.empty())
				name = "body_#";

			auto replacement = std::format("{}", index);
			auto ofs = size_t{};
			for (auto pos = name.find('#', ofs); pos != std::string::npos; pos = name.find('#', ofs))
			{
				name.replace(pos, 1, replacement);
				ofs = pos + replacement.size();
			}

			if (ofs == 0 && count > 1)
				name += std::format("_{}", index);

			return name;
		}
		int SelectPaletteIndex(EGeneratorSelector selector, int index, int count, int palette_count, std::default_random_engine& rng)
		{
			switch (selector)
			{
				case EGeneratorSelector::Random:
				{
					auto range = std::uniform_int_distribution<int>(0, palette_count - 1);
					return range(rng);
				}
				case EGeneratorSelector::Linear:
				{
					return palette_count <= 1 ? 0 : static_cast<int>(std::lround(LinearT(index, count) * (palette_count - 1)));
				}
				default:
				{
					throw std::runtime_error("Unknown body generator selector");
				}
			}
		}
		pr::json::Value const& ResolveShape(NamedShapeMap const& shapes, std::string_view shape_name)
		{
			for (auto const& [name, shape] : shapes)
			{
				if (name == shape_name)
					return *shape;
			}

			throw std::runtime_error(pr::FmtS("Scene references unknown shape '%.*s'", static_cast<int>(shape_name.size()), shape_name.data()));
		}
		BodyDesc ReadShape(pr::json::Value const& jshape)
		{
			BodyDesc desc;
			auto const& jshape_obj = jshape.to_object();
			auto shape_type = jshape_obj["type"].to<std::string>();
			if (shape_type == "box")
			{
				desc.shape_type = BodyDesc::EShape::Box;
				desc.box_dimensions = ReadVec3(jshape_obj["dimensions"], 0.0f);
			}
			else if (shape_type == "sphere")
			{
				desc.shape_type = BodyDesc::EShape::Sphere;
				desc.sphere_radius = jshape_obj["radius"].to<float>();
			}
			else if (shape_type == "line")
			{
				desc.shape_type = BodyDesc::EShape::Line;
				desc.line_length = jshape_obj["length"].to<float>();

				if (auto* thickness = jshape_obj.find("thickness"))
					desc.line_thickness = thickness->to<float>();
			}
			else if (shape_type == "triangle")
			{
				desc.shape_type = BodyDesc::EShape::Triangle;

				auto const& verts = jshape_obj["vertices"].to_array();
				if (verts.size() < 3)
					throw std::runtime_error("Triangle shape requires 3 vertices");

				desc.tri_verts[0] = ReadVec3(verts[0], 1.0f);
				desc.tri_verts[1] = ReadVec3(verts[1], 1.0f);
				desc.tri_verts[2] = ReadVec3(verts[2], 1.0f);
			}
			else if (shape_type == "polytope")
			{
				desc.shape_type = BodyDesc::EShape::Polytope;

				auto const& verts = jshape_obj["vertices"].to_array();
				if (verts.size() < 4)
					throw std::runtime_error("Polytope shape requires at least 4 non-coplanar vertices");

				for (auto const& v : verts)
					desc.polytope_verts.push_back(ReadVec3(v, 1.0f));
			}
			else
			{
				throw std::runtime_error(pr::FmtS("Unknown shape type: '%s'", shape_type.c_str()));
			}

			return desc;
		}
		BodyDesc ReadShapeRef(pr::json::Value const& jshape, NamedShapeMap const& shapes)
		{
			if (auto const* shape_name = jshape.as<std::string>())
				return ReadShape(ResolveShape(shapes, *shape_name));

			return ReadShape(jshape);
		}
		void AssignShape(BodyDesc& body, BodyDesc shape)
		{
			body.shape_type = shape.shape_type;
			body.box_dimensions = shape.box_dimensions;
			body.sphere_radius = shape.sphere_radius;
			body.line_length = shape.line_length;
			body.line_thickness = shape.line_thickness;
			body.tri_verts[0] = shape.tri_verts[0];
			body.tri_verts[1] = shape.tri_verts[1];
			body.tri_verts[2] = shape.tri_verts[2];
			body.polytope_verts = std::move(shape.polytope_verts);
		}
		BodyDesc ReadGeneratorShape(pr::json::Value const& jshape, NamedShapeMap const& shapes, EGeneratorSelector selector, int index, int count, std::default_random_engine& rng)
		{
			auto const* shape_name = jshape.as<std::string>();
			auto const& shape = shape_name != nullptr
				? ResolveShape(shapes, *shape_name)
				: jshape;

			auto const& jshape_obj = shape.to_object();
			auto shape_type = jshape_obj["type"].to<std::string>();
			if (shape_type == "box")
			{
				auto desc = BodyDesc{};
				desc.shape_type = BodyDesc::EShape::Box;
				desc.box_dimensions = ReadVec3Range(jshape_obj["dimensions"], 0.0f, selector, index, count, rng);
				return desc;
			}
			if (shape_type == "sphere")
			{
				auto desc = BodyDesc{};
				desc.shape_type = BodyDesc::EShape::Sphere;
				desc.sphere_radius = ReadFloatRange(jshape_obj["radius"], selector, index, count, rng);
				return desc;
			}
			if (shape_type == "line")
			{
				auto desc = BodyDesc{};
				desc.shape_type = BodyDesc::EShape::Line;
				desc.line_length = ReadFloatRange(jshape_obj["length"], selector, index, count, rng);
				if (auto const* thickness = jshape_obj.find("thickness"))
					desc.line_thickness = ReadFloatRange(*thickness, selector, index, count, rng);
				return desc;
			}

			return ReadShape(shape);
		}
		NamedShapeMap ReadNamedShapes(pr::json::Object const& jscene)
		{
			auto shapes = NamedShapeMap{};
			if (auto const* jshapes = jscene.find("shapes"))
			{
				for (auto const& shape : jshapes->to_object())
				{
					auto const& shape_obj = shape.val.to_object();
					auto const* jname = shape_obj.find("name");
					if (jname == nullptr)
						throw std::runtime_error(pr::FmtS("Scene shape declaration '%s' requires a 'name' field", shape.key.c_str()));

					auto shape_name = jname->to<std::string>();
					if (shape_name.empty())
						throw std::runtime_error(pr::FmtS("Scene shape declaration '%s' has an empty name", shape.key.c_str()));

					for (auto const& existing_shape : shapes)
					{
						if (existing_shape.first == shape_name)
							throw std::runtime_error(pr::FmtS("Scene shape name '%s' is declared more than once", shape_name.c_str()));
					}

					shapes.push_back({std::move(shape_name), &shape.val});
				}
			}
			return shapes;
		}
		void AppendGeneratedBodies(SceneDesc& desc, pr::json::Value const& jv_generator, NamedShapeMap const& shapes, std::default_random_engine& rng)
		{
			auto const& jgen = jv_generator.to_object();
			auto selector = ReadGeneratorSelector(jgen);
			auto instance_count = 1;
			if (auto const* jcount = jgen.find("instance_count"))
				instance_count = jcount->to<int>();
			if (instance_count <= 0)
				throw std::runtime_error("Body generator 'instance_count' must be greater than zero");

			auto const* jshape = jgen.find("shape");
			if (jshape == nullptr)
				throw std::runtime_error("Body generator requires a 'shape' field");

			auto shape_palette_count = std::min(instance_count, 16);
			if (auto const* jpalette_count = jgen.find("shape_palette_count"))
				shape_palette_count = jpalette_count->to<int>();
			if (shape_palette_count <= 0)
				throw std::runtime_error("Body generator 'shape_palette_count' must be greater than zero");

			shape_palette_count = std::min(shape_palette_count, instance_count);

			auto shape_palette = std::vector<BodyDesc>{};
			shape_palette.reserve(shape_palette_count);
			for (auto shape_index = 0; shape_index != shape_palette_count; ++shape_index)
				shape_palette.push_back(ReadGeneratorShape(*jshape, shapes, selector, shape_index, shape_palette_count, rng));

			auto name = std::string{};
			if (auto const* jname = jgen.find("name"))
				name = jname->to<std::string>();

			for (auto body_index = 0; body_index != instance_count; ++body_index)
			{
				auto palette_index = SelectPaletteIndex(selector, body_index, instance_count, shape_palette_count, rng);
				auto body = shape_palette[palette_index];
				body.name = GeneratedName(name, body_index, instance_count);

				if (auto const* jcolour = jgen.find("colour"))
					body.colour = ReadColourRange(*jcolour, selector, body_index, instance_count, rng);
				if (auto const* jmass = jgen.find("mass"))
					body.mass = ReadFloatRange(*jmass, selector, body_index, instance_count, rng);
				if (auto const* jposition = jgen.find("position"))
					body.position = ReadVec3Range(*jposition, 1.0f, selector, body_index, instance_count, rng);
				if (auto const* jrotation = jgen.find("rotation"))
					body.rotation = ReadVec3Range(*jrotation, 0.0f, selector, body_index, instance_count, rng);
				if (auto const* jvelocity = jgen.find("velocity"))
					body.velocity = ReadVec3Range(*jvelocity, 0.0f, selector, body_index, instance_count, rng);
				if (auto const* jangular_velocity = jgen.find("angular_velocity"))
					body.angular_velocity = ReadVec3Range(*jangular_velocity, 0.0f, selector, body_index, instance_count, rng);
				if (auto const* jsleeping = jgen.find("sleeping"))
					body.sleeping = jsleeping->to<bool>();

				desc.bodies.push_back(std::move(body));
			}
		}
	}

	// Read a 3-element JSON array as a position vector (w=1) or direction vector (w=0).
	v4 ReadVec3(pr::json::Value const& arr, float w)
	{
		auto const& a = arr.to_array();
		if (a.size() < 3)
			throw std::runtime_error("Expected a 3-element array for vector");

		return v4{
			a[0].to<float>(),
			a[1].to<float>(),
			a[2].to<float>(),
			w
		};
	}

	// Parse a single body definition from a JSON object
	BodyDesc ReadBody(pr::json::Value const& jv_body)
	{
		BodyDesc desc;
		auto const& jbody = jv_body.to_object();

		// Name
		if (auto* jname = jbody.find("name"))
			desc.name = jname->to<std::string>();
		
		// Colour
		if (auto* jcolour = jbody.find("colour"))
			desc.colour = ReadColour(*jcolour);

		// Mass
		if (auto* jmass = jbody.find("mass"))
			desc.mass = jmass->to<float>();

		// Position
		if (auto* jpos = jbody.find("position"))
			desc.position = ReadVec3(*jpos, 1.0f);

		// Rotation — Euler angles in degrees, applied in X, Y, Z order
		if (auto* jrot = jbody.find("rotation"))
			desc.rotation = ReadVec3(*jrot, 0.0f);

		// Velocity (optional, defaults to zero)
		if (auto* jvel = jbody.find("velocity"))
			desc.velocity = ReadVec3(*jvel, 0.0f);

		// Angular velocity (optional, defaults to zero)
		if (auto* javl = jbody.find("angular_velocity"))
			desc.angular_velocity = ReadVec3(*javl, 0.0f);

		// Initial sleep state
		if (auto* jsleeping = jbody.find("sleeping"))
			desc.sleeping = jsleeping->to<bool>();

		if (auto* jshape = jbody.find("shape"); jshape != nullptr && jshape->as<std::string>() == nullptr)
			AssignShape(desc, ReadShape(*jshape));

		return desc;
	}
	BodyDesc ReadBody(pr::json::Value const& jv_body, NamedShapeMap const& shapes)
	{
		auto desc = ReadBody(jv_body);
		auto const& jbody = jv_body.to_object();
		if (auto* jshape = jbody.find("shape"))
			AssignShape(desc, ReadShapeRef(*jshape, shapes));
		else
			throw std::runtime_error(pr::FmtS("Body '%s' requires a 'shape' field", desc.name.c_str()));

		return desc;
	}

	// Parse a ground plane definition from a JSON object
	GroundPlaneDesc ReadGroundPlane(pr::json::Value const& jgp_)
	{
		GroundPlaneDesc ground;
		auto const& jgp = jgp_.to_object();

		if (auto* s = jgp.find("size"))
			ground.size = v2(s->to_array()[0].to<float>(), s->to_array()[1].to<float>());

		if (auto* h = jgp.find("height"))
			ground.height = h->to<float>();

		if (auto* c = jgp.find("colour"))
			ground.colour = ReadColour(*c);

		if (auto* t = jgp.find("texture"))
			ground.texture = t->to<std::string>();

		return ground;
	}

	// Parse a ground plane definition from a JSON object
	CameraDesc ReadCamera(pr::json::Value const& jcam)
	{
		CameraDesc camera;
		auto const& jcam_obj = jcam.to_object();

		if (auto* jposition = jcam_obj.find("position"))
			camera.position = ReadVec3(*jposition, 1.0f);

		if (auto* jlookat = jcam_obj.find("lookat"))
			camera.lookat = ReadVec3(*jlookat, 1.0f);

		return camera;
	}

	// Parse a scene description from a JSON file
	SceneDesc LoadFromFile(std::filesystem::path const& filepath)
	{
		auto doc = pr::json::Read(filepath, json::Options{.AllowComments = true, .AllowTrailingCommas = true});
		auto const& jscene = doc.to_object()["scene"].to_object();

		SceneDesc desc;
		desc.filepath = filepath;

		// Description
		if (auto* jdesc = jscene.find("description"))
			desc.description = jdesc->to<std::string>();

		// Gravity
		if (auto* jgravity = jscene.find("gravity"))
			desc.gravity = ReadVec3(*jgravity, 0.0f);

		// Generated scene content
		if (jscene.find("colour_seed") != nullptr)
			throw std::runtime_error("Scene property 'colour_seed' has been replaced by 'seed'");

		if (auto* jseed = jscene.find("seed"))
			desc.seed = static_cast<unsigned int>(jseed->to<int>());

		auto scene_rng = std::default_random_engine(desc.seed);
		auto shapes = ReadNamedShapes(jscene);

		// Material properties
		if (auto* jmat = jscene.find("material"))
		{
			if (auto* jelasticity = jmat->to_object().find("elasticity"))
				desc.elasticity = jelasticity->to<float>();

			if (auto* jfriction = jmat->to_object().find("friction"))
				desc.friction = jfriction->to<float>();
		}

		// Physics settings
		if (auto* jphysics = jscene.find("physics"))
		{
			auto const& jphysics_obj = jphysics->to_object();
			if (auto* jsubsteps = jphysics_obj.find("substeps"))
			{
				desc.physics_substeps = jsubsteps->to<int>();
				if (desc.physics_substeps < 1)
					throw std::runtime_error("Scene physics.substeps must be at least 1");
			}
			if (auto* jsolver_iterations = jphysics_obj.find("solver_iterations"))
			{
				desc.physics_solver_iterations = jsolver_iterations->to<int>();
				if (desc.physics_solver_iterations < 0)
					throw std::runtime_error("Scene physics.solver_iterations must be non-negative");
			}
			if (auto* jposition_iterations = jphysics_obj.find("position_iterations"))
			{
				desc.physics_position_iterations = jposition_iterations->to<int>();
				if (desc.physics_position_iterations < 0)
					throw std::runtime_error("Scene physics.position_iterations must be non-negative");
			}
			if (auto* jcontact_sort_propagation_scale = jphysics_obj.find("contact_sort_propagation_scale"))
			{
				desc.physics_contact_sort_propagation_scale = jcontact_sort_propagation_scale->to<float>();
				if (desc.physics_contact_sort_propagation_scale < 0.0f)
					throw std::runtime_error("Scene physics.contact_sort_propagation_scale must be non-negative");
			}
			if (auto* jbroadphase_aabb_margin = jphysics_obj.find("broadphase_aabb_margin"))
			{
				desc.physics_broadphase_aabb_margin = jbroadphase_aabb_margin->to<float>();
				if (desc.physics_broadphase_aabb_margin < 0.0f)
					throw std::runtime_error("Scene physics.broadphase_aabb_margin must be non-negative");
			}
			if (auto* jcontact_sort_shock_iterations = jphysics_obj.find("contact_sort_shock_iterations"))
			{
				desc.physics_contact_sort_shock_iterations = jcontact_sort_shock_iterations->to<int>();
				if (desc.physics_contact_sort_shock_iterations < 0)
					throw std::runtime_error("Scene physics.contact_sort_shock_iterations must be non-negative");
			}
			if (auto* jcontact_slop_scale = jphysics_obj.find("contact_slop_scale"))
			{
				desc.physics_contact_slop_scale = jcontact_slop_scale->to<float>();
				if (desc.physics_contact_slop_scale < 0.0f)
					throw std::runtime_error("Scene physics.contact_slop_scale must be non-negative");
			}
			if (auto* jsupport_contact_slop_scale = jphysics_obj.find("support_contact_slop_scale"))
			{
				desc.physics_support_contact_slop_scale = jsupport_contact_slop_scale->to<float>();
				if (desc.physics_support_contact_slop_scale < 0.0f)
					throw std::runtime_error("Scene physics.support_contact_slop_scale must be non-negative");
			}
			if (auto* jwarm_start_scale = jphysics_obj.find("warm_start_scale"))
			{
				desc.physics_warm_start_scale = jwarm_start_scale->to<float>();
				if (desc.physics_warm_start_scale < 0.0f)
					throw std::runtime_error("Scene physics.warm_start_scale must be non-negative");
			}
			if (auto* jmax_collision_pairs = jphysics_obj.find("max_collision_pairs"))
			{
				desc.physics_max_collision_pairs = jmax_collision_pairs->to<int>();
				if (desc.physics_max_collision_pairs < 1)
					throw std::runtime_error("Scene physics.max_collision_pairs must be at least 1");
			}
			if (auto* jselective_refresh_passes = jphysics_obj.find("selective_refresh_passes"))
			{
				desc.physics_selective_refresh_passes = jselective_refresh_passes->to<int>();
				if (desc.physics_selective_refresh_passes < 0)
					throw std::runtime_error("Scene physics.selective_refresh_passes must be non-negative");
			}
			if (auto* jselective_refresh_max_pairs = jphysics_obj.find("selective_refresh_max_pairs"))
			{
				desc.physics_selective_refresh_max_pairs = jselective_refresh_max_pairs->to<int>();
				if (desc.physics_selective_refresh_max_pairs < 1)
					throw std::runtime_error("Scene physics.selective_refresh_max_pairs must be at least 1");
			}
			if (auto* jselective_refresh_body_limit = jphysics_obj.find("selective_refresh_body_limit"))
			{
				desc.physics_selective_refresh_body_limit = jselective_refresh_body_limit->to<int>();
				if (desc.physics_selective_refresh_body_limit < 0)
					throw std::runtime_error("Scene physics.selective_refresh_body_limit must be non-negative");
			}
			if (auto* jselective_refresh_contact_limit = jphysics_obj.find("selective_refresh_contact_limit"))
			{
				desc.physics_selective_refresh_contact_limit = jselective_refresh_contact_limit->to<int>();
				if (desc.physics_selective_refresh_contact_limit < 0)
					throw std::runtime_error("Scene physics.selective_refresh_contact_limit must be non-negative");
			}
			if (auto* jselective_refresh_solver_iterations = jphysics_obj.find("selective_refresh_solver_iterations"))
			{
				desc.physics_selective_refresh_solver_iterations = jselective_refresh_solver_iterations->to<int>();
				if (desc.physics_selective_refresh_solver_iterations < 0)
					throw std::runtime_error("Scene physics.selective_refresh_solver_iterations must be non-negative");
			}
			if (auto* jselective_refresh_position_iterations = jphysics_obj.find("selective_refresh_position_iterations"))
			{
				desc.physics_selective_refresh_position_iterations = jselective_refresh_position_iterations->to<int>();
				if (desc.physics_selective_refresh_position_iterations < 0)
					throw std::runtime_error("Scene physics.selective_refresh_position_iterations must be non-negative");
			}
			if (auto* jselective_refresh_bias_scale = jphysics_obj.find("selective_refresh_bias_scale"))
			{
				desc.physics_selective_refresh_bias_scale = jselective_refresh_bias_scale->to<float>();
				if (desc.physics_selective_refresh_bias_scale < 0.0f)
					throw std::runtime_error("Scene physics.selective_refresh_bias_scale must be non-negative");
			}
			if (auto* jselective_refresh_restitution_scale = jphysics_obj.find("selective_refresh_restitution_scale"))
			{
				desc.physics_selective_refresh_restitution_scale = jselective_refresh_restitution_scale->to<float>();
				if (desc.physics_selective_refresh_restitution_scale < 0.0f)
					throw std::runtime_error("Scene physics.selective_refresh_restitution_scale must be non-negative");
			}
			if (auto* jselective_refresh_adaptive_body_limit = jphysics_obj.find("selective_refresh_adaptive_body_limit"))
			{
				desc.physics_selective_refresh_adaptive_body_limit = jselective_refresh_adaptive_body_limit->to<int>();
				if (desc.physics_selective_refresh_adaptive_body_limit < 0)
					throw std::runtime_error("Scene physics.selective_refresh_adaptive_body_limit must be non-negative");
			}
			if (auto* jselective_refresh_adaptive_solver_iterations = jphysics_obj.find("selective_refresh_adaptive_solver_iterations"))
			{
				desc.physics_selective_refresh_adaptive_solver_iterations = jselective_refresh_adaptive_solver_iterations->to<int>();
				if (desc.physics_selective_refresh_adaptive_solver_iterations < 0)
					throw std::runtime_error("Scene physics.selective_refresh_adaptive_solver_iterations must be non-negative");
			}
			if (auto* jselective_refresh_support_only = jphysics_obj.find("selective_refresh_support_only"))
			{
				desc.physics_selective_refresh_support_only = jselective_refresh_support_only->to<bool>();
			}
			if (auto* jselective_refresh_resolve_support_only = jphysics_obj.find("selective_refresh_resolve_support_only"))
			{
				desc.physics_selective_refresh_resolve_support_only = jselective_refresh_resolve_support_only->to<bool>();
			}
			if (auto* jselective_refresh_depth_slop = jphysics_obj.find("selective_refresh_depth_slop"))
			{
				desc.physics_selective_refresh_depth_slop = jselective_refresh_depth_slop->to<float>();
				if (desc.physics_selective_refresh_depth_slop < 0.0f)
					throw std::runtime_error("Scene physics.selective_refresh_depth_slop must be non-negative");
			}
			if (auto* jselective_refresh_support_depth_slop = jphysics_obj.find("selective_refresh_support_depth_slop"))
			{
				desc.physics_selective_refresh_support_depth_slop = jselective_refresh_support_depth_slop->to<float>();
				if (desc.physics_selective_refresh_support_depth_slop < 0.0f)
					throw std::runtime_error("Scene physics.selective_refresh_support_depth_slop must be non-negative");
			}
			if (auto* jselective_refresh_closing_speed_slop = jphysics_obj.find("selective_refresh_closing_speed_slop"))
			{
				desc.physics_selective_refresh_closing_speed_slop = jselective_refresh_closing_speed_slop->to<float>();
				if (desc.physics_selective_refresh_closing_speed_slop < 0.0f)
					throw std::runtime_error("Scene physics.selective_refresh_closing_speed_slop must be non-negative");
			}
			if (auto* jselective_refresh_support_alignment = jphysics_obj.find("selective_refresh_support_alignment"))
			{
				desc.physics_selective_refresh_support_alignment = jselective_refresh_support_alignment->to<float>();
				if (desc.physics_selective_refresh_support_alignment < 0.0f || desc.physics_selective_refresh_support_alignment > 1.0f)
					throw std::runtime_error("Scene physics.selective_refresh_support_alignment must be in [0,1]");
			}
			if (auto* jselective_refresh_aabb_margin = jphysics_obj.find("selective_refresh_aabb_margin"))
			{
				desc.physics_selective_refresh_aabb_margin = jselective_refresh_aabb_margin->to<float>();
				if (desc.physics_selective_refresh_aabb_margin < 0.0f)
					throw std::runtime_error("Scene physics.selective_refresh_aabb_margin must be non-negative");
			}
		}

		// Ground plane
		if (auto* jground = jscene.find("ground_plane"))
			desc.ground = ReadGroundPlane(*jground);

		// Bodies
		if (auto* jbodies = jscene.find("bodies"))
		{
			for (auto const& jbody : jbodies->to_array())
				desc.bodies.push_back(ReadBody(jbody, shapes));
		}

		// Generated bodies
		if (auto* jgenerators = jscene.find("body_generators"))
		{
			for (auto const& jgenerator : jgenerators->to_array())
				AppendGeneratedBodies(desc, jgenerator, shapes, scene_rng);
		}

		// Camera
		if (auto* jcamera = jscene.find("camera"))
		{
			desc.camera = ReadCamera(*jcamera);
		}

		return desc;
	}
}
