#include "src/forward.h"
#include "src/utils/scene_loader.h"
#include "src/utils/scene_loader_internal.h"

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
		// Read a two-component float vector from a JSON array.
		v2 ReadVec2(pr::json::Value const& arr)
		{
			auto const& a = arr.to_array();
			if (a.size() < 2)
				throw std::runtime_error("Expected a 2-element array for vector");

			return v2{
				a[0].to<float>(),
				a[1].to<float>(),
			};
		}
		// Read a two-component double vector from a JSON array.
		pr::physics::terrain::v2d ReadVec2d(pr::json::Value const& arr)
		{
			auto const& a = arr.to_array();
			if (a.size() < 2)
				throw std::runtime_error("Expected a 2-element array for vector");

			return pr::physics::terrain::v2d{
				a[0].to<double>(),
				a[1].to<double>(),
			};
		}
		// Read a two-component integer vector from a JSON array.
		iv2 ReadInt2(pr::json::Value const& arr)
		{
			auto const& a = arr.to_array();
			if (a.size() < 2)
				throw std::runtime_error("Expected a 2-element array for integer vector");

			return iv2{
				a[0].to<int>(),
				a[1].to<int>(),
			};
		}
		// Read a three-component integer vector from a JSON array.
		iv3 ReadInt3(pr::json::Value const& arr)
		{
			auto const& a = arr.to_array();
			if (a.size() < 3)
				throw std::runtime_error("Expected a 3-element array for integer vector");

			return iv3{
				a[0].to<int>(),
				a[1].to<int>(),
				a[2].to<int>(),
			};
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
		// Select an integer using the generator's random or linearly distributed policy.
		int SelectInt(int min_value, int max_value, EGeneratorSelector selector, int index, int count, std::default_random_engine& rng)
		{
			switch (selector)
			{
				case EGeneratorSelector::Random:
				{
					auto range = std::uniform_int_distribution<int>(std::min(min_value, max_value), std::max(min_value, max_value));
					return range(rng);
				}
				case EGeneratorSelector::Linear:
				{
					auto const t = LinearT(index, count);
					return static_cast<int>(std::lround(Lerp(static_cast<float>(min_value), static_cast<float>(max_value), t)));
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
		// Read an integer or select one from a two-value range.
		int ReadIntRange(pr::json::Value const& jv, EGeneratorSelector selector, int index, int count, std::default_random_engine& rng)
		{
			if (auto const* arr = jv.as<pr::json::Array>(); arr != nullptr && arr->size() == 2 && IsNumber((*arr)[0]) && IsNumber((*arr)[1]))
				return SelectInt((*arr)[0].to<int>(), (*arr)[1].to<int>(), selector, index, count, rng);

			return jv.to<int>();
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
			else if (shape_type == "compound")
			{
				desc.shape_type = BodyDesc::EShape::Compound;

				auto const& children = jshape_obj["children"].to_array();
				if (children.empty())
					throw std::runtime_error("Compound shape requires at least one child");

				for (auto const& jchild : children)
				{
					// Each child is a primitive shape object with an optional body-space placement.
					auto child = ReadShape(jchild);
					if (child.shape_type == BodyDesc::EShape::Compound)
						throw std::runtime_error("Compound shape children must be primitive shapes");

					auto const& jchild_obj = jchild.to_object();
					if (auto const* jpos = jchild_obj.find("position"))
						child.position = ReadVec3(*jpos, 1.0f);
					if (auto const* jrot = jchild_obj.find("rotation"))
						child.rotation = ReadVec3(*jrot, 0.0f);

					desc.compound_children.push_back(std::move(child));
				}
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
			body.compound_children = std::move(shape.compound_children);
		}

		// Generate a uniformly distributed direction over the unit sphere.
		v4 RandomDirection(std::default_random_engine& rng)
		{
			auto unit = std::uniform_real_distribution<float>(0.0f, 1.0f);
			auto const z = 2.0f * unit(rng) - 1.0f;
			auto const azimuth = constants<float>::tau * unit(rng);
			auto const radial = std::sqrt(std::max(0.0f, 1.0f - z * z));
			return v4{radial * std::cos(azimuth), radial * std::sin(azimuth), z, 0.0f};
		}

		// Generate a non-degenerate radial point cloud whose convex hull forms a varied faceted body.
		// Axis anchors bound every principal extent while the remaining random directions produce the
		// irregular facets. Independent positive and negative radii avoid imposing central symmetry.
		BodyDesc ReadRandomConvexShape(pr::json::Object const& jshape, int index, int count, std::default_random_engine& rng)
		{
			auto point_count = 12;
			if (auto const* jpoint_count = jshape.find("point_count"))
				point_count = ReadIntRange(*jpoint_count, EGeneratorSelector::Random, index, count, rng);
			if (point_count < 6)
				throw std::runtime_error("Random convex shape 'point_count' must be at least 6");

			auto radius_min = 0.7f;
			auto radius_max = 1.0f;
			if (auto const* jradius = jshape.find("radius"))
			{
				if (auto const* range = jradius->as<pr::json::Array>(); range != nullptr && range->size() == 2 && IsNumber((*range)[0]) && IsNumber((*range)[1]))
				{
					radius_min = (*range)[0].to<float>();
					radius_max = (*range)[1].to<float>();
				}
				else
				{
					radius_min = radius_max = jradius->to<float>();
				}
			}
			if (!(radius_min > 0.0f) || !(radius_max > 0.0f) || !std::isfinite(radius_min) || !std::isfinite(radius_max))
				throw std::runtime_error("Random convex shape 'radius' values must be finite and positive");

			auto aspect = v4{1.0f, 1.0f, 1.0f, 0.0f};
			if (auto const* jaspect = jshape.find("aspect"))
				aspect = ReadVec3Range(*jaspect, 0.0f, EGeneratorSelector::Random, index, count, rng);
			if (!(aspect.x > 0.0f) || !(aspect.y > 0.0f) || !(aspect.z > 0.0f) || !IsFinite(aspect))
				throw std::runtime_error("Random convex shape 'aspect' values must be finite and positive");

			// The six axis anchors guarantee that sparse random samples cannot produce a nearly planar
			// or needle-like hull with an ill-conditioned inertia tensor.
			auto desc = BodyDesc{};
			desc.shape_type = BodyDesc::EShape::Polytope;
			desc.polytope_verts.reserve(point_count);
			auto radius = std::uniform_real_distribution<float>(std::min(radius_min, radius_max), std::max(radius_min, radius_max));
			auto const anchors = std::array{
				v4::XAxis(),
				v4::YAxis(),
				v4::ZAxis(),
			};
			for (auto const& direction : anchors)
			{
				desc.polytope_verts.push_back((direction * radius(rng) * aspect).w1());
				desc.polytope_verts.push_back((-direction * radius(rng) * aspect).w1());
			}

			// Additional antipodal direction pairs retain the guaranteed origin enclosure. An odd
			// requested count receives one final radial point without changing that invariant.
			auto const remaining_count = point_count - isize(desc.polytope_verts);
			for (auto pair_index = 0; pair_index != remaining_count / 2; ++pair_index)
			{
				auto const direction = RandomDirection(rng);
				desc.polytope_verts.push_back((direction * radius(rng) * aspect).w1());
				desc.polytope_verts.push_back((-direction * radius(rng) * aspect).w1());
			}
			if ((remaining_count & 1) != 0)
				desc.polytope_verts.push_back((RandomDirection(rng) * radius(rng) * aspect).w1());

			return desc;
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
			if (shape_type == "polytope")
			{
				auto desc = BodyDesc{};
				desc.shape_type = BodyDesc::EShape::Polytope;

				auto const& vertices = jshape_obj["vertices"].to_array();
				if (vertices.size() < 4)
					throw std::runtime_error("Polytope shape requires at least 4 non-coplanar vertices");

				desc.polytope_verts.reserve(vertices.size());
				for (auto const& vertex : vertices)
					desc.polytope_verts.push_back(ReadVec3Range(vertex, 1.0f, selector, index, count, rng));

				return desc;
			}
			if (shape_type == "random_convex")
				return ReadRandomConvexShape(jshape_obj, index, count, rng);

			return ReadShape(shape);
		}

		// Apply a uniform scale after selecting the base shape so named polytopes and primitive
		// dimensions use the same finite palette mechanism.
		void ScaleShape(BodyDesc& shape, float scale)
		{
			if (!(scale > 0.0f) || !std::isfinite(scale))
				throw std::runtime_error("Body generator 'scale' must be finite and positive");

			shape.box_dimensions *= scale;
			shape.sphere_radius *= scale;
			shape.line_length *= scale;
			shape.line_thickness *= scale;
			for (auto& vertex : shape.tri_verts)
				vertex = (vertex * scale).w1();
			for (auto& vertex : shape.polytope_verts)
				vertex = (vertex * scale).w1();

			// Compound children scale about the body origin, so their placements scale with their shapes.
			for (auto& child : shape.compound_children)
			{
				ScaleShape(child, scale);
				child.position = (child.position * scale).w1();
			}
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

		// Append generated rigid bodies.
		void AppendGeneratedBodies(SceneDesc& desc, pr::json::Value const& jv_generator, NamedShapeMap const& shapes, std::default_random_engine& rng)
		{
			auto const& jgen = jv_generator.to_object();
			auto selector = ReadGeneratorSelector(jgen);
			auto instance_count = 1;
			if (auto const* jcount = jgen.find("instance_count"))
				instance_count = jcount->to<int>();
			if (instance_count <= 0)
				throw std::runtime_error("Body generator 'instance_count' must be greater than zero");

			auto unique_shapes = false;
			if (auto const* junique_shapes = jgen.find("unique_shapes"))
				unique_shapes = junique_shapes->to<bool>();

			auto const* jshape = jgen.find("shape");
			if (jshape == nullptr)
				throw std::runtime_error("Body generator requires a 'shape' field");

			auto shape_palette_count = std::min(instance_count, 16);
			if (auto const* jpalette_count = jgen.find("shape_palette_count"))
				shape_palette_count = jpalette_count->to<int>();
			if (shape_palette_count <= 0)
				throw std::runtime_error("Body generator 'shape_palette_count' must be greater than zero");

			// Unique mode intentionally builds one descriptor per body so scene loading exercises
			// collision-shape construction and derived-data caching for non-shared geometry.
			shape_palette_count = unique_shapes ? instance_count : std::min(shape_palette_count, instance_count);

			auto shape_palette = std::vector<BodyDesc>{};
			shape_palette.reserve(shape_palette_count);
			for (auto shape_index = 0; shape_index != shape_palette_count; ++shape_index)
			{
				auto shape = ReadGeneratorShape(*jshape, shapes, selector, shape_index, shape_palette_count, rng);
				if (auto const* jscale = jgen.find("scale"))
				{
					auto const scale = ReadFloatRange(*jscale, EGeneratorSelector::Linear, shape_index, shape_palette_count, rng);
					ScaleShape(shape, scale);
				}
				shape_palette.push_back(std::move(shape));
			}

			auto const* jmass = jgen.find("mass");
			auto const* jdensity = jgen.find("density");
			if (jmass != nullptr && jdensity != nullptr)
				throw std::runtime_error("Body generator 'mass' and 'density' are mutually exclusive");

			auto name = std::string{};
			if (auto const* jname = jgen.find("name"))
				name = jname->to<std::string>();

			for (auto body_index = 0; body_index != instance_count; ++body_index)
			{
				auto palette_index = unique_shapes
					? body_index
					: SelectPaletteIndex(selector, body_index, instance_count, shape_palette_count, rng);
				auto body = shape_palette[palette_index];
				body.name = GeneratedName(name, body_index, instance_count);

				if (auto const* jcolour = jgen.find("colour"))
					body.colour = ReadColourRange(*jcolour, selector, body_index, instance_count, rng);
				if (jmass != nullptr)
					body.mass = ReadFloatRange(*jmass, selector, body_index, instance_count, rng);
				if (jdensity != nullptr)
				{
					auto const density = ReadFloatRange(*jdensity, selector, body_index, instance_count, rng);
					if (!(density > 0.0f) || !std::isfinite(density))
						throw std::runtime_error("Body generator 'density' must be finite and positive");

					body.density = density;
				}
				if (auto const* jposition = jgen.find("position"))
					body.position = ReadVec3Range(*jposition, 1.0f, selector, body_index, instance_count, rng);
				if (auto const* jrotation = jgen.find("rotation"))
					body.rotation = ReadVec3Range(*jrotation, 0.0f, selector, body_index, instance_count, rng);
				if (auto const* jz_axis = jgen.find("z_axis"))
				{
					auto const axis = ReadVec3Range(*jz_axis, 0.0f, selector, body_index, instance_count, rng);
					if (!IsFinite(axis) || !(LengthSq(axis) > Sqr(math::tiny<float>)))
						throw std::runtime_error("Body generator 'z_axis' requires finite non-zero directions");

					body.z_axis = Normalise(axis);
				}
				if (jgen.find("rotation") != nullptr && body.z_axis)
					throw std::runtime_error("Body generator 'rotation' and 'z_axis' are mutually exclusive");
				if (auto const* jvelocity = jgen.find("velocity"))
					body.velocity = ReadVec3Range(*jvelocity, 0.0f, selector, body_index, instance_count, rng);
				if (auto const* jangular_velocity = jgen.find("angular_velocity"))
					body.angular_velocity = ReadVec3Range(*jangular_velocity, 0.0f, selector, body_index, instance_count, rng);
				if (auto const* jsleeping = jgen.find("sleeping"))
					body.sleeping = jsleeping->to<bool>();
				if (auto const* jnever_sleep = jgen.find("never_sleep"))
					body.never_sleep = jnever_sleep->to<bool>();
				if (body.sleeping && body.never_sleep)
					throw std::runtime_error("A generated body cannot start sleeping and be marked never_sleep");

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
		if (auto* jdensity = jbody.find("density"))
		{
			if (jbody.find("mass") != nullptr)
				throw std::runtime_error("Body 'mass' and 'density' are mutually exclusive");

			auto const density = jdensity->to<float>();
			if (!(density > 0.0f) || !std::isfinite(density))
				throw std::runtime_error("Body 'density' must be finite and positive");

			desc.density = density;
		}

		// Position
		if (auto* jpos = jbody.find("position"))
			desc.position = ReadVec3(*jpos, 1.0f);

		// Rotation — Euler angles in degrees, applied in X, Y, Z order
		if (auto* jrot = jbody.find("rotation"))
			desc.rotation = ReadVec3(*jrot, 0.0f);

		// Axis alignment is a concise, stable representation for rods whose endpoints define their orientation.
		if (auto* jz_axis = jbody.find("z_axis"))
		{
			if (jbody.find("rotation") != nullptr)
				throw std::runtime_error("Body 'rotation' and 'z_axis' are mutually exclusive");

			auto const axis = ReadVec3(*jz_axis, 0.0f);
			if (!IsFinite(axis) || !(LengthSq(axis) > Sqr(math::tiny<float>)))
				throw std::runtime_error("Body 'z_axis' requires a finite non-zero direction");

			desc.z_axis = Normalise(axis);
		}

		// Velocity (optional, defaults to zero)
		if (auto* jvel = jbody.find("velocity"))
			desc.velocity = ReadVec3(*jvel, 0.0f);

		// Angular velocity (optional, defaults to zero)
		if (auto* javl = jbody.find("angular_velocity"))
			desc.angular_velocity = ReadVec3(*javl, 0.0f);

		// Initial sleep state
		if (auto* jsleeping = jbody.find("sleeping"))
			desc.sleeping = jsleeping->to<bool>();

		// Automatic sleep immunity keeps deliberately persistent motion active.
		if (auto* jnever_sleep = jbody.find("never_sleep"))
			desc.never_sleep = jnever_sleep->to<bool>();
		if (desc.sleeping && desc.never_sleep)
			throw std::runtime_error("A body cannot start sleeping and be marked never_sleep");

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

	// Parse a camera definition from a JSON object.
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

	// Parse one canonical terrain source shared by rendering and sampled GPU collision.
	TerrainDesc ReadTerrain(pr::json::Value const& jterrain)
	{
		// Apply optional source identity, preview bounds, and collision density over the shared defaults.
		auto terrain = TerrainDesc{};
		auto const& jterrain_obj = jterrain.to_object();
		if (auto const* jseed = jterrain_obj.find("seed"))
			terrain.surface.m_seed = static_cast<uint32_t>(jseed->to<int64_t>());
		if (auto const* jmaterial = jterrain_obj.find("material_id"))
			terrain.surface.m_material_id = jmaterial->to<int>();
		if (auto const* jcentre = jterrain_obj.find("centre"))
			terrain.centre_xy = ReadVec2d(*jcentre);
		if (auto const* jradius = jterrain_obj.find("radius"))
			terrain.radius_m = jradius->to<double>();
		if (auto const* jintervals = jterrain_obj.find("intervals"))
			terrain.intervals = jintervals->to<int>();
		if (auto const* spacing = jterrain_obj.find("surface_spacing"))
			terrain.surface_spacing = spacing->to<float>();

		// Recipe fields map directly to the owning evaluator's validated configuration.
		if (auto const* recipe = jterrain_obj.find("recipe"))
		{
			auto const& obj = recipe->to_object();

			// Retain the configured default when a scalar property is absent.
			auto scalar = [](auto const& source, char const* name, auto& value)
			{
				if (auto const* field = source.find(name))
					value = field->template to<std::remove_reference_t<decltype(value)>>();
			};

			// Apply the common octave parameters to any explicitly supplied spatial band.
			auto band = [&](char const* name, auto& config)
			{
				if (auto const* field = obj.find(name))
				{
					auto const& source = field->to_object();
					scalar(source, "amplitude", config.m_amplitude);
					scalar(source, "wavelength", config.m_wavelength_m);
					scalar(source, "octaves", config.m_octave_count);
					scalar(source, "lacunarity", config.m_lacunarity);
					scalar(source, "persistence", config.m_persistence);
				}
			};

			// Map shared family controls before applying the optional ridge and warp-specific parameters.
			scalar(obj, "sea_level_bias", terrain.surface.m_sea_level_bias_m);
			scalar(obj, "uplift_height", terrain.surface.m_uplift_height_m);
			scalar(obj, "mountain_base_height", terrain.surface.m_mountain_base_height_m);
			scalar(obj, "basin_depth", terrain.surface.m_basin_depth_m);
			scalar(obj, "basin_threshold", terrain.surface.m_basin_threshold);
			scalar(obj, "supported_coordinate_abs", terrain.surface.m_supported_coordinate_abs_m);
			band("regional_base", terrain.surface.m_regional_base);
			band("region_selector", terrain.surface.m_region_selector);
			band("region_uplift", terrain.surface.m_region_uplift);
			band("plains", terrain.surface.m_plains);
			band("hills", terrain.surface.m_hills);
			band("mountains", terrain.surface.m_mountains);
			band("basin_selector", terrain.surface.m_basin_selector);
			if (auto const* field = obj.find("mountains"))
			{
				scalar(field->to_object(), "roundness", terrain.surface.m_mountains.m_roundness);
				scalar(field->to_object(), "weight_gain", terrain.surface.m_mountains.m_weight_gain);
			}
			if (auto const* field = obj.find("domain_warp"))
			{
				auto const& source = field->to_object();
				scalar(source, "amplitude", terrain.surface.m_domain_warp.m_amplitude_m);
				scalar(source, "wavelength", terrain.surface.m_domain_warp.m_wavelength_m);
				scalar(source, "octaves", terrain.surface.m_domain_warp.m_octave_count);
				scalar(source, "lacunarity", terrain.surface.m_domain_warp.m_lacunarity);
				scalar(source, "persistence", terrain.surface.m_domain_warp.m_persistence);
			}
		}

		// Reject unusable collision density and evaluator recipes before scene creation allocates resources.
		if (!std::isfinite(terrain.surface_spacing) || terrain.surface_spacing <= 0)
			throw std::runtime_error("Terrain surface_spacing must be finite and positive");

		// Delegate field-domain and octave validation to the canonical source constructor.
		static_cast<void>(pr::physics::terrain::landscape::BaselineSurface(terrain.surface));

		// Display choices affect the preview only, not the physical terrain recipe.
		if (auto const* jdisplay = jterrain_obj.find("display"))
		{
			auto const display = jdisplay->to<std::string>();
			if (display == "neutral")
				terrain.display = ETerrainDisplayMode::Neutral;
			else if (display == "elevation")
				terrain.display = ETerrainDisplayMode::Elevation;
			else if (display == "slope")
				terrain.display = ETerrainDisplayMode::Slope;
			else
				throw std::runtime_error(std::format("Unknown terrain display mode '{}'", display));
		}

		// Keep the requested preview mesh within its basic radius and grid-size preconditions.
		if (!std::isfinite(terrain.radius_m) || terrain.radius_m <= 0.0)
			throw std::runtime_error("Terrain radius must be finite and greater than zero");
		if (terrain.intervals < 4)
			throw std::runtime_error("Terrain intervals must be at least 4");

		// Return the validated source and preview settings without creating scene resources.
		return terrain;
	}
	// Parse the water surface used by buoyancy and the sandbox visual mesh.
	WaterDesc ReadWater(pr::json::Value const& jwater)
	{
		auto water = WaterDesc{};
		auto const& jwater_obj = jwater.to_object();

		auto level = 0.0;
		if (auto const* jlevel = jwater_obj.find("level"))
			level = jlevel->to<double>();

		if (auto const* jsize = jwater_obj.find("size"))
		{
			water.size = ReadVec2(*jsize);
			if (water.size.x < 0.0f || water.size.y < 0.0f)
				throw std::runtime_error("Water size components must be non-negative");
			if ((water.size.x == 0.0f) != (water.size.y == 0.0f))
				throw std::runtime_error("Water size components must both be zero or both be positive");
		}

		if (auto const* jgrid = jwater_obj.find("grid"))
		{
			water.grid = ReadInt2(*jgrid);
			if (water.grid.x < 1 || water.grid.y < 1)
				throw std::runtime_error("Water grid components must be at least 1");
		}

		if (auto const* jcolour = jwater_obj.find("colour"))
			water.colour = ReadColour(*jcolour);

		// Waves are sine elements whose 'phase_speed' is the angular frequency in rad/s.
		auto elements = std::vector<physics::terrain::water::WaterFieldElement>{};
		if (auto const* jwaves = jwater_obj.find("waves"))
		{
			for (auto const& jwave : jwaves->to_array())
			{
				auto const& jwave_obj = jwave.to_object();
				auto const* jdirection = jwave_obj.find("direction");
				if (jdirection == nullptr)
					throw std::runtime_error("Water wave requires a 'direction' field");

				auto const* jwavelength = jwave_obj.find("wavelength");
				auto const* jperiod = jwave_obj.find("period");
				if (jwavelength != nullptr && jperiod != nullptr)
					throw std::runtime_error("Water wave cannot specify both 'wavelength' and 'period'");
				if (jwavelength == nullptr && jperiod == nullptr)
					throw std::runtime_error("Water wave requires a 'wavelength' or 'period' field");

				auto const* jamplitude = jwave_obj.find("amplitude");
				if (jamplitude == nullptr)
					throw std::runtime_error("Water wave requires an 'amplitude' field");

				auto const* jphase_speed = jwave_obj.find("phase_speed");
				elements.push_back(physics::terrain::water::SineWave(
					ReadVec2(*jdirection),
					jamplitude->to<float>(),
					(jwavelength != nullptr ? jwavelength : jperiod)->to<float>(),
					jphase_speed != nullptr ? jphase_speed->to<float>() : 0.0f
				));
			}
		}

		water.surface = physics::terrain::water::WaterField(level, elements);
		return water;
	}


	// Parse a side-boundary value for the atmosphere block.
	physics::atmosphere::EAtmosphereBoundary ReadAtmosphereBoundary(pr::json::Value const& jboundary)
	{
		// Text keeps scene files readable while mapping to the physics API enum at the parser boundary.
		auto const value = jboundary.to<std::string>();
		if (value == "solid")
			return physics::atmosphere::EAtmosphereBoundary::Solid;
		if (value == "open")
			return physics::atmosphere::EAtmosphereBoundary::Open;
		throw std::runtime_error(std::format("Unknown atmosphere boundary '{}'", value));
	}

	// Level the floor near the open sides of 'config'. Columns within 'm_open_edge_band' of an open side are set to the mean floor height of
	// those columns, and the next band blends smoothly back to the original floor. Solid columns are not changed.
	static void LevelOpenEdgeFloor(physics::atmosphere::AtmosphereConfig& config)
	{
		// Find each column's distance, in columns, to the nearest open side.
		using EBoundary = physics::atmosphere::EAtmosphereBoundary;
		auto& grid = config.m_grid;
		auto const& sides = config.m_boundaries;
		auto const band = config.m_open_edge_band;
		auto const nx = grid.m_cell_count.x;
		auto const ny = grid.m_cell_count.y;
		auto EdgeDistance = [&](int x, int y)
		{
			// Sides that are not open do not constrain the floor.
			auto d = std::numeric_limits<int>::max();
			if (sides.m_x_min == EBoundary::Open) d = std::min(d, x);
			if (sides.m_x_max == EBoundary::Open) d = std::min(d, nx - 1 - x);
			if (sides.m_y_min == EBoundary::Open) d = std::min(d, y);
			if (sides.m_y_max == EBoundary::Open) d = std::min(d, ny - 1 - y);
			return d;
		};

		// The level height is the mean floor of the air columns inside the band, so the levelled region keeps about the same air volume.
		auto sum = 0.0;
		auto count = 0;
		for (int y = 0; y != ny; ++y)
		{
			for (int x = 0; x != nx; ++x)
			{
				// Solid columns hold no air and are excluded.
				auto const floor = grid.m_floor_heights[y * nx + x];
				if (EdgeDistance(x, y) >= band || floor >= grid.m_lid_z)
					continue;

				sum += floor;
				++count;
			}
		}
		if (count == 0)
			return;

		// Blend each air column toward the level height. The weight is 1 inside the band and eases to 0 across the next band.
		auto const level = static_cast<float>(sum / count);
		for (int y = 0; y != ny; ++y)
		{
			for (int x = 0; x != nx; ++x)
			{
				// Leave solid columns, and columns beyond the blend band, unchanged.
				auto& floor = grid.m_floor_heights[y * nx + x];
				auto const d = EdgeDistance(x, y);
				if (d >= 2 * band || floor >= grid.m_lid_z)
					continue;

				auto const t = std::clamp(static_cast<float>(d - band + 1) / static_cast<float>(band + 1), 0.0f, 1.0f);
				auto const weight = 1.0f - t * t * (3.0f - 2.0f * t);
				floor += weight * (level - floor);
			}
		}
	}

	// Parse the optional GPU atmosphere solver and visualisation block. 'terrain' and 'water' are the scene's parsed blocks, or null when absent.
	// They are needed when the atmosphere floor follows the scene terrain.
	AtmosphereDesc ReadAtmosphere(pr::json::Value const& jatmosphere, TerrainDesc const* terrain, WaterDesc const* water)
	{
		// The first atmosphere scene is flat-floored, but the parsed structures mirror the reusable solver API.
		auto desc = AtmosphereDesc{};
		auto const& obj = jatmosphere.to_object();
		auto& config = desc.m_config;
		config.m_grid = physics::atmosphere::AtmosphereGrid{ .m_cell_count = iv3{64, 64, 8}, .m_origin = v4{-640.0f, -640.0f, 0.0f, 1.0f}, .m_dx = 20.0f, .m_lid_z = 400.0f, .m_first_layer_thickness = 8.0f, .m_layer_stretch_power = 0.75f };
		config.m_reference = physics::atmosphere::AtmosphereReferenceProfile{ .m_temperature_at_origin = 288.0f, .m_lapse_rate = -0.0065f, .m_min_temperature = 220.0f };

		if (auto const* grid = obj.find("grid"))
		{
			auto const& jgrid = grid->to_object();
			if (auto const* value = jgrid.find("cell_count"))
				config.m_grid.m_cell_count = ReadInt3(*value);
			if (auto const* value = jgrid.find("origin"))
				config.m_grid.m_origin = ReadVec3(*value, 1.0f);
			if (auto const* value = jgrid.find("dx"))
				config.m_grid.m_dx = value->to<float>();
			if (auto const* value = jgrid.find("lid_z"))
				config.m_grid.m_lid_z = value->to<float>();
			if (auto const* value = jgrid.find("first_layer_thickness"))
				config.m_grid.m_first_layer_thickness = value->to<float>();
			if (auto const* value = jgrid.find("layer_stretch_power"))
				config.m_grid.m_layer_stretch_power = value->to<float>();
		}

		if (auto const* boundaries = obj.find("boundaries"))
		{
			auto const& b = boundaries->to_object();
			if (auto const* value = b.find("x_min")) config.m_boundaries.m_x_min = ReadAtmosphereBoundary(*value);
			if (auto const* value = b.find("x_max")) config.m_boundaries.m_x_max = ReadAtmosphereBoundary(*value);
			if (auto const* value = b.find("y_min")) config.m_boundaries.m_y_min = ReadAtmosphereBoundary(*value);
			if (auto const* value = b.find("y_max")) config.m_boundaries.m_y_max = ReadAtmosphereBoundary(*value);
			if (auto const* value = b.find("z_min")) config.m_boundaries.m_z_min = ReadAtmosphereBoundary(*value);
			if (auto const* value = b.find("z_max")) config.m_boundaries.m_z_max = ReadAtmosphereBoundary(*value);
		}

		if (auto const* wall_drag = obj.find("wall_drag"))
		{
			auto const& d = wall_drag->to_object();
			if (auto const* value = d.find("x_min")) config.m_wall_drag.m_x_min = value->to<float>();
			if (auto const* value = d.find("x_max")) config.m_wall_drag.m_x_max = value->to<float>();
			if (auto const* value = d.find("y_min")) config.m_wall_drag.m_y_min = value->to<float>();
			if (auto const* value = d.find("y_max")) config.m_wall_drag.m_y_max = value->to<float>();
			if (auto const* value = d.find("z_min")) config.m_wall_drag.m_z_min = value->to<float>();
			if (auto const* value = d.find("z_max")) config.m_wall_drag.m_z_max = value->to<float>();
		}

		if (auto const* reference = obj.find("reference"))
		{
			auto const& r = reference->to_object();
			if (auto const* value = r.find("temperature_at_origin")) config.m_reference.m_temperature_at_origin = value->to<float>();
			if (auto const* value = r.find("lapse_rate")) config.m_reference.m_lapse_rate = value->to<float>();
			if (auto const* value = r.find("min_temperature")) config.m_reference.m_min_temperature = value->to<float>();
		}

		if (auto const* value = obj.find("open_edge_band"))
			config.m_open_edge_band = value->to<int>();
		if (auto const* value = obj.find("vorticity_confinement"))
			config.m_vorticity_confinement = value->to<float>();
		if (auto const* value = obj.find("vertical_viscosity"))
			config.m_vertical_viscosity = value->to<float>();

		// Outside air is authored as rectangles in the horizontal plane. Positions outside every rectangle get calm, reference-temperature air.
		// 'wind_noise' adds a fixed pseudo-random offset to each boundary column's wind so symmetric flows have a disturbance to grow from.
		struct OutsideAirRegion
		{
			v2 m_min;
			v2 m_max;
			float m_wind_noise;
			physics::atmosphere::AtmosphereOutsideAir m_air;
		};
		auto outside_air_regions = std::vector<OutsideAirRegion>{};
		if (auto const* regions = obj.find("outside_air"))
		{
			for (auto const& jregion : regions->to_array())
			{
				// Read one region; omitted bounds cover the whole plane.
				auto const& r = jregion.to_object();
				auto region = OutsideAirRegion{ .m_min = v2{ -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max() }, .m_max = v2{ std::numeric_limits<float>::max(), std::numeric_limits<float>::max() }, .m_wind_noise = 0.0f, .m_air = {} };
				if (auto const* value = r.find("min")) region.m_min = ReadVec2(*value);
				if (auto const* value = r.find("max")) region.m_max = ReadVec2(*value);
				if (auto const* value = r.find("wind")) region.m_air.m_wind = ReadVec2(*value);
				if (auto const* value = r.find("wind_noise")) region.m_wind_noise = value->to<float>();
				if (auto const* value = r.find("temperature_offset")) region.m_air.m_temperature_offset = value->to<float>();
				outside_air_regions.push_back(region);
			}
		}

		if (auto const* sources = obj.find("heat_sources"))
		{
			for (auto const& jsource : sources->to_array())
			{
				auto const& s = jsource.to_object();
				auto source = physics::atmosphere::AtmosphereHeatSource{};
				if (auto const* value = s.find("centre")) source.m_centre = ReadVec3(*value, 1.0f);
				if (auto const* value = s.find("radius")) source.m_radius = value->to<float>();
				if (auto const* value = s.find("heating_rate")) source.m_heating_rate = value->to<float>();
				if (auto const* value = s.find("target_temperature")) source.m_target_temperature = value->to<float>();
				if (auto const* value = s.find("relaxation_rate")) source.m_relaxation_rate = value->to<float>();
				desc.m_heat_sources.push_back(source);
			}
		}

		if (auto const* cylinders = obj.find("cylinders"))
		{
			for (auto const& jcylinder : cylinders->to_array())
			{
				// Read one floor-to-lid cylinder.
				auto const& c = jcylinder.to_object();
				auto cylinder = AtmosphereCylinderDesc{};
				if (auto const* value = c.find("centre")) cylinder.m_centre = ReadVec2(*value);
				if (auto const* value = c.find("radius")) cylinder.m_radius = value->to<float>();
				desc.m_cylinders.push_back(cylinder);
			}
		}

		// The step rate sets the solver time step, so it must be positive.
		if (auto const* value = obj.find("step_rate"))
		{
			desc.m_step_rate = value->to<float>();
			if (!(desc.m_step_rate > 0.0f))
				throw std::runtime_error("atmosphere.step_rate must be positive");
		}

		if (auto const* tracers = obj.find("tracers"))
		{
			auto const& t = tracers->to_object();
			if (auto const* value = t.find("count")) desc.m_tracers.m_particle_count = value->to<int>();
			if (auto const* value = t.find("seed")) desc.m_tracers.m_seed = static_cast<uint32_t>(value->to<int64_t>());
			if (auto const* value = t.find("max_age")) desc.m_tracers.m_max_age = value->to<float>();
			if (auto const* value = t.find("ground_density")) desc.m_tracers.m_ground_density = value->to<float>();
			if (auto const* value = t.find("break_density")) desc.m_tracers.m_break_density = value->to<float>();
			if (auto const* value = t.find("upper_density")) desc.m_tracers.m_upper_density = value->to<float>();
			if (auto const* value = t.find("break_height")) desc.m_tracers.m_break_height = value->to<float>();
		}

		if (auto const* visual = obj.find("visual"))
		{
			auto const& v = visual->to_object();
			if (auto const* value = v.find("colour_by"))
			{
				auto const mode = value->to<std::string>();
				if (mode == "temperature")
					desc.m_visual.m_colour_by = EAtmosphereColourBy::Temperature;
				else if (mode == "speed")
					desc.m_visual.m_colour_by = EAtmosphereColourBy::Speed;
				else
					throw std::runtime_error(std::format("Unknown atmosphere colour_by '{}'", mode));
			}
			if (auto const* value = v.find("temperature_range"))
			{
				auto const& range = value->to_array();
				desc.m_visual.m_min_temperature = range[0].to<float>();
				desc.m_visual.m_max_temperature = range[1].to<float>();
			}
			if (auto const* value = v.find("speed_range"))
			{
				auto const& range = value->to_array();
				desc.m_visual.m_min_speed = range[0].to<float>();
				desc.m_visual.m_max_speed = range[1].to<float>();
			}
			if (auto const* value = v.find("particle_size")) desc.m_visual.m_particle_size = value->to<float>();
			if (auto const* value = v.find("grid_line_limit")) desc.m_visual.m_grid_line_limit = value->to<int>();
			if (auto const* value = v.find("show_grid")) desc.m_visual.m_show_grid = value->to<bool>();
			if (auto const* value = v.find("show_particles")) desc.m_visual.m_show_particles = value->to<bool>();
			if (auto const* value = v.find("show_heat_sources")) desc.m_visual.m_show_heat_sources = value->to<bool>();
			if (auto const* value = v.find("show_obstacles")) desc.m_visual.m_show_obstacles = value->to<bool>();
		}

		// The floor is either flat at the grid origin or follows the scene terrain. Over water the floor is the water surface, so lakes are flat.
		if (auto const* value = obj.find("floor"))
		{
			auto const mode = value->to<std::string>();
			if (mode == "terrain")
				desc.m_terrain_floor = true;
			else if (mode != "flat")
				throw std::runtime_error(std::format("Unknown atmosphere floor '{}'", mode));
		}
		if (desc.m_terrain_floor && terrain == nullptr)
			throw std::runtime_error("atmosphere.floor 'terrain' needs a scene 'terrain' block");

		// Cylinders become solid columns by raising the floor of every column whose centre lies inside one up to the lid.
		if (desc.m_terrain_floor || !desc.m_cylinders.empty())
		{
			// The terrain source is evaluated once per column, so it only lives for the build.
			auto const& grid = config.m_grid;
			auto const surface = desc.m_terrain_floor ? std::make_optional<pr::physics::terrain::landscape::BaselineSurface>(terrain->surface) : std::nullopt;
			config.m_grid.m_floor_heights = physics::atmosphere::AtmosphereGrid::BuildFloorHeights(iv2{ grid.m_cell_count.x, grid.m_cell_count.y }, grid.m_origin, grid.m_dx, [&](v2 pos)
			{
				// Cylinders reach the lid wherever they are.
				for (auto const& cylinder : desc.m_cylinders)
				{
					if (LengthSq(pos - cylinder.m_centre) <= cylinder.m_radius * cylinder.m_radius)
						return grid.m_lid_z;
				}
				if (!surface)
					return grid.m_origin.z;

				// Air rests on the higher of the ground and the still water surface.
				auto height = surface->Sample(pr::physics::terrain::v2d{ pos.x, pos.y }).m_height;
				if (water != nullptr)
					height = std::max(height, water->surface.Level());

				return static_cast<float>(height);
			});

			// Uniform outside wind needs level ground near open sides. See AtmosphereConfig::m_open_edge_band. Level the terrain floor
			// across the band of each open side, then blend back to the terrain over the next band so the air floor has no step.
			// The visual and collision terrain is not changed, so the air floor departs from it near open sides.
			if (desc.m_terrain_floor && config.m_open_edge_band > 0)
				LevelOpenEdgeFloor(config);
		}

		config.Validate();
		desc.m_tracers.Validate();

		// Sample the authored regions once per boundary column. The scene's outside air does not change over time.
		auto column = uint32_t{};
		desc.m_outside_air = config.m_grid.BuildOutsideAir([&](v2 pos)
		{
			// Each call is the next boundary column, so the column index seeds that column's repeatable wind noise.
			auto noise = [seed = column++](uint32_t channel)
			{
				// Mix the column and channel bits, then map the hash to [-1, 1].
				auto h = seed * 0x9E3779B9u ^ channel * 0x85EBCA6Bu;
				h ^= h >> 16;
				h *= 0x7FEB352Du;
				h ^= h >> 15;
				h *= 0x846CA68Bu;
				h ^= h >> 16;
				return static_cast<float>(h) / static_cast<float>(std::numeric_limits<uint32_t>::max()) * 2.0f - 1.0f;
			};

			// The first region containing the sample position supplies the air.
			for (auto const& region : outside_air_regions)
			{
				// Bounds are inclusive so a region edge on the domain edge still applies.
				if (pos.x < region.m_min.x || pos.x > region.m_max.x || pos.y < region.m_min.y || pos.y > region.m_max.y)
					continue;

				auto air = region.m_air;
				air.m_wind += region.m_wind_noise * v2{ noise(0), noise(1) };
				return air;
			}
			return physics::atmosphere::AtmosphereOutsideAir{};
		});
		return desc;
	}

	// Parse a scene description from a JSON file
	SceneDesc LoadFromFile(std::filesystem::path const& filepath)
	{
		auto doc = pr::json::Read(filepath, json::Options{.AllowComments = true, .AllowTrailingCommas = true});
		auto const& jscene = doc.to_object()["scene"].to_object();

		SceneDesc desc;
		desc.filepath = filepath;

		// Runtime catalogue metadata keeps menu grouping and labels editable with the scene.
		if (auto const* jdemo = jscene.find("demo"))
		{
			auto const& demo = jdemo->to_object();
			auto metadata = SceneMetadata{};
			if (auto const* field = demo.find("command"))
				metadata.m_command = field->to<std::string>();
			if (auto const* field = demo.find("name"))
				metadata.m_name = field->to<std::string>();
			if (auto const* field = demo.find("group"))
				metadata.m_group = field->to<std::string>();
			if (auto const* field = demo.find("order"))
				metadata.m_order = field->to<int>();
			if (auto const* field = jscene.find("description"))
				metadata.m_description = field->to<std::string>();
			if (metadata.m_name.empty() || metadata.m_group.empty())
				throw std::runtime_error("Scene demo metadata requires non-empty 'name' and 'group' fields");

			desc.metadata = std::move(metadata);
		}

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
				auto const max_substeps = physics::EngineConfig{}.max_internal_substeps;
				if (desc.physics_substeps < 1 || desc.physics_substeps > max_substeps)
					throw std::runtime_error(std::format("Scene physics.substeps must be in the range [1, {}]", max_substeps));
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

		// Terrain preview
		if (auto* jterrain = jscene.find("terrain"))
			desc.terrain = ReadTerrain(*jterrain);

		// Water surface
		if (auto* jwater = jscene.find("water"))
			desc.water = ReadWater(*jwater);

		// Atmosphere solver
		if (auto* jatmosphere = jscene.find("atmosphere"))
			desc.atmosphere = ReadAtmosphere(*jatmosphere, desc.terrain ? &*desc.terrain : nullptr, desc.water ? &*desc.water : nullptr);

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

		// Articulations and constraints share the same named-shape namespace as rigid bodies.
		AppendMultibodyDescriptions(desc, jscene, [&](pr::json::Value const& value)
		{
			return ReadBody(value, shapes);
		});

		// Camera
		if (auto* jcamera = jscene.find("camera"))
		{
			desc.camera = ReadCamera(*jcamera);
		}

		return desc;
	}

	// Read only the inexpensive menu metadata from a JSON scene file.
	std::optional<SceneMetadata> LoadMetadataFromFile(std::filesystem::path const& filepath)
	{
		auto const document = pr::json::Read(filepath, json::Options{.AllowComments = true, .AllowTrailingCommas = true});
		auto const& scene = document.to_object()["scene"].to_object();
		auto const* value = scene.find("demo");
		if (value == nullptr)
			return {};

		auto const& demo = value->to_object();
		auto const* name = demo.find("name");
		auto const* group = demo.find("group");
		if (name == nullptr || group == nullptr)
			throw std::runtime_error(pr::FmtS("Scene '%ls' has incomplete demo metadata", filepath.c_str()));

		auto metadata = SceneMetadata{};
		if (auto const* command = demo.find("command"))
			metadata.m_command = command->to<std::string>();
		metadata.m_name = name->to<std::string>();
		metadata.m_group = group->to<std::string>();
		if (auto const* description = scene.find("description"))
			metadata.m_description = description->to<std::string>();
		if (auto const* order = demo.find("order"))
			metadata.m_order = order->to<int>();
		if (metadata.m_name.empty() || metadata.m_group.empty())
			throw std::runtime_error(pr::FmtS("Scene '%ls' has empty demo metadata", filepath.c_str()));

		return metadata;
	}

	#if PR_UNITTESTS
	namespace tests
	{
		namespace
		{
			// Parse only the sandbox terrain block from an in-memory scene document.
			SceneDesc ParseTerrain(std::string_view text)
			{
				auto document = pr::json::Read(text);
				auto const& scene = document.to_object()["scene"].to_object();
				auto desc = SceneDesc{};
				if (auto const* jterrain = scene.find("terrain"))
					desc.terrain = ReadTerrain(*jterrain);
				return desc;
			}
		}

		PRUnitTestClass(SceneLoaderTerrainTests)
		{
			PRUnitTestMethod(ParsesTerrainPreviewSettings, Quick)
			{
				auto const desc = ParseTerrain(R"json(
				{
					"scene": {
						"terrain": {
							"seed": 77,
							"material_id": 2,
							"centre": [125.5, -88.25],
							"radius": 2048.0,
							"intervals": 96,
							"display": "slope"
						}
					}
				})json");

				PR_EXPECT(desc.terrain.has_value());
				PR_EXPECT(desc.terrain->surface.m_seed == 77u);
				PR_EXPECT(desc.terrain->surface.m_material_id == 2);
				PR_EXPECT(desc.terrain->centre_xy.x == 125.5);
				PR_EXPECT(desc.terrain->centre_xy.y == -88.25);
				PR_EXPECT(desc.terrain->radius_m == 2048.0);
				PR_EXPECT(desc.terrain->intervals == 96);
				PR_EXPECT(desc.terrain->display == ETerrainDisplayMode::Slope);
			}

			PRUnitTestMethod(RejectsInvalidTerrainPreviewSettings, Quick)
			{
				PR_THROWS(ParseTerrain(R"json({"scene":{"terrain":{"radius":0.0}}})json"), std::exception);
				PR_THROWS(ParseTerrain(R"json({"scene":{"terrain":{"intervals":3}}})json"), std::exception);
				PR_THROWS(ParseTerrain(R"json({"scene":{"terrain":{"display":"wireframe"}}})json"), std::exception);
			}

			// Collision spacing is independent of buoyancy; the visual and physical terrain share one validated recipe.
			PRUnitTestMethod(ParsesTerrainCollisionRecipe, Quick)
			{
				auto const defaults = ParseTerrain(R"json({"scene":{"terrain":{}}})json");
				PR_EXPECT(defaults.terrain->surface_spacing == physics::surface::DefaultSpacing);
				auto const desc = ParseTerrain(R"json({"scene":{"terrain":{"surface_spacing":0.2,"recipe":{"sea_level_bias":-3.0,"regional_base":{"amplitude":1.6,"wavelength":4.0,"octaves":1}}}}})json");
				PR_EXPECT(desc.terrain->surface_spacing == 0.2f);
				PR_EXPECT(desc.terrain->surface.m_sea_level_bias_m == -3.0);
				PR_EXPECT(desc.terrain->surface.m_regional_base.m_amplitude == 1.6);
				PR_EXPECT(desc.terrain->surface.m_regional_base.m_wavelength_m == 4.0);
				PR_EXPECT(desc.terrain->surface.m_regional_base.m_octave_count == 1);
			}

			// Reject unusable sampling and recipe inputs before a scene creates GPU resources.
			PRUnitTestMethod(RejectsInvalidTerrainCollisionSettings, Quick)
			{
				PR_THROWS(ParseTerrain(R"json({"scene":{"terrain":{"surface_spacing":0}}})json"), std::exception);
				PR_THROWS(ParseTerrain(R"json({"scene":{"terrain":{"surface_spacing":-0.1}}})json"), std::exception);
				PR_THROWS(ParseTerrain(R"json({"scene":{"terrain":{"recipe":{"regional_base":{"wavelength":0}}}}})json"), std::exception);
			}
		};

		PRUnitTestClass(SceneLoaderAtmosphereTests)
		{
			// A terrain floor rests on the ground, or on the water surface over lakes, and stays below the lid so every column holds air.
			// The demo's sides are all open, so the floor is level inside the edge band and follows the terrain beyond the blend band.
			PRUnitTestMethod(TerrainFloorFollowsGroundAndWater, Quick)
			{
				auto const desc = LoadFromFile("projects\\tests\\physics-sandbox\\scenes\\climate_terrain.json");
				PR_EXPECT(desc.atmosphere.has_value() && desc.atmosphere->m_terrain_floor);

				auto const& grid = desc.atmosphere->m_config.m_grid;
				auto const surface = pr::physics::terrain::landscape::BaselineSurface(desc.terrain->surface);
				auto const level = static_cast<float>(desc.water->surface.Level());
				auto const band = desc.atmosphere->m_config.m_open_edge_band;
				auto const edge_level = grid.FloorHeight(iv2{ 0, 0 });
				auto water_columns = 0;
				auto max_floor = -std::numeric_limits<float>::max();
				for (int y = 0; y != grid.m_cell_count.y; ++y)
				{
					for (int x = 0; x != grid.m_cell_count.x; ++x)
					{
						// Compare each column with the terrain at its centre.
						auto const centre = v2{ grid.m_origin.x + (x + 0.5f) * grid.m_dx, grid.m_origin.y + (y + 0.5f) * grid.m_dx };
						auto const ground = static_cast<float>(surface.Sample(pr::physics::terrain::v2d{ centre.x, centre.y }).m_height);
						auto const floor = grid.FloorHeight(iv2{ x, y });
						auto const edge = std::min({ x, y, grid.m_cell_count.x - 1 - x, grid.m_cell_count.y - 1 - y });
						if (edge < band)
							PR_EXPECT(FEql(floor, edge_level));
						else if (edge >= 2 * band)
							PR_EXPECT(FEql(floor, std::max(ground, level)));

						PR_EXPECT(!grid.ColumnSolid(iv2{ x, y }));
						water_columns += ground < level ? 1 : 0;
						max_floor = std::max(max_floor, floor);
					}
				}

				// The demo region is chosen to contain both lakes and high ground.
				PR_EXPECT(water_columns > 0);
				PR_EXPECT(max_floor > level + 200.0f);
			}

			// A terrain floor needs a terrain source.
			PRUnitTestMethod(TerrainFloorNeedsTerrain, Quick)
			{
				auto document = pr::json::Read(std::string_view{ R"json({"floor":"terrain"})json" });
				PR_THROWS(ReadAtmosphere(document, nullptr, nullptr), std::exception);
				auto unknown = pr::json::Read(std::string_view{ R"json({"floor":"bumpy"})json" });
				PR_THROWS(ReadAtmosphere(unknown, nullptr, nullptr), std::exception);
			}
		};
	}
	#endif
}
