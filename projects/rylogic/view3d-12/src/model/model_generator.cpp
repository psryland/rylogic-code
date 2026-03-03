//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/model/model_generator.h"
#include "pr/view3d-12/model/animation.h"
#include "pr/view3d-12/model/skin.h"
#include "pr/view3d-12/model/skeleton.h"
#include "pr/view3d-12/model/model_tree.h"
#include "pr/view3d-12/model/vertex_layout.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/resource/stock_resources.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "pr/view3d-12/texture/texture_2d.h"

namespace pr::rdr12
{
	// Implementation functions
	namespace model_generator
	{
		// Bake a transform into 'cache'
		template <typename VType>
		static void BakeTransform(ModelGenerator::Cache<VType>& cache, m4x4 const& a2b)
		{
			// Apply the 'bake' transform to every vertex
			cache.m_bbox = a2b * cache.m_bbox;
			for (auto& v : cache.m_vcont)
			{
				v.m_vert = a2b * v.m_vert;
				v.m_norm = a2b * v.m_norm;
			}

			// If the transform is left handed, flip the faces
			if (Determinant3(a2b) < 0)
			{
				// Check each nugget for faces
				for (auto& nug : cache.m_ncont)
				{
					switch (nug.m_topo)
					{
						case ETopo::TriList:
						{
							FlipTriListFaces<VType>(cache, nug.m_irange);
							break;
						}
						case ETopo::TriStrip:
						{
							FlipTriStripFaces<VType>(cache, nug.m_irange); break;
							break;
						}
					}
				}
			}
		}

		// Flip the winding order of faces in a triangle list
		template <typename VType>
		static void FlipTriListFaces(ModelGenerator::Cache<VType>& cache, Range irange)
		{
			assert(irange.size() % 3 == 0);
			auto iptr = cache.m_icont.template begin<int64_t>();
			for (int64_t i = irange.begin(), iend = irange.end(); i != iend; i += 3)
				swap(*(iptr + 1), *(iptr + 2));
		}

		// Flip the winding order of faces in a triangle strip
		template <typename VType>
		static void FlipTriStripFaces(ModelGenerator::Cache<VType>& cache, Range irange)
		{
			assert(irange.size() % 2 == 0);
			auto iptr = cache.m_icont.template begin<int64_t>();
			for (int64_t i = irange.begin(), iend = irange.end(); i != iend; i += 2)
				swap(*(iptr + 0), *(iptr + 1));
		}

		// Generate normals for the triangle list given by index range 'irange' in 'cache'
		template <typename VType>
		static void GenerateNormals(ModelGenerator::Cache<VType>& cache, Range irange, float gen_normals)
		{
			auto iptr = cache.m_icont.template begin<int64_t>() + s_cast<ptrdiff_t>(irange.begin());

			geometry::GenerateNormals(
				isize(irange), iptr, gen_normals, isize(cache.m_vcont),
				[&](int64_t idx)
				{
					return GetP(cache.m_vcont[s_cast<size_t>(idx)]);
				},
				[&](int64_t idx, int64_t orig, v4 const& norm)
				{
					assert(idx <= isize(cache.m_vcont));
					if (idx == isize(cache.m_vcont)) cache.m_vcont.push_back(cache.m_vcont[orig]);
					SetN(cache.m_vcont[s_cast<size_t>(idx)], norm);
				},
				[&](int64_t i0, int64_t i1, int64_t i2)
				{
					*iptr++ = i0;
					*iptr++ = i1;
					*iptr++ = i2;
				});
		}

		// Generate normals for the triangle list nuggets in 'cache'
		template <typename VType>
		static void GenerateNormals(ModelGenerator::Cache<VType>& cache, float gen_normals)
		{
			assert(gen_normals >= 0 && "Smoothing threshold must be a positive number");

			// Check each nugget for faces
			for (auto& nug : cache.m_ncont)
			{
				switch (nug.m_topo)
				{
					case ETopo::TriList:
					{
						GenerateNormals<VType>(cache, nug.m_irange, gen_normals);
						break;
					}
					case ETopo::TriStrip:
					{
						throw std::exception("Generate normals isn't supported for TriStrip");
					}
				}
			}
		}
	};

	// Create a model from 'cache'
	template <typename VType>
	ModelPtr ModelGenerator::Create(ResourceFactory& factory, ModelGenerator::Cache<VType>& cache, ModelGenerator::CreateOptions const* opts)
	{
		// Sanity check 'cache'
		assert("No nuggets given" && !cache.m_ncont.empty());
		for (auto& nug : cache.m_ncont)
		{
			// Invalid range means "full range"
			if (nug.m_vrange == Range::Reset())
				nug.m_vrange = Range(0, cache.VCount());
			if (nug.m_irange == Range::Reset())
				nug.m_irange = Range(0, cache.ICount());

			assert("Nugget range invalid" && nug.m_vrange.begin() >= 0 && nug.m_vrange.end() <= cache.VCount());
			assert("Nugget range invalid" && nug.m_irange.begin() >= 0 && nug.m_irange.end() <= cache.ICount());
		}

		// Bake a transform into the model
		if (opts && opts->has(ModelGenerator::CreateOptions::EOptions::BakeTransform))
			model_generator::BakeTransform(cache, opts->m_bake);

		// Generate normals
		if (opts && opts->has(ModelGenerator::CreateOptions::EOptions::NormalGeneration))
			model_generator::GenerateNormals(cache, opts->m_gen_normals);

		// Create the model
		ModelDesc mdesc = ModelDesc()
			.vbuf(ResDesc::VBuf<VType>(cache.VCount(), cache.m_vcont))
			.ibuf(ResDesc::IBuf(cache.ICount(), cache.m_icont.stride(), cache.m_icont))
			.bbox(cache.m_bbox)
			.m2root(cache.m_m2root)
			.name(cache.m_name.c_str());
		auto model = factory.CreateModel(mdesc);

		// Create the render nuggets
		for (auto& nug : cache.m_ncont)
		{
			// If the model geom has valid texture data but no texture..
			if (AllSet(nug.m_geom, EGeom::Tex0))
			{
				// Use the texture from the options
				if (nug.m_tex_diffuse == nullptr)
					nug.m_tex_diffuse = opts && opts->m_tex_diffuse != nullptr
						? opts->m_tex_diffuse
						: factory.rdr().store().StockTexture(EStockTexture::White);

				if (nug.m_sam_diffuse == nullptr)
					nug.m_sam_diffuse = opts && opts->m_sam_diffuse != nullptr
						? opts->m_sam_diffuse
						: factory.rdr().store().StockSampler(EStockSampler::AnisotropicWrap);
			}

			// Create the nugget
			model->CreateNugget(factory, nug);
		}

		// Return the freshly minted model.
		return model;
	}
	template ModelPtr ModelGenerator::Create<Vert>(ResourceFactory& factory, ModelGenerator::Cache<Vert>& cache, ModelGenerator::CreateOptions const* opts);

	// Points/Sprites *********************************************************************
	ModelPtr ModelGenerator::Points(ResourceFactory& factory, std::span<v4 const> points, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::PointSize(isize(points));
		auto colours = opts ? opts->m_colours : std::span<Colour32 const>{};
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::Points(points, colours,
			[&](v4 p, Colour32 c, auto, auto) { SetPC(*vptr++, p, Colour(c)); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::PointList, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}

	// Lines ******************************************************************************
	ModelPtr ModelGenerator::Lines(ResourceFactory& factory, int num_lines, std::span<v4 const> points, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::LineSize(num_lines);
		auto colours = opts ? opts->m_colours : std::span<Colour32 const>{};
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::Lines(num_lines, points, colours,
			[&](v4 p, Colour32 c, auto, auto) { SetPC(*vptr++, p, Colour(c)); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::LineList, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::LinesD(ResourceFactory& factory, int num_lines, std::span<v4 const> points, std::span<v4 const> directions, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::LineSize(num_lines);
		auto colours = opts ? opts->m_colours : std::span<Colour32 const>{};
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();
		assert(vcount == isize(points));
		assert(vcount == isize(directions));

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::LinesD(num_lines, points.data(), directions.data(), colours,
			[&](v4 p, Colour32 c, auto, auto) { SetPC(*vptr++, p, Colour(c)); },
			[&](int idx) { *iptr++ = idx; }
		);
		
		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::LineList, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::LineStrip(ResourceFactory& factory, int num_lines, std::span<v4 const> points, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::LineStripSize(num_lines);
		auto colours = opts ? opts->m_colours : std::span<Colour32 const>{};
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();
		assert(vcount == isize(points));

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::LinesStrip(num_lines, points.data(), colours,
			[&](v4 p, Colour32 c, auto, auto) { SetPC(*vptr++, p, Colour(c)); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::LineStrip, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}

	// Quad *******************************************************************************
	ModelPtr ModelGenerator::Quad(ResourceFactory& factory, CreateOptions const* opts)
	{
		v4 const verts[] = { v4{-1,-1,0,1}, v4{+1,-1,0,1}, v4{-1,+1,0,1}, v4{+1,+1,0,1} };
		return Quad(factory, 1, { &verts[0], 4 }, opts);
	}
	ModelPtr ModelGenerator::Quad(ResourceFactory& factory, int num_quads, std::span<v4 const> verts, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::QuadSize(num_quads);
		auto colours = opts ? opts->m_colours : std::span<Colour32 const>{};
		auto t2s = opts && opts->has(ModelGenerator::CreateOptions::EOptions::TextureToSurface) ? opts->m_t2s : m4x4::Identity();
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();
		assert(vcount == isize(verts));

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::Quad(num_quads, verts.data(), colours, t2s,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::TriList, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::Quad(ResourceFactory& factory, v2 const& anchor, v4 const& quad_w, v4 const& quad_h, iv2 const& divisions, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::QuadSize(divisions);
		auto colour = opts && !opts->m_colours.empty() ? opts->m_colours[0] : Colour32White;
		auto t2s = opts && opts->has(ModelGenerator::CreateOptions::EOptions::TextureToSurface) ? opts->m_t2s : m4x4::Identity();
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::Quad(anchor, quad_w, quad_h, divisions, colour, t2s,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::TriList, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::Quad(ResourceFactory& factory, AxisId axis_id, v2 const& anchor, float width, float height, iv2 const& divisions, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::QuadSize(divisions);
		auto colour = opts && !opts->m_colours.empty() ? opts->m_colours[0] : Colour32White;
		auto t2s = opts && opts->has(ModelGenerator::CreateOptions::EOptions::TextureToSurface) ? opts->m_t2s : m4x4::Identity();
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::Quad(axis_id, anchor, width, height, divisions, colour, t2s,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::TriList, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::QuadStrip(ResourceFactory& factory, int num_quads, std::span<v4 const> verts, float width, std::span<v4 const> normals, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::QuadStripSize(num_quads);
		auto colours = opts ? opts->m_colours : std::span<Colour32 const>{};
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::QuadStrip(num_quads, verts.data(), width, isize(normals), normals.data(), colours,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::TriStrip, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::QuadPatch(ResourceFactory& factory, int dimx, int dimy, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::QuadPatchSize(dimx, dimy);
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::QuadPatch(dimx, dimy,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::TriStrip, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}

	// Shape2d ****************************************************************************
	ModelPtr ModelGenerator::Ellipse(ResourceFactory& factory, float dimx, float dimy, bool solid, int facets, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::EllipseSize(solid, facets);
		auto colour = opts && !opts->m_colours.empty() ? opts->m_colours[0] : Colour32White;
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::Ellipse(dimx, dimy, solid, facets, colour,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(solid ? ETopo::TriStrip : ETopo::LineStrip, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::Pie(ResourceFactory& factory, float dimx, float dimy, float ang0, float ang1, float radius0, float radius1, bool solid, int facets, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::PieSize(solid, ang0, ang1, facets);
		auto colour = opts && !opts->m_colours.empty() ? opts->m_colours[0] : Colour32White;
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::Pie(dimx, dimy, ang0, ang1, radius0, radius1, solid, facets, colour,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(solid ? ETopo::TriStrip : ETopo::LineStrip, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::RoundedRectangle(ResourceFactory& factory, float dimx, float dimy, float corner_radius, bool solid, int facets, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::RoundedRectangleSize(solid, corner_radius, facets);
		auto colour = opts && !opts->m_colours.empty() ? opts->m_colours[0] : Colour32White;
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::RoundedRectangle(dimx, dimy, solid, corner_radius, facets, colour,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(solid ? ETopo::TriStrip : ETopo::LineStrip, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::Polygon(ResourceFactory& factory, std::span<v2 const> points, bool solid, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::PolygonSize(isize(points), solid);
		auto colours = opts ? opts->m_colours : std::span<Colour32 const>{};
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::Polygon(points, solid, colours,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(solid ? ETopo::TriList : ETopo::LineStrip, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}

	// Boxes ******************************************************************************
	ModelPtr ModelGenerator::Box(ResourceFactory& factory, float rad, CreateOptions const* opts)
	{
		return Box(factory, v4(rad), opts);
	}
	ModelPtr ModelGenerator::Box(ResourceFactory& factory, v4 rad, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::BoxSize(1);
		auto colour = opts && !opts->m_colours.empty() ? opts->m_colours[0] : Colour32White;

		// Generate the geometry
		Cache cache{vcount, icount, 0, sizeof(uint16_t)};
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.data<uint16_t>();
		auto props = geometry::Box(rad, m4x4::Identity(), colour,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = s_cast<uint16_t>(idx); }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::TriList, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::Boxes(ResourceFactory& factory, int num_boxes, std::span<v4 const> points, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::BoxSize(num_boxes);
		auto colours = opts ? opts->m_colours : std::span<Colour32 const>{};
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::Boxes(num_boxes, points.data(), m4x4::Identity(), colours,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::TriList, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}	
	ModelPtr ModelGenerator::BoxList(ResourceFactory& factory, int num_boxes, std::span<v4 const> positions, v4 rad, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::BoxSize(num_boxes);
		auto colours = opts ? opts->m_colours : std::span<Colour32 const>{};
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::BoxList(num_boxes, positions, rad, colours,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::TriList, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::BoxList(ResourceFactory& factory, std::span<BBox const> boxes, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::BoxSize(isize(boxes));
		auto colours = opts ? opts->m_colours : std::span<Colour32 const>{};
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::BoxList(boxes, colours,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::TriList, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}

	// Sphere *****************************************************************************
	ModelPtr ModelGenerator::Geosphere(ResourceFactory& factory, float radius, int divisions, CreateOptions const* opts)
	{
		return Geosphere(factory, v4(radius, radius, radius, 0.0f), divisions, opts);
	}
	ModelPtr ModelGenerator::Geosphere(ResourceFactory& factory, v4 radius, int divisions, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::GeosphereSize(divisions);
		auto colour = opts && !opts->m_colours.empty() ? opts->m_colours[0] : Colour32White;
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::Geosphere(radius, divisions, colour,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::TriList, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::Sphere(ResourceFactory& factory, float radius, int wedges, int layers, CreateOptions const* opts)
	{
		return Sphere(factory, v4{ radius, radius, radius, 0 }, wedges, layers, opts);
	}
	ModelPtr ModelGenerator::Sphere(ResourceFactory& factory, v4 const& radius, int wedges, int layers, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::SphereSize(wedges, layers);
		auto colour = opts && !opts->m_colours.empty() ? opts->m_colours[0] : Colour32White;
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::Sphere(radius, wedges, layers, colour,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::TriList, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}

	// Cylinder ***************************************************************************
	ModelPtr ModelGenerator::Cylinder(ResourceFactory& factory, float radius0, float radius1, float height, float xscale, float yscale, int wedges, int layers, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::CylinderSize(wedges, layers);
		auto colours = opts ? opts->m_colours : std::span<Colour32 const>{};
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::Cylinder(radius0, radius1, height, xscale, yscale, wedges, layers, colours,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::TriList, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}

	// Extrude ****************************************************************************
	ModelPtr ModelGenerator::Extrude(ResourceFactory& factory, std::span<v2 const> cs, std::span<v4 const> path, bool closed, bool smooth_cs, CreateOptions const* opts)
	{
		assert(isize(path) >= 2);

		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::ExtrudeSize(isize(cs), isize(path), closed, smooth_cs);
		auto colours = opts ? opts->m_colours : std::span<Colour32 const>{};
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Convert a stream of points into a stream of transforms
		// At each vertex, ori.z should be the tangent to the extrusion path.
		auto ori = m4x4::Identity();
		auto yaxis = Perpendicular(path[1] - path[0], v4::YAxis());
		auto make_path = [&](int p_, int pcount_)
		{
			size_t p = p_, pcount = pcount_;
			if (p == 0)
			{
				auto tang = path[1] - path[0];
				if (!FEql(tang, v4::Zero()))
				{
					yaxis = Perpendicular(tang, yaxis);
					ori.rot = OriFromDir(tang, AxisId::PosZ, yaxis);
				}
			}
			else if (p == pcount - 1)
			{
				auto tang = path[p] - path[p - 1];
				if (!FEql(tang, v4::Zero()))
				{
					yaxis = Perpendicular(tang, yaxis);
					ori.rot = OriFromDir(tang, AxisId::PosZ, yaxis);
				}
			}
			else
			{
				auto a = Normalise(path[p] - path[p - 1], v4::Zero());
				auto b = Normalise(path[p + 1] - path[p], v4::Zero());
				auto tang = a + b;
				if (!FEql(tang, v4::Zero()))
				{
					yaxis = Perpendicular(tang, yaxis);
					ori.rot = OriFromDir(tang, AxisId::PosZ, yaxis);
				}
			}
			ori.pos = path[p];
			return ori;
		};

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::Extrude(cs, make_path, isize(path), closed, smooth_cs, colours,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::TriList, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::Extrude(ResourceFactory& factory, std::span<v2 const> cs, std::span<m4x4 const> path, bool closed, bool smooth_cs, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::ExtrudeSize(isize(cs), isize(path), closed, smooth_cs);
		auto colours = opts ? opts->m_colours : std::span<Colour32 const>{};
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Path transform stream source
		auto make_path = [&](int p, int) { return path[p]; };

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::Extrude(cs, make_path, isize(path), closed, smooth_cs, colours,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create a nugget
		cache.m_ncont.push_back(NuggetDesc(ETopo::TriList, props.m_geom).alpha_geom(props.m_has_alpha));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}

	// Mesh *******************************************************************************
	ModelPtr ModelGenerator::Mesh(ResourceFactory& factory, MeshCreationData const& cdata, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::MeshSize(isize(cdata.m_verts), isize(cdata.m_idxbuf));
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::Mesh(
			cdata.m_verts,
			cdata.m_idxbuf,
			cdata.m_colours,
			cdata.m_normals,
			cdata.m_tex_coords,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Create the nuggets
		cache.m_ncont.insert(cache.m_ncont.begin(), cdata.m_nuggets.begin(), cdata.m_nuggets.end());
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}

	// SkyBox *****************************************************************************
	ModelPtr ModelGenerator::SkyboxGeosphere(ResourceFactory& factory, Texture2DPtr sky_texture, float radius, int divisions, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::SkyboxGeosphereSize(divisions);
		auto colour = opts && !opts->m_colours.empty() ? opts->m_colours[0] : Colour32White;
		auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();

		// Generate the geometry
		Cache cache{ vcount, icount, 0, idx_stride };
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.begin<int>();
		auto props = geometry::SkyboxGeosphere(radius, divisions, colour,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = idx; }
		);

		// Model nugget properties for the sky box
		cache.m_ncont.push_back(NuggetDesc(ETopo::TriList, props.m_geom).alpha_geom(props.m_has_alpha).tex_diffuse(sky_texture).pso<EPipeState::CullMode>(D3D12_CULL_MODE_FRONT));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::SkyboxGeosphere(ResourceFactory& factory, std::filesystem::path const& texture_path, float radius, int divisions, CreateOptions const* opts)
	{
		// One texture per nugget
		TextureDesc desc = TextureDesc(AutoId, ResDesc()).name("skybox");
		auto tex = factory.CreateTexture2D(texture_path, desc);
		return SkyboxGeosphere(factory, tex, radius, divisions, opts);
	}
	ModelPtr ModelGenerator::SkyboxFiveSidedCube(ResourceFactory& factory, Texture2DPtr sky_texture, float radius, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::SkyboxFiveSidedCubicDomeSize();
		auto colour = opts && !opts->m_colours.empty() ? opts->m_colours[0] : Colour32White;

		// Generate the geometry
		Cache cache(vcount, icount, 0, sizeof(uint16_t));
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.data<uint16_t>();
		auto props = geometry::SkyboxFiveSidedCubicDome(radius, colour,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](size_t idx) { *iptr++ = s_cast<uint16_t>(idx); });

		// Model nugget properties for the sky box
		cache.m_ncont.push_back(NuggetDesc(ETopo::TriList, props.m_geom).alpha_geom(props.m_has_alpha).tex_diffuse(sky_texture).pso<EPipeState::CullMode>(D3D12_CULL_MODE_FRONT));
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::SkyboxFiveSidedCube(ResourceFactory& factory, std::filesystem::path const& texture_path, float radius, CreateOptions const* opts)
	{
		// One texture per nugget
		TextureDesc desc = TextureDesc(AutoId, ResDesc()).name("skybox");
		auto tex = factory.CreateTexture2D(texture_path, desc);
		return SkyboxFiveSidedCube(factory, tex, radius, opts);
	}
	ModelPtr ModelGenerator::SkyboxSixSidedCube(ResourceFactory& factory, Texture2DPtr (&sky_texture)[6], float radius, CreateOptions const* opts)
	{
		// Calculate the required buffer sizes
		auto [vcount, icount] = geometry::SkyboxSixSidedCubeSize();
		auto colour = opts && !opts->m_colours.empty() ? opts->m_colours[0] : Colour32White;

		// Generate the geometry
		Cache cache(vcount, icount, 0, sizeof(uint16_t));
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.data<uint16_t>();
		auto props = geometry::SkyboxSixSidedCube(radius, colour,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](size_t idx) { *iptr++ = s_cast<uint16_t>(idx); });

		// Create the nuggets, one per face. Expected order: +X, -X, +Y, -Y, +Z, -Z
		for (int i = 0; i != 6; ++i)
		{
			// Create the render nugget for this face of the sky box
			cache.m_ncont.push_back(NuggetDesc(ETopo::TriList, props.m_geom)
				.vrange(Range(i * 4, (i + 1) * 4))
				.irange(Range(i * 6, (i + 1) * 6))
				.alpha_geom(props.m_has_alpha)
				.tex_diffuse(sky_texture[i])
				.pso<EPipeState::CullMode>(D3D12_CULL_MODE_FRONT));
		}
		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::SkyboxSixSidedCube(ResourceFactory& factory, std::filesystem::path const& texture_path_pattern, float radius, CreateOptions const* opts)
	{
		wstring256 tpath = texture_path_pattern.wstring();
		auto ofs = tpath.find(L"??");
		if (ofs == std::string::npos)
			throw std::runtime_error(FmtS("Skybox texture path '%S' does not include '??' characters", texture_path_pattern.c_str()));

		Texture2DPtr tex[6] = {}; int i = 0;
		for (auto face : { L"+X", L"-X", L"+Y", L"-Y", L"+Z", L"-Z" })
		{
			// Load the texture for this face of the sky box
			tpath[ofs + 0] = face[0];
			tpath[ofs + 1] = face[1];
			TextureDesc desc = TextureDesc(AutoId, ResDesc()).name("skybox");
			tex[i++] = factory.CreateTexture2D(tpath.c_str(), desc);
		}

		return SkyboxSixSidedCube(factory, tex, radius, opts);
	}

	// ModelFile **************************************************************************
	// Load a P3D model from a stream, emitting models for each mesh via 'out'.
	void ModelGenerator::LoadP3DModel(ResourceFactory& factory, std::istream& src, IModelOut& out, CreateOptions const* opts)
	{
		using namespace geometry;

		// Model output helpers
		struct Mat :p3d::Material
		{
			ResourceFactory& m_factory;
			mutable Texture2DPtr m_tex_diffuse;

			Mat(ResourceFactory& factory, p3d::Material&& m)
				:p3d::Material(std::forward<p3d::Material>(m))
				,m_factory(factory)
				,m_tex_diffuse()
			{}

			// The material base colour
			Colour32 Tint() const
			{
				return this->m_diffuse.argb();
			}

			// The diffuse texture resolved to a renderer texture resource
			Texture2DPtr TexDiffuse() const
			{
				// Lazy load the texture
				if (m_tex_diffuse == nullptr)
				{
					for (auto& tex : m_textures)
					{
						if (tex.m_type != p3d::Texture::EType::Diffuse)
							continue;
						
						TextureDesc desc = TextureDesc(AutoId, ResDesc()).has_alpha(AllSet(tex.m_flags, p3d::Texture::EFlags::Alpha)).name(tex.m_filepath.c_str());
						m_tex_diffuse = m_factory.CreateTexture2D(tex.m_filepath.c_str(), desc);
						break;
					}
				}
				return m_tex_diffuse;
			}
		};
		struct ModelOut
		{
			// Notes:
			//  - Each 'mesh' can contain nested child meshes.
			//    Create models for each and emit the tree structure of models
			using MatCont = std::vector<Mat>;

			ResourceFactory& m_factory;
			CreateOptions const* m_opts;
			IModelOut& m_out;
			Cache<> m_cache;
			MatCont m_mats;

			ModelOut(ResourceFactory& factory, CreateOptions const* opts, IModelOut& out)
				: m_factory(factory)
				, m_opts(opts)
				, m_out(out)
				, m_cache(0, 0, 0, sizeof(uint32_t))
				, m_mats()
			{}

			// Functor called from 'ExtractMaterials'
			bool operator ()(p3d::Material&& mat)
			{
				m_mats.push_back(Mat(m_factory, std::forward<p3d::Material>(mat)));
				return false;
			}

			// Functor called from 'ExtractMeshes'
			bool operator()(p3d::Mesh&& mesh)
			{
				ModelTree tree;
				BuildModelTree(tree, mesh, 0, m4x4::Identity());
				return m_out.Model(std::move(tree)) == IModelOut::Stop;
			}

			// Recursive function for populating a model tree
			void BuildModelTree(ModelTree& tree, p3d::Mesh const& mesh, int level, m4x4 const& p2w)
			{
				auto o2w = p2w * mesh.m_o2p;
				auto big_indices = mesh.vcount() > 0xFFFF;

				// Convert a 'p3d::Mesh' into a 'rdr::Model'
				m_cache.Reset();

				// Name/Bounding box
				m_cache.m_name = mesh.m_name;
				m_cache.m_bbox = mesh.m_bbox;
				m_cache.m_m2root = o2w;

				// Copy the verts
				m_cache.m_vcont.resize(mesh.vcount());
				auto vptr = m_cache.m_vcont.data();
				for (auto const& mvert : mesh.fat_verts())
				{
					SetPCNT(*vptr, GetP(mvert), GetC(mvert), GetN(mvert), GetT(mvert));
					++vptr;
				}

				// Copy nuggets
				m_cache.m_icont.resize(mesh.icount(), big_indices ? sizeof(uint16_t) : sizeof(uint32_t));
				m_cache.m_ncont.reserve(mesh.ncount());
				auto iptr = m_cache.m_icont.begin<uint32_t>();
				auto vrange = Range::Zero();
				auto irange = Range::Zero();
				for (auto const& nug : mesh.nuggets())
				{
					// Copy the indices
					vrange = Range::Reset();
					for (auto i : nug.indices())
					{
						vrange.grow(i);
						*iptr++ = i;
					}

					// The index range in the model buffer
					irange.m_beg = irange.m_end;
					irange.m_end = irange.m_beg + nug.icount();

					// The basic nugget
					NuggetDesc nugget = NuggetDesc(nug.m_topo, nug.m_geom).vrange(vrange).irange(irange);

					// Resolve the material
					for (auto& m : m_mats)
					{
						if (nug.m_mat != m.m_id) continue;
						nugget.tex_diffuse(m.TexDiffuse());
						nugget.tint(m.Tint());
						nugget.alpha_tint(nugget.m_tint.a != 0xff);
						break;
					}

					// Add a nugget
					m_cache.m_ncont.push_back(nugget);
				}

				// Return the renderer model
				auto model = Create(m_factory, m_cache, m_opts);
				tree.push_back(ModelTreeNode{
					.m_o2p = m4x4::Identity(),
					.m_name = mesh.m_name,
					.m_model = model,
					.m_level = level,
				});

				// Add the children (in depth first order)
				for (auto& child : mesh.m_children)
					BuildModelTree(tree, child, level + 1, o2w);
			}
		};
		ModelOut model_out(factory, opts, out);

		// Material read from the p3d model, extended with associated renderer resources.
		p3d::ExtractMaterials<std::istream, ModelOut&>(src, model_out);

		// Load each mesh in the P3D stream and emit it as a model
		p3d::ExtractMeshes<std::istream, ModelOut&>(src, model_out);
	}
	void ModelGenerator::Load3DSModel(ResourceFactory& factory, std::istream& src, IModelOut& out, CreateOptions const* opts)
	{
		using namespace geometry;

		// 3DS Material read from the 3ds model, extended with associated renderer resources
		struct Mat : max_3ds::Material
		{
			mutable Texture2DPtr m_tex_diffuse;

			Mat() {}
			Mat(max_3ds::Material&& m)
				:max_3ds::Material(std::forward<max_3ds::Material>(m))
				, m_tex_diffuse()
			{}

			// The material base colour
			Colour32 Tint() const
			{
				return this->m_diffuse.argb();
			}

			// The diffuse texture resolved to a renderer texture resource
			Texture2DPtr TexDiffuse(ResourceFactory& factory) const
			{
				if (m_tex_diffuse == nullptr && !m_textures.empty())
				{
					auto& tex = m_textures[0];
					TextureDesc desc = TextureDesc(AutoId, ResDesc()).name(tex.m_filepath.c_str());
					m_tex_diffuse = factory.CreateTexture2D(tex.m_filepath.c_str(), desc);

					// todo
					//SamplerDesc sam_desc{
					//	tex.m_tiling == 0 ? D3D11_TEXTURE_ADDRESS_CLAMP :
					//	tex.m_tiling == 1 ? D3D11_TEXTURE_ADDRESS_WRAP :
					//	D3D11_TEXTURE_ADDRESS_BORDER,
					//	D3D11_FILTER_ANISOTROPIC};
				}
				return m_tex_diffuse;
			}
		};
		std::vector<Mat> mats;
		max_3ds::ReadMaterials(src, [&](max_3ds::Material&& mat)
		{
			mats.emplace_back(std::forward<max_3ds::Material>(mat));
			return false;
		});

		// Load each mesh in the stream and emit it as a model
		Cache cache{ 0, 0, 0, sizeof(uint16_t) };
		max_3ds::ReadObjects(src, [&](max_3ds::Object&& obj)
		{
			cache.Reset();

			// Name/Bounding box
			cache.m_name = obj.m_name;
			cache.m_bbox = BBox::Reset();
			cache.m_m2root = obj.m_mesh.m_o2p; //todo: hierarchy needed

			// Populate 'cache' from the 3DS data
			auto vout = [&](v4 const& p, Colour const& c, v4 const& n, v2 const& t)
			{
				Vert vert;
				SetPCNT(vert, cache.m_bbox.Grow(p), c, n, t);
				cache.m_vcont.push_back(vert);
			};
			auto iout = [&](uint16_t i0, uint16_t i1, uint16_t i2)
			{
				cache.m_icont.push_back<uint16_t>(i0);
				cache.m_icont.push_back<uint16_t>(i1);
				cache.m_icont.push_back<uint16_t>(i2);
			};
			auto nout = [&](ETopo topo, EGeom geom, Mat const& mat, Range vrange, Range irange)
			{
				cache.m_ncont.push_back(NuggetDesc(topo, geom)
					.vrange(vrange)
					.irange(irange)
					.tex_diffuse(mat.TexDiffuse(factory))
					.tint(mat.Tint())
					.alpha_tint(mat.Tint().a != 0xff));
			};
			auto matlookup = [&](std::string const& name)
			{
				for (auto& mat : mats)
				{
					if (mat.m_name != name) continue;
					return mat;
				}
				return Mat{};
			};
			max_3ds::CreateModel(obj, matlookup, vout, iout, nout);

			// Emit the model. 'out' returns true to stop searching
			// 3DS models cannot nest, so each 'Model Tree' is one root node only
			auto model = Create(factory, cache, opts);
			return out.Model(ModelTree{
				ModelTreeNode{
					.m_o2p = m4x4::Identity(),
					.m_name = obj.m_name,
					.m_model = model,
					.m_level = 0,
				}
			});
		});
	}
	void ModelGenerator::LoadSTLModel(ResourceFactory& factory, std::istream& src, IModelOut& out, CreateOptions const* opts)
	{
		using namespace geometry;

		// Parse the model file in the STL stream
		Cache cache{0, 0, 0, sizeof(uint16_t)};
		stl::Read(src, stl::Options{}, [&](stl::Model&& mesh)
			{
				cache.Reset();

				// Name/Bounding box
				cache.m_name = mesh.m_header;
				cache.m_bbox = BBox::Reset();

				// Copy the verts
				cache.m_vcont.resize(mesh.m_verts.size());
				auto vptr = cache.m_vcont.data();
				for (int i = 0, iend = int(mesh.m_verts.size()); i != iend; ++i)
				{
					SetPCNT(*vptr, cache.m_bbox.Grow(mesh.m_verts[i]), ColourWhite, mesh.m_norms[i/3], v2Zero);
					++vptr;
				}

				// Copy nuggets
				auto vcount = int(cache.m_vcont.size());
				if (vcount > 0xFFFF)
				{
					// Use 32bit indices
					cache.m_icont.resize(vcount, sizeof(uint32_t));
					auto ibuf = cache.m_icont.data<uint32_t>();
					for (uint32_t i = 0; vcount-- != 0;)
						*ibuf++ = i++;
				}
				else
				{
					// Use 16bit indices
					cache.m_icont.resize(vcount, sizeof(uint16_t));
					auto ibuf = cache.m_icont.data<uint16_t>();
					for (uint16_t i = 0; vcount-- != 0;)
						*ibuf++ = i++;
				}
				cache.m_ncont.push_back(NuggetDesc(ETopo::TriList, EGeom::Vert|EGeom::Norm));

				// Emit the model. 'out' returns true to stop searching
				// STL models cannot nest, so each 'Model Tree' is one root node only
				auto model = Create(factory, cache, opts);
				return out.Model(ModelTree{
					ModelTreeNode{
						.m_o2p = m4x4::Identity(),
						.m_name = mesh.m_header,
						.m_model = model,
						.m_level = 0,
					}
				});
			});
	}
	void ModelGenerator::LoadFBXModel(ResourceFactory& factory, std::istream& src, IModelOut& out, CreateOptions const* opts)
	{
		// Notes:
		//  - The model generator handles loading models and animation, however, LDraw separates these
		//    into two separate calls.

		using namespace geometry;

		// FBX read output adapter
		struct ReadOutput :fbx::IReadOutput
		{
			ResourceFactory& m_factory;
			CreateOptions const* m_opts;
			IModelOut& m_out;
			Cache<> m_cache;
			std::unordered_map<uint32_t, ModelPtr> m_models;
			std::unordered_map<uint32_t, SkeletonPtr> m_skels;
			
			ReadOutput(ResourceFactory& factory, IModelOut& out, CreateOptions const* opts)
				: m_factory(factory)
				, m_opts(opts)
				, m_out(out)
				, m_cache(0, 0, 0, sizeof(uint32_t))
				, m_models()
				, m_skels()
			{}

			// Create a user-side mesh from 'mesh' and return an opaque handle to it (or null)
			virtual void CreateMesh(fbx::Mesh const& mesh, std::span<fbx::Material const> materials) override
			{
				m_cache.Reset();

				// Name/Bounding box
				m_cache.m_name = mesh.m_name;
				m_cache.m_bbox = mesh.m_bbox;
				m_cache.m_m2root = m4x4::Identity();

				// Copy the verts
				auto vcount = isize(mesh.m_vbuf);
				m_cache.m_vcont.resize(vcount, {});
				auto vptr = m_cache.m_vcont.data();
				for (auto const& v : mesh.m_vbuf)
				{
					SetPCNTI(*vptr, v.m_vert, v.m_colr, v.m_norm, v.m_tex0, v.m_idx0);
					++vptr;
				}

				// Copy indices
				auto icount = isize(mesh.m_ibuf);
				auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();
				m_cache.m_icont.resize(icount, idx_stride);
				if (idx_stride == sizeof(uint32_t))
				{
					// Use 32bit indices
					auto iptr = m_cache.m_icont.data<uint32_t>();
					memcpy(iptr, mesh.m_ibuf.data(), mesh.m_ibuf.size() * sizeof(int));
				}
				else
				{
					// Use 16bit indices
					auto isrc = mesh.m_ibuf.data();
					auto idst = m_cache.m_icont.begin<int>();
					for (auto count = mesh.m_ibuf.size(); count-- != 0;)
						*idst++ = *isrc++;
				}

				// Copy the nuggets
				m_cache.m_ncont.resize(mesh.m_nbuf.size());
				auto nptr = m_cache.m_ncont.data();
				for (auto const& n : mesh.m_nbuf)
				{
					auto const& mat = materials[n.m_mat_id];
					*nptr++ = NuggetDesc{ n.m_topo, n.m_geom }.vrange(n.m_vrange).irange(n.m_irange).tint(mat.m_diffuse).flags(ENuggetFlag::RangesCanOverlap);
				}

				// Emit the model.
				auto model = Create(m_factory, m_cache, m_opts);

				// Add skinning data if present and requested
				if (fbx::Skin skin = mesh.m_skin; skin && AllSet(m_out.Parts(), ESceneParts::Skins))
				{
					assert(m_skels.contains(skin.m_skel_id) && m_skels[skin.m_skel_id] != nullptr);
					constexpr int max_influences_per_vertex = _countof(Skinfluence::m_bones);

					auto const& skel = *m_skels[skin.m_skel_id].get();
					auto id_to_idx16 = [&skel](uint32_t id)
					{
						auto idx = s_cast<int16_t>(index_of(skel.m_bone_ids, id));
						assert(idx >= 0 && idx < isize(skel.m_bone_ids) && "Bone id not found in skeleton");
						return idx;
					};
					auto norm_to_u16 = [](double w)
					{
						return s_cast<uint16_t>(std::clamp(w, 0.0, 1.0) * 65535);
					};

					// Read the influences per vertex
					vector<Skinfluence> influences(skin.vert_count());
					for (int vidx = 0, vidx_count = skin.vert_count(); vidx != vidx_count; ++vidx)
					{
						auto const influence_count = skin.influence_count(vidx);
						if (influence_count > max_influences_per_vertex)
							OutputDebugStringA(PR_LINK "Unsupported number of bone influences\n");

						// Convert the bone weights to the compressed format
						auto ibase = skin.m_offsets[vidx];
						auto& influence = influences[vidx];
						for (int i = 0; i != influence_count && i != max_influences_per_vertex; ++i)
						{
							influence.m_bones[i] = id_to_idx16(skin.m_bones[ibase + i]);
							influence.m_weights[i] = norm_to_u16(skin.m_weights[ibase + i]);
						}
					}

					model->m_skin = Skin(m_factory, influences, skin.m_skel_id);
				}

				// Store the created mesh
				m_models[mesh.m_mesh_id] = model;
			}

			// Create a model from a hierarchy of mesh instances
			virtual void CreateModel(std::span<fbx::MeshTree const> mesh_tree) override
			{
				ModelTree tree;
				for (auto const& node : mesh_tree)
				{
					tree.push_back(ModelTreeNode{
						.m_o2p = node.m_o2p,
						.m_name = node.m_name,
						.m_model = m_models[node.m_mesh_id],
						.m_level = node.m_level,
					});
				}
				m_out.Model(std::move(tree));
			}

			// Create a skeleton from a hierarchy of bone instances
			virtual void CreateSkeleton(fbx::Skeleton const& fbxskel)
			{
				// Type conversion from fbx to rdr12
				constexpr auto ToU8 = [](int x) -> uint8_t { return s_cast<uint8_t>(x); };
				constexpr auto ToString32 = [](std::string const& x) -> string32 { return static_cast<string32>(x); };

				auto bone_names = transform<Skeleton::Names>(fbxskel.m_bone_names, ToString32);
				auto hierarchy = transform<Skeleton::Hierarchy>(fbxskel.m_hierarchy, ToU8);

				SkeletonPtr skel(rdr12::New<Skeleton>(fbxskel.m_skel_id, fbxskel.m_bone_ids, bone_names, fbxskel.m_o2bp, hierarchy), true);
				m_skels[fbxskel.m_skel_id] = skel;
				
				/*{
					std::ofstream ofile("E:\\Dump\\LDraw\\PendulumAnimDump2.txt");
					ofile << "Skel: " << skel->m_id << "\n";
					ofile << "  BoneIds:\n"; for (auto x : skel->m_bone_ids) ofile << "    " << x << "\n";
					ofile << "  Names:\n"; for (auto x : skel->m_names) ofile << "    " << x << "\n";
					ofile << "  O2BP:\n"; for (auto x : skel->m_o2bp) ofile << "    " << x << "\n";
					ofile << "  Hierarchy:\n"; for (auto x : skel->m_hierarchy) ofile << "    " << (int)x << "\n";
				}*/

				m_out.Skeleton(std::move(skel));
			}

			// Create an animation from 
			virtual bool CreateAnimation(fbx::Animation const& fbxanim)
			{
				// Create an animation for 'fbxanim'
				KeyFrameAnimationPtr anim(rdr12::New<KeyFrameAnimation>(fbxanim.m_skel_id, fbxanim.m_duration, fbxanim.m_frame_rate), true);

				// Read the key frame data
				anim->m_bone_map = fbxanim.m_bone_map;
				anim->m_rotation = fbxanim.m_rotation;
				anim->m_position = fbxanim.m_position;
				anim->m_scale = fbxanim.m_scale;

				// Save the animation
				return m_out.Animation(std::move(anim)) == IModelOut::EResult::Continue;
			}
		} read_out{ factory, out, opts };

		// Load the fbx scene
		fbx::Scene scene(src, fbx::LoadOptions{
			.space_conversion = fbx::ESpaceConversion::TransformRoot,
			.pivot_handling = fbx::EPivotHandling::Retain,
			.target_axes = {
				.right = fbx::ECoordAxis::PosX,
				.up = fbx::ECoordAxis::PosZ,
				.front = fbx::ECoordAxis::NegY,
			},
			.target_unit_meters = 1.0f,
		});
		#if 0
		{
			//hack
			std::ofstream ofile("E:\\Dump\\LDraw\\SceneDump.txt");
			scene.Dump(ofile, fbx::DumpOptions{
				.m_parts =
					ESceneParts::MainObjects |
					ESceneParts::NodeHierarchy |
					ESceneParts::None,
			});
		}
		#endif
		scene.Read(read_out, fbx::ReadOptions{
			.m_parts = out.Parts(),
			.m_frame_range = out.FrameRange(),
			.m_mesh_filter = std::bind(&IModelOut::ModelFilter, &out, _1),
			.m_skel_filter = std::bind(&IModelOut::SkeletonFilter, &out, _1),
			.m_anim_filter = std::bind(&IModelOut::AnimationFilter, &out, _1),
			.m_progress = std::bind(&IModelOut::Progress, &out, _1, _2, _3, _4),
		});
	}
	void ModelGenerator::LoadGLTFModel(ResourceFactory& factory, std::istream& src, IModelOut& out, CreateOptions const* opts)
	{
		using namespace geometry;

		// glTF read output adapter
		struct ReadOutput :gltf::IReadOutput
		{
			ResourceFactory& m_factory;
			CreateOptions const* m_opts;
			IModelOut& m_out;
			Cache<> m_cache;
			std::unordered_map<uint32_t, ModelPtr> m_models;
			std::unordered_map<uint32_t, SkeletonPtr> m_skels;
			
			ReadOutput(ResourceFactory& factory, IModelOut& out, CreateOptions const* opts)
				: m_factory(factory)
				, m_opts(opts)
				, m_out(out)
				, m_cache(0, 0, 0, sizeof(uint32_t))
				, m_models()
				, m_skels()
			{}

			// Create a user-side mesh from 'mesh' and return an opaque handle to it (or null)
			virtual void CreateMesh(gltf::Mesh const& mesh, std::span<gltf::Material const> materials) override
			{
				m_cache.Reset();

				// Name/Bounding box
				m_cache.m_name = mesh.m_name;
				m_cache.m_bbox = mesh.m_bbox;
				m_cache.m_m2root = m4x4::Identity();

				// Copy the verts
				auto vcount = isize(mesh.m_vbuf);
				m_cache.m_vcont.resize(vcount, {});
				auto vptr = m_cache.m_vcont.data();
				for (auto const& v : mesh.m_vbuf)
				{
					SetPCNTI(*vptr, v.m_vert, v.m_colr, v.m_norm, v.m_tex0, v.m_idx0);
					++vptr;
				}

				// Copy indices
				auto icount = isize(mesh.m_ibuf);
				auto idx_stride = vcount > 0xFFFF ? isizeof<uint32_t>() : isizeof<uint16_t>();
				m_cache.m_icont.resize(icount, idx_stride);
				if (idx_stride == sizeof(uint32_t))
				{
					auto iptr = m_cache.m_icont.data<uint32_t>();
					memcpy(iptr, mesh.m_ibuf.data(), mesh.m_ibuf.size() * sizeof(int));
				}
				else
				{
					auto isrc = mesh.m_ibuf.data();
					auto idst = m_cache.m_icont.begin<int>();
					for (auto count = mesh.m_ibuf.size(); count-- != 0;)
						*idst++ = *isrc++;
				}

				// Copy the nuggets
				m_cache.m_ncont.resize(mesh.m_nbuf.size());
				auto nptr = m_cache.m_ncont.data();
				for (auto const& n : mesh.m_nbuf)
				{
					auto const& mat = materials[n.m_mat_id];
					*nptr++ = NuggetDesc{ n.m_topo, n.m_geom }.vrange(n.m_vrange).irange(n.m_irange).tint(mat.m_diffuse).flags(ENuggetFlag::RangesCanOverlap);
				}

				// Emit the model
				auto model = Create(m_factory, m_cache, m_opts);

				// Add skinning data if present and requested
				if (gltf::Skin skin = mesh.m_skin; skin && AllSet(m_out.Parts(), ESceneParts::Skins))
				{
					assert(m_skels.contains(skin.m_skel_id) && m_skels[skin.m_skel_id] != nullptr);
					constexpr int max_influences_per_vertex = _countof(Skinfluence::m_bones);

					auto const& skel = *m_skels[skin.m_skel_id].get();
					auto id_to_idx16 = [&skel](uint32_t id)
					{
						auto idx = s_cast<int16_t>(index_of(skel.m_bone_ids, id));
						assert(idx >= 0 && idx < isize(skel.m_bone_ids) && "Bone id not found in skeleton");
						return idx;
					};
					auto norm_to_u16 = [](double w)
					{
						return s_cast<uint16_t>(std::clamp(w, 0.0, 1.0) * 65535);
					};

					// Read the influences per vertex
					vector<Skinfluence> influences(skin.vert_count());
					for (int vidx = 0, vidx_count = skin.vert_count(); vidx != vidx_count; ++vidx)
					{
						auto const influence_count = skin.influence_count(vidx);
						if (influence_count > max_influences_per_vertex)
							OutputDebugStringA(PR_LINK "Unsupported number of bone influences\n");

						auto ibase = skin.m_offsets[vidx];
						auto& influence = influences[vidx];
						for (int i = 0; i != influence_count && i != max_influences_per_vertex; ++i)
						{
							influence.m_bones[i] = id_to_idx16(skin.m_bones[ibase + i]);
							influence.m_weights[i] = norm_to_u16(skin.m_weights[ibase + i]);
						}
					}

					model->m_skin = Skin(m_factory, influences, skin.m_skel_id);
				}

				// Store the created mesh
				m_models[mesh.m_mesh_id] = model;
			}

			// Create a model from a hierarchy of mesh instances
			virtual void CreateModel(std::span<gltf::MeshTree const> mesh_tree) override
			{
				ModelTree tree;
				for (auto const& node : mesh_tree)
				{
					tree.push_back(ModelTreeNode{
						.m_o2p = node.m_o2p,
						.m_name = node.m_name,
						.m_model = m_models[node.m_mesh_id],
						.m_level = node.m_level,
					});
				}
				m_out.Model(std::move(tree));
			}

			// Create a skeleton from a hierarchy of bone instances
			virtual void CreateSkeleton(gltf::Skeleton const& gltfskel)
			{
				constexpr auto ToU8 = [](int x) -> uint8_t { return s_cast<uint8_t>(x); };
				constexpr auto ToString32 = [](std::string const& x) -> string32 { return static_cast<string32>(x); };

				auto bone_names = transform<Skeleton::Names>(gltfskel.m_bone_names, ToString32);
				auto hierarchy = transform<Skeleton::Hierarchy>(gltfskel.m_hierarchy, ToU8);

				SkeletonPtr skel(rdr12::New<Skeleton>(gltfskel.m_skel_id, gltfskel.m_bone_ids, bone_names, gltfskel.m_o2bp, hierarchy), true);
				m_skels[gltfskel.m_skel_id] = skel;
				m_out.Skeleton(std::move(skel));
			}

			// Create an animation
			virtual bool CreateAnimation(gltf::Animation const& gltfanim)
			{
				KeyFrameAnimationPtr anim(rdr12::New<KeyFrameAnimation>(gltfanim.m_skel_id, gltfanim.m_duration, gltfanim.m_frame_rate), true);

				anim->m_bone_map = gltfanim.m_bone_map;
				anim->m_rotation = gltfanim.m_rotation;
				anim->m_position = gltfanim.m_position;
				anim->m_scale = gltfanim.m_scale;

				return m_out.Animation(std::move(anim)) == IModelOut::EResult::Continue;
			}
		} read_out{ factory, out, opts };

		// Load the glTF scene from stream
		gltf::Scene scene(src, gltf::LoadOptions{});
		scene.Read(read_out, gltf::ReadOptions{
			.m_parts = out.Parts(),
			.m_mesh_filter = std::bind(&IModelOut::ModelFilter, &out, _1),
			.m_skel_filter = std::bind(&IModelOut::SkeletonFilter, &out, _1),
			.m_anim_filter = std::bind(&IModelOut::AnimationFilter, &out, _1),
			.m_progress = std::bind(&IModelOut::Progress, &out, _1, _2, _3, _4),
		});
	}
	void ModelGenerator::LoadModel(geometry::EModelFileFormat format, ResourceFactory& factory, std::istream& src, IModelOut& mout, CreateOptions const* opts)
	{
		using namespace geometry;
		switch (format)
		{
			case EModelFileFormat::P3D:    LoadP3DModel(factory, src, mout, opts); break;
			case EModelFileFormat::Max3DS: Load3DSModel(factory, src, mout, opts); break;
			case EModelFileFormat::STL:    LoadSTLModel(factory, src, mout, opts); break;
			case EModelFileFormat::FBX:    LoadFBXModel(factory, src, mout, opts); break;
			case EModelFileFormat::GLTF:   LoadGLTFModel(factory, src, mout, opts); break;
			case EModelFileFormat::GLB:    LoadGLTFModel(factory, src, mout, opts); break;
			default: throw std::runtime_error("Unsupported model file format");
		}
	}

	// Text *******************************************************************************
	// Create a quad containing text.
	// 'text' is the complete text to render into the quad.
	// 'formatting' defines regions in the text to apply formatting to.
	// 'layout' is global text layout information.
	// 'scale' controls the size of the output quad. Scale of 1 => 100pt = 1m
	// 'axis_id' is the forward direction of the quad
	// 'dim_out' is 'xy' = size of text in pixels, 'zw' = size of quad in pixels
	ModelPtr ModelGenerator::Text(ResourceFactory& factory, std::wstring_view text, std::span<TextFormat const> formatting, TextLayout const& layout, float scale, AxisId axis_id, v4& dim_out, CreateOptions const* opts)
	{
		// Texture sizes are in physical pixels, but D2D operates in DIP so we need to determine
		// the size in physical pixels on this device that correspond to the returned metrics.
		// From: https://msdn.microsoft.com/en-us/library/windows/desktop/ff684173%28v=vs.85%29.aspx?f=255&MSPPError=-2147217396
		// "Direct2D automatically performs scaling to match the DPI setting.
		//  In Direct2D, coordinates are measured in units called device-independent pixels (DIPs).
		//  A DIP is defined as 1/96th of a logical inch. In Direct2D, all drawing operations are
		//  specified in DIPs and then scaled to the current DPI setting."
		D3DPtr<IDWriteFactory> dwrite;
		Check(DWriteCreateFactory(DWRITE_FACTORY_TYPE_SHARED, __uuidof(IDWriteFactory), (IUnknown**)dwrite.address_of()));

		// Get the default format
		auto def = !formatting.empty() && !formatting[0].empty() ? formatting[0] : TextFormat();

		// Determine if the model requires alpha blending.
		// Consider alpha = 0 as not requiring blending, Alpha clip will be used instead
		auto has_alpha = HasAlpha(layout.m_bk_colour) || HasAlpha(def.m_font.m_colour);

		// Create the default font
		D3DPtr<IDWriteTextFormat> text_format;
		Check(dwrite->CreateTextFormat(def.m_font.m_name.c_str(), nullptr, def.m_font.m_weight, def.m_font.m_style, def.m_font.m_stretch, def.m_font.m_size, L"en-US", text_format.address_of()));

		// Create a text layout interface
		D3DPtr<IDWriteTextLayout> text_layout;
		Check(dwrite->CreateTextLayout(text.data(), UINT32(text.size()), text_format.get(), layout.m_dim.x, layout.m_dim.y, text_layout.address_of()));
		text_layout->SetTextAlignment(layout.m_align_h);
		text_layout->SetParagraphAlignment(layout.m_align_v);
		text_layout->SetWordWrapping(layout.m_word_wrapping);

		// Apply the formatting
		for (auto& fmt : formatting)
		{
			// A null range can be used to set the default font/style for the whole string
			if (fmt.empty())
				continue;

			// Font changes
			if (fmt.m_font.m_name      != def.m_font.m_name      ) text_layout->SetFontFamilyName(fmt.m_font.m_name.c_str() , fmt.m_range);
			if (fmt.m_font.m_size      != def.m_font.m_size      ) text_layout->SetFontSize      (fmt.m_font.m_size         , fmt.m_range);
			if (fmt.m_font.m_weight    != def.m_font.m_weight    ) text_layout->SetFontWeight    (fmt.m_font.m_weight       , fmt.m_range);
			if (fmt.m_font.m_style     != def.m_font.m_style     ) text_layout->SetFontStyle     (fmt.m_font.m_style        , fmt.m_range);
			if (fmt.m_font.m_stretch   != def.m_font.m_stretch   ) text_layout->SetFontStretch   (fmt.m_font.m_stretch      , fmt.m_range);
			if (fmt.m_font.m_underline != def.m_font.m_underline ) text_layout->SetUnderline     (fmt.m_font.m_underline    , fmt.m_range);
			if (fmt.m_font.m_strikeout != def.m_font.m_strikeout ) text_layout->SetStrikethrough (fmt.m_font.m_strikeout    , fmt.m_range);

			// Record if any of the text has alpha
			has_alpha |= HasAlpha(fmt.m_font.m_colour);
		}

		// Measure the formatted text
		DWRITE_TEXT_METRICS metrics;
		Check(text_layout->GetMetrics(&metrics));

		// The size of the text in device independent pixels, including padding.
		auto dip_size = v2(
			metrics.widthIncludingTrailingWhitespace + layout.m_padding.left + layout.m_padding.right,
			metrics.height + layout.m_padding.top + layout.m_padding.bottom);

		// DIP is defined as 1/96th of a logical inch (= 0.2645833 mm/px)
		// Font size 12pt is 16px high = 4.233mm (1pt = 1/72th of an inch)
		// Can choose the size arbitrarily so defaulting to 1pt = 1cm. 'scale' can be used to adjust this.
		constexpr float pt_to_px = 96.0f / 72.0f;  // This is used to find the required texture size.
		const float pt_to_m = 0.00828491f * scale; // This is used to create the quad as a multiple of the text size.

		// Determine the required texture size. This is controlled by the font size only.
		// DWrite draws in absolute pixels so there is no point in trying to scale the texture.
		auto text_size = dip_size;
		auto texture_size = Max(Ceil(text_size * pt_to_px), { 1, 1 });

		// Create a texture large enough to contain the text, and render the text into it
		constexpr auto format = DXGI_FORMAT_B8G8R8A8_UNORM;
		ResDesc td = ResDesc::Tex2D(Image{ s_cast<int>(texture_size.x), s_cast<int>(texture_size.y), nullptr, format }, 1)
			.heap_flags(D3D12_HEAP_FLAG_SHARED)
			.usage(EUsage::RenderTarget|EUsage::SimultaneousAccess)
			.clear(format, To<D3DCOLORVALUE>(layout.m_bk_colour))
			;
		TextureDesc tdesc = TextureDesc(AutoId, td).has_alpha(has_alpha).name("text_quad");
		auto tex = factory.CreateTexture2D(tdesc);

		// Render the text using DWrite
		{
			// Get a D2D device context to draw on the texture
			auto dc = tex->GetD2DeviceContext();

			// Apply different colours to text ranges
			for (auto& fmt : formatting)
			{
				if (fmt.empty()) continue;
				if (fmt.m_font.m_colour != def.m_font.m_colour)
				{
					D3DPtr<ID2D1SolidColorBrush> brush;
					Check(dc->CreateSolidColorBrush(To<D3DCOLORVALUE>(fmt.m_font.m_colour), brush.address_of()));
					brush->SetOpacity(fmt.m_font.m_colour.a);

					// Apply the colour
					text_layout->SetDrawingEffect(brush.get(), fmt.m_range);
				}
			}

			// Create the default text colour brush
			D3DPtr<ID2D1SolidColorBrush> brush_fr;
			Check(dc->CreateSolidColorBrush(To<D3DCOLORVALUE>(def.m_font.m_colour), brush_fr.address_of()));
			brush_fr->SetOpacity(def.m_font.m_colour.a);

			// Create the default text colour brush
			D3DPtr<ID2D1SolidColorBrush> brush_bk;
			Check(dc->CreateSolidColorBrush(To<D3DCOLORVALUE>(layout.m_bk_colour), brush_bk.address_of()));
			brush_bk->SetOpacity(layout.m_bk_colour.a);

			// Draw the string
			dc->BeginDraw();
			dc->Clear(To<D3DCOLORVALUE>(layout.m_bk_colour));
			dc->SetTextAntialiasMode(D2D1_TEXT_ANTIALIAS_MODE_DEFAULT);
			dc->DrawTextLayout({ layout.m_padding.left, layout.m_padding.top }, text_layout.get(), brush_fr.get(), D2D1_DRAW_TEXT_OPTIONS_NONE);
			Check(dc->EndDraw());
		}

		// Create a quad using this texture
		auto [vcount, icount] = geometry::QuadSize(1);

		// Return the text metrics size and the texture size
		dim_out = v4(text_size, texture_size);

		// Set the texture coordinates to match the text metrics and the quad size
		auto t2q = m4x4::Scale(text_size.x/texture_size.x, text_size.y/texture_size.y, 1.0f, v4::Origin()) * m4x4(v4::XAxis(), -v4::YAxis(), v4::ZAxis(), v4(0, 1, 0, 1));

		// Generate the geometry
		Cache cache{vcount, icount, 0, sizeof(uint16_t)};
		auto vptr = cache.m_vcont.data();
		auto iptr = cache.m_icont.data<uint16_t>();
		auto props = geometry::Quad(axis_id, layout.m_anchor, text_size.x * pt_to_m, text_size.y * pt_to_m, iv2::Zero(), Colour32White, t2q,
			[&](v4 p, Colour32 c, v4 n, v2 t) { SetPCNT(*vptr++, p, Colour(c), n, t); },
			[&](int idx) { *iptr++ = s_cast<uint16_t>(idx); });

		// Create a nugget
		cache.m_ncont.push_back(
			NuggetDesc(ETopo::TriList, props.m_geom & ~EGeom::Norm)
			.tex_diffuse(tex)
			.sam_diffuse(factory.CreateSampler(EStockSampler::AnisotropicClamp))
			.alpha_geom(has_alpha));

		cache.m_bbox = props.m_bbox;

		// Create the model
		return Create(factory, cache, opts);
	}
	ModelPtr ModelGenerator::Text(ResourceFactory& factory, std::wstring_view text, std::span<TextFormat const> formatting, TextLayout const& layout, float scale, AxisId axis_id, CreateOptions const* opts)
	{
		v4 dim_out;
		return Text(factory, text, formatting, layout, scale, axis_id, dim_out, opts);
	}
	ModelPtr ModelGenerator::Text(ResourceFactory& factory, std::wstring_view text, TextFormat const& formatting, TextLayout const& layout, float scale, AxisId axis_id, v4& dim_out, CreateOptions const* opts)
	{
		return Text(factory, text, { &formatting, 1 }, layout, scale, axis_id, dim_out, opts);
	}
	ModelPtr ModelGenerator::Text(ResourceFactory& factory, std::wstring_view text, TextFormat const& formatting, TextLayout const& layout, float scale, AxisId axis_id, CreateOptions const* opts)
	{
		v4 dim_out;
		return Text(factory, text, { &formatting, 1 }, layout, scale, axis_id, dim_out, opts);
	}
}
