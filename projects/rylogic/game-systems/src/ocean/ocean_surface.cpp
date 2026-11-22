//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "pr/game-systems/forward.h"
#include "pr/game-systems/ocean/ocean_surface.h"
#include "pr/game-systems/ocean/ocean_surface.hlsli"

#ifdef NDEBUG
#define PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(file) PR_STRINGISE(game-systems/src/shaders/hlsl/compiled/release/##file)
#else
#define PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(file) PR_STRINGISE(game-systems/src/shaders/hlsl/compiled/debug/##file)
#endif

namespace pr::game_systems::ocean
{
	namespace compiled
	{
		// The ocean's build-time shader bytecode
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(ocean_forward_vs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(ocean_forward_ps0.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(ocean_forward_ps1.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(ocean_forward_ps2.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(ocean_foam_cs.h)
	}

	namespace
	{
		// Throw when a COM operation fails.
		void CheckHr(HRESULT hr, char const* message)
		{
			if (FAILED(hr))
				throw std::runtime_error(message);
		}

		// Add two size values while rejecting overflow.
		size_t CheckedAdd(size_t lhs, size_t rhs, char const* message)
		{
			if (rhs > std::numeric_limits<size_t>::max() - lhs)
				throw std::overflow_error(message);

			return lhs + rhs;
		}

		// Multiply two size values while rejecting overflow.
		size_t CheckedMul(size_t lhs, size_t rhs, char const* message)
		{
			if (lhs != 0 && rhs > std::numeric_limits<size_t>::max() / lhs)
				throw std::overflow_error(message);

			return lhs * rhs;
		}

		// Convert a bounded size or logical count to the signed View3D ABI width.
		template <typename TValue>
		int CheckedToInt(TValue value, char const* message)
		{
			// Reject negative signed inputs and values outside the public object's count domain.
			if constexpr (std::is_signed_v<TValue>)
			{
				if (value < 0)
					throw std::overflow_error(message);
			}
			using Unsigned = std::make_unsigned_t<TValue>;
			if (static_cast<Unsigned>(value) > static_cast<Unsigned>(std::numeric_limits<int>::max()))
				throw std::overflow_error(message);

			return static_cast<int>(value);
		}

		// Require one option value to be finite and greater than zero.
		void RequirePositiveFinite(double value, char const* message)
		{
			if (!std::isfinite(value) || value <= 0.0)
				throw std::invalid_argument(message);
		}
	}

	// Retains the fixed layout of one ocean surface so that each frame can pack the complete procedural constants block.
	class OceanSurfaceImpl final : public OceanSurface
	{
		// Fixed fields are written once; Update overwrites the per-frame fields.
		OceanConstants m_constants;
		double m_radius_m;

		// CPU copies of the shader's wave elements, bathymetry heights, and wind-sea table and winds, so Height evaluates the drawn surface exactly.
		std::vector<WaterFieldElement> m_elements;
		std::vector<float> m_bathymetry;
		std::vector<WindSeaTableRow> m_table;
		std::vector<float2> m_winds;
		bool m_has_frame = false;

		// Height limits of the stored wind sea before shore sharpening (see WindSeaLimits), which each frame's shore sharpness raises.
		float m_wave_swash_m = 0.0f;
		float m_wave_height_m = 0.0f;

		// Foam settings that the history update applies each frame (see SetFoam), and the wave clock of the last history update, or NaN before
		// the first.
		double m_foam_tile_m = 1.0;
		double m_foam_fade_rate = 0.0;
		double m_foam_churn_mps = 0.0;
		std::array<double, 4> m_foam_churn = {};
		double m_foam_time_s = std::numeric_limits<double>::quiet_NaN();

		// An upper bound of every foam history texel, kept with the history update's own float operations, so it stays at or above every texel to
		// within a rounding error. When it reaches zero the history is empty, and the update and its sampling are skipped until crest foam can form.
		float m_foam_bound = 0.0f;

		bool m_show_ripple_strength = false;
		bool m_ripples = true;
		bool m_foam = true;

	public:

		// Validate the options and create the surface's procedural View3D object, returning the owner and the object.
		static std::unique_ptr<OceanSurface> Create(pr::view3d::OceanOptions const& options, char const* name, pr::view3d::Object& out_object)
		{
			// Validate the ABI inputs before copying them to the GPU. The wind-sea data is validated when it is stored below.
			auto const wind_sea_grid = ToWindSeaGrid(options.m_wind_sea_grid);
			if (options.m_wind_sea.m_elements == nullptr)
				throw std::invalid_argument("Water surface requires initial wave elements");
			if (options.m_bathymetry_heights == nullptr || options.m_bathymetry_width < 2 || options.m_bathymetry_height < 2)
				throw std::invalid_argument("Water surface bathymetry requires at least 2 x 2 heights");
			if (!std::isfinite(options.m_bathymetry_origin_x_m) || !std::isfinite(options.m_bathymetry_origin_y_m))
				throw std::invalid_argument("Water surface bathymetry origin must be finite");
			RequirePositiveFinite(options.m_bathymetry_cell_m, "Water surface bathymetry cell size must be finite and positive");
			RequirePositiveFinite(options.m_radius_m, "Ocean radius must be finite and positive");
			RequirePositiveFinite(options.m_half_height_m, "Water surface bounds height must be finite and positive");
			if (!std::isfinite(options.m_floor_height_m))
				throw std::invalid_argument("Water surface floor height must be finite");
			if (options.m_vertices_per_side < 3 || options.m_vertices_per_side > 4097 || options.m_vertices_per_side % 2 == 0)
				throw std::invalid_argument("Water surface lattice requires an odd vertex count per side between 3 and 4097");

			// The buffer regions have fixed sizes, so later wind-sea data is written in place: room for the most elements, then the bathymetry
			// heights, then the table rows on a 16-byte boundary, then the winds, then the foam history on a 16-byte boundary.
			auto const element_region = size_t{OceanMaxElements} * sizeof(WaterFieldElement);
			auto const node_count = CheckedMul(static_cast<size_t>(options.m_bathymetry_width), static_cast<size_t>(options.m_bathymetry_height), "Water surface bathymetry is too large");
			auto const bathymetry_bytes = CheckedMul(node_count, sizeof(float), "Water surface bathymetry is too large");
			auto const table_offset = (CheckedAdd(element_region, bathymetry_bytes, "Water surface buffer is too large") + 15) & ~size_t{15};
			auto const table_bytes = static_cast<size_t>(wind_sea_grid.band_count) * static_cast<size_t>(wind_sea_grid.speed_count) * sizeof(WindSeaTableRow);
			auto const winds_offset = CheckedAdd(table_offset, table_bytes, "Water surface buffer is too large");
			auto const wind_count = CheckedMul(static_cast<size_t>(wind_sea_grid.dims.x) * static_cast<size_t>(wind_sea_grid.dims.y), static_cast<size_t>(wind_sea_grid.level_count), "Water surface wind grid is too large");
			auto const winds_end = CheckedAdd(winds_offset, CheckedMul(wind_count, sizeof(float2), "Water surface wind grid is too large"), "Water surface buffer is too large");
			auto const history_offset = (winds_end + 15) & ~size_t{15};
			auto const history_bytes = size_t{4} * OceanFoamHistorySide * OceanFoamHistorySide * sizeof(float);
			auto const buffer_size = CheckedAdd(history_offset, history_bytes, "Water surface buffer is too large");
			if (buffer_size > std::numeric_limits<uint32_t>::max())
				throw std::invalid_argument("Water surface buffer exceeds 4 GB");

			// Record the fixed layout in the constants shared with the shader, then store the initial wind sea.
			auto owner = std::make_unique<OceanSurfaceImpl>();
			owner->m_radius_m = options.m_radius_m;
			owner->m_bathymetry.assign(options.m_bathymetry_heights, options.m_bathymetry_heights + node_count);
			owner->m_constants = {};
			owner->m_constants.layout = pr::hlsl::uint4(static_cast<uint32_t>(options.m_vertices_per_side), 0U, 0U, static_cast<uint32_t>(element_region));
			owner->m_constants.bathymetry = pr::hlsl::float4(static_cast<float>(options.m_bathymetry_origin_x_m), static_cast<float>(options.m_bathymetry_origin_y_m), static_cast<float>(options.m_bathymetry_cell_m), 0.0f);
			owner->m_constants.bathymetry_dims = pr::hlsl::uint4(static_cast<uint32_t>(options.m_bathymetry_width), static_cast<uint32_t>(options.m_bathymetry_height), 0U, 0U);
			owner->m_constants.mesh.w = static_cast<float>(options.m_floor_height_m);
			owner->m_constants.wind_sea = wind_sea_grid;
			owner->m_constants.wind_sea_offsets = pr::hlsl::uint4(static_cast<uint32_t>(table_offset), static_cast<uint32_t>(winds_offset), 0U, 0U);
			owner->m_constants.foam_history = pr::hlsl::uint4(static_cast<uint32_t>(history_offset), 0U, 0U, 0U);
			owner->SetFoam(DefaultFoam);
			owner->m_table.resize(table_bytes / sizeof(WindSeaTableRow));
			owner->m_winds.resize(wind_count);
			owner->StoreWindSea(options.m_wind_sea);

			// Fill the buffer from the stored copies. The foam history starts empty, as zeros.
			auto buffer = std::vector<uint8_t>(buffer_size);
			std::memcpy(buffer.data(), owner->m_elements.data(), owner->m_elements.size() * sizeof(WaterFieldElement));
			std::memcpy(buffer.data() + element_region, options.m_bathymetry_heights, bathymetry_bytes);
			std::memcpy(buffer.data() + table_offset, owner->m_table.data(), table_bytes);
			std::memcpy(buffer.data() + winds_offset, owner->m_winds.data(), wind_count * sizeof(float2));

			// The lattice and curtain indices depend only on the vertex count, so they are built once and shared by every surface.
			auto const side = static_cast<uint32_t>(options.m_vertices_per_side);
			auto const& indices = LatticeIndices(side);
			auto const surface_index_count = SurfaceIndexCount(side);

			// Use the caller's pixel shaders when given (they supply the ground height for the shore), otherwise the default ones, which have no shore.
			auto const has_overrides = options.m_forward_pixel[0].m_bytecode != nullptr;
			for (auto const& ps : options.m_forward_pixel)
			{
				if ((ps.m_bytecode != nullptr) != has_overrides || (ps.m_bytecode != nullptr && ps.m_size == 0))
					throw std::invalid_argument("Ocean pixel shader overrides require all three shaders or none");
			}
			auto const forward_pixel = has_overrides ? std::array<pr::view3d::ShaderByteCode, 3>{options.m_forward_pixel[0], options.m_forward_pixel[1], options.m_forward_pixel[2]} : std::array<pr::view3d::ShaderByteCode, 3>
			{
				pr::view3d::ShaderByteCode{compiled::ocean_forward_ps0, sizeof(compiled::ocean_forward_ps0)},
				pr::view3d::ShaderByteCode{compiled::ocean_forward_ps1, sizeof(compiled::ocean_forward_ps1)},
				pr::view3d::ShaderByteCode{compiled::ocean_forward_ps2, sizeof(compiled::ocean_forward_ps2)},
			};

			// Create the forward shader with its copied constants, and the ocean pixel shaders that add the foam (see ocean_forward_ps.hlsli). The
			// buffer is bound to the object once it exists. The object takes its own shader reference.
			auto const shader_options = pr::view3d::ShaderOptions
			{
				.m_struct_size = sizeof(pr::view3d::ShaderOptions),
				.m_version = pr::view3d::ShaderOptions::CurrentVersion,
				.m_stage = pr::view3d::EShaderStage::Vertex,
				.m_bytecode = compiled::ocean_forward_vs,
				.m_bytecode_size = sizeof(compiled::ocean_forward_vs),
				.m_dbg_name = "OceanForward",
				.m_procedural =
				{
					.m_rdr_step = pr::view3d::ERenderStep::ForwardRender,
					.m_constants = &owner->m_constants,
					.m_constants_size = sizeof(OceanConstants),
					.m_forward_pixel_model = pr::view3d::EForwardPixelModel::SimpleDetail,
					.m_forward_pixel =
					{
						forward_pixel[0],
						forward_pixel[1],
						forward_pixel[2],
					},
				},
			};
			auto forward = pr::view3d::ShaderPtr(View3D_ShaderCreate(shader_options));
			if (forward == nullptr)
				throw std::runtime_error("Failed to create the water surface shader");

			// Translucent water is drawn from both sides so it is also visible from below, and it never casts shadows.
			// The curtain is opaque, so the domain wall and what lies beyond it do not show through.
			auto nuggets = std::array<pr::view3d::Nugget, 2>{};
			auto& nugget = nuggets[0];
			nugget.m_topo = pr::view3d::ETopo::TriList;
			nugget.m_geom = pr::view3d::EGeom::Vert | pr::view3d::EGeom::Colr | pr::view3d::EGeom::Norm;
			nugget.m_v0 = 0;
			nugget.m_v1 = 1;
			nugget.m_i0 = 0;
			nugget.m_i1 = CheckedToInt(surface_index_count, "Water surface index count exceeds the View3D ABI");
			nugget.m_shaders[0] = {forward.get(), pr::view3d::ERenderStep::ForwardRender, 0};
			nugget.m_shaders[1] = {nullptr, pr::view3d::ERenderStep::Invalid, 0};
			nugget.m_nflags = pr::view3d::ENuggetFlag::GeometryHasAlpha | pr::view3d::ENuggetFlag::ShadowCastExclude;
			nugget.m_cull_mode = pr::view3d::ECullMode::None;
			nugget.m_fill_mode = pr::view3d::EFillMode::Default;
			nugget.m_tint = 0xFFFFFFFFU;
			nugget.m_rel_reflec = 1.0f;
			auto& curtain = nuggets[1];
			curtain = nugget;
			curtain.m_i0 = nugget.m_i1;
			curtain.m_i1 = CheckedToInt(indices.size(), "Water surface index count exceeds the View3D ABI");
			curtain.m_nflags = pr::view3d::ENuggetFlag::ShadowCastExclude;

			// The bounds cover the domain at any water level, down to the domain floor; the shader writes world positions directly.
			auto const bounds_lo = std::min(-options.m_half_height_m, options.m_floor_height_m);
			auto const bounds_hi = options.m_half_height_m;
			auto const options_create = pr::view3d::ObjectCreateOptions
			{
				.m_struct_size = sizeof(pr::view3d::ObjectCreateOptions),
				.m_version = pr::view3d::ObjectCreateOptions::CurrentVersion,
				.m_vertex_source = pr::view3d::EVertexSource::ProceduralVertexId,
				.m_vcount_logical = CheckedToInt(static_cast<size_t>(side) * side + size_t{2} * OceanCurtainSegments, "Water surface vertex count exceeds the View3D ABI"),
				.m_bbox =
				{
					pr::view3d::Vec4{0.0f, 0.0f, static_cast<float>(0.5 * (bounds_hi + bounds_lo)), 1.0f},
					pr::view3d::Vec4{static_cast<float>(options.m_radius_m), static_cast<float>(options.m_radius_m), static_cast<float>(0.5 * (bounds_hi - bounds_lo)), 0.0f},
				},
			};
			auto placeholder = pr::view3d::Vertex{};
			placeholder.pos = {0.0f, 0.0f, 0.0f, 1.0f};
			auto context_id = GUID{};
			CheckHr(CoCreateGuid(&context_id), "Failed to allocate the water surface object context");
			auto object = View3D_ObjectCreateU32(name, 0xFFFFFFFFU, 1, curtain.m_i1, static_cast<int>(nuggets.size()), &placeholder, indices.data(), nuggets.data(), options_create, context_id);
			if (object == nullptr)
				throw std::runtime_error("Failed to create the water surface object");

			// Small-scale roughness comes from a baked slope map, projected onto the surface in layers that Update sets from the wind.
			// The object keeps its own references to the texture and sampler.
			try
			{
				// The foam history is updated by a compute shader, so the buffer allows unordered access. The shaders take their own reference.
				auto gpu_buffer = D3DPtr<ID3D12Resource>(static_cast<ID3D12Resource*>(View3D_ProceduralBufferCreate(buffer.size(), buffer.data(), buffer.size(), TRUE, "OceanBuffer")), false);
				if (gpu_buffer == nullptr)
					throw std::runtime_error("Failed to create the water surface buffer");

				View3D_ObjectProceduralBufferSet(object, 0, gpu_buffer.get());

				// The pixel shader may declare a shore source at slot 1, which must always hold a buffer. Until SetShoreSource binds one, the
				// surface's own buffer stands in, and the zeroed shore constants mark the source as absent.
				View3D_ObjectProceduralBufferSet(object, 1, gpu_buffer.get());

				// The full mip chain averages the slopes away with distance, so far water does not alias. The averaged mean square slope keeps
				// the lost roughness, which View3D turns into a blurred, weaker reflection.
				if (options.m_detail_map == nullptr)
					throw std::invalid_argument("The ocean detail map is required (see BakeDetailMap)");

				auto texture_options = pr::view3d::TextureOptions{};
				texture_options.m_format = DXGI_FORMAT_R8G8B8A8_UNORM;
				texture_options.m_resource_state = D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE;
				texture_options.m_mips = 0;
				texture_options.m_multisamp = {1, 0};
				texture_options.m_t2s = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
				texture_options.m_dbg_name = "OceanDetail";
				auto texture = pr::view3d::TexturePtr(View3D_TextureCreate(RoughnessMapSize, RoughnessMapSize, options.m_detail_map, size_t{RoughnessMapSize} * RoughnessMapSize * sizeof(uint32_t), texture_options));
				auto sampler = pr::view3d::SamplerPtr(View3D_SamplerCreate(pr::view3d::SamplerOptions{D3D12_FILTER_ANISOTROPIC, D3D12_TEXTURE_ADDRESS_MODE_WRAP, D3D12_TEXTURE_ADDRESS_MODE_WRAP, D3D12_TEXTURE_ADDRESS_MODE_WRAP, "OceanDetail"}));
				if (texture == nullptr || sampler == nullptr)
					throw std::runtime_error("Failed to create the water detail map");

				// Both the surface and the curtain need the map, because the water's pixel shaders replace the detail-normal variants of the stock shaders.
				for (auto index = 0; index != static_cast<int>(nuggets.size()); ++index)
					View3D_ObjectNuggetDetailNormalsSet(object, texture.get(), sampler.get(), nullptr, index);
			}
			catch (...)
			{
				// The caller never receives the object, so release it here.
				View3D_ObjectDelete(object);
				throw;
			}

			out_object = object;
			return owner;
		}

		// Pack one frame's state into the constants and hand them to the object.
		void Update(pr::view3d::Object object, pr::view3d::OceanFrame const& frame) override
		{
			// Pack validates the frame before the object sees it.
			if (object == nullptr)
				throw std::invalid_argument("Water surface object is required");

			m_constants = Pack(frame);
			m_has_frame = true;

			// The history is updated only while it can hold foam: while crests can make foam, or until the last trails have faded. An empty
			// history is not read, and the next update starts without history, as the first one does.
			auto const foam_history = m_foam && (m_constants.foam_crest.x > 0.0f || m_foam_bound > 0.0f);
			if (foam_history)
			{
				UpdateFoam(frame);
			}
			else
			{
				m_foam_time_s = std::numeric_limits<double>::quiet_NaN();
				m_foam_bound = 0.0f;
				m_constants.foam_history.z = 0U;
			}

			View3D_ObjectProceduralConstantsSet(object, &m_constants, sizeof(m_constants));
			if (foam_history)
				View3D_ObjectProceduralBufferDispatch(object, compiled::ocean_foam_cs, sizeof(compiled::ocean_foam_cs), &m_constants, sizeof(m_constants), OceanFoamHistorySide / 8, OceanFoamHistorySide / 8, 2);

			// The roughness layers share the material, so updating them each frame replaces nothing. They are drawn at the reference wind with no
			// heading, and the pixel shader scales and turns them to the local wind (see RippleStrength in ocean_forward_ps.hlsli).
			auto const layers = RoughnessLayers(frame);
			View3D_ObjectDetailNormalLayersSet(object, layers.data(), static_cast<int>(layers.size()), 0.0f);
		}

		// Replace the wind-sea data of 'object', laid out by the surface's fixed grid. Null elements keep the current elements and table.
		// The drawn surface and Height both use the new data straight away.
		void SetWindSea(pr::view3d::Object object, pr::view3d::OceanWindSea const& wind_sea) override
		{
			// StoreWindSea validates everything before it changes anything, so a rejected update leaves the surface as it was.
			if (object == nullptr)
				throw std::invalid_argument("Water surface object is required");

			StoreWindSea(wind_sea);

			// Write only the regions that changed. Each write lands between frames, and the constants carry the matching element count and swash.
			if (wind_sea.m_elements != nullptr)
			{
				View3D_ObjectProceduralBufferWrite(object, 0, m_elements.data(), m_elements.size() * sizeof(WaterFieldElement));
				View3D_ObjectProceduralBufferWrite(object, m_constants.wind_sea_offsets.x, m_table.data(), m_table.size() * sizeof(WindSeaTableRow));
			}
			View3D_ObjectProceduralBufferWrite(object, m_constants.wind_sea_offsets.y, m_winds.data(), m_winds.size() * sizeof(float2));
			View3D_ObjectProceduralConstantsSet(object, &m_constants, sizeof(m_constants));
		}

		// Draw the ripple strength as colour instead of the shaded surface: blue for calm, green for a fresh breeze, and red at twice its roughness or
		// more. Takes effect from the next Update.
		void ShowRippleStrength(bool show) override
		{
			// Pack adds the flag to each frame's constants.
			m_show_ripple_strength = show;
		}

		// Draw or hide the small-scale ripple normals, and the foam and lip. Takes effect from the next Update.
		void SetEffects(bool ripples, bool foam) override
		{
			// Pack adds the flags to each frame's constants. Foam that is turned back on starts with no history, because its history was not updated.
			if (foam && !m_foam)
				m_foam_time_s = std::numeric_limits<double>::quiet_NaN();

			m_ripples = ripples;
			m_foam = foam;
		}

		// Validate 'foam' and store it in the constants. Pack copies the constants, so the foam is drawn from the next Update.
		void SetFoam(pr::view3d::OceanFoam const& foam) override
		{
			// Every value is used directly by the shaders. The shore foam depth, softness, tile size, crest range, and crest depth divide, and the
			// radius bounds the shore foam.
			auto const non_negative = { foam.m_crest_strength, foam.m_bump_m, foam.m_lip_strength, foam.m_churn_mps };
			for (auto const value : non_negative)
			{
				if (!std::isfinite(value) || value < 0.0)
					throw std::invalid_argument("Water foam values must be finite and non-negative");
			}
			RequirePositiveFinite(foam.m_shore_foam_depth_m, "Water shore foam depth must be finite and positive");
			RequirePositiveFinite(foam.m_edge_softness, "Water foam edge softness must be finite and positive");
			RequirePositiveFinite(foam.m_crest_threshold, "Water foam crest threshold must be finite and positive");
			RequirePositiveFinite(foam.m_tile_m, "Water foam tile size must be finite and positive");
			RequirePositiveFinite(foam.m_radius_m, "Water foam radius must be finite and positive");
			RequirePositiveFinite(foam.m_fade_m, "Water foam fade must be finite and positive");
			RequirePositiveFinite(foam.m_lip_height_m, "Water lip height must be finite and positive");
			RequirePositiveFinite(foam.m_crest_range, "Water foam crest range must be finite and positive");
			RequirePositiveFinite(foam.m_trail_time_s, "Water foam trail time must be finite and positive");
			RequirePositiveFinite(foam.m_crest_depth_m, "Water foam crest depth must be finite and positive");

			// The churn speed and fade rate change the history update rather than the shading, so Update applies them each frame.
			m_constants.foam_shore = pr::hlsl::float4(static_cast<float>(foam.m_shore_foam_depth_m), static_cast<float>(foam.m_edge_softness), 0.0f, 0.0f);
			m_constants.foam_crest = pr::hlsl::float4(static_cast<float>(foam.m_crest_strength), static_cast<float>(foam.m_crest_threshold), static_cast<float>(foam.m_crest_range), 0.0f);
			m_constants.foam_pattern = pr::hlsl::float4(static_cast<float>(foam.m_tile_m), static_cast<float>(foam.m_bump_m), static_cast<float>(foam.m_radius_m), static_cast<float>(foam.m_fade_m));
			m_constants.shore_lip = pr::hlsl::float4(static_cast<float>(foam.m_lip_height_m), static_cast<float>(foam.m_lip_strength), static_cast<float>(foam.m_crest_depth_m), 0.0f);
			m_foam_tile_m = foam.m_tile_m;
			m_foam_churn_mps = foam.m_churn_mps;
			m_foam_fade_rate = 1.0 / foam.m_trail_time_s;
		}

		// Bind the ground source that the pixel shaders read for the shore foam and lip, or the surface's own buffer when 'buffer' is null.
		// 'params' fill the 'shore_source' constants. Takes effect from the next Update.
		void SetShoreSource(pr::view3d::Object object, ID3D12Resource* buffer, std::span<uint32_t const, 12> params) override
		{
			// Without a source, the surface's own buffer stands in at slot 1 (see Create) and zeroed params mark the source as absent.
			if (object == nullptr)
				throw std::invalid_argument("Ocean object is required");
			if (buffer == nullptr)
			{
				View3D_ObjectProceduralBufferSet(object, 1, View3D_ObjectProceduralBufferGet(object, 0));
				std::memset(&m_constants.shore_source[0], 0, sizeof(m_constants.shore_source));
				return;
			}

			// The shaders read the source in place, so the ocean shares the caller's buffer.
			View3D_ObjectProceduralBufferSet(object, 1, buffer);
			static_assert(sizeof(m_constants.shore_source) == 12 * sizeof(uint32_t));
			std::memcpy(&m_constants.shore_source[0], params.data(), sizeof(m_constants.shore_source));
		}

		// Return the height (m) of the drawn surface at the world position 'x_m', 'y_m' in the last frame passed to Update.
		// Positions beyond the domain radius take the height at the domain edge, where the surface ends.
		double Height(double x_m, double y_m) const override
		{
			// The drawn surface is defined by the last frame's constants.
			if (!m_has_frame)
				throw std::logic_error("Water surface has no frame to sample");
			if (!std::isfinite(x_m) || !std::isfinite(y_m))
				throw std::invalid_argument("Water surface sample position must be finite");

			// Mirror the vertex shader's inputs (see Main and OceanSample in ocean_forward_vs.hlsl). A position on a lattice vertex then gets
			// that vertex's height exactly, and the camera position is always the centre vertex.
			auto const& c = m_constants;
			auto world_x = static_cast<float>(x_m);
			auto world_y = static_cast<float>(y_m);
			auto const centre_distance = std::hypot(world_x, world_y);
			if (centre_distance > c.level_radii.y)
			{
				// Vertices beyond the wall collapse onto it.
				world_x *= c.level_radii.y / centre_distance;
				world_y *= c.level_radii.y / centre_distance;
			}
			auto const world_xy = float2(world_x, world_y);
			auto const distance = std::hypot(world_x - c.camera_time.x, world_y - c.camera_time.y);
			auto const half_cells = 0.5f * static_cast<float>(c.layout.x - 1);
			auto const central_extent = OceanCentralExtent(c.mesh.x, half_cells, c.level_radii.z);
			auto const warp_power = OceanWarpPower(c.mesh.y, c.camera_time.z - c.level_radii.x, c.level_radii.z);
			auto const spacing = OceanSpacingAtDistance(distance, central_extent, c.level_radii.z, warp_power, half_cells);
			auto const wave_fade = OceanWaveFade(distance, c.level_radii.w);

			// Positions beyond the wave radius, and a calm sea, are flat, as in the shader.
			if (wave_fade <= 0.0f || OceanCalm(c.waves.y))
				return static_cast<double>(c.level_radii.x);

			// Interpolate the bathymetry as the shader does.
			auto const grid = WaterBathymetryGrid
			{
				.origin = float2(c.bathymetry.x, c.bathymetry.y),
				.cell_size = c.bathymetry.z,
				.pad = 0.0f,
				.dims = int2(static_cast<int>(c.bathymetry_dims.x), static_cast<int>(c.bathymetry_dims.y)),
				.pad2 = int2(0, 0),
			};
			auto const cell = WaterBathymetryLocate(grid, world_xy);
			auto const terrain = WaterBathymetryInterpolate(cell, m_bathymetry[cell.index.x], m_bathymetry[cell.index.y], m_bathymetry[cell.index.z], m_bathymetry[cell.index.w]);
			auto const depth = c.level_radii.x - terrain;

			// Evaluate the same local wind sea and waves as the vertex shader, from the CPU copies.
			struct SeaData
			{
				OceanSurfaceImpl const& m_owner;

				// Wind vector 'index' and table row 'index', indexed as in wind_sea.hlsli.
				float2 Wind(int index) const
				{
					return m_owner.m_winds[static_cast<size_t>(index)];
				}
				WindSeaTableRow Row(int index) const
				{
					return m_owner.m_table[static_cast<size_t>(index)];
				}
			};
			struct ElementSource
			{
				OceanSurfaceImpl const& m_owner;
				OceanSea const& m_sea;

				// Return element 'index' with its local amplitude and crest sharpness.
				WaterFieldElement Element(uint32_t index) const
				{
					return OceanSeaElement(m_owner.m_elements[index], m_sea);
				}
			};
			auto const sea = OceanSeaAt(SeaData{*this}, c.wind_sea, world_xy);
			auto const depth_effects = (c.layout.y & OceanFlagDepthEffects) != 0;
			auto const sample = OceanWaves(ElementSource{*this, sea}, c.layout.z, world_xy, c.camera_time.w, depth, c.waves.x, spacing, wave_fade, depth_effects, c.mesh.z, c.waves.z, c.shore_lip.z);
			return static_cast<double>(c.level_radii.x) + sample.surface.displacement_foam.z;
		}

		// Return the constants of the last frame passed to Update.
		OceanConstants const& Constants() const override
		{
			// Before the first frame the constants do not describe a drawn surface.
			if (!m_has_frame)
				throw std::logic_error("Water surface has no frame to sample");

			return m_constants;
		}

		// Build the shared index list for surfaces with 'vertices_per_side' vertices per side, so creating them later does not. Any thread may call this.
		static void PrepareLattice(int32_t vertices_per_side)
		{
			// Accept the same vertex counts as Create.
			if (vertices_per_side < 3 || vertices_per_side > 4097 || vertices_per_side % 2 == 0)
				throw std::invalid_argument("Water surface lattice requires an odd vertex count per side between 3 and 4097");

			LatticeIndices(static_cast<uint32_t>(vertices_per_side));
		}
		// Return the detail map (see BakeWaterDetailMap). It depends on nothing else, so it can be baked ahead of time.
		static std::vector<uint32_t> BakeDetailMap()
		{
			// The packed pixels hold R, G, B, A from the low byte up, which is the RGBA8 texture's byte order.
			static_assert(RoughnessMapSize == pr::view3d::OceanDetailMapSize);
			return BakeWaterDetailMap();
		}

	private:

		// Texels per side of the water detail map. The foam pattern's smallest bubbles need this many texels.
		static constexpr int RoughnessMapSize = 512;

		// Grid points per side on which the ripple slopes are summed before they are interpolated to the map size. The ripples have at most 16 waves
		// across the tile, so this grid holds them closely, at a quarter of the cost of summing at every texel.
		static constexpr int RippleBakeSize = 256;

		// One projection of the roughness map onto the surface. Different tile sizes and angles stop the layers from lining up, so the sum does not
		// visibly repeat. 'angle_rad' turns the layer from the wind heading, and 'weight' scales its share of the roughness. 'noise_cell_m' is the
		// world size of the noise patches that vary the layer's weight and bend its rows of tiles, which hides the tiling when many tiles are in view.
		struct RoughnessLayer
		{
			double tile_m;
			double angle_rad;
			double weight;
			double noise_cell_m;
		};
		static constexpr std::array<RoughnessLayer, 3> RoughnessLayerSet =
		{{
			{.tile_m = 6.1, .angle_rad = -0.30, .weight = 1.0, .noise_cell_m = 37.0},
			{.tile_m = 2.3, .angle_rad = 0.35, .weight = 0.8, .noise_cell_m = 29.0},
			{.tile_m = 0.83, .angle_rad = -0.12, .weight = 0.6, .noise_cell_m = 23.0},
		}};

		// How strongly the noise varies each layer's weight, in [0, 1].
		static constexpr double RoughnessWeightNoise = 0.7;

		// Largest shift of each layer's tiles by the noise, as a fraction of the noise cell. Shifts approaching a tile or more stop the rows of tiles
		// from lining up, while this fraction keeps the map's local stretch small (see pr::rdr12::materials::DetailNormalLayer).
		static constexpr double RoughnessWarp = 0.15;

		// Wavelength of the strongest ripples in the map, as a fraction of the tile. Each layer moves at the deep-water speed of this wavelength.
		static constexpr double RoughnessDominantWavelength = 1.0 / 5.0;


		// Return the number of lattice indices in the shared index list of 'side' vertices per side (see LatticeIndices); the curtain's follow them.
		static size_t SurfaceIndexCount(uint32_t side)
		{
			// Two triangles per lattice cell.
			return CheckedMul(static_cast<size_t>(side - 1) * (side - 1), size_t{6}, "Water surface index count overflowed");
		}

		// Return the shared index list of a lattice with 'side' vertices per side: the lattice's triangles, then the curtain's. The list is built on first
		// use for each side and kept for the life of the process, so the returned list stays valid. Any thread may call this.
		static std::vector<uint32_t> const& LatticeIndices(uint32_t side)
		{
			// Each vertex count has its own list. There is normally only one. Map nodes never move, so a list stays valid after the lock is released.
			static auto mutex = std::mutex{};
			static auto cache = std::unordered_map<uint32_t, std::vector<uint32_t>>{};
			auto lock = std::lock_guard{mutex};
			auto [entry, added] = cache.try_emplace(side);
			auto& indices = entry->second;
			if (!added)
				return indices;

			// Triangulate the lattice with each cell's diagonal pointing away from the centre, so the four quadrants mirror each other and the
			// warped disk has no directional bias. The list holds millions of indices, so they are written through a pointer into a sized list.
			auto const half = static_cast<int32_t>(side / 2);
			indices.resize(SurfaceIndexCount(side) + size_t{6} * OceanCurtainSegments);
			auto out = indices.data();
			for (auto y = uint32_t{}; y != side - 1; ++y)
			{
				// Each cell contributes two triangles.
				for (auto x = uint32_t{}; x != side - 1; ++x)
				{
					// The cell's quadrant decides which diagonal splits it.
					auto const v00 = y * side + x;
					auto const v10 = v00 + 1;
					auto const v01 = v00 + side;
					auto const v11 = v01 + 1;
					auto const dx = 2 * static_cast<int32_t>(x) + 1 - 2 * half;
					auto const dy = 2 * static_cast<int32_t>(y) + 1 - 2 * half;
					if (dx * dy > 0)
					{
						// The diagonal joins v00 and v11.
						out[0] = v00; out[1] = v10; out[2] = v11; out[3] = v00; out[4] = v11; out[5] = v01;
					}
					else
					{
						// The diagonal joins v10 and v01.
						out[0] = v00; out[1] = v10; out[2] = v01; out[3] = v10; out[4] = v11; out[5] = v01;
					}
					out += 6;
				}
			}

			// The curtain follows the lattice's vertex IDs as top and bottom pairs, one quad per segment, closing the ring at the last segment.
			auto const curtain_base = side * side;
			for (auto s = uint32_t{}; s != OceanCurtainSegments; ++s)
			{
				// Join this segment's top and bottom to the next segment's.
				auto const t0 = curtain_base + 2 * s;
				auto const t1 = curtain_base + 2 * ((s + 1) % OceanCurtainSegments);
				out[0] = t0; out[1] = t0 + 1; out[2] = t1 + 1; out[3] = t0; out[4] = t1 + 1; out[5] = t1;
				out += 6;
			}
			return indices;
		}


		// Return the RGBA8 detail map of the water: a tileable slope map of ripples that travel mainly along +u, and the foam pattern. R and G hold
		// the u and v slopes (per tile), mapped from [-1, 1] to [0, 1] and scaled so the steepest texel uses the full range. B holds half the sum of
		// the squared scaled slopes. A holds the foam pattern (see BakeFoamPattern).
		static std::vector<uint32_t> BakeWaterDetailMap()
		{
			// Each ripple has an integer wave vector so the map tiles. Its angular spread follows cos^2 of its angle from +u, and its slope falls
			// as 1/|k| so the finer ripples are weaker. Phases come from a fixed hash so every run bakes the same map.
			constexpr auto n = RippleBakeSize;
			constexpr auto tau_d = pr::math::constants<double>::tau;
			struct Ripple { int kx, ky; double cos_phase, sin_phase, slope_u, slope_v; };
			auto ripples = std::vector<Ripple>{};
			for (auto kx = 1; kx != 17; ++kx)
			{
				// Keep wave vectors in the ring 3 <= |k| <= 16.
				for (auto ky = -16; ky != 17; ++ky)
				{
					// Skip wave vectors outside the ring.
					auto const k = std::hypot(double(kx), double(ky));
					if (k < 3.0 || k > 16.0)
						continue;

					auto hash = static_cast<uint32_t>(kx * 73856093) ^ static_cast<uint32_t>((ky + 32) * 19349663);
					hash = (hash ^ (hash >> 16)) * 0x7feb352dU;
					hash = (hash ^ (hash >> 15)) * 0x846ca68bU;
					hash ^= hash >> 16;
					auto const phase = tau_d * static_cast<double>(hash) / 4294967296.0;
					auto const spread = (kx * kx) / (k * k);
					ripples.push_back({kx, ky, std::cos(phase), std::sin(phase), spread / k * kx / k, spread / k * ky / k});
				}
			}

			// Wave vectors are integers, so every phase is a whole number of 1/n turns and comes from a table.
			auto cos_table = std::vector<double>(n);
			auto sin_table = std::vector<double>(n);
			for (auto i = 0; i != n; ++i)
			{
				// One table entry per 1/n turn.
				cos_table[i] = std::cos(tau_d * i / n);
				sin_table[i] = std::sin(tau_d * i / n);
			}

			// The height is the sum of sin(k.p + phase) ripples, so its slope is the sum of k * cos(k.p + phase).
			auto slopes = std::vector<double>(2 * n * n);
			auto steepest = 0.0;
			for (auto y = 0; y != n; ++y)
			{
				// Sum every ripple at each texel of this row.
				for (auto x = 0; x != n; ++x)
				{
					// Accumulate the u and v slopes.
					auto su = 0.0;
					auto sv = 0.0;
					for (auto const& ripple : ripples)
					{
						// cos(a + phase) from the table entry for 'a'.
						auto const index = ((ripple.kx * x + ripple.ky * y) % n + n) % n;
						auto const c = cos_table[index] * ripple.cos_phase - sin_table[index] * ripple.sin_phase;
						su += ripple.slope_u * c;
						sv += ripple.slope_v * c;
					}
					slopes[2 * (y * n + x) + 0] = su;
					slopes[2 * (y * n + x) + 1] = sv;
					steepest = std::max({steepest, std::abs(su), std::abs(sv)});
				}
			}

			// Interpolate the slopes up to the map size, then normalise and pack. Bytes are R, G, B, A from the low byte up.
			constexpr auto size = RoughnessMapSize;
			auto const foam = BakeFoamPattern();
			auto pixels = std::vector<uint32_t>(size * size);
			for (auto i = 0; i != size * size; ++i)
			{
				// Blend the four surrounding grid points, wrapping at the tile's edge. Grid point 'j' and texel 'j * size / n' are at the same place.
				auto const x = double(i % size) * n / size;
				auto const y = double(i / size) * n / size;
				auto const x0 = static_cast<int>(x);
				auto const y0 = static_cast<int>(y);
				auto const fx = x - x0;
				auto const fy = y - y0;
				auto const slope = [&](int gx, int gy, int axis)
				{
					// The slope along 'axis' at a wrapped grid point.
					return slopes[2 * ((gy % n) * n + (gx % n)) + axis];
				};
				auto const blend = [&](int axis)
				{
					// Bilinear blend along x, then y.
					auto const lo = std::lerp(slope(x0, y0, axis), slope(x0 + 1, y0, axis), fx);
					auto const hi = std::lerp(slope(x0, y0 + 1, axis), slope(x0 + 1, y0 + 1, axis), fx);
					return std::lerp(lo, hi, fy);
				};

				// Map the slopes from [-1, 1] to [0, 255], and half the mean square slope from [0, 1].
				auto const su = blend(0) / steepest;
				auto const sv = blend(1) / steepest;
				auto const r = static_cast<uint32_t>(std::lround((su + 1.0) * 127.5));
				auto const g = static_cast<uint32_t>(std::lround((sv + 1.0) * 127.5));
				auto const b = static_cast<uint32_t>(std::lround((su * su + sv * sv) * 0.5 * 255.0));
				pixels[i] = (uint32_t{foam[i]} << 24) | (b << 16) | (g << 8) | r;
			}
			return pixels;
		}

		// Return the tileable foam pattern, RoughnessMapSize bytes square. The pattern is a network of foam filaments around large cells of open
		// water, with round bubbles of many sizes punched through it. The highest values lie on the filaments and furthest from bubble centres,
		// so as the threshold rises, dense foam first opens into round bubbles, the bubbles in the middle of each cell merge into open water with a
		// bubbly edge, and finally only thin filaments remain before they vanish. The values are spread evenly over [0, 255], so the pattern above
		// the threshold '255 * (1 - amount)' covers the fraction 'amount' of the tile.
		static std::vector<uint8_t> BakeFoamPattern()
		{
			// Random values in [0, 1) for each cell of each layer. Cells are wrapped by the caller, so the pattern tiles. A fixed hash makes every run
			// bake the same pattern.
			constexpr auto n = RoughnessMapSize;
			auto const random = [](int x, int y, uint32_t seed)
			{
				// Mix the cell and seed into one well-scrambled value.
				auto hash = static_cast<uint32_t>(x) * 0x8da6b343U ^ static_cast<uint32_t>(y) * 0xd8163841U ^ seed * 0xcb1ab31fU;
				hash = (hash ^ (hash >> 16)) * 0x7feb352dU;
				hash = (hash ^ (hash >> 15)) * 0x846ca68bU;
				hash ^= hash >> 16;
				return static_cast<double>(hash) / 4294967296.0;
			};
			auto const wrap = [](int i, int cells)
			{
				// The non-negative remainder.
				return ((i % cells) + cells) % cells;
			};

			// Smooth noise in [0, 1] at tile position (u, v), with 'cells' cells across the tile, for bending the holes and varying the density.
			auto const noise = [&](double u, double v, int cells, uint32_t seed)
			{
				// Blend the random values at the four surrounding lattice points.
				auto const x = u * cells;
				auto const y = v * cells;
				auto const x0 = static_cast<int>(std::floor(x));
				auto const y0 = static_cast<int>(std::floor(y));
				auto const fx = x - x0;
				auto const fy = y - y0;
				auto const sx = fx * fx * (3.0 - 2.0 * fx);
				auto const sy = fy * fy * (3.0 - 2.0 * fy);
				auto const a = random(wrap(x0, cells), wrap(y0, cells), seed);
				auto const b = random(wrap(x0 + 1, cells), wrap(y0, cells), seed);
				auto const c = random(wrap(x0, cells), wrap(y0 + 1, cells), seed);
				auto const d = random(wrap(x0 + 1, cells), wrap(y0 + 1, cells), seed);
				return std::lerp(std::lerp(a, b, sx), std::lerp(c, d, sx), sy);
			};

			// Return the closeness of tile position (u, v) to the nearest border between cells, with 'cells' cells across the tile: 1 on a border,
			// falling towards 0 over about 'width' cells. Each cell has one random point and covers the positions nearer to it than to any other
			// point, so the borders form a network of irregular polygons, like the filaments left by dissolving foam. The gap between the nearest
			// and second-nearest distances is zero on a border and grows towards the middle of a cell.
			auto const filaments = [&](double u, double v, int cells, double width, uint32_t seed)
			{
				// Points stay inside their own cell, so the 3x3 cells around the position contain the two nearest points.
				auto const x = u * cells;
				auto const y = v * cells;
				auto const cx = static_cast<int>(std::floor(x));
				auto const cy = static_cast<int>(std::floor(y));
				auto nearest = 1.0e9;
				auto second = 1.0e9;
				for (auto dy = -1; dy != 2; ++dy)
				{
					// Each row of neighbouring cells.
					for (auto dx = -1; dx != 2; ++dx)
					{
						// Keep the two smallest distances.
						auto const wx = wrap(cx + dx, cells);
						auto const wy = wrap(cy + dy, cells);
						auto const px = cx + dx + 0.1 + 0.8 * random(wx, wy, seed);
						auto const py = cy + dy + 0.1 + 0.8 * random(wx, wy, seed + 1);
						auto const distance = std::hypot(x - px, y - py);
						second = std::min(second, std::max(nearest, distance));
						nearest = std::min(nearest, distance);
					}
				}
				return std::exp(-(second - nearest) / width);
			};

			// Return how far tile position (u, v) is from the centre of the nearest bubble, as a fraction of that bubble's radius, up to 1 at and
			// beyond its rim. Each of the 'cells' cells across the tile has one bubble of random position and radius. Distances measured in radii
			// make every contour below 1 a circle, so the bubbles open as round holes.
			auto const bubbles = [&](double u, double v, int cells, uint32_t seed)
			{
				// Centres stay inside their own cell and radii stay under half a cell, so the 3x3 cells around the position hold every bubble that
				// can reach it.
				auto const x = u * cells;
				auto const y = v * cells;
				auto const cx = static_cast<int>(std::floor(x));
				auto const cy = static_cast<int>(std::floor(y));
				auto closest = 1.0;
				for (auto dy = -1; dy != 2; ++dy)
				{
					// Each row of neighbouring cells.
					for (auto dx = -1; dx != 2; ++dx)
					{
						// Keep the smallest distance in radii.
						auto const wx = wrap(cx + dx, cells);
						auto const wy = wrap(cy + dy, cells);
						auto const px = cx + dx + 0.15 + 0.7 * random(wx, wy, seed);
						auto const py = cy + dy + 0.15 + 0.7 * random(wx, wy, seed + 1);
						auto const radius = 0.2 + 0.28 * random(wx, wy, seed + 2);
						closest = std::min(closest, std::hypot(x - px, y - py) / radius);
					}
				}
				return closest;
			};

			// Score each texel. Large filaments shape the foam into cells and fine filaments split the cells, with noise varying their thickness so
			// some filaments outlast others. Bubbles of three sizes pierce the foam; where the filaments are weak the bubbles are the first to open,
			// so open water has a bubbly edge. Noise bends the filaments into curves, and broad noise varies the density, so some areas fill first.
			auto scores = std::vector<double>(n * n);
			for (auto y = 0; y != n; ++y)
			{
				// Each row of texels.
				for (auto x = 0; x != n; ++x)
				{
					// Bend the filaments with two octaves of noise. The bubbles are bent only slightly, so they stay round.
					auto const u = (x + 0.5) / n;
					auto const v = (y + 0.5) / n;
					auto const wu = u + 0.3 / 4.0 * (2.0 * noise(u, v, 8, 10) - 1.0) + 0.1 / 4.0 * (2.0 * noise(u, v, 24, 12) - 1.0);
					auto const wv = v + 0.3 / 4.0 * (2.0 * noise(u, v, 8, 11) - 1.0) + 0.1 / 4.0 * (2.0 * noise(u, v, 24, 13) - 1.0);
					auto const bu = u + 0.004 * (2.0 * noise(u, v, 40, 14) - 1.0);
					auto const bv = v + 0.004 * (2.0 * noise(u, v, 40, 15) - 1.0);

					// Combine the network and the bubbles.
					auto const thickness = 0.5 + noise(u, v, 12, 16);
					auto const large = filaments(wu, wv, 4, 0.12 * thickness, 20);
					auto const fine = filaments(wu, wv, 11, 0.08 * thickness, 21);
					auto const bubble = std::min({bubbles(bu, bv, 14, 30), bubbles(bu, bv, 29, 33), bubbles(bu, bv, 57, 36)});
					auto const density = 0.5 * noise(u, v, 3, 50) + 0.3 * noise(u, v, 7, 51) + 0.2 * noise(u, v, 15, 52);
					scores[y * n + x] = 0.5 * large + 0.2 * fine + 0.3 * bubble + 0.15 * (density - 0.5);
				}
			}
			// Replace each score by its rank, so the values are spread evenly. Ties keep a stable order, so every run bakes the same pattern.
			auto order = std::vector<int>(n * n);
			std::iota(order.begin(), order.end(), 0);
			std::stable_sort(order.begin(), order.end(), [&](int lhs, int rhs)
			{
				// Lower scores rank first.
				return scores[lhs] < scores[rhs];
			});
			auto pattern = std::vector<uint8_t>(n * n);
			for (auto rank = 0; rank != n * n; ++rank)
				pattern[order[rank]] = static_cast<uint8_t>(std::lround(255.0 * rank / (n * n - 1)));

			return pattern;
		}

		// Return the roughness layers for one frame at the reference steepness (see OceanRippleSteepness), unturned. The pixel shader scales and turns
		// them to the local wind. Each layer drifts along its own direction at the speed of its ripples.
		static std::array<pr::view3d::DetailNormalLayer, RoughnessLayerSet.size()> RoughnessLayers(pr::view3d::OceanFrame const& frame)
		{
			// Each layer's ripples travel at the deep-water speed of the map's dominant wavelength.
			constexpr auto tau_d = pr::math::constants<double>::tau;
			constexpr auto gravity = 9.81;

			auto layers = std::array<pr::view3d::DetailNormalLayer, RoughnessLayerSet.size()>{};
			for (auto i = size_t{}; i != layers.size(); ++i)
			{
				// The u axis points along the layer's travel. Each row has a magnitude of 1/tile, so world slope = height scale * map slope / tile.
				auto const& layer = RoughnessLayerSet[i];
				auto const angle = layer.angle_rad;
				auto const dx = std::cos(angle) / layer.tile_m;
				auto const dy = std::sin(angle) / layer.tile_m;
				auto const speed = std::sqrt(gravity * layer.tile_m * RoughnessDominantWavelength / tau_d);
				auto const offset = -std::fmod(speed * frame.m_wave_time_s / layer.tile_m, 1.0);
				layers[i].m_row_u = pr::view3d::Vec4{static_cast<float>(dx), static_cast<float>(dy), 0.0f, static_cast<float>(offset)};
				layers[i].m_row_v = pr::view3d::Vec4{static_cast<float>(-dy), static_cast<float>(dx), 0.0f, 0.0f};
				layers[i].m_height_scale = static_cast<float>(OceanRippleReferenceSteepness * layer.weight * layer.tile_m);
				layers[i].m_weight_noise = static_cast<float>(RoughnessWeightNoise);
				layers[i].m_noise_frequency = static_cast<float>(layer.tile_m / layer.noise_cell_m);
				layers[i].m_warp = static_cast<float>(RoughnessWarp * layer.noise_cell_m / layer.tile_m);
			}
			return layers;
		}

		// Advance the foam history constants to 'frame', which Pack has already validated: move the history windows to the camera, flip the
		// slot to write, and move the noise that makes the foam pattern wander. Before the first update, the previous windows are placed far from any texel, so the
		// first update reads no history.
		void UpdateFoam(pr::view3d::OceanFrame const& frame)
		{
			// A clock that runs backwards or jumps, such as after a pause or reset, ages the foam by at most a quarter second.
			auto& c = m_constants;
			auto const first = std::isnan(m_foam_time_s);
			auto const dt = first ? 0.0 : std::clamp(frame.m_wave_time_s - m_foam_time_s, 0.0, 0.25);
			m_foam_time_s = frame.m_wave_time_s;

			// Centre each window on the camera's texel, keeping the windows of the last update for reading the history.
			auto const origin = [](double position, float texel)
			{
				// The camera's texel, less half the window.
				return static_cast<int>(std::floor(position / texel)) - OceanFoamHistorySide / 2;
			};
			auto const far_away = std::numeric_limits<int>::min() / 2;
			c.foam_prev_origins = first ? pr::hlsl::int4(far_away, far_away, far_away, far_away) : c.foam_origins;
			c.foam_origins = pr::hlsl::int4(
				origin(frame.m_camera_x_m, OceanFoamNearTexel), origin(frame.m_camera_y_m, OceanFoamNearTexel),
				origin(frame.m_camera_x_m, OceanFoamFarTexel), origin(frame.m_camera_y_m, OceanFoamFarTexel));
			c.foam_history.y = first ? 0U : 1U - c.foam_history.y;
			c.foam_history.z = 1U;

			// The history update fades the trails by this update's share of the fade rate.
			c.foam_fade = pr::hlsl::float4(static_cast<float>(dt), static_cast<float>(m_foam_fade_rate), 0.0f, 0.0f);

			// Track the history's bound as water_foam_cs.hlsl updates each texel: fade, then renew with crest foam, which is at most the crest strength.
			auto const previous_bound = first ? 0.0f : m_foam_bound;
			m_foam_bound = std::max(std::max(previous_bound - c.foam_fade.y * c.foam_fade.x, 0.0f), c.foam_crest.x);

			// The warp noise moves so that the pattern wanders at about the churn speed: a noise change of one moves the pattern by twice the warp
			// shift, and smooth noise changes by about one per cell. Each offset wraps at the noise period, where the noise repeats, so it never jumps.
			auto const churn_cells = m_foam_churn_mps * dt / (2.0 * OceanFoamWarpShift * m_foam_tile_m);
			auto const dirs = std::array<double, 4>{OceanFoamChurnDirX.x, OceanFoamChurnDirX.y, OceanFoamChurnDirY.x, OceanFoamChurnDirY.y};
			auto const period = static_cast<double>(OceanFoamNoisePeriod);
			for (int i = 0; i != 4; ++i)
			{
				// Keep each offset in [0, period) so the float offset keeps its precision.
				auto const offset = m_foam_churn[i] + churn_cells * dirs[i];
				m_foam_churn[i] = offset - period * std::floor(offset / period);
			}
			c.foam_churn = pr::hlsl::float4(
				static_cast<float>(m_foam_churn[0]), static_cast<float>(m_foam_churn[1]),
				static_cast<float>(m_foam_churn[2]), static_cast<float>(m_foam_churn[3]));
		}

		// Validate one frame and return the complete constants block that draws it.
		OceanConstants Pack(pr::view3d::OceanFrame const& frame) const
		{
			// Reject values the shader cannot use; the caller owns sensible magnitudes.
			if (!std::isfinite(frame.m_camera_x_m) || !std::isfinite(frame.m_camera_y_m) || !std::isfinite(frame.m_camera_z_m) || !std::isfinite(frame.m_wave_time_s) || !std::isfinite(frame.m_level_m))
				throw std::invalid_argument("Water surface camera, clock, and level must be finite");
			RequirePositiveFinite(frame.m_wave_radius_m, "Water surface wave radius must be finite and positive");
			RequirePositiveFinite(frame.m_min_spacing_m, "Water surface vertex spacing must be finite and positive");
			if (!std::isfinite(frame.m_warp_power) || frame.m_warp_power < 1.0)
				throw std::invalid_argument("Water surface warp power must be at least one");
			RequirePositiveFinite(frame.m_breaking_ratio, "Water surface breaking ratio must be finite and positive");
			if (!(frame.m_shore_sharpness >= 0.0 && frame.m_shore_sharpness <= WaterFieldMaxCrestSharpness))
				throw std::invalid_argument(std::format("Water surface shore sharpness must be in [0, {}]", WaterFieldMaxCrestSharpness));

			// The disk reaches from the camera to the far side of the domain, wherever the camera is.
			auto constants = m_constants;
			auto const outer_radius = std::hypot(frame.m_camera_x_m, frame.m_camera_y_m) + m_radius_m;
			constants.camera_time = pr::hlsl::float4(static_cast<float>(frame.m_camera_x_m), static_cast<float>(frame.m_camera_y_m), static_cast<float>(frame.m_camera_z_m), static_cast<float>(frame.m_wave_time_s));
			constants.level_radii = pr::hlsl::float4(static_cast<float>(frame.m_level_m), static_cast<float>(m_radius_m), static_cast<float>(outer_radius), static_cast<float>(frame.m_wave_radius_m));
			constants.mesh = pr::hlsl::float4(static_cast<float>(frame.m_min_spacing_m), static_cast<float>(frame.m_warp_power), static_cast<float>(frame.m_breaking_ratio), m_constants.mesh.w);
			constants.layout.y = (frame.m_depth_effects != 0 ? OceanFlagDepthEffects : 0U) | (m_ripples ? OceanFlagRipples : 0U) | (m_foam ? OceanFlagFoam : 0U) | (m_show_ripple_strength ? OceanFlagShowRippleStrength : 0U);
			constants.waves = WavesConstants(static_cast<float>(frame.m_shore_sharpness));
			auto const colour = pr::Colour(pr::Colour32(frame.m_colour_argb));
			constants.colour = pr::hlsl::float4(colour.r, colour.g, colour.b, colour.a);
			return constants;
		}

		// Return the wind-sea grid of 'options', validated as the physics wind sea validates it.
		static WindSeaGrid ToWindSeaGrid(pr::view3d::OceanWindSeaGrid const& options)
		{
			// Every evaluator indexes the data from this layout, so it must be usable before anything is sized from it.
			if (!std::isfinite(options.m_origin_x_m) || !std::isfinite(options.m_origin_y_m))
				throw std::invalid_argument("Water wind-sea grid origin must be finite");
			RequirePositiveFinite(options.m_cell_m, "Water wind-sea cell size must be finite and positive");
			RequirePositiveFinite(options.m_speed_step_mps, "Water wind-sea speed step must be finite and positive");
			if (options.m_width < 2 || options.m_width > 4096 || options.m_height < 2 || options.m_height > 4096)
				throw std::invalid_argument("Water wind-sea grid requires between 2 and 4096 nodes along each side");
			if (options.m_band_count < 1 || options.m_band_count > static_cast<int32_t>(OceanMaxBands))
				throw std::invalid_argument(std::format("Water wind sea requires between 1 and {} bands", OceanMaxBands));
			if (options.m_speed_count < 2 || options.m_speed_count > 65536)
				throw std::invalid_argument("Water wind-sea table requires between 2 and 65536 rows per band");
			if (options.m_level_count < 1 || options.m_level_count > options.m_band_count)
				throw std::invalid_argument("Water wind-sea level count must be in [1, band_count]");

			auto grid = WindSeaGrid{};
			grid.origin = float2(static_cast<float>(options.m_origin_x_m), static_cast<float>(options.m_origin_y_m));
			grid.cell_size = static_cast<float>(options.m_cell_m);
			grid.speed_step = static_cast<float>(options.m_speed_step_mps);
			grid.dims = int2(options.m_width, options.m_height);
			grid.band_count = options.m_band_count;
			grid.speed_count = options.m_speed_count;
			grid.level_count = options.m_level_count;
			return grid;
		}

		// Validate 'wind_sea' against the fixed grid, then store it in the CPU copies and the constants, with the swash allowance it implies.
		// Null elements keep the current elements and table. Nothing changes unless every value is valid.
		void StoreWindSea(pr::view3d::OceanWindSea const& wind_sea)
		{
			// Validate the elements and table, which are replaced together.
			auto const& grid = m_constants.wind_sea;
			auto const elements = static_cast<WaterFieldElement const*>(wind_sea.m_elements);
			if (elements != nullptr)
			{
				if (wind_sea.m_element_count < 1 || wind_sea.m_element_count > static_cast<int32_t>(OceanMaxElements))
					throw std::invalid_argument(std::format("Water wind sea requires between 1 and {} wave elements", OceanMaxElements));
				if (wind_sea.m_table == nullptr)
					throw std::invalid_argument("Water wind-sea elements require a table");
				for (auto i = 0; i != wind_sea.m_element_count; ++i)
				{
					// An element names no band, or one of the grid's bands.
					if (WaterFieldWindSeaBand(elements[i]) >= grid.band_count)
						throw std::invalid_argument("Water wind-sea element names a band beyond the grid");
				}
				for (auto i = size_t{}; i != m_table.size(); ++i)
				{
					// Every evaluator uses the rows directly. See WindSeaTableRow.
					auto const row = &wind_sea.m_table[4 * i];
					if (!std::isfinite(row[0]) || row[0] < 0.0f || !std::isfinite(row[1]) || row[1] < 0.0f || !std::isfinite(row[3]) || row[3] < 0.0f)
						throw std::invalid_argument("Water wind-sea table rows must be finite and non-negative");
					if (!(row[2] >= 0.0f && row[2] <= WaterFieldMaxCrestSharpness))
						throw std::invalid_argument(std::format("Water wind-sea crest sharpness must be in [0, {}]", WaterFieldMaxCrestSharpness));
				}
			}

			// Validate the winds, which are always replaced, and find the strongest wind of each level in the same pass. This pass covers the whole grid on
			// every update, so it uses plain arithmetic and comparisons only, which stay cheap in unoptimised builds. A range comparison rejects NaN and infinity.
			if (wind_sea.m_winds == nullptr)
				throw std::invalid_argument("Water wind sea requires winds");

			auto const level_count = static_cast<size_t>(grid.level_count);
			auto level_speed_sq = std::array<float, OceanMaxBands>{};
			for (auto n = size_t{}; n != m_winds.size(); n += level_count)
			{
				// Winds are stored level by level within each node.
				auto const node = &wind_sea.m_winds[2 * n];
				for (auto l = size_t{}; l != level_count; ++l)
				{
					// Wind vectors may point anywhere, but must be finite.
					auto const x = node[2 * l + 0];
					auto const y = node[2 * l + 1];
					if (!(x >= -FLT_MAX && x <= FLT_MAX && y >= -FLT_MAX && y <= FLT_MAX))
						throw std::invalid_argument("Water wind-sea winds must be finite");

					auto const speed_sq = x * x + y * y;
					if (speed_sq > level_speed_sq[l])
						level_speed_sq[l] = speed_sq;
				}
			}

			// Store the validated data.
			if (elements != nullptr)
			{
				// The elements are kept in order of increasing wavelength, so shaders can stop at the first wave that a filter fully removes or keeps.
				m_elements.assign(elements, elements + wind_sea.m_element_count);
				std::ranges::sort(m_elements, {}, [](WaterFieldElement const& element)
				{
					return element.wave.y;
				});
				std::memcpy(m_table.data(), wind_sea.m_table, m_table.size() * sizeof(WindSeaTableRow));
				m_constants.layout.z = static_cast<uint32_t>(wind_sea.m_element_count);
			}
			std::memcpy(m_winds.data(), wind_sea.m_winds, m_winds.size() * sizeof(float2));

			// The strongest wind of each level bounds every blend of its node winds, and so bounds the waves.
			auto level_speed = std::array<float, OceanMaxBands>{};
			for (auto l = size_t{}; l != level_count; ++l)
				level_speed[l] = std::sqrt(level_speed_sq[l]);

			auto const limits = WindSeaLimits(level_speed);
			m_wave_swash_m = limits.swash;
			m_wave_height_m = limits.height;
			m_constants.waves = WavesConstants(m_constants.waves.z);
		}

		// Return the 'waves' constants for the stored wind sea with crests sharpened towards 'shore_sharpness' near the breaking height.
		// Shore sharpening raises a crest of sharpness r to at most (1 + r)(1 + shore_sharpness) times its amplitude, so the height bound grows by that factor.
		pr::hlsl::float4 WavesConstants(float shore_sharpness) const
		{
			// The swash allowance does not depend on crest shape.
			return pr::hlsl::float4(m_wave_swash_m, m_wave_height_m * (1.0f + shore_sharpness), shore_sharpness, 0.0f);
		}

		// Height limits of the stored wind sea.
		struct WindSeaBounds
		{
			// Swash allowance (m), see WaterFieldSwashDepth.
			float swash;

			// Largest possible rise or fall (m) of the drawn surface from the still-water level.
			float height;
		};

		// Return the height limits of the stored wind sea, given the strongest stored wind speed (m/s) of each level. The swash allowance is derived
		// from the amplitude bounds exactly as the physics water field derives it (see WaterField::UpdateBounds and WindSea), so the drawn waves run up
		// the shore as far as the physics waves.
		WindSeaBounds WindSeaLimits(std::array<float, OceanMaxBands> const& level_speed) const
		{
			// The grid gives each band's levels and the table layout.
			auto const& grid = m_constants.wind_sea;

			// Elements outside the wind sea add their own squares and amplitude bounds. Each band with elements adds the largest x * x * w of the
			// table rows its wind can reach, including the row after the strongest wind's row, because local values interpolate between two rows.
			// Its elements' crest heights are at most the largest reachable amplitude times one plus the largest reachable sharpness, because the
			// alignment factor of WindSeaAmplitude is at most one.
			auto amplitude_squares = 0.0f;
			auto amplitude_sum = 0.0f;
			auto band_elements = std::array<int, OceanMaxBands>{};
			for (auto const& element : m_elements)
			{
				// Wind-sea elements are counted per band below.
				auto const band = WaterFieldWindSeaBand(element);
				if (band >= 0)
				{
					++band_elements[static_cast<size_t>(band)];
					continue;
				}
				auto const amplitude = WaterFieldElementAmplitudeBound(element);
				amplitude_squares += amplitude * amplitude;
				amplitude_sum += amplitude;
			}
			for (auto b = 0; b != grid.band_count; ++b)
			{
				// Skip bands without elements.
				if (band_elements[static_cast<size_t>(b)] == 0)
					continue;

				auto l1 = 0;
				auto frac = 0.0f;
				auto const l0 = WindSeaLevelIndex(grid, b, l1, frac);
				auto const max_speed = std::max(level_speed[static_cast<size_t>(l0)], level_speed[static_cast<size_t>(l1)]);
				auto const last = std::min(static_cast<int>(max_speed / grid.speed_step) + 1, grid.speed_count - 1);
				auto square_bound = 0.0f;
				auto amplitude_bound = 0.0f;
				auto sharpness_bound = 0.0f;
				for (auto r = 0; r <= last; ++r)
				{
					// See WindSeaTableRow: x * x * w is twice the band's variance.
					auto const& row = m_table[static_cast<size_t>(b * grid.speed_count + r)];
					square_bound = std::max(square_bound, row.x * row.x * row.w);
					amplitude_bound = std::max(amplitude_bound, row.x);
					sharpness_bound = std::max(sharpness_bound, row.z);
				}
				amplitude_squares += square_bound;
				amplitude_sum += band_elements[static_cast<size_t>(b)] * amplitude_bound * (1.0f + sharpness_bound);
			}

			// Shoaling can raise each amplitude by at most WaterFieldMaxShoaling, while the breaking limit, lattice filter, and fade only lower it.
			return WindSeaBounds{
				.swash = WaterFieldSwashDepth(amplitude_squares),
				.height = WaterFieldMaxShoaling * amplitude_sum,
			};
		}

		// The foam each surface starts with: shore foam in water shallower than 0.15 m, whitecaps from a crest steepness of 0.3, and a 4 m foam
		// texture tile. Shore foam and the lip are drawn within 150 m of the camera and fade out over the last 50 m. The water darkens over
		// its last 5 cm. Crests are fully covered at a crest steepness of 0.5, and leave trails that fade
		// over 6 s. Crest foam is gone in water shallower than 0.2 m. The pattern wanders at about 0.1 m/s.
		static constexpr pr::view3d::OceanFoam DefaultFoam =
		{
			.m_shore_foam_depth_m = 0.15,
			.m_edge_softness = 0.15,
			.m_crest_strength = 1.0,
			.m_crest_threshold = 0.3,
			.m_tile_m = 4.0,
			.m_bump_m = 0.04,
			.m_radius_m = 150.0,
			.m_fade_m = 50.0,
			.m_lip_height_m = 0.05,
			.m_lip_strength = 0.7,
			.m_crest_range = 0.2,
			.m_trail_time_s = 6.0,
			.m_crest_depth_m = 0.2,
			.m_churn_mps = 0.1,
		};


	};

	// Create an ocean surface and its View3D object.
	std::unique_ptr<OceanSurface> OceanSurface::Create(pr::view3d::OceanOptions const& options, char const* name, pr::view3d::Object& out_object)
	{
		return OceanSurfaceImpl::Create(options, name, out_object);
	}

	// Build the shared index list for 'vertices_per_side'.
	void OceanSurface::PrepareLattice(int32_t vertices_per_side)
	{
		OceanSurfaceImpl::PrepareLattice(vertices_per_side);
	}

	// Bake the detail map that OceanOptions requires.
	std::vector<uint32_t> OceanSurface::BakeDetailMap()
	{
		return OceanSurfaceImpl::BakeDetailMap();
	}
}
