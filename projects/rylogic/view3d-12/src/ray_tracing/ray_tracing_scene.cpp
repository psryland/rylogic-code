//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/ray_tracing/ray_tracing_scene.h"
#include "pr/view3d-12/ray_tracing/ray_tracing_model.h"
#include "pr/view3d-12/ray_tracing/ray_tracing_resource.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/instance/instance.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/model/pose.h"
#include "pr/view3d-12/model/skinned_geometry.h"
#include "pr/view3d-12/model/vertex_layout.h"
#include "pr/view3d-12/utility/barrier_batch.h"
#include "view3d-12/src/shaders/common.h"

namespace pr::rdr12
{
	namespace
	{
		// Sync with ray_tracing_cbuf.hlsli.
		constexpr UINT RayTracingInstanceMask_Default = 0x01;
		constexpr UINT RayTracingInstanceMask_Caustic = 0x02;
		constexpr UINT RayTracingMaterialIndexLimit = 0x01000000;
		constexpr UINT RayTracingGeometryFlag_HasGeometry = 0x01;
		constexpr UINT RayTracingGeometryFlag_HasNormals = 0x02;
		constexpr UINT RayTracingGeometryFlag_Index16 = 0x04;
		constexpr UINT RayTracingGeometryFlag_Index32 = 0x08;

		struct SkinnedBlasKey
		{
			Model const* m_model;
			Pose const* m_pose;

			friend bool operator == (SkinnedBlasKey const&, SkinnedBlasKey const&) = default;
		};
		struct SkinnedBlasKeyHash
		{
			size_t operator () (SkinnedBlasKey const& key) const
			{
				auto hash = std::hash<Model const*>{}(key.m_model);
				hash = hash * 16777619u ^ std::hash<Pose const*>{}(key.m_pose);
				return hash;
			}
		};

		// Copy a View3D object-to-world transform into DXR's row-major 3x4 instance transform.
		void CopyTransform(m4x4 const& o2w, float (&transform)[3][4])
		{
			for (auto row = 0; row != 3; ++row)
			{
				auto const r = o2w.row(row);
				transform[row][0] = r.x;
				transform[row][1] = r.y;
				transform[row][2] = r.z;
				transform[row][3] = r.w;
			}
		}

		// Add POD data to a signature used to decide whether the TLAS must be rebuilt.
		template <typename T>
		void AddSignature(uint64_t& signature, T const& value)
		{
			signature = s_cast<uint64_t>(hash::HashBytes64(&value, &value + 1, signature));
		}

		// Return true if the first caustic proof pass should treat 'inst' as a transparent/refractive light contributor.
		bool IsCausticContributor(BaseInstance const& inst, Model const& model)
		{
			if (HasAlpha(inst))
				return true;

			for (auto const* nugget = model.m_nuggets.get(); nugget != nullptr; nugget = nugget->m_next.get())
			{
				if (AllSet(nugget->m_nflags, ENuggetFlag::Hidden))
					continue;
				if (nugget->RequiresAlpha())
					return true;
			}

			return false;
		}

	}

	struct RayTracingScene::Data
	{
		struct SkinnedBlas
		{
			D3DPtr<ID3D12Resource> m_blas;
			D3DPtr<ID3D12Resource> m_scratch;
			D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO m_prebuild_info;
			RayTracingGeometryStats m_stats;
			PosePtr m_pose;
			uint64_t m_model_revision;
			uint64_t m_pose_revision;
			int64_t m_vcount;

			// Create an empty dynamic skinned BLAS entry.
			SkinnedBlas()
				: m_blas()
				, m_scratch()
				, m_prebuild_info()
				, m_stats()
				, m_pose()
				, m_model_revision()
				, m_pose_revision()
				, m_vcount()
			{}
		};

		D3DPtr<ID3D12Resource> m_tlas;
		D3DPtr<ID3D12Resource> m_scratch;
		D3DPtr<ID3D12Resource> m_instances;
		D3DPtr<ID3D12Resource> m_materials;
		D3DPtr<ID3D12Resource> m_shading_vertices;
		D3DPtr<ID3D12Resource> m_shading_indices16;
		D3DPtr<ID3D12Resource> m_shading_indices32;
		D3DPtr<ID3D12Resource> m_shading_geometry;
		D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO m_prebuild_info;
		std::unordered_map<SkinnedBlasKey, SkinnedBlas, SkinnedBlasKeyHash> m_skinned_blas;
		uint64_t m_material_signature;
		uint64_t m_shading_geometry_signature;
		int m_material_count;
		int m_shading_vertex_count;
		int m_shading_index16_count;
		int m_shading_index32_count;
		int m_shading_geometry_count;
		int m_shading_geometry_fallback_count;

		// Create an empty heavy-weight TLAS data object.
		Data()
			: m_tlas()
			, m_scratch()
			, m_instances()
			, m_materials()
			, m_shading_vertices()
			, m_shading_indices16()
			, m_shading_indices32()
			, m_shading_geometry()
			, m_prebuild_info()
			, m_skinned_blas()
			, m_material_signature(hash::FNV_offset_basis64)
			, m_shading_geometry_signature(hash::FNV_offset_basis64)
			, m_material_count()
			, m_shading_vertex_count()
			, m_shading_index16_count()
			, m_shading_index32_count()
			, m_shading_geometry_count()
			, m_shading_geometry_fallback_count()
		{}
	};

	namespace
	{
		struct SkinnedBlasBuildResult
		{
			RayTracingGeometryStats m_stats;
			D3D12_GPU_VIRTUAL_ADDRESS m_address;
			ID3D12Resource* m_vertex_buffer;
			uint64_t m_pose_revision;
			bool m_cache_hit;
		};

		struct BufferCopy
		{
			ID3D12Resource* m_src;
			uint64_t m_src_offset;
			uint64_t m_dst_offset;
			uint64_t m_size;
		};

		struct InstanceBuildInput
		{
			RayTracingSceneStats m_stats;
			vector<D3D12_RAYTRACING_INSTANCE_DESC, 1024> m_instances;
			vector<SkinnedBlasKey, 32> m_used_skinned_blas;
			vector<shaders::rt::RayTracingMaterial, 1024> m_materials;
			vector<shaders::rt::RayTracingGeometry, 1024> m_shading_geometry;
			vector<BufferCopy, 1024> m_vertex_copies;
			vector<BufferCopy, 1024> m_index16_copies;
			vector<BufferCopy, 1024> m_index32_copies;
			uint64_t m_signature;
			uint64_t m_material_signature;
			uint64_t m_shading_geometry_signature;
			int m_shading_vertex_count;
			int m_shading_index16_count;
			int m_shading_index32_count;
			int m_shading_geometry_fallback_count;

			// Create empty TLAS build input.
			InstanceBuildInput()
				: m_stats()
				, m_instances()
				, m_used_skinned_blas()
				, m_materials()
				, m_shading_geometry()
				, m_vertex_copies()
				, m_index16_copies()
				, m_index32_copies()
				, m_signature(hash::FNV_offset_basis64)
				, m_material_signature(hash::FNV_offset_basis64)
				, m_shading_geometry_signature(hash::FNV_offset_basis64)
				, m_shading_vertex_count()
				, m_shading_index16_count()
				, m_shading_index32_count()
				, m_shading_geometry_fallback_count()
			{}
		};

		// Release the TLAS resources owned by 'data'.
		void ReleaseTlas(Renderer& rdr, RayTracingScene::Data& data)
		{
			rdr.DeferRelease(data.m_tlas);
			rdr.DeferRelease(data.m_scratch);
			rdr.DeferRelease(data.m_instances);
		}

		// Release the material buffer that closest-hit shaders use to shade reflected instances.
		void ReleaseMaterialBuffer(Renderer& rdr, RayTracingScene::Data& data)
		{
			rdr.DeferRelease(data.m_materials);
			data.m_material_count = 0;
			data.m_material_signature = hash::FNV_offset_basis64;
		}

		// Release the packed geometry buffers that closest-hit shaders use to reconstruct triangle attributes.
		void ReleaseShadingGeometryBuffers(Renderer& rdr, RayTracingScene::Data& data)
		{
			rdr.DeferRelease(data.m_shading_vertices);
			rdr.DeferRelease(data.m_shading_indices16);
			rdr.DeferRelease(data.m_shading_indices32);
			rdr.DeferRelease(data.m_shading_geometry);
			data.m_shading_vertex_count = 0;
			data.m_shading_index16_count = 0;
			data.m_shading_index32_count = 0;
			data.m_shading_geometry_count = 0;
			data.m_shading_geometry_fallback_count = 0;
			data.m_shading_geometry_signature = hash::FNV_offset_basis64;
		}

		// Release the dynamic BLAS resources owned by 'entry'.
		void ReleaseSkinnedBlas(Renderer& rdr, RayTracingScene::Data::SkinnedBlas& entry)
		{
			rdr.DeferRelease(entry.m_blas);
			rdr.DeferRelease(entry.m_scratch);
			entry.m_pose = nullptr;
			entry.m_model_revision = 0;
			entry.m_pose_revision = 0;
			entry.m_vcount = 0;
		}

		// Remove dynamic skinned BLAS entries that were not referenced by the current TLAS build.
		void PruneSkinnedBlas(Renderer& rdr, RayTracingScene::Data& data, std::span<SkinnedBlasKey const> used)
		{
			for (auto iter = begin(data.m_skinned_blas); iter != end(data.m_skinned_blas); )
			{
				// Keep cache entries that are still referenced by at least one skinned instance in this frame's TLAS.
				if (std::find(begin(used), end(used), iter->first) != end(used))
				{
					++iter;
					continue;
				}

				// Anything no longer reachable from the scene can release its GPU resources through the deferred-release path.
				ReleaseSkinnedBlas(rdr, iter->second);
				iter = data.m_skinned_blas.erase(iter);
			}
		}

		// Return the fixed-function material approximation used by the current reflection hit shader.
		shaders::rt::RayTracingMaterial MakeMaterial(BaseInstance const& inst, Nugget const& nugget)
		{
			auto const* inst_tint = inst.find<Colour32>(EInstComp::TintColour32);
			auto& base_colour = nugget.mat().ComponentOrDefault<materials::BaseColour>();
			auto const tint = Colour((inst_tint != nullptr ? *inst_tint : Colour32White) * base_colour.m_tint);

			shaders::rt::RayTracingMaterial material = {};
			material.diffuse = tint.rgba;
			return material;
		}

		// Append materials in the same order as the BLAS geometry descriptors and return the TLAS InstanceID material base.
		UINT AppendMaterials(InstanceBuildInput& result, BaseInstance const& inst, RayTracingGeometryBuildInput const& geometry)
		{
			if (geometry.m_nuggets.size() != geometry.m_geometry.size())
				throw std::runtime_error("Ray tracing material and geometry counts do not match");

			auto const material_base = result.m_materials.size();
			if (material_base + geometry.m_nuggets.size() > RayTracingMaterialIndexLimit)
				throw std::runtime_error("Ray tracing material table exceeds the DXR InstanceID range");

			for (auto const* nugget : geometry.m_nuggets)
			{
				auto material = MakeMaterial(inst, *nugget);
				AddSignature(result.m_material_signature, material);
				result.m_materials.push_back(material);
			}
			AddSignature(result.m_material_signature, result.m_materials.size());
			return s_cast<UINT>(material_base);
		}

		// Return the transform used to bring model-space normals into world space for closest-hit shading.
		m4x4 NormalToWorld(BaseInstance const& inst, Model const& model)
		{
			auto n2w = GetO2W(inst);
			n2w.x = Normalise(n2w.x, v4::Zero());
			n2w.y = Normalise(Cross(n2w.z, n2w.x), v4::Zero());
			n2w.z = Cross(n2w.x, n2w.y);
			return model.m_m2root * n2w;
		}

		// Append RT hit-shading metadata in the same order as material records and BLAS geometry descriptors.
		UINT AppendShadingGeometry(InstanceBuildInput& result, BaseInstance const& inst, Model const& model, RayTracingGeometryBuildInput const& geometry, ID3D12Resource* vertex_buffer, uint64_t source_revision)
		{
			if (geometry.m_nuggets.size() != geometry.m_geometry.size())
				throw std::runtime_error("Ray tracing shading geometry and BLAS geometry counts do not match");

			auto const geometry_base = result.m_shading_geometry.size();
			if (geometry_base + geometry.m_nuggets.size() > RayTracingMaterialIndexLimit)
				throw std::runtime_error("Ray tracing geometry table exceeds the DXR InstanceID range");

			auto const supported_vertex_layout = model.m_vstride.size() == sizeof(Vert);
			auto const supports_shading_sidecar = !model.m_skin && supported_vertex_layout && vertex_buffer != nullptr;
			auto const index16 = model.m_ib_view.Format == DXGI_FORMAT_R16_UINT;
			auto const index32 = model.m_ib_view.Format == DXGI_FORMAT_R32_UINT;
			auto const has_shading_sidecar = supports_shading_sidecar && (index16 || index32);
			auto const normal_to_world = has_shading_sidecar ? NormalToWorld(inst, model) : m4x4::Identity();
			for (auto const* nugget : geometry.m_nuggets)
			{
				auto record = shaders::rt::RayTracingGeometry{};
				if (has_shading_sidecar)
				{
					auto const vertex_offset = result.m_shading_vertex_count;
					auto const index_offset = index16 ? result.m_shading_index16_count : result.m_shading_index32_count;
					auto const vertex_count = s_cast<int>(nugget->m_vrange.size());
					auto const index_count = s_cast<int>(nugget->m_irange.size());

					record.ranges = pr::hlsl::uint4{
						s_cast<uint32_t>(vertex_offset),
						s_cast<uint32_t>(nugget->m_vrange.m_beg),
						s_cast<uint32_t>(index_offset),
						s_cast<uint32_t>(index_count),
					};
					record.flags = pr::hlsl::uint4{
						RayTracingGeometryFlag_HasGeometry |
						(AllSet(nugget->m_geom, EGeom::Norm) ? RayTracingGeometryFlag_HasNormals : 0U) |
						(index16 ? RayTracingGeometryFlag_Index16 : RayTracingGeometryFlag_Index32),
						s_cast<uint32_t>(vertex_count),
						0,
						0,
					};
					record.normal_to_world = normal_to_world;

					result.m_vertex_copies.push_back(BufferCopy{
						.m_src = vertex_buffer,
						.m_src_offset = s_cast<uint64_t>(nugget->m_vrange.m_beg) * sizeof(Vert),
						.m_dst_offset = s_cast<uint64_t>(vertex_offset) * sizeof(shaders::rt::RayTracingVertex),
						.m_size = s_cast<uint64_t>(vertex_count) * sizeof(Vert),
					});

					auto& index_copies = index16 ? result.m_index16_copies : result.m_index32_copies;
					auto const index_stride = s_cast<uint64_t>(model.m_istride.size());
					index_copies.push_back(BufferCopy{
						.m_src = const_cast<ID3D12Resource*>(model.m_ib.get()),
						.m_src_offset = s_cast<uint64_t>(nugget->m_irange.m_beg) * index_stride,
						.m_dst_offset = s_cast<uint64_t>(index_offset) * index_stride,
						.m_size = s_cast<uint64_t>(index_count) * index_stride,
					});

					result.m_shading_vertex_count += vertex_count;
					if (index16)
						result.m_shading_index16_count += index_count;
					else
						result.m_shading_index32_count += index_count;
				}
				else
				{
					// Keep a one-for-one geometry table entry so the shader can fall back to the material table instead of changing InstanceID semantics.
					++result.m_shading_geometry_fallback_count;
				}

				AddSignature(result.m_shading_geometry_signature, record);
				if ((record.flags.x & RayTracingGeometryFlag_HasGeometry) != 0)
				{
					AddSignature(result.m_shading_geometry_signature, source_revision);
					AddSignature(result.m_shading_geometry_signature, reinterpret_cast<uintptr_t>(vertex_buffer));
					AddSignature(result.m_shading_geometry_signature, reinterpret_cast<uintptr_t>(model.m_ib.get()));
				}
				result.m_shading_geometry.push_back(record);
			}

			AddSignature(result.m_shading_geometry_signature, result.m_shading_geometry.size());
			AddSignature(result.m_shading_geometry_signature, result.m_shading_vertex_count);
			AddSignature(result.m_shading_geometry_signature, result.m_shading_index16_count);
			AddSignature(result.m_shading_geometry_signature, result.m_shading_index32_count);
			return s_cast<UINT>(geometry_base);
		}

		// Create or refresh the material buffer consumed by reflection closest-hit shaders.
		void EnsureMaterialBuffer(Renderer& rdr, GfxCmdList& cmd_list, GpuUploadBuffer& upload, RayTracingScene::Data& data, std::span<shaders::rt::RayTracingMaterial const> materials, uint64_t material_signature)
		{
			if (data.m_materials != nullptr && data.m_material_count == isize(materials) && data.m_material_signature == material_signature)
				return;

			ReleaseMaterialBuffer(rdr, data);
			if (materials.empty())
				return;

			auto desc = ResDesc::Buf<shaders::rt::RayTracingMaterial>(isize(materials), materials)
				.def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			data.m_materials = CreateRayTracingResource(rdr, cmd_list, upload, desc, "RayTracing:materials");
			data.m_material_count = isize(materials);
			data.m_material_signature = material_signature;
		}

		// Copy the packed hit-shading source buffers and upload the geometry metadata table.
		void EnsureShadingGeometryBuffers(Renderer& rdr, GfxCmdList& cmd_list, GpuUploadBuffer& upload, RayTracingScene::Data& data, InstanceBuildInput const& input)
		{
			auto const buffers_current =
				data.m_shading_geometry != nullptr &&
				data.m_shading_geometry_signature == input.m_shading_geometry_signature &&
				data.m_shading_vertex_count == input.m_shading_vertex_count &&
				data.m_shading_index16_count == input.m_shading_index16_count &&
				data.m_shading_index32_count == input.m_shading_index32_count &&
				data.m_shading_geometry_count == isize(input.m_shading_geometry);
			if (buffers_current)
				return;

			ReleaseShadingGeometryBuffers(rdr, data);
			if (input.m_shading_geometry.empty())
				return;

			auto metadata_desc = ResDesc::Buf<shaders::rt::RayTracingGeometry>(
				isize(input.m_shading_geometry),
				std::span<shaders::rt::RayTracingGeometry const>(input.m_shading_geometry.data(), input.m_shading_geometry.size()))
				.def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			data.m_shading_geometry = CreateRayTracingResource(rdr, cmd_list, upload, metadata_desc, "RayTracing:shading geometry");

			if (input.m_shading_vertex_count != 0)
			{
				auto desc = ResDesc::Buf<shaders::rt::RayTracingVertex>(input.m_shading_vertex_count, {})
					.def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
				data.m_shading_vertices = CreateRayTracingResource(rdr, cmd_list, upload, desc, "RayTracing:shading vertices");
			}
			if (input.m_shading_index16_count != 0)
			{
				auto desc = ResDesc::Buf<uint16_t>(input.m_shading_index16_count, {})
					.def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
				data.m_shading_indices16 = CreateRayTracingResource(rdr, cmd_list, upload, desc, "RayTracing:shading indices16");
			}
			if (input.m_shading_index32_count != 0)
			{
				auto desc = ResDesc::Buf<uint32_t>(input.m_shading_index32_count, {})
					.def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
				data.m_shading_indices32 = CreateRayTracingResource(rdr, cmd_list, upload, desc, "RayTracing:shading indices32");
			}

			BarrierBatch before_copy(cmd_list);
			for (auto const& copy : input.m_vertex_copies)
			{
				before_copy.Transition(copy.m_src, D3D12_RESOURCE_STATE_COPY_SOURCE);
				before_copy.Transition(data.m_shading_vertices.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			}
			for (auto const& copy : input.m_index16_copies)
			{
				before_copy.Transition(copy.m_src, D3D12_RESOURCE_STATE_COPY_SOURCE);
				before_copy.Transition(data.m_shading_indices16.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			}
			for (auto const& copy : input.m_index32_copies)
			{
				before_copy.Transition(copy.m_src, D3D12_RESOURCE_STATE_COPY_SOURCE);
				before_copy.Transition(data.m_shading_indices32.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			}
			before_copy.Commit();

			for (auto const& copy : input.m_vertex_copies)
				cmd_list.CopyBufferRegion(data.m_shading_vertices.get(), copy.m_dst_offset, copy.m_src, copy.m_src_offset, copy.m_size);
			for (auto const& copy : input.m_index16_copies)
				cmd_list.CopyBufferRegion(data.m_shading_indices16.get(), copy.m_dst_offset, copy.m_src, copy.m_src_offset, copy.m_size);
			for (auto const& copy : input.m_index32_copies)
				cmd_list.CopyBufferRegion(data.m_shading_indices32.get(), copy.m_dst_offset, copy.m_src, copy.m_src_offset, copy.m_size);

			BarrierBatch after_copy(cmd_list);
			if (data.m_shading_vertices != nullptr)
				after_copy.Transition(data.m_shading_vertices.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			if (data.m_shading_indices16 != nullptr)
				after_copy.Transition(data.m_shading_indices16.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			if (data.m_shading_indices32 != nullptr)
				after_copy.Transition(data.m_shading_indices32.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			for (auto const& copy : input.m_vertex_copies)
				after_copy.Transition(copy.m_src, DefaultResState(copy.m_src));
			for (auto const& copy : input.m_index16_copies)
				after_copy.Transition(copy.m_src, DefaultResState(copy.m_src));
			for (auto const& copy : input.m_index32_copies)
				after_copy.Transition(copy.m_src, DefaultResState(copy.m_src));
			after_copy.Commit();

			data.m_shading_vertex_count = input.m_shading_vertex_count;
			data.m_shading_index16_count = input.m_shading_index16_count;
			data.m_shading_index32_count = input.m_shading_index32_count;
			data.m_shading_geometry_count = isize(input.m_shading_geometry);
			data.m_shading_geometry_fallback_count = input.m_shading_geometry_fallback_count;
			data.m_shading_geometry_signature = input.m_shading_geometry_signature;
		}

		// Build or update a BLAS for the current pose of a skinned model.
		SkinnedBlasBuildResult BuildSkinnedBlas(Renderer& rdr, GfxCmdList& cmd_list, GpuUploadBuffer& upload, RayTracingScene::Data& data, Model& model, PosePtr const& pose)
		{
			// Dynamic skinned BLAS ownership is per scene and per pose. A model can be instanced with multiple poses, so the model-owned static BLAS cache is
			// deliberately not used for skinned geometry.
			auto const key = SkinnedBlasKey{ &model, pose.get() };
			auto [iter, added] = data.m_skinned_blas.try_emplace(key);
			auto& entry = iter->second;
			entry.m_pose = pose;

			// Ensure the compute-skinned vertex buffer is current before building the geometry descriptors. RT consumes the same post-skinning `Vert` layout as the
			// raster paths, so the BLAS sees the deformed pose rather than the bind-pose mesh.
			auto const vbuf = rdr.SkinnedGeometry().VBuf(cmd_list, upload, model, pose);
			auto input = RayTracingBuildGeometryInput(model, vbuf.m_resource->GetGPUVirtualAddress(), rdr.RayTracing().Available(), true);
			if (!input.m_stats.m_rt_available || input.m_geometry.empty())
			{
				// If the model cannot produce ray-traceable geometry, discard the cache entry immediately so the TLAS builder treats the instance as unsupported.
				ReleaseSkinnedBlas(rdr, entry);
				data.m_skinned_blas.erase(iter);
				return SkinnedBlasBuildResult{
					.m_stats = input.m_stats,
					.m_address = 0,
					.m_vertex_buffer = vbuf.m_resource,
					.m_pose_revision = vbuf.m_pose_revision,
					.m_cache_hit = false,
				};
			}

			auto const model_revision = model.m_ray_tracing.Revision();
			auto const rebuild =
				added ||
				entry.m_blas == nullptr ||
				entry.m_vcount != model.m_vcount ||
				entry.m_model_revision != model_revision ||
				entry.m_stats.m_geometry_count != input.m_stats.m_geometry_count;
			auto const update = !rebuild && entry.m_pose_revision != vbuf.m_pose_revision;
			if (!rebuild && !update)
			{
				// The existing BLAS already matches the model topology and pose revision, so the TLAS can reuse the acceleration structure as-is.
				return SkinnedBlasBuildResult{
					.m_stats = entry.m_stats,
					.m_address = entry.m_blas->GetGPUVirtualAddress(),
					.m_vertex_buffer = vbuf.m_resource,
					.m_pose_revision = entry.m_pose_revision,
					.m_cache_hit = true,
				};
			}

			D3DPtr<ID3D12Device5> device;
			Check(rdr.D3DDevice()->QueryInterface<ID3D12Device5>(device.address_of()));

			// Dynamic skinned BLASes are created with ALLOW_UPDATE. Pose-only changes use the cheaper update path, while topology/resource changes fall back to a
			// full rebuild because the allocation sizes or geometry descriptors may no longer match.
			auto build_flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS(
				D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE |
				D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_ALLOW_UPDATE);
			auto inputs = D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS{
				.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL,
				.Flags = rebuild
					? build_flags
					: D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS(build_flags | D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PERFORM_UPDATE),
				.NumDescs = s_cast<UINT>(input.m_geometry.size()),
				.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY,
				.pGeometryDescs = input.m_geometry.data(),
			};

			if (rebuild)
			{
				// Rebuilds recalculate the required allocation sizes and replace the old BLAS/scratch resources. The old resources are deferred because the GPU may
				// still reference them from previously submitted frames.
				auto prebuild_inputs = inputs;
				prebuild_inputs.Flags = build_flags;

				D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO prebuild_info = {};
				device->GetRaytracingAccelerationStructurePrebuildInfo(&prebuild_inputs, &prebuild_info);
				if (prebuild_info.ResultDataMaxSizeInBytes == 0)
					throw std::runtime_error(std::format("Failed to get dynamic BLAS prebuild information for model '{}'", model.m_name));

				ReleaseSkinnedBlas(rdr, entry);
				entry.m_pose = pose;
				entry.m_prebuild_info = prebuild_info;
				entry.m_stats = input.m_stats;
				entry.m_stats.m_blas_size = prebuild_info.ResultDataMaxSizeInBytes;
				entry.m_stats.m_scratch_size = std::max(prebuild_info.ScratchDataSizeInBytes, prebuild_info.UpdateScratchDataSizeInBytes);
				entry.m_model_revision = model_revision;
				entry.m_vcount = model.m_vcount;

				auto scratch_desc = ResDesc::Buf(s_cast<int64_t>(entry.m_stats.m_scratch_size), 1, std::span<std::byte const>{}, 1)
					.usage(EUsage::UnorderedAccess)
					.def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				entry.m_blas = CreateRayTracingAccelerationStructure(rdr, prebuild_info.ResultDataMaxSizeInBytes, std::format("{}:SkinnedBLAS", model.m_name));
				entry.m_scratch = CreateRayTracingResource(rdr, cmd_list, upload, scratch_desc, std::format("{}:SkinnedBLAS scratch", model.m_name));
			}

			// BuildRaytracingAccelerationStructure reads the skinned vertex buffer and index buffer as SRVs. The skinned vertex transition also orders the compute
			// dispatch that produced the current pose before the BLAS build that consumes it.
			BarrierBatch barriers(cmd_list);
			barriers.Transition(vbuf.m_resource, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			barriers.Transition(model.m_ib.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			barriers.Commit();

			D3DPtr<ID3D12GraphicsCommandList4> cmd_list4;
			Check(cmd_list.get()->QueryInterface<ID3D12GraphicsCommandList4>(cmd_list4.address_of()));

			// Updates read from and write to the same BLAS resource, which is the standard in-place refit/update pattern for D3D12 acceleration structures.
			auto const blas_address = entry.m_blas->GetGPUVirtualAddress();
			auto build_desc = D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC{
				.DestAccelerationStructureData = blas_address,
				.Inputs = inputs,
				.SourceAccelerationStructureData = rebuild ? 0 : blas_address,
				.ScratchAccelerationStructureData = entry.m_scratch->GetGPUVirtualAddress(),
			};
			cmd_list4->BuildRaytracingAccelerationStructure(&build_desc, 0, nullptr);

			BarrierBatch build_barriers(cmd_list);
			build_barriers.UAV(entry.m_blas.get());
			build_barriers.Commit();

			// Record the pose revision that the BLAS now represents so repeated instances of the same model/pose can use a cache hit within later frames.
			entry.m_pose_revision = vbuf.m_pose_revision;
			return SkinnedBlasBuildResult{
				.m_stats = entry.m_stats,
				.m_address = blas_address,
				.m_vertex_buffer = vbuf.m_resource,
				.m_pose_revision = entry.m_pose_revision,
				.m_cache_hit = false,
			};
		}

		// Build the instance descriptors used to create a top-level acceleration structure for 'instances'.
		InstanceBuildInput BuildInstanceInput(Renderer& rdr, GfxCmdList& cmd_list, GpuUploadBuffer& upload, RayTracingScene::Data& data, std::span<BaseInstance const* const> instances)
		{
			// The signature includes both the scene-level instance list and the per-instance AS inputs. If it is unchanged, the existing TLAS still describes the
			// same set of instances and acceleration-structure addresses.
			InstanceBuildInput result;
			result.m_stats.m_rt_available = rdr.RayTracing().Available();
			result.m_stats.m_scene_instance_count = isize(instances);
			AddSignature(result.m_signature, result.m_stats.m_scene_instance_count);

			if (!result.m_stats.m_rt_available)
				return result;

			auto instance_index = 0;
			for (auto const* inst : instances)
			{
				// Hash the raw instance pointer as well as the generated descriptor so removing and re-adding otherwise similar instances still invalidates the TLAS.
				AddSignature(result.m_signature, reinterpret_cast<uintptr_t>(inst));
				if (inst == nullptr)
				{
					++result.m_stats.m_excluded_no_model_count;
					continue;
				}

				auto const* model_ptr = inst->find<ModelPtr>(EInstComp::ModelPtr);
				if (model_ptr == nullptr || *model_ptr == nullptr)
				{
					++result.m_stats.m_excluded_no_model_count;
					continue;
				}

				// DXR instance transforms are affine 3x4 matrices. Keep the initial eligibility conservative until non-affine handling is explicit.
				auto const flags = GetFlags(*inst);
				auto const& o2w = GetO2W(*inst);
				if (AllSet(flags, EInstFlag::NonAffine) || !IsAffine(o2w))
				{
					++result.m_stats.m_excluded_non_affine_count;
					continue;
				}

				auto model_ref = *model_ptr;
				auto& model = *model_ref.get();
				auto const instance_mask = RayTracingInstanceMask_Default | (IsCausticContributor(*inst, model) ? RayTracingInstanceMask_Caustic : 0U);
				auto desc = D3D12_RAYTRACING_INSTANCE_DESC{};
				CopyTransform(o2w, desc.Transform);
				desc.InstanceMask = instance_mask;
				desc.InstanceContributionToHitGroupIndex = 0;
				desc.Flags = D3D12_RAYTRACING_INSTANCE_FLAG_TRIANGLE_CULL_DISABLE;

				auto cache_hit = false;
				auto pose_revision = uint64_t{};
				auto source_revision = uint64_t{};
				auto* shading_vertex_buffer = static_cast<ID3D12Resource*>(nullptr);
				auto geometry = RayTracingGeometryBuildInput{};
				if (model.m_skin)
				{
					// Skinned models require a live pose because tracing bind-pose geometry would disagree with raster. Missing poses are excluded until the caller
					// provides enough animation state to build a current-pose BLAS.
					auto pose = FindPose(*inst);
					if (pose == nullptr)
					{
						++result.m_stats.m_excluded_skinned_count;
						continue;
					}

					// Dynamic skinned BLASes are keyed by model and pose, then referenced by the TLAS instance descriptor just like static model-owned BLASes.
					auto skinned_blas = BuildSkinnedBlas(rdr, cmd_list, upload, data, model, pose);
					if (skinned_blas.m_address == 0)
					{
						++result.m_stats.m_excluded_unsupported_count;
						continue;
					}

					auto const key = SkinnedBlasKey{ &model, pose.get() };
					result.m_used_skinned_blas.push_back(key);
					desc.AccelerationStructure = skinned_blas.m_address;
					cache_hit = skinned_blas.m_cache_hit;
					pose_revision = skinned_blas.m_pose_revision;
					source_revision = skinned_blas.m_pose_revision;
					shading_vertex_buffer = skinned_blas.m_vertex_buffer;
					geometry = RayTracingBuildGeometryInput(model, 0, true, true);

					AddSignature(result.m_signature, key.m_model);
					AddSignature(result.m_signature, key.m_pose);
					AddSignature(result.m_signature, pose_revision);
					AddSignature(result.m_signature, model.m_ray_tracing.Revision());
				}
				else
				{
					// Static models keep their BLAS beside the model because their geometry does not depend on per-scene pose state.
					auto const was_built = model.m_ray_tracing.Built();
					auto const geom_stats = model.m_ray_tracing.Build(rdr, cmd_list, upload, model);
					if (!model.m_ray_tracing.Built())
					{
						++result.m_stats.m_excluded_unsupported_count;
						continue;
					}

					desc.AccelerationStructure = model.m_ray_tracing.AccelerationStructureAddress();
					cache_hit = was_built;
					source_revision = model.m_ray_tracing.Revision();
					shading_vertex_buffer = model.m_vb.get();
					geometry = RayTracingBuildGeometryInput(model, 0, true, false);

					AddSignature(result.m_signature, model.m_ray_tracing.Revision());
					AddSignature(result.m_signature, geom_stats.m_geometry_count);
				}
				if (geometry.m_nuggets.empty())
				{
					++result.m_stats.m_excluded_unsupported_count;
					continue;
				}

				// Once the instance has a valid BLAS address, append the final descriptor and keep all per-geometry side tables aligned with the DXR ids.
				desc.InstanceID = AppendMaterials(result, *inst, geometry);
				auto const geometry_id = AppendShadingGeometry(result, *inst, model, geometry, shading_vertex_buffer, source_revision);
				if (geometry_id != desc.InstanceID)
					throw std::runtime_error("Ray tracing material and shading geometry tables are out of sync");

				AddSignature(result.m_signature, desc);
				result.m_instances.push_back(desc);
				++result.m_stats.m_tlas_instance_count;
				++instance_index;

				if (cache_hit)
					++result.m_stats.m_blas_cached_count;
				else
					++result.m_stats.m_blas_build_count;
			}
			return result;
		}
	}

	// Create empty ray tracing scene diagnostics.
	RayTracingSceneStats::RayTracingSceneStats()
		: m_rt_available()
		, m_scene_instance_count()
		, m_tlas_instance_count()
		, m_blas_build_count()
		, m_blas_cached_count()
		, m_excluded_no_model_count()
		, m_excluded_skinned_count()
		, m_excluded_unsupported_count()
		, m_excluded_non_affine_count()
		, m_tlas_size()
		, m_scratch_size()
		, m_instance_desc_size()
	{}

	// Create empty per-scene ray tracing state.
	RayTracingScene::RayTracingScene()
		: m_data()
		, m_stats()
		, m_signature(hash::FNV_offset_basis64)
	{}

	// Move per-scene ray tracing state without copying GPU resource ownership.
	RayTracingScene::RayTracingScene(RayTracingScene&& rhs) noexcept
		: m_data(std::move(rhs.m_data))
		, m_stats(rhs.m_stats)
		, m_signature(rhs.m_signature)
	{}

	// Move per-scene ray tracing state without copying GPU resource ownership.
	RayTracingScene& RayTracingScene::operator =(RayTracingScene&& rhs) noexcept
	{
		if (&rhs == this)
			return *this;

		m_data = std::move(rhs.m_data);
		m_stats = rhs.m_stats;
		m_signature = rhs.m_signature;
		return *this;
	}

	// Destroy the per-scene ray tracing state.
	RayTracingScene::~RayTracingScene() = default;

	// Return true if this scene currently owns a built TLAS.
	bool RayTracingScene::Built() const
	{
		return m_data != nullptr && m_data->m_tlas != nullptr;
	}

	// Return the latest TLAS build diagnostics.
	RayTracingSceneStats const& RayTracingScene::Stats() const
	{
		return m_stats;
	}

	// Return the TLAS GPU virtual address, or zero if no TLAS is built.
	D3D12_GPU_VIRTUAL_ADDRESS RayTracingScene::AccelerationStructureAddress() const
	{
		return Built()
			? m_data->m_tlas->GetGPUVirtualAddress()
			: D3D12_GPU_VIRTUAL_ADDRESS{};
	}

	// Return the material buffer used by reflection closest-hit shaders.
	ID3D12Resource* RayTracingScene::MaterialBuffer() const
	{
		return m_data != nullptr
			? const_cast<ID3D12Resource*>(m_data->m_materials.get())
			: nullptr;
	}

	// Return the number of material records in the material buffer.
	int RayTracingScene::MaterialCount() const
	{
		return m_data != nullptr
			? m_data->m_material_count
			: 0;
	}

	// Return the packed vertex buffer used by RT closest-hit shading.
	ID3D12Resource* RayTracingScene::ShadingVertexBuffer() const
	{
		return m_data != nullptr
			? const_cast<ID3D12Resource*>(m_data->m_shading_vertices.get())
			: nullptr;
	}

	// Return the number of packed RT shading vertices.
	int RayTracingScene::ShadingVertexCount() const
	{
		return m_data != nullptr
			? m_data->m_shading_vertex_count
			: 0;
	}

	// Return the packed 16-bit index buffer used by RT closest-hit shading.
	ID3D12Resource* RayTracingScene::ShadingIndex16Buffer() const
	{
		return m_data != nullptr
			? const_cast<ID3D12Resource*>(m_data->m_shading_indices16.get())
			: nullptr;
	}

	// Return the number of 16-bit indices in the packed RT shading index buffer.
	int RayTracingScene::ShadingIndex16Count() const
	{
		return m_data != nullptr
			? m_data->m_shading_index16_count
			: 0;
	}

	// Return the packed 32-bit index buffer used by RT closest-hit shading.
	ID3D12Resource* RayTracingScene::ShadingIndex32Buffer() const
	{
		return m_data != nullptr
			? const_cast<ID3D12Resource*>(m_data->m_shading_indices32.get())
			: nullptr;
	}

	// Return the number of 32-bit indices in the packed RT shading index buffer.
	int RayTracingScene::ShadingIndex32Count() const
	{
		return m_data != nullptr
			? m_data->m_shading_index32_count
			: 0;
	}

	// Return the geometry metadata buffer used by RT closest-hit shading.
	ID3D12Resource* RayTracingScene::ShadingGeometryBuffer() const
	{
		return m_data != nullptr
			? const_cast<ID3D12Resource*>(m_data->m_shading_geometry.get())
			: nullptr;
	}

	// Return the number of geometry records in the RT closest-hit shading buffer.
	int RayTracingScene::ShadingGeometryCount() const
	{
		return m_data != nullptr
			? m_data->m_shading_geometry_count
			: 0;
	}

	// Return the number of geometry records that must fall back to material-only shading.
	int RayTracingScene::ShadingGeometryFallbackCount() const
	{
		return m_data != nullptr
			? m_data->m_shading_geometry_fallback_count
			: 0;
	}

	// Release TLAS and dynamic BLAS resources after deferring GPU lifetime management through the renderer.
	void RayTracingScene::DeferRelease(Renderer& rdr)
	{
		if (m_data == nullptr)
			return;

		ReleaseTlas(rdr, *m_data);
		ReleaseMaterialBuffer(rdr, *m_data);
		ReleaseShadingGeometryBuffers(rdr, *m_data);
		for (auto& [key, entry] : m_data->m_skinned_blas)
			ReleaseSkinnedBlas(rdr, entry);

		m_data.reset();
	}

	// Mark the scene's TLAS as invalid because the scene content changed.
	void RayTracingScene::Invalidate(Renderer& rdr)
	{
		DeferRelease(rdr);
		m_signature = hash::FNV_offset_basis64;
	}

	// Build or reuse the scene TLAS for the provided instances.
	RayTracingSceneStats RayTracingScene::Build(Renderer& rdr, GfxCmdList& cmd_list, GpuUploadBuffer& upload, std::span<BaseInstance const* const> instances)
	{
		if (m_data == nullptr)
			m_data = std::make_unique<Data>();

		// Collect all TLAS instance descriptors up front. This may also build/update model BLASes, because the TLAS descriptor needs the final BLAS GPU address for
		// each included instance.
		auto input = BuildInstanceInput(rdr, cmd_list, upload, *m_data, instances);
		m_stats = input.m_stats;
		if (!m_stats.m_rt_available)
		{
			// If DXR is unavailable, release any previous AS resources so callers never accidentally trace stale GPU state.
			DeferRelease(rdr);
			m_signature = input.m_signature;
			return m_stats;
		}
		if (input.m_instances.empty())
		{
			// An empty ray-traceable scene has no TLAS. Skinned BLAS cache entries are also pruned because there are no instances referencing them.
			ReleaseTlas(rdr, *m_data);
			ReleaseMaterialBuffer(rdr, *m_data);
			ReleaseShadingGeometryBuffers(rdr, *m_data);
			PruneSkinnedBlas(rdr, *m_data, std::span<SkinnedBlasKey const>{});
			m_signature = input.m_signature;
			return m_stats;
		}

		// Keep only dynamic skinned BLASes that were referenced by the descriptors collected for this frame.
		PruneSkinnedBlas(rdr, *m_data, std::span<SkinnedBlasKey const>(input.m_used_skinned_blas.data(), input.m_used_skinned_blas.size()));
		EnsureMaterialBuffer(
			rdr,
			cmd_list,
			upload,
			*m_data,
			std::span<shaders::rt::RayTracingMaterial const>(input.m_materials.data(), input.m_materials.size()),
			input.m_material_signature);
		EnsureShadingGeometryBuffers(rdr, cmd_list, upload, *m_data, input);

		if (Built() && m_signature == input.m_signature)
		{
			// The existing TLAS is still valid. Refresh the diagnostic byte counts from the cached prebuild info without recording any GPU work.
			m_stats.m_tlas_size = m_data->m_prebuild_info.ResultDataMaxSizeInBytes;
			m_stats.m_scratch_size = m_data->m_prebuild_info.ScratchDataSizeInBytes;
			m_stats.m_instance_desc_size = input.m_instances.size() * sizeof(D3D12_RAYTRACING_INSTANCE_DESC);
			return m_stats;
		}

		D3DPtr<ID3D12Device5> device;
		Check(rdr.D3DDevice()->QueryInterface<ID3D12Device5>(device.address_of()));

		// Instance descriptors are normal GPU-read buffers. Only the TLAS result resource needs the special acceleration-structure state.
		auto instance_desc = ResDesc::Buf<D3D12_RAYTRACING_INSTANCE_DESC>(
			isize(input.m_instances),
			std::span<D3D12_RAYTRACING_INSTANCE_DESC const>(input.m_instances.data(), input.m_instances.size()))
			.def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		auto instance_buffer = CreateRayTracingResource(rdr, cmd_list, upload, instance_desc, "RayTracing:TLAS instances");

		D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS inputs = {
			.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL,
			.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE,
			.NumDescs = s_cast<UINT>(input.m_instances.size()),
			.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY,
			.InstanceDescs = instance_buffer->GetGPUVirtualAddress(),
		};

		D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO prebuild_info = {};
		device->GetRaytracingAccelerationStructurePrebuildInfo(&inputs, &prebuild_info);
		if (prebuild_info.ResultDataMaxSizeInBytes == 0)
			throw std::runtime_error("Failed to get TLAS prebuild information");

		// Replacing the TLAS also replaces the instance descriptor buffer and scratch allocation. Defer old resource release because submitted frames may still
		// reference the previous TLAS.
		ReleaseTlas(rdr, *m_data);
		m_data->m_prebuild_info = prebuild_info;
		m_data->m_instances = instance_buffer;
		m_signature = input.m_signature;
		m_stats.m_tlas_size = prebuild_info.ResultDataMaxSizeInBytes;
		m_stats.m_scratch_size = prebuild_info.ScratchDataSizeInBytes;
		m_stats.m_instance_desc_size = input.m_instances.size() * sizeof(D3D12_RAYTRACING_INSTANCE_DESC);

		auto scratch_desc = ResDesc::Buf(s_cast<int64_t>(prebuild_info.ScratchDataSizeInBytes), 1, std::span<std::byte const>{}, 1)
			.usage(EUsage::UnorderedAccess)
			.def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		m_data->m_tlas = CreateRayTracingAccelerationStructure(rdr, prebuild_info.ResultDataMaxSizeInBytes, "RayTracing:TLAS");
		m_data->m_scratch = CreateRayTracingResource(rdr, cmd_list, upload, scratch_desc, "RayTracing:TLAS scratch");

		D3DPtr<ID3D12GraphicsCommandList4> cmd_list4;
		Check(cmd_list.get()->QueryInterface<ID3D12GraphicsCommandList4>(cmd_list4.address_of()));

		// The TLAS build consumes the uploaded instance descriptor buffer and writes a fresh top-level acceleration structure for the current scene signature.
		D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC build_desc = {
			.DestAccelerationStructureData = m_data->m_tlas->GetGPUVirtualAddress(),
			.Inputs = inputs,
			.SourceAccelerationStructureData = 0,
			.ScratchAccelerationStructureData = m_data->m_scratch->GetGPUVirtualAddress(),
		};
		cmd_list4->BuildRaytracingAccelerationStructure(&build_desc, 0, nullptr);

		// Make the finished TLAS visible to later ray dispatches in the frame.
		BarrierBatch build_barriers(cmd_list);
		build_barriers.UAV(m_data->m_tlas.get());
		build_barriers.Commit();

		return m_stats;
	}
}
