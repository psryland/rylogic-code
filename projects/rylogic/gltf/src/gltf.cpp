//********************************
// glTF Model loader
//  Copyright (c) Rylogic Ltd 2025
//********************************
#include <concepts>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <string_view>
#include <functional>
#include <atomic>
#include <mutex>
#include <sstream>

#include <Windows.h>

#define CGLTF_IMPLEMENTATION
#include "cgltf/cgltf.h"

#include "pr/common/to.h"
#include "pr/common/cast.h"
#include "pr/common/bit_fields.h"
#include "pr/geometry/gltf.h"
#include "pr/geometry/common.h"
#include "pr/container/vector.h"
#include "pr/math/math.h"
#include "pr/math/conversion.h"

using namespace pr;
using namespace pr::math;
using namespace pr::geometry::gltf;

namespace pr
{
	template <typename T> requires (requires (T t) { t == nullptr; })
	static constexpr T NullCheck(T ptr, std::string_view msg)
	{
		if (!(ptr == nullptr)) return ptr;
		throw std::runtime_error(std::string(msg));
	}

	// Convert cgltf result to string
	inline std::string ResultToString(cgltf_result result)
	{
		switch (result)
		{
			case cgltf_result_success: return "success";
			case cgltf_result_data_too_short: return "data too short";
			case cgltf_result_unknown_format: return "unknown format";
			case cgltf_result_invalid_json: return "invalid JSON";
			case cgltf_result_invalid_gltf: return "invalid glTF";
			case cgltf_result_invalid_options: return "invalid options";
			case cgltf_result_file_not_found: return "file not found";
			case cgltf_result_io_error: return "IO error";
			case cgltf_result_out_of_memory: return "out of memory";
			case cgltf_result_legacy_gltf: return "legacy glTF";
			default: return "unknown error";
		}
	}

	// Convert from cgltf types to Rylogic types
	inline v2 ToV2(cgltf_float const* f)
	{
		return v2{ f[0], f[1] };
	}
	inline v3 ToV3(cgltf_float const* f)
	{
		return v3{ f[0], f[1], f[2] };
	}
	inline v4 ToV4(cgltf_float const* f, float w)
	{
		return v4{ f[0], f[1], f[2], w };
	}
	inline quat ToQuat(cgltf_float const* f)
	{
		return quat{ f[0], f[1], f[2], f[3] };
	}
	inline Colour ToColour(cgltf_float const* f, int count)
	{
		return Colour{
			count > 0 ? f[0] : 1.0f,
			count > 1 ? f[1] : 1.0f,
			count > 2 ? f[2] : 1.0f,
			count > 3 ? f[3] : 1.0f,
		};
	}
	inline m4x4 ToM4x4(cgltf_float const* m)
	{
		// glTF uses column-major matrices: m[col*4+row]
		return m4x4{
			v4{ m[0], m[1], m[2], m[3] },
			v4{ m[4], m[5], m[6], m[7] },
			v4{ m[8], m[9], m[10], m[11] },
			v4{ m[12], m[13], m[14], m[15] },
		};
	}
	inline m4x4 NodeLocalTransform(cgltf_node const* node)
	{
		cgltf_float mat[16];
		cgltf_node_transform_local(node, mat);
		return ToM4x4(mat);
	}
	inline m4x4 NodeWorldTransform(cgltf_node const* node)
	{
		cgltf_float mat[16];
		cgltf_node_transform_world(node, mat);
		return ToM4x4(mat);
	}
	inline geometry::ETopo ToETopo(cgltf_primitive_type type)
	{
		using namespace pr::geometry;
		switch (type)
		{
			case cgltf_primitive_type_points: return ETopo::PointList;
			case cgltf_primitive_type_lines: return ETopo::LineList;
			case cgltf_primitive_type_line_strip: return ETopo::LineStrip;
			case cgltf_primitive_type_triangles: return ETopo::TriList;
			case cgltf_primitive_type_triangle_strip: return ETopo::TriStrip;
			default: return ETopo::TriList;
		}
	}
}
namespace pr::geometry::gltf
{
	static constexpr Vert NoVert = {
		.m_vert = Zero<v4>(), 
		.m_colr = ColourWhite,
		.m_norm = Zero<v4>(),
		.m_tex0 = Zero<v2>(),
		.m_idx0 = {NoIndex, 0} };

	// Model data types (owning versions of the public span-based types)
	struct MaterialData
	{
		uint32_t m_mat_id = NoId;
		std::string m_name = "default";
		Colour m_ambient = ColourBlack;
		Colour m_diffuse = ColourWhite;
		Colour m_specular = ColourZero;
		std::string m_tex_diff;

		operator Material() const
		{
			return Material{
				.m_mat_id = m_mat_id,
				.m_name = m_name,
				.m_ambient = m_ambient,
				.m_diffuse = m_diffuse,
				.m_specular = m_specular,
				.m_tex_diff = m_tex_diff,
			};
		}
	};
	struct SkinData
	{
		uint32_t m_skel_id = NoId;
		vector<int> m_offsets;
		vector<uint32_t> m_bones;
		vector<float> m_weights;

		void Reset()
		{
			m_skel_id = NoId;
			m_offsets.resize(0);
			m_bones.resize(0);
			m_weights.resize(0);
		}
		operator Skin() const
		{
			return Skin{
				.m_skel_id = m_skel_id,
				.m_offsets = m_offsets,
				.m_bones = m_bones,
				.m_weights = m_weights,
			};
		}
	};
	struct SkeletonData
	{
		uint32_t m_skel_id = NoId;
		std::string m_name;
		vector<uint32_t> m_bone_ids;
		vector<std::string> m_bone_names;
		vector<m4x4> m_o2bp;
		vector<int> m_hierarchy;

		void Reset()
		{
			m_skel_id = NoId;
			m_name = "";
			m_bone_ids.resize(0);
			m_bone_names.resize(0);
			m_o2bp.resize(0);
			m_hierarchy.resize(0);
		}
		operator Skeleton() const
		{
			assert(
				isize(m_bone_ids) == isize(m_bone_names) &&
				isize(m_bone_ids) == isize(m_o2bp) &&
				isize(m_bone_ids) == isize(m_hierarchy)
			);
			return Skeleton{
				.m_skel_id = m_skel_id,
				.m_name = m_name,
				.m_bone_ids = m_bone_ids,
				.m_bone_names = m_bone_names,
				.m_o2bp = m_o2bp,
				.m_hierarchy = m_hierarchy,
			};
		}
	};
	struct AnimationData
	{
		uint32_t m_skel_id = NoId;
		double m_duration = 0;
		double m_frame_rate = 30.0;
		std::string m_name;
		vector<uint16_t, 0> m_bone_map;
		vector<quat, 0> m_rotation;
		vector<v3, 0> m_position;
		vector<v3, 0> m_scale;

		void Reset()
		{
			m_skel_id = NoId;
			m_duration = 0.0;
			m_frame_rate = 30.0;
			m_bone_map.resize(0);
			m_rotation.resize(0);
			m_position.resize(0);
			m_scale.resize(0);
		}
		operator Animation() const
		{
			return Animation{
				.m_skel_id = m_skel_id,
				.m_duration = m_duration,
				.m_frame_rate = m_frame_rate,
				.m_name = m_name,
				.m_bone_map = m_bone_map,
				.m_rotation = m_rotation,
				.m_position = m_position,
				.m_scale = m_scale,
			};
		}
	};
	struct MeshData
	{
		uint32_t m_mesh_id = NoId;
		std::string m_name;
		vector<Vert> m_vbuf;
		vector<int> m_ibuf;
		vector<Nugget> m_nbuf;
		SkinData m_skin = {};
		BBox m_bbox = {};

		void Reset()
		{
			m_mesh_id = NoId;
			m_name.resize(0);
			m_vbuf.resize(0);
			m_ibuf.resize(0);
			m_nbuf.resize(0);
			m_skin.Reset();
			m_bbox = BBox::Reset();
		}
		operator Mesh() const
		{
			return Mesh{
				.m_mesh_id = m_mesh_id,
				.m_name = m_name,
				.m_vbuf = m_vbuf,
				.m_ibuf = m_ibuf,
				.m_nbuf = m_nbuf,
				.m_skin = Skin{
					.m_skel_id = m_skin.m_skel_id,
					.m_offsets = m_skin.m_offsets,
					.m_bones = m_skin.m_bones,
					.m_weights = m_skin.m_weights,
				},
				.m_bbox = m_bbox,
			};
		}
	};

	// Return the index of an element in the cgltf data arrays
	inline uint32_t MeshIndex(cgltf_data const& data, cgltf_mesh const* mesh)
	{
		return mesh ? s_cast<uint32_t>(mesh - data.meshes) : NoId;
	}
	inline uint32_t MaterialIndex(cgltf_data const& data, cgltf_material const* mat)
	{
		return mat ? s_cast<uint32_t>(mat - data.materials) : NoId;
	}
	inline uint32_t SkinIndex(cgltf_data const& data, cgltf_skin const* skin)
	{
		return skin ? s_cast<uint32_t>(skin - data.skins) : NoId;
	}
	inline uint32_t NodeIndex(cgltf_data const& data, cgltf_node const* node)
	{
		return node ? s_cast<uint32_t>(node - data.nodes) : NoId;
	}

	// Walk the node hierarchy depth first
	template <std::invocable<cgltf_node const*, int> Callback>
	inline void WalkHierarchy(cgltf_node const* node, int level, Callback cb)
	{
		if (!cb(node, level))
			return;
		for (cgltf_size i = 0; i != node->children_count; ++i)
			WalkHierarchy(node->children[i], level + 1, cb);
	}

	// Read data from a scene and output it to the caller
	struct Reader
	{
		struct Influence
		{
			vector<uint32_t, 8> m_bones;
			vector<float, 8> m_weights;
		};
		using Materials = vector<Material, 2>;

		cgltf_data const& m_data;
		ReadOptions const& m_opts;
		IReadOutput& m_out;

		// Cache
		MeshData m_mesh;
		Materials m_materials;

		Reader(cgltf_data const& data, ReadOptions const& opts, IReadOutput& out)
			: m_data(data)
			, m_opts(opts)
			, m_out(out)
			, m_mesh()
			, m_materials()
		{
			m_materials.push_back({});
		}

		// Read the scene
		void Do()
		{
			if (AllSet(m_opts.m_parts, EParts::Materials))
				ReadMaterials();
			if (AllSet(m_opts.m_parts, EParts::Skeletons))
				ReadSkeletons();
			if (AllSet(m_opts.m_parts, EParts::Meshes))
				ReadGeometry();
			if (AllSet(m_opts.m_parts, EParts::Animation))
				ReadAnimation();
		}

		// Read the materials
		void ReadMaterials()
		{
			if (m_data.materials_count == 0)
			{
				m_materials.resize(1, {});
				return;
			}

			m_materials.resize(0);
			m_materials.reserve(m_data.materials_count);
			for (cgltf_size i = 0; i != m_data.materials_count; ++i)
			{
				Progress(1LL + i, m_data.materials_count, "Reading materials...");

				auto const& cgmat = m_data.materials[i];
				Material mat = {};
				mat.m_mat_id = s_cast<uint32_t>(i);
				mat.m_name = cgmat.name ? cgmat.name : "";

				// Map PBR base color to diffuse
				if (cgmat.has_pbr_metallic_roughness)
				{
					auto const& pbr = cgmat.pbr_metallic_roughness;
					mat.m_diffuse = ToColour(pbr.base_color_factor, 4);

					// Base color texture URI
					if (pbr.base_color_texture.texture && pbr.base_color_texture.texture->image)
					{
						auto* img = pbr.base_color_texture.texture->image;
						if (img->uri)
							mat.m_tex_diff = img->uri;
					}
				}
				else if (cgmat.has_pbr_specular_glossiness)
				{
					auto const& pbr = cgmat.pbr_specular_glossiness;
					mat.m_diffuse = ToColour(pbr.diffuse_factor, 4);
				}

				// Map emissive to ambient
				mat.m_ambient = ToColour(cgmat.emissive_factor, 3);

				m_materials.push_back(mat);
			}
		}

		// Read meshes from the glTF scene
		void ReadGeometry()
		{
			// glTF meshes are separate from nodes. Nodes reference meshes by pointer.
			// First, output each unique mesh. Then build a tree of node instances.
			for (cgltf_size mi = 0; mi != m_data.meshes_count; ++mi)
			{
				auto const& cgmesh = m_data.meshes[mi];

				// Filter check: see if any node using this mesh passes the filter
				if (m_opts.m_mesh_filter)
				{
					auto used = false;
					for (cgltf_size ni = 0; ni != m_data.nodes_count && !used; ++ni)
					{
						auto const& node = m_data.nodes[ni];
						if (node.mesh != &cgmesh) continue;
						std::string_view name = node.name ? node.name : "";
						used = m_opts.m_mesh_filter(name);
					}
					if (!used) continue;
				}

				ReadMesh(cgmesh, s_cast<uint32_t>(mi));
				m_out.CreateMesh(m_mesh, m_materials);
			}

			// Build the mesh tree from the scene hierarchy
			auto const* scene = m_data.scene ? m_data.scene : (m_data.scenes_count > 0 ? &m_data.scenes[0] : nullptr);
			if (scene == nullptr) return;

			vector<MeshTree> mesh_tree;
			mesh_tree.reserve(m_data.nodes_count);

			for (cgltf_size ni = 0; ni != scene->nodes_count; ++ni)
			{
				Progress(1LL + ni, scene->nodes_count, "Reading models...");
				WalkHierarchy(scene->nodes[ni], 0, [this, &mesh_tree](cgltf_node const* node, int level) -> bool
				{
					if (node->mesh == nullptr)
						return true; // continue walking children

					auto name = std::string_view(node->name ? node->name : "");
					if (m_opts.m_mesh_filter && !m_opts.m_mesh_filter(name))
						return true;

					auto mesh_id = MeshIndex(m_data, node->mesh);
					auto o2p = level == 0 ? NodeWorldTransform(node) : NodeLocalTransform(node);

					mesh_tree.push_back(MeshTree{
						.m_o2p = o2p,
						.m_name = name,
						.m_mesh_id = mesh_id,
						.m_level = level,
					});
					return true;
				});
			}

			m_out.CreateModel(mesh_tree);
		}

		// Read a single cgltf mesh
		void ReadMesh(cgltf_mesh const& cgmesh, uint32_t mesh_id)
		{
			auto& mesh = m_mesh;
			mesh.Reset();
			mesh.m_mesh_id = mesh_id;
			mesh.m_name = cgmesh.name ? cgmesh.name : "";

			for (cgltf_size pi = 0; pi != cgmesh.primitives_count; ++pi)
			{
				auto const& prim = cgmesh.primitives[pi];

				// Get accessors for the standard attributes
				auto const* pos_acc = cgltf_find_accessor(&prim, cgltf_attribute_type_position, 0);
				if (pos_acc == nullptr) continue;

				auto const* nrm_acc = cgltf_find_accessor(&prim, cgltf_attribute_type_normal, 0);
				auto const* tex_acc = cgltf_find_accessor(&prim, cgltf_attribute_type_texcoord, 0);
				auto const* col_acc = cgltf_find_accessor(&prim, cgltf_attribute_type_color, 0);

				auto topo = ToETopo(prim.type);
				auto mat_id = MaterialIndex(m_data, prim.material);

				// Start a new nugget
				Nugget nugget = {
					.m_mat_id = mat_id,
					.m_topo = topo,
					.m_geom = EGeom::Vert,
				};

				auto vbase = isize(mesh.m_vbuf);
				auto ibase = isize(mesh.m_ibuf);

				// Read vertices
				for (cgltf_size vi = 0; vi != pos_acc->count; ++vi)
				{
					Vert v = {};
					
					cgltf_float pos[3] = {};
					cgltf_accessor_read_float(pos_acc, vi, pos, 3);
					v.m_vert = v4{ pos[0], pos[1], pos[2], 1.0f };

					if (nrm_acc)
					{
						cgltf_float nrm[3] = {};
						cgltf_accessor_read_float(nrm_acc, vi, nrm, 3);
						v.m_norm = v4{ nrm[0], nrm[1], nrm[2], 0.0f };
						nugget.m_geom |= EGeom::Norm;
					}

					if (tex_acc)
					{
						cgltf_float tex[2] = {};
						cgltf_accessor_read_float(tex_acc, vi, tex, 2);
						v.m_tex0 = v2{ tex[0], tex[1] };
						nugget.m_geom |= EGeom::Tex0;
					}

					if (col_acc)
					{
						cgltf_float col[4] = { 1, 1, 1, 1 };
						auto num = cgltf_num_components(col_acc->type);
						cgltf_accessor_read_float(col_acc, vi, col, num);
						v.m_colr = ToColour(col, s_cast<int>(num));
						nugget.m_geom |= EGeom::Colr;
					}

					v.m_idx0 = iv2{ s_cast<int>(vbase + vi), 0 };

					mesh.m_vbuf.push_back(v);
					mesh.m_bbox.Grow(v.m_vert);
				}

				// Read indices
				if (prim.indices)
				{
					for (cgltf_size ii = 0; ii != prim.indices->count; ++ii)
					{
						auto idx = s_cast<int>(vbase + cgltf_accessor_read_index(prim.indices, ii));
						mesh.m_ibuf.push_back(idx);
					}
				}
				else
				{
					// No index buffer, generate sequential indices
					for (cgltf_size vi = 0; vi != pos_acc->count; ++vi)
						mesh.m_ibuf.push_back(s_cast<int>(vbase + vi));
				}

				nugget.m_vrange.grow(vbase);
				nugget.m_vrange.grow(isize(mesh.m_vbuf) - 1);
				nugget.m_irange.grow(ibase);
				nugget.m_irange.grow(isize(mesh.m_ibuf) - 1);
				mesh.m_nbuf.push_back(nugget);
			}

			// Read skin data if this mesh is used by a skinned node
			if (AllSet(m_opts.m_parts, EParts::Skins))
				ReadSkin(cgmesh);
		}

		// Read skin data for nodes that use this mesh
		void ReadSkin(cgltf_mesh const& cgmesh)
		{
			auto& skin = m_mesh.m_skin;
			skin.Reset();

			// Find a node that references this mesh and has a skin
			cgltf_skin const* cgskin = nullptr;
			for (cgltf_size ni = 0; ni != m_data.nodes_count; ++ni)
			{
				if (m_data.nodes[ni].mesh != &cgmesh) continue;
				if (m_data.nodes[ni].skin == nullptr) continue;
				cgskin = m_data.nodes[ni].skin;
				break;
			}
			if (cgskin == nullptr)
				return;

			skin.m_skel_id = SkinIndex(m_data, cgskin);

			// Read joint/weight attributes from the mesh primitives
			for (cgltf_size pi = 0; pi != cgmesh.primitives_count; ++pi)
			{
				auto const& prim = cgmesh.primitives[pi];
				auto const* joints_acc = cgltf_find_accessor(&prim, cgltf_attribute_type_joints, 0);
				auto const* weights_acc = cgltf_find_accessor(&prim, cgltf_attribute_type_weights, 0);
				if (joints_acc == nullptr || weights_acc == nullptr) continue;

				auto vert_count = s_cast<int>(joints_acc->count);
				auto influences_per_vert = s_cast<int>(cgltf_num_components(joints_acc->type));

				skin.m_offsets.reserve(vert_count + 1);
				skin.m_bones.reserve(vert_count * influences_per_vert);
				skin.m_weights.reserve(vert_count * influences_per_vert);

				int count = 0;
				for (int vi = 0; vi != vert_count; ++vi)
				{
					skin.m_offsets.push_back(count);

					cgltf_uint joint_indices[4] = {};
					cgltf_float joint_weights[4] = {};
					cgltf_accessor_read_uint(joints_acc, vi, joint_indices, influences_per_vert);
					cgltf_accessor_read_float(weights_acc, vi, joint_weights, influences_per_vert);

					for (int j = 0; j != influences_per_vert; ++j)
					{
						if (joint_weights[j] <= 0.0f) continue;

						// Map joint index to the node id of the bone
						auto joint_idx = joint_indices[j];
						if (joint_idx < cgskin->joints_count)
						{
							auto bone_id = NodeIndex(m_data, cgskin->joints[joint_idx]);
							skin.m_bones.push_back(bone_id);
							skin.m_weights.push_back(joint_weights[j]);
							count++;
						}
					}
				}
				skin.m_offsets.push_back(count);
			}
		}

		// Read skeletons from the glTF scene
		void ReadSkeletons()
		{
			for (cgltf_size si = 0; si != m_data.skins_count; ++si)
			{
				Progress(1LL + si, m_data.skins_count, "Reading skeletons...");
				auto const& cgskin = m_data.skins[si];

				std::string_view name = cgskin.name ? cgskin.name : "";
				if (m_opts.m_skel_filter && !m_opts.m_skel_filter(name))
					continue;

				SkeletonData skel;
				skel.m_skel_id = s_cast<uint32_t>(si);
				skel.m_name = std::string(name);

				auto bone_count = s_cast<int>(cgskin.joints_count);
				skel.m_bone_ids.reserve(bone_count);
				skel.m_bone_names.reserve(bone_count);
				skel.m_o2bp.reserve(bone_count);
				skel.m_hierarchy.reserve(bone_count);

				// Build a joint index map for hierarchy levels
				std::unordered_map<cgltf_node const*, int> joint_map;
				joint_map.reserve(bone_count);
				for (cgltf_size ji = 0; ji != cgskin.joints_count; ++ji)
					joint_map[cgskin.joints[ji]] = s_cast<int>(ji);

				// Read inverse bind matrices
				vector<m4x4> ibm(bone_count, m4x4::Identity());
				if (cgskin.inverse_bind_matrices)
				{
					for (int bi = 0; bi != bone_count; ++bi)
					{
						cgltf_float mat[16];
						cgltf_accessor_read_float(cgskin.inverse_bind_matrices, bi, mat, 16);
						ibm[bi] = ToM4x4(mat);
					}
				}

				// Build the skeleton data
				for (cgltf_size ji = 0; ji != cgskin.joints_count; ++ji)
				{
					auto const* joint = cgskin.joints[ji];
					auto bone_id = NodeIndex(m_data, joint);
					auto bone_name = std::string(joint->name ? joint->name : "");

					// Determine hierarchy level by walking up the parent chain
					int level = 0;
					for (auto* p = joint->parent; p != nullptr; p = p->parent)
					{
						if (joint_map.count(p) == 0) break;
						++level;
					}

					skel.m_bone_ids.push_back(bone_id);
					skel.m_bone_names.push_back(bone_name);
					skel.m_o2bp.push_back(ibm[ji]);
					skel.m_hierarchy.push_back(level);
				}

				m_out.CreateSkeleton(skel);
			}
		}

		// Read animation data from the scene
		void ReadAnimation()
		{
			for (cgltf_size ai = 0; ai != m_data.animations_count; ++ai)
			{
				Progress(1LL + ai, m_data.animations_count, "Reading animation...");

				auto const& cganim = m_data.animations[ai];
				std::string_view name = cganim.name ? cganim.name : "";
				if (m_opts.m_anim_filter && !m_opts.m_anim_filter(name))
					continue;

				// For each skin, extract the animation data
				for (cgltf_size si = 0; si != m_data.skins_count; ++si)
				{
					auto const& cgskin = m_data.skins[si];
					AnimationData anim;
					anim.m_name = std::string(name);
					anim.m_skel_id = s_cast<uint32_t>(si);

					// Determine the time range and frame rate
					double time_min = std::numeric_limits<double>::max();
					double time_max = std::numeric_limits<double>::lowest();

					// Build a set of nodes in this skin
					std::unordered_map<cgltf_node const*, int> joint_map;
					for (cgltf_size ji = 0; ji != cgskin.joints_count; ++ji)
						joint_map[cgskin.joints[ji]] = s_cast<int>(ji);

					// Find the time range from channels targeting this skin's joints
					bool has_channels = false;
					for (cgltf_size ci = 0; ci != cganim.channels_count; ++ci)
					{
						auto const& chan = cganim.channels[ci];
						if (chan.target_node == nullptr) continue;
						if (joint_map.count(chan.target_node) == 0) continue;
						if (chan.sampler == nullptr || chan.sampler->input == nullptr) continue;

						has_channels = true;
						auto const* input = chan.sampler->input;
						if (input->has_min) time_min = std::min(time_min, static_cast<double>(input->min[0]));
						if (input->has_max) time_max = std::max(time_max, static_cast<double>(input->max[0]));
					}
					if (!has_channels) continue;
					if (time_min >= time_max) continue;

					anim.m_frame_rate = 30.0; // glTF doesn't specify frame rate, default to 30fps
					anim.m_duration = time_max - time_min;
					auto num_keys = s_cast<int>(std::ceil(anim.m_duration * anim.m_frame_rate)) + 1;
					auto bone_count = s_cast<int>(cgskin.joints_count);

					// Build bone map
					anim.m_bone_map.reserve(bone_count);
					for (cgltf_size ji = 0; ji != cgskin.joints_count; ++ji)
						anim.m_bone_map.push_back(s_cast<uint16_t>(NodeIndex(m_data, cgskin.joints[ji])));

					// Allocate space for M bones x N frames (interleaved)
					anim.m_rotation.resize(bone_count * num_keys, quat::Identity());
					anim.m_position.resize(bone_count * num_keys, v3::Zero());
					anim.m_scale.resize(bone_count * num_keys, v3::One());

					// Sample each channel
					for (cgltf_size ci = 0; ci != cganim.channels_count; ++ci)
					{
						auto const& chan = cganim.channels[ci];
						if (chan.target_node == nullptr || chan.sampler == nullptr) continue;
						auto it = joint_map.find(chan.target_node);
						if (it == joint_map.end()) continue;

						auto bone_idx = it->second;
						auto const* sampler = chan.sampler;
						auto const* input_acc = sampler->input;
						auto const* output_acc = sampler->output;
						if (input_acc == nullptr || output_acc == nullptr) continue;

						// Read the keyframe times and values
						auto key_count = s_cast<int>(input_acc->count);
						vector<float> times(key_count);
						for (int k = 0; k != key_count; ++k)
							cgltf_accessor_read_float(input_acc, k, &times[k], 1);

						// Sample at each frame
						for (int f = 0; f != num_keys; ++f)
						{
							auto time = s_cast<float>(time_min + f / anim.m_frame_rate);
							auto idx = f * bone_count + bone_idx;

							// Find the two surrounding keyframes
							int k0 = 0, k1 = 0;
							for (int k = 0; k < key_count - 1; ++k)
							{
								if (times[k + 1] >= time) { k0 = k; k1 = k + 1; break; }
								k0 = k1 = key_count - 1;
							}

							// Interpolation parameter
							float t = 0;
							if (k0 != k1 && times[k1] > times[k0])
								t = (time - times[k0]) / (times[k1] - times[k0]);

							switch (chan.target_path)
							{
								case cgltf_animation_path_type_rotation:
								{
									cgltf_float v0[4], v1[4];
									cgltf_accessor_read_float(output_acc, k0, v0, 4);
									cgltf_accessor_read_float(output_acc, k1, v1, 4);
									auto q0 = ToQuat(v0);
									auto q1 = ToQuat(v1);
									anim.m_rotation[idx] = Slerp(q0, q1, t);
									break;
								}
								case cgltf_animation_path_type_translation:
								{
									cgltf_float v0[3], v1[3];
									cgltf_accessor_read_float(output_acc, k0, v0, 3);
									cgltf_accessor_read_float(output_acc, k1, v1, 3);
									anim.m_position[idx] = Lerp(ToV3(v0), ToV3(v1), t);
									break;
								}
								case cgltf_animation_path_type_scale:
								{
									cgltf_float v0[3], v1[3];
									cgltf_accessor_read_float(output_acc, k0, v0, 3);
									cgltf_accessor_read_float(output_acc, k1, v1, 3);
									anim.m_scale[idx] = Lerp(ToV3(v0), ToV3(v1), t);
									break;
								}
								default: break;
							}
						}
					}

					// Check for default channels and trim
					bool has_rot = false, has_pos = false, has_scl = false;
					for (auto const& r : anim.m_rotation) has_rot |= !FEql(r, quat::Identity());
					for (auto const& p : anim.m_position) has_pos |= !FEql(p, v3::Zero());
					for (auto const& s : anim.m_scale) has_scl |= !FEql(s, v3::One());
					if (!has_rot) anim.m_rotation.resize(0);
					if (!has_pos) anim.m_position.resize(0);
					if (!has_scl) anim.m_scale.resize(0);

					if (!m_out.CreateAnimation(anim))
						return;
				}
			}
		}

		// Report progress
		void Progress(int64_t step, int64_t total, char const* message, int nest = 0) const
		{
			if (m_opts.m_progress == nullptr) return;
			if (m_opts.m_progress(step, total, message, nest)) return;
			throw std::runtime_error("user cancelled");
		}
	};

	// Dump the structure of a glTF file to a stream
	struct Dumper
	{
		cgltf_data const& m_data;
		DumpOptions const& m_opts;
		std::ostream& m_out;

		Dumper(cgltf_data const& data, DumpOptions const& opts, std::ostream& out)
			: m_data(data)
			, m_opts(opts)
			, m_out(out)
		{
			out << std::showpos;
		}

		void Do() const
		{
			if (AllSet(m_opts.m_parts, EParts::MainObjects))
				DumpMainObjects();
			if (AllSet(m_opts.m_parts, EParts::NodeHierarchy))
				DumpHierarchy();
			if (AllSet(m_opts.m_parts, EParts::Meshes))
				DumpGeometry();
			if (AllSet(m_opts.m_parts, EParts::Skeletons))
				DumpSkeletons();
			if (AllSet(m_opts.m_parts, EParts::Animation))
				DumpAnimation();
		}

		void DumpMainObjects() const
		{
			int ind = 0;
			m_out << "Main Objects:\n";
			{
				++ind;

				// Asset info
				m_out << Indent(ind) << "Asset:\n";
				{
					++ind;
					if (m_data.asset.generator) m_out << Indent(ind) << "Generator: " << m_data.asset.generator << "\n";
					if (m_data.asset.version) m_out << Indent(ind) << "Version: " << m_data.asset.version << "\n";
					--ind;
				}

				m_out << Indent(ind) << "Meshes: " << m_data.meshes_count << "\n";
				for (cgltf_size i = 0; i != m_data.meshes_count; ++i)
				{
					++ind;
					auto const& mesh = m_data.meshes[i];
					m_out << Indent(ind) << "MESH: " << (mesh.name ? mesh.name : "(unnamed)") << " (" << i << ")\n";
					--ind;
				}

				m_out << Indent(ind) << "Materials: " << m_data.materials_count << "\n";
				for (cgltf_size i = 0; i != m_data.materials_count; ++i)
				{
					++ind;
					auto const& mat = m_data.materials[i];
					m_out << Indent(ind) << "MAT: " << (mat.name ? mat.name : "(unnamed)") << " (" << i << ")\n";
					--ind;
				}

				m_out << Indent(ind) << "Skins: " << m_data.skins_count << "\n";
				for (cgltf_size i = 0; i != m_data.skins_count; ++i)
				{
					++ind;
					auto const& skin = m_data.skins[i];
					m_out << Indent(ind) << "SKIN: " << (skin.name ? skin.name : "(unnamed)") << " (" << i << ") Joints: " << skin.joints_count << "\n";
					--ind;
				}

				m_out << Indent(ind) << "Animations: " << m_data.animations_count << "\n";
				for (cgltf_size i = 0; i != m_data.animations_count; ++i)
				{
					++ind;
					auto const& anim = m_data.animations[i];
					m_out << Indent(ind) << "ANIM: " << (anim.name ? anim.name : "(unnamed)") << " (" << i << ") Channels: " << anim.channels_count << "\n";
					--ind;
				}

				--ind;
			}
		}

		void DumpHierarchy() const
		{
			auto const* scene = m_data.scene ? m_data.scene : (m_data.scenes_count > 0 ? &m_data.scenes[0] : nullptr);
			if (scene == nullptr) return;

			m_out << "Node Hierarchy:\n";
			for (cgltf_size ni = 0; ni != scene->nodes_count; ++ni)
			{
				WalkHierarchy(scene->nodes[ni], 1, [this](cgltf_node const* node, int level) -> bool
				{
					m_out << Indent(level) << "NODE: " << (node->name ? node->name : "(unnamed)") << " (" << NodeIndex(m_data, node) << ")\n";
					{
						auto o2p = NodeLocalTransform(node);
						m_out << Indent(level + 1) << "O2P: " << o2p << "\n";
						if (node->mesh) m_out << Indent(level + 1) << "Mesh: " << (node->mesh->name ? node->mesh->name : "(unnamed)") << "\n";
						if (node->skin) m_out << Indent(level + 1) << "Skin: " << (node->skin->name ? node->skin->name : "(unnamed)") << "\n";
					}
					return true;
				});
			}
		}

		void DumpGeometry() const
		{
			int ind = 0;
			m_out << "Geometry:\n";
			for (cgltf_size mi = 0; mi != m_data.meshes_count; ++mi)
			{
				auto const& mesh = m_data.meshes[mi];
				++ind;
				m_out << Indent(ind) << "Mesh (ID: " << mi << "):\n";
				{
					++ind;
					m_out << Indent(ind) << "Name: " << (mesh.name ? mesh.name : "(unnamed)") << "\n";
					m_out << Indent(ind) << "Primitives: " << mesh.primitives_count << "\n";
					for (cgltf_size pi = 0; pi != mesh.primitives_count; ++pi)
					{
						auto const& prim = mesh.primitives[pi];
						++ind;
						m_out << Indent(ind) << "Primitive " << pi << ":\n";
						{
							++ind;
							m_out << Indent(ind) << "Type: " << prim.type << "\n";
							m_out << Indent(ind) << "Attributes: " << prim.attributes_count << "\n";
							for (cgltf_size ai = 0; ai != prim.attributes_count; ++ai)
							{
								auto const& attr = prim.attributes[ai];
								m_out << Indent(ind + 1) << (attr.name ? attr.name : "?") << " count=" << (attr.data ? attr.data->count : 0) << "\n";
							}
							if (prim.indices) m_out << Indent(ind) << "Indices: " << prim.indices->count << "\n";
							if (prim.material) m_out << Indent(ind) << "Material: " << (prim.material->name ? prim.material->name : "(unnamed)") << "\n";
							--ind;
						}
						--ind;
					}
					--ind;
				}
				--ind;
			}
		}

		void DumpSkeletons() const
		{
			int ind = 0;
			m_out << "Skins/Skeletons:\n";
			for (cgltf_size si = 0; si != m_data.skins_count; ++si)
			{
				auto const& skin = m_data.skins[si];
				++ind;
				m_out << Indent(ind) << "Skin (ID: " << si << "):\n";
				{
					++ind;
					m_out << Indent(ind) << "Name: " << (skin.name ? skin.name : "(unnamed)") << "\n";
					m_out << Indent(ind) << "Joints: " << skin.joints_count << "\n";
					auto limit = std::min(skin.joints_count, s_cast<cgltf_size>(m_opts.m_summary_length));
					for (cgltf_size ji = 0; ji != limit; ++ji)
					{
						auto const* joint = skin.joints[ji];
						m_out << Indent(ind + 1) << "Joint " << ji << ": " << (joint->name ? joint->name : "(unnamed)") << " (Node " << NodeIndex(m_data, joint) << ")\n";
					}
					if (limit < skin.joints_count)
						m_out << Indent(ind + 1) << "... (" << skin.joints_count - limit << " more)\n";
					--ind;
				}
				--ind;
			}
		}

		void DumpAnimation() const
		{
			int ind = 0;
			m_out << "Animation:\n";
			for (cgltf_size ai = 0; ai != m_data.animations_count; ++ai)
			{
				auto const& anim = m_data.animations[ai];
				++ind;
				m_out << Indent(ind) << "Animation (ID: " << ai << "):\n";
				{
					++ind;
					m_out << Indent(ind) << "Name: " << (anim.name ? anim.name : "(unnamed)") << "\n";
					m_out << Indent(ind) << "Samplers: " << anim.samplers_count << "\n";
					m_out << Indent(ind) << "Channels: " << anim.channels_count << "\n";
					auto limit = std::min(anim.channels_count, s_cast<cgltf_size>(m_opts.m_summary_length));
					for (cgltf_size ci = 0; ci != limit; ++ci)
					{
						auto const& chan = anim.channels[ci];
						char const* path = "?";
						switch (chan.target_path)
						{
							case cgltf_animation_path_type_translation: path = "translation"; break;
							case cgltf_animation_path_type_rotation: path = "rotation"; break;
							case cgltf_animation_path_type_scale: path = "scale"; break;
							case cgltf_animation_path_type_weights: path = "weights"; break;
							default: break;
						}
						m_out << Indent(ind + 1) << "Channel " << ci << ": " << path << " -> " << (chan.target_node && chan.target_node->name ? chan.target_node->name : "?") << "\n";
					}
					if (limit < anim.channels_count)
						m_out << Indent(ind + 1) << "... (" << anim.channels_count - limit << " more)\n";
					--ind;
				}
				--ind;
			}
		}

		// Indent helper
		static std::string_view Indent(int amount)
		{
			constexpr static char const space[] = "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t";
			constexpr static int len = (int)_countof(space);
			return std::string_view(space, amount < len ? amount : len);
		}
	};

	// Loaded scene data
	struct SceneData
	{
		cgltf_data* m_gltfdata;
		std::vector<char> m_file_buffer; // Holds data when loaded from stream

		// Load from file
		SceneData(char const* filepath, LoadOptions const&)
			: m_gltfdata(nullptr)
			, m_file_buffer()
		{
			cgltf_options options = {};
			auto result = cgltf_parse_file(&options, filepath, &m_gltfdata);
			if (result != cgltf_result_success)
				throw std::runtime_error("glTF parse error: " + ResultToString(result));

			result = cgltf_load_buffers(&options, m_gltfdata, filepath);
			if (result != cgltf_result_success)
			{
				cgltf_free(m_gltfdata);
				m_gltfdata = nullptr;
				throw std::runtime_error("glTF buffer load error: " + ResultToString(result));
			}

			result = cgltf_validate(m_gltfdata);
			if (result != cgltf_result_success)
			{
				cgltf_free(m_gltfdata);
				m_gltfdata = nullptr;
				throw std::runtime_error("glTF validation error: " + ResultToString(result));
			}
		}

		// Load from stream
		SceneData(std::istream& src, LoadOptions const& opts)
			: m_gltfdata(nullptr)
			, m_file_buffer()
		{
			if (!src.good())
				throw std::runtime_error("glTF input stream is unhealthy");

			// Read the entire stream into memory
			m_file_buffer.assign(std::istreambuf_iterator<char>(src), std::istreambuf_iterator<char>());
			if (m_file_buffer.empty())
				throw std::runtime_error("glTF input stream is empty");

			cgltf_options options = {};
			auto result = cgltf_parse(&options, m_file_buffer.data(), m_file_buffer.size(), &m_gltfdata);
			if (result != cgltf_result_success)
				throw std::runtime_error("glTF parse error: " + ResultToString(result));

			// Load buffer data. For GLB files this wires up the embedded binary chunk.
			// For glTF with external .bin files, this needs a path hint to resolve relative URIs.
			auto path_hint = opts.filename.empty() ? nullptr : std::string(opts.filename).c_str();
			result = cgltf_load_buffers(&options, m_gltfdata, path_hint);
			if (result != cgltf_result_success)
			{
				cgltf_free(m_gltfdata);
				m_gltfdata = nullptr;
				throw std::runtime_error("glTF buffer load error: " + ResultToString(result));
			}

			result = cgltf_validate(m_gltfdata);
			if (result != cgltf_result_success)
			{
				cgltf_free(m_gltfdata);
				m_gltfdata = nullptr;
				throw std::runtime_error("glTF validation error: " + ResultToString(result));
			}
		}

		~SceneData()
		{
			if (m_gltfdata)
				cgltf_free(m_gltfdata);
		}

		SceneData(SceneData const&) = delete;
		SceneData& operator=(SceneData const&) = delete;

		operator cgltf_data const& () const
		{
			return *m_gltfdata;
		}
	};

	// An RAII dll reference
	struct Context
	{
	private:
		using SceneCont = std::vector<std::shared_ptr<SceneData>>;

		ErrorHandler m_error_cb;
		mutable std::mutex m_mutex;
		SceneCont m_scenes;

	public:

		Context(ErrorHandler error_cb)
			: m_error_cb(error_cb)
			, m_mutex()
			, m_scenes()
		{
		}
		Context(Context&&) = delete;
		Context(Context const&) = delete;
		Context& operator=(Context&&) = delete;
		Context& operator=(Context const&) = delete;

		// Report errors
		void ReportError(std::string_view msg) const
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_error_cb(msg);
		}

		// Add 'gltfscene' to this context
		SceneData* AddScene(std::shared_ptr<SceneData>&& scene)
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_scenes.push_back(scene);
			return m_scenes.back().get();
		}
	};
}

extern "C"
{
	std::mutex g_mutex;
	std::vector<std::unique_ptr<Context>> g_contexts;
	HINSTANCE g_instance;

	// DLL entry point
	BOOL APIENTRY DllMain(HINSTANCE hInstance, DWORD ul_reason_for_call, LPVOID)
	{
		g_instance = hInstance;
		switch (ul_reason_for_call)
		{
			case DLL_PROCESS_ATTACH: break;
			case DLL_PROCESS_DETACH: break;
			case DLL_THREAD_ATTACH: break;
			case DLL_THREAD_DETACH: break;
		}
		return TRUE;
	}

	// Create a dll context
	__declspec(dllexport) Context* __stdcall Gltf_Initialise(ErrorHandler error_cb)
	{
		try
		{
			std::lock_guard<std::mutex> lock(g_mutex);
			g_contexts.push_back(std::unique_ptr<Context>(new Context(error_cb)));
			return g_contexts.back().get();
		}
		catch (std::exception const& ex)
		{
			error_cb(ex.what());
			return nullptr;
		}
	}

	// Release a dll context
	__declspec(dllexport) void __stdcall Gltf_Release(Context* ctx)
	{
		try
		{
			if (ctx == nullptr) return;
			std::lock_guard<std::mutex> lock(g_mutex);
			g_contexts.erase(std::remove_if(begin(g_contexts), end(g_contexts), [ctx](auto& p) { return p.get() == ctx; }), end(g_contexts));
		}
		catch (std::exception const& ex)
		{
			ctx->ReportError(ex.what());
		}
	}

	// Load a glTF scene from a file path
	__declspec(dllexport) SceneData* __stdcall Gltf_Scene_LoadFile(Context& ctx, char const* filepath, LoadOptions const& opts)
	{
		try
		{
			return ctx.AddScene(std::shared_ptr<SceneData>{ new SceneData(filepath, opts) });
		}
		catch (std::exception const& ex)
		{
			ctx.ReportError(ex.what());
			return nullptr;
		}
	}

	// Load a glTF scene from a stream
	__declspec(dllexport) SceneData* __stdcall Gltf_Scene_Load(Context& ctx, std::istream& src, LoadOptions const& opts)
	{
		try
		{
			return ctx.AddScene(std::shared_ptr<SceneData>{ new SceneData(src, opts) });
		}
		catch (std::exception const& ex)
		{
			ctx.ReportError(ex.what());
			return nullptr;
		}
	}

	// Read the hierarchy from the scene
	__declspec(dllexport) void __stdcall Gltf_Scene_Read(Context& ctx, SceneData& scene, ReadOptions const& options, IReadOutput& out)
	{
		try
		{
			NullCheck(scene.m_gltfdata, "Scene is null");
			Reader reader(*scene.m_gltfdata, options, out);
			reader.Do();
		}
		catch (std::exception const& ex)
		{
			ctx.ReportError(ex.what());
		}
	}

	// Dump info about the scene to 'out'
	__declspec(dllexport) void __stdcall Gltf_Scene_Dump(Context& ctx, SceneData const& scene, DumpOptions const& options, std::ostream& out)
	{
		try
		{
			NullCheck(scene.m_gltfdata, "Scene is null");
			Dumper dumper(*scene.m_gltfdata, options, out);
			dumper.Do();
		}
		catch (std::exception const& ex)
		{
			ctx.ReportError(ex.what());
		}
	}

	// Static function signature checks
	void Gltf::StaticChecks()
	{
		#define PR_GLTF_API_CHECK(prefix, name, function_type)\
		static_assert(std::is_same_v<Gltf::prefix##name##Fn, decltype(&Gltf_##prefix##name)>, "Function signature mismatch for Gltf_"#prefix#name);
		PR_GLTF_API(PR_GLTF_API_CHECK);
		#undef PR_GLTF_API_CHECK
	}
}
