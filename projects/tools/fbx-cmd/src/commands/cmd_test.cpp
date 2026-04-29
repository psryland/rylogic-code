#include "src/forward.h"

using namespace pr;
using namespace pr::geometry;

namespace fbx_cmd
{
	void test()
	{
		std::filesystem::path ifilepath{
			//"E:\\Rylogic\\Code\\art\\models\\pendulum\\pendulum.fbx"
			//"E:\\Rylogic\\Code\\art\\models\\AnimCharacter\\AnimatedCharacter.fbx"
			"E:\\Dump\\biplane.fbx"
		};
		std::filesystem::path p3doutpath{
			"E:\\Dump\\model.p3d"
		};
		std::filesystem::path ofilepath{
			"E:\\Dump\\LDraw\\fbx-round-trip.fbx"
		};
		std::filesystem::path dfilepath{
			"E:\\Dump\\LDraw\\fbx-dump-animchar.txt"
		};

		std::ifstream ifile(ifilepath, std::ios::binary);
		//' std::ofstream ofile(ofilepath, std::ios::binary);
		std::ofstream dfile(dfilepath);

		//' dll.Fbx_RoundTripTest(ifile, ofile);
		//' dll.Fbx_DumpStream(ifile, dfile);
		fbx::Scene scene(ifile);
		scene.Dump(dfile, {
			.m_parts =
				fbx::EParts::NodeHierarchy |
				fbx::EParts::Meshes |
				fbx::EParts::Skeletons |
				fbx::EParts::Skins |
				fbx::EParts::None,
		});

		#if 0
		// Convert the models the p3d
		p3d::File file = {};
		{
			// Materials
			for (auto const& [unique_id, mat] : fbxscene.m_materials)
			{
				p3d::Material material;
				material.m_id = std::format("mat-{}", unique_id);
				material.m_diffuse = mat.m_diffuse;
				file.m_scene.m_materials.push_back(material);
			}

			// Models
			for (fbx::Mesh const& mesh : fbxscene.m_meshes)
			{
				// Ignoring 'mesh.m_level'
				p3d::Mesh m;
				m.m_name = mesh.m_name;
				m.m_bbox = mesh.m_bbox;
				m.m_o2p = mesh.m_o2p;
				for (auto& v : mesh.m_vbuf)
				{
					m.add_vert({ v.m_vert, v.m_colr, v.m_norm, v.m_tex0 });
				}
				for (auto& n : mesh.m_nbuf)
				{
					p3d::Nugget nugget{ n.m_topo, n.m_geom, std::format("mat-{}", n.m_mat_id) };
					nugget.m_vidx.append<int>(std::span{ mesh.m_ibuf }.subspan(n.m_irange.begin(), n.m_irange.size()));
					m.add_nugget(nugget);
				}
				file.m_scene.m_meshes.push_back(std::move(m));
			}
		}

		if (std::ofstream ofile(p3doutpath, std::ios::binary); ofile)
			p3d::Write(ofile, file);
		#endif
	}
}