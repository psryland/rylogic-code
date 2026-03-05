//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/model/vertex_layout.h"
#include "pr/view3d-12/model/model_desc.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/model/model.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/texture/texture_cube.h"
#include "pr/view3d-12/texture/texture_loader.h"
#include "pr/view3d-12/sampler/sampler_desc.h"
#include "pr/view3d-12/sampler/sampler.h"
#include "pr/view3d-12/shaders/shader_forward.h"
#include "pr/view3d-12/shaders/shader_point_sprites.h"
#include "pr/view3d-12/shaders/shader_thick_line.h"
#include "pr/view3d-12/shaders/shader_arrow_head.h"
#include "pr/view3d-12/ldraw/ldraw_reader_text.h"
#include "pr/view3d-12/utility/update_resource.h"

namespace pr::rdr12
{
	ResourceFactory::ResourceFactory(Renderer& rdr)
		: m_rdr(rdr)
		, m_gsync(rdr.D3DDevice())
		, m_keep_alive(m_gsync)
		, m_gfx_cmd_alloc_pool(m_gsync)
		, m_gfx_cmd_list(rdr.D3DDevice(), m_gfx_cmd_alloc_pool.Get(), nullptr, "ResourceFactory", EColours::LightGreen)
		, m_upload_buffer(m_gsync, 1ULL * 1024 * 1024)
		, m_mipmap_gen(rdr, m_gsync, m_gfx_cmd_list)
		, m_flush_required()
	{}
	ResourceFactory::~ResourceFactory()
	{
		// Wait till stock resources are created
		FlushToGpu(EGpuFlush::Block);
	}

	// Renderer access
	ID3D12Device4* ResourceFactory::d3d() const
	{
		return rdr().d3d();
	}
	Renderer& ResourceFactory::rdr() const
	{
		return m_rdr;
	}

	// Access the command list associated with this factory instance
	GfxCmdList& ResourceFactory::CmdList()
	{
		m_flush_required = true;
		return m_gfx_cmd_list;
	}

	// Access the upload buffer associated with this factory instance
	GpuUploadBuffer& ResourceFactory::UploadBuffer()
	{
		m_flush_required = true;
		return m_upload_buffer;
	}

	// Flush creation commands to the GPU. Returns the sync point for when they've been executed
	uint64_t ResourceFactory::FlushToGpu(EGpuFlush flush)
	{
		if (!m_flush_required || flush == EGpuFlush::DontFlush)
			return m_gsync.LastAddedSyncPoint();

		// Close the command list
		m_gfx_cmd_list.Close();

		pix::BeginEvent(rdr().GfxQueue(), s_cast<uint32_t>(EColours::LightGreen), "ResourceManager Flush");

		// Execute the command list
		rdr().ExecuteGfxCommandLists({ m_gfx_cmd_list });

		m_flush_required = false;

		// Add a sync point
		auto sync_point = m_gsync.AddSyncPoint(rdr().GfxQueue());
		m_gfx_cmd_list.SyncPoint(sync_point);

		// Reset the command list
		m_gfx_cmd_list.Reset(m_gfx_cmd_alloc_pool.Get());

		// Wait till done?
		if (flush == EGpuFlush::Block)
			Wait(sync_point);

		pix::EndEvent(rdr().GfxQueue());
		return sync_point;
	}

	// Wait for the GPU to finish processing the internal command list
	void ResourceFactory::Wait(uint64_t sync_point) const
	{
		m_gsync.Wait(sync_point);
	}

	// Create and initialise a resource
	D3DPtr<ID3D12Resource> ResourceFactory::CreateResource(ResDesc const& desc, std::string_view name)
	{
		D3DPtr<ID3D12Resource> res;
		auto device = rdr().D3DDevice();
		auto has_init_data = !desc.Data.empty();

		if (desc.Width == 0)
			return res;

		// Buffer resources specify the Width as the size in bytes, even though for textures width is the pixel count.
		ResDesc rd = desc;
		if (desc.Dimension == D3D12_RESOURCE_DIMENSION_BUFFER)
			rd.Width *= desc.ElemStride;

		// Create a GPU visible resource that will hold the created texture/verts/indices/etc.
		// Create in the COMMON state to prevent a D3D12 warning "Buffers are effectively created in state D3D12_RESOURCE_STATE_COMMON"
		// COMMON state is implicitly promoted to the first state transition.
		assert(desc.Check());
		Check(device->CreateCommittedResource(
			&desc.HeapProps, desc.HeapFlags, &rd, D3D12_RESOURCE_STATE_COMMON,
			desc.ClearValue ? &*desc.ClearValue : nullptr,
			__uuidof(ID3D12Resource), (void**)res.address_of()));

		// Assume common state until the resource is initialised
		DefaultResState(res.get(), D3D12_RESOURCE_STATE_COMMON);
		DebugName(res, name);

		// If initialisation data is provided, initialise using an UploadBuffer
		if (has_init_data)
		{
			// Copy the initialisation data for each array slice into the resource
			auto array_length = desc.Dimension == D3D12_RESOURCE_DIMENSION_TEXTURE3D ? 1 : s_cast<int>(desc.DepthOrArraySize);
			for (auto i = 0; i != array_length; ++i)
			{
				// Note: here 'desc.Data' is an array of mip-level-zero images.
				// The span of images expected by 'UpdateSubresource' is for each mip level.
				UpdateSubresourceScope map(m_gfx_cmd_list, m_upload_buffer, res.get(), i, 0, 1, desc.DataAlignment);
				map.Write(desc.Data[i], AllSet(desc.MiscFlags, ResDesc::EMiscFlags::PartialInitData));
				map.Commit(EFinalState::Override, desc.DefaultState);
			}

			// Generate mip maps for the texture (if needed)
			// 'm_mipmap_gen' should use the same cmd-list as the resource manager, so that mips are generated as
			// textures are created. Remember cmd-lists are executed serially.
			if (desc.MipLevels != 1)
				m_mipmap_gen.Generate(res.get());
		}

		// Set the true default state for the resource now that it's initialised
		DefaultResState(res.get(), desc.DefaultState);

		// Transition the resource to the default state
		BarrierBatch barriers(m_gfx_cmd_list);
		barriers.Transition(res.get(), desc.DefaultState);
		barriers.Commit();

		// Trace the pointers of resources that have been created
		#if 0
		{
			OutputDebugStringA(std::format("{} -> 0x{:016x}\n", name, uintptr_t(res.get())).c_str());
			if (name == "Cube.003-ibuf")
				name = name;
		}
		#endif

		m_keep_alive.Add(res);
		m_flush_required = true;
		return res;
	}

	// Create a model.
	ModelPtr ResourceFactory::CreateModel(ModelDesc const& mdesc, D3DPtr<ID3D12Resource> vb, D3DPtr<ID3D12Resource> ib)
	{
		if (mdesc.m_vb.Width == 0)
			throw std::runtime_error("Attempt to create 0-length model vertex buffer");
		if (mdesc.m_ib.Width == 0)
			throw std::runtime_error("Attempt to create 0-length model index buffer");

		// Create a V/I buffers
		vb = vb ? vb : CreateResource(mdesc.m_vb, string32(mdesc.m_name).append("-vbuf"));
		ib = ib ? ib : CreateResource(mdesc.m_ib, string32(mdesc.m_name).append("-ibuf"));

		// Set the size and alignment of the vertex/index element types
		SizeAndAlign16 vstride(mdesc.m_vb.ElemStride, mdesc.m_vb.DataAlignment);
		SizeAndAlign16 istride(mdesc.m_ib.ElemStride, mdesc.m_ib.DataAlignment);

		// Create the model
		ModelPtr ptr(rdr12::New<Model>(rdr(),
			s_cast<int64_t>(mdesc.m_vb.Width),
			s_cast<int64_t>(mdesc.m_ib.Width),
			vstride,
			istride,
			vb.get(),
			ib.get(),
			mdesc.m_bbox,
			mdesc.m_m2root,
			mdesc.m_name
		), true);
		assert(m_rdr.mem_tracker().add(ptr.m_ptr));
		return ptr;
	}
	ModelPtr ResourceFactory::CreateModel(ModelDesc const& mdesc)
	{
		return CreateModel(mdesc, nullptr, nullptr);
	}
	ModelPtr ResourceFactory::CreateModel(EStockModel id)
	{
		switch (id)
		{
			case EStockModel::Basis:
			{
				// Basis/focus point model
				constexpr Vert verts[] = {
					{v4(0.0f,  0.0f,  0.0f, 1.0f), Colour(0xFFFF0000), v4::Zero(), v2::Zero()},
					{v4(1.0f,  0.0f,  0.0f, 1.0f), Colour(0xFFFF0000), v4::Zero(), v2::Zero()},
					{v4(0.0f,  0.0f,  0.0f, 1.0f), Colour(0xFF00FF00), v4::Zero(), v2::Zero()},
					{v4(0.0f,  1.0f,  0.0f, 1.0f), Colour(0xFF00FF00), v4::Zero(), v2::Zero()},
					{v4(0.0f,  0.0f,  0.0f, 1.0f), Colour(0xFF0000FF), v4::Zero(), v2::Zero()},
					{v4(0.0f,  0.0f,  1.0f, 1.0f), Colour(0xFF0000FF), v4::Zero(), v2::Zero()},
				};
				constexpr uint16_t idxs[] = {
					0, 1, 2, 3, 4, 5,
				};
				constexpr BBox bbox = {
					v4(0.5f, 0.5f, 0.5f, 1.0f),
					v4(1, 1, 1, 0)
				};

				ModelDesc mdesc = ModelDesc().vbuf(verts).ibuf(idxs).bbox(bbox).name("basis");
				auto ptr = CreateModel(mdesc);

				NuggetDesc nug(ETopo::LineList, EGeom::Vert | EGeom::Colr);
				nug.m_nflags = SetBits(nug.m_nflags, ENuggetFlag::ShadowCastExclude, true);
				ptr->CreateNugget(*this, nug);

				return ptr;
			}
			case EStockModel::UnitQuad:
			{
				// Unit quad in Z = 0 plane
				constexpr Vert verts[] = {
					{v4(-0.5f,-0.5f, 0, 1), Colour(0xFFFFFFFF), v4::ZAxis(), v2(0.0000f,0.9999f)},
					{v4(0.5f,-0.5f, 0, 1), Colour(0xFFFFFFFF), v4::ZAxis(), v2(0.9999f,0.9999f)},
					{v4(0.5f, 0.5f, 0, 1), Colour(0xFFFFFFFF), v4::ZAxis(), v2(0.9999f,0.0000f)},
					{v4(-0.5f, 0.5f, 0, 1), Colour(0xFFFFFFFF), v4::ZAxis(), v2(0.0000f,0.0000f)},
				};
				constexpr uint16_t idxs[] = {
					0, 1, 2, 0, 2, 3
				};
				constexpr BBox bbox = {
					v4::Origin(),
					v4(1, 1, 0, 0)
				};

				ModelDesc mdesc = ModelDesc().vbuf(verts).ibuf(idxs).bbox(bbox).name("unit quad");
				auto ptr = CreateModel(mdesc);

				NuggetDesc nug(ETopo::TriList, EGeom::Vert | EGeom::Colr | EGeom::Norm | EGeom::Tex0);
				ptr->CreateNugget(*this, nug);
				return ptr;
			}
			case EStockModel::BBoxModel:
			{
				// Bounding box cube
				constexpr Vert verts[] = {
					{v4(-0.5f, -0.5f, -0.5f, 1.0f), Colour(0xFF0000FF), v4::Zero(), v2::Zero()},
					{v4(+0.5f, -0.5f, -0.5f, 1.0f), Colour(0xFF0000FF), v4::Zero(), v2::Zero()},
					{v4(+0.5f, +0.5f, -0.5f, 1.0f), Colour(0xFF0000FF), v4::Zero(), v2::Zero()},
					{v4(-0.5f, +0.5f, -0.5f, 1.0f), Colour(0xFF0000FF), v4::Zero(), v2::Zero()},
					{v4(-0.5f, -0.5f, +0.5f, 1.0f), Colour(0xFF0000FF), v4::Zero(), v2::Zero()},
					{v4(+0.5f, -0.5f, +0.5f, 1.0f), Colour(0xFF0000FF), v4::Zero(), v2::Zero()},
					{v4(+0.5f, +0.5f, +0.5f, 1.0f), Colour(0xFF0000FF), v4::Zero(), v2::Zero()},
					{v4(-0.5f, +0.5f, +0.5f, 1.0f), Colour(0xFF0000FF), v4::Zero(), v2::Zero()},
				};
				constexpr uint16_t idxs[] = {
					0, 1, 1, 2, 2, 3, 3, 0,
					4, 5, 5, 6, 6, 7, 7, 4,
					0, 4, 1, 5, 2, 6, 3, 7,
				};
				constexpr BBox bbox = {
					v4::Origin(),
					v4(1, 1, 1, 0)
				};

				ModelDesc mdesc = ModelDesc().vbuf(verts).ibuf(idxs).bbox(bbox).name("bbox cube");
				auto ptr = CreateModel(mdesc);

				NuggetDesc nug(ETopo::LineList, EGeom::Vert | EGeom::Colr);
				nug.m_nflags = SetBits(nug.m_nflags, ENuggetFlag::ShadowCastExclude, true);
				ptr->CreateNugget(*this, nug);

				return ptr;
			}
			case EStockModel::SelectionBox:
			{
				// Selection box
				// Create the selection box model
				constexpr float sz = 1.0f;
				constexpr float dd = 0.8f;
				constexpr Vert verts[] = {
					{v4(-sz, -sz, -sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(-dd, -sz, -sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(-sz, -dd, -sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(-sz, -sz, -dd, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},

					{v4(sz, -sz, -sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(sz, -dd, -sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(dd, -sz, -sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(sz, -sz, -dd, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},

					{v4(sz, sz, -sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(dd, sz, -sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(sz, dd, -sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(sz, sz, -dd, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},

					{v4(-sz, sz, -sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(-sz, dd, -sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(-dd, sz, -sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(-sz, sz, -dd, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},

					{v4(-sz, -sz, sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(-dd, -sz, sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(-sz, -dd, sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(-sz, -sz, dd, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},

					{v4(sz, -sz, sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(sz, -dd, sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(dd, -sz, sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(sz, -sz, dd, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},

					{v4(sz, sz, sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(dd, sz, sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(sz, dd, sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(sz, sz, dd, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},

					{v4(-sz, sz, sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(-sz, dd, sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(-dd, sz, sz, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
					{v4(-sz, sz, dd, 1.0f), Colour(0xFFFFFFFF), v4::Zero(), v2::Zero()},
				};
				constexpr uint16_t idxs[] = {
					0,  1,  0,  2,  0,  3,
					4,  5,  4,  6,  4,  7,
					8,  9,  8, 10,  8, 11,
					12, 13, 12, 14, 12, 15,
					16, 17, 16, 18, 16, 19,
					20, 21, 20, 22, 20, 23,
					24, 25, 24, 26, 24, 27,
					28, 29, 28, 30, 28, 31,
				};
				constexpr BBox bbox = {
					v4::Origin(),
					v4(1, 1, 1, 0)
				};

				ModelDesc mdesc = ModelDesc().vbuf(verts).ibuf(idxs).bbox(bbox).name("selection box");
				auto ptr = CreateModel(mdesc);

				NuggetDesc nug(ETopo::LineList, EGeom::Vert);
				nug.m_nflags = SetBits(nug.m_nflags, ENuggetFlag::ShadowCastExclude, true);
				ptr->CreateNugget(*this, nug);

				return ptr;
			}
			default:
			{
				throw std::runtime_error("Unknown stock model type");
			}
		}
	}

	// Create a new nugget
	Nugget* ResourceFactory::CreateNugget(NuggetDesc const& ndata, Model* model)
	{
		auto ptr = rdr12::New<Nugget>(ndata, model);
		assert(rdr().mem_tracker().add(ptr));
		return ptr;
	}

	// Create a new texture instance.
	Texture2DPtr ResourceFactory::CreateTexture2D(TextureDesc const& desc)
	{
		if (desc.m_tdesc.DepthOrArraySize != 1)
			throw std::runtime_error("Expected a 2D texture");

		D3DPtr<ID3D12Resource> res;

		// If a uri is given, see if the DX resource already exists
		if (desc.m_uri != 0)
		{
			ResourceStore::Access store(rdr());
			res = store.FindRes(desc.m_uri);
			if (res == nullptr)
			{
				// If not, create the resource and add it to the lookup
				res = CreateResource(desc.m_tdesc, desc.m_name.c_str());

				// Record the uri for reuse
				store.Add(desc.m_uri, res.get());
			}
		}

		// Otherwise, just create the texture
		else
		{
			res = CreateResource(desc.m_tdesc, desc.m_name.c_str());
		}

		// Allocate a new texture instance
		Texture2DPtr inst(rdr12::New<Texture2D>(rdr(), res.get(), desc), true);
		assert(rdr().mem_tracker().add(inst.get()));

		// Add the texture instance pointer (not ref counted) to the store.
		// The caller owns the texture, when released it will be removed automatically.
		{
			ResourceStore::Access store(rdr());
			store.Add(inst.get());
		}

		return inst;
	}
	Texture2DPtr ResourceFactory::CreateTexture2D(std::filesystem::path const& resource_path, TextureDesc const& desc_)
	{
		if (resource_path.empty())
			throw std::runtime_error("A resource path must be given");

		// Create the texture resource
		D3DPtr<ID3D12Resource> res;
		TextureDesc desc = desc_;

		// Accept stock texture strings: #black, #white, #checker, etc
		// This is handy for model files that contain string paths for textures.
		// The code that loads these models doesn't need to handle strings such as '#white' as a special case
		if (*resource_path.c_str() == '#')
		{
			auto stock = Enum<EStockTexture>::TryParse(std::wstring_view{ resource_path.c_str() + 1 }, false);
			if (!stock)
				throw std::runtime_error(Fmt("Unknown stock texture name: %s", resource_path.string().c_str() + 1));

			// Create a stock texture
			return CreateTexture(*stock);
		}

		// Create a texture from embedded resource ("@<module>:<res_type>:<res_name>" means embedded resource)
		else if (*resource_path.c_str() == '@')
		{
			auto uri = resource_path.wstring();

			desc.m_uri = MakeId(uri.c_str());
			if (desc.m_name.empty())
				desc.m_name = To<string32>(str::FindLastOf(uri.c_str(), L":"));

			// Look for an existing Dx12 resource corresponding to the uri
			ResourceStore::Access store(rdr());
			res = store.FindRes(desc.m_uri);
			if (res == nullptr)
			{
				// Parse the embedded resource string: "@<module>:<res_type>:<res_name>"
				HMODULE hmodule;
				wstring32 res_type, res_name;
				ParseEmbeddedResourceUri(uri, hmodule, res_type, res_name);

				// Get the embedded resource
				auto emb = resource::Read<uint8_t>(res_name.c_str(), res_type.c_str(), hmodule);
				auto data = std::span{ emb.m_data, emb.m_len };

				// Create the texture data
				auto [images, tdesc] = LoadImageData(data, 1, false, 0, &rdr().Features());
				desc.m_tdesc = tdesc;
				desc.m_tdesc.Data = images;

				// Create the texture
				res = CreateResource(desc.m_tdesc, desc.m_name.c_str());

				// Record the uri for reuse
				store.Add(desc.m_uri, res.get());
			}
		}

		// Otherwise, create from a file on disk
		else
		{
			using namespace std::filesystem;
			auto filepath = resource_path.lexically_normal();

			// Generate an id from the filepath
			desc.m_uri = MakeId(filepath.c_str());
			if (desc.m_name.empty())
				desc.m_name = To<string32>(filepath.filename().c_str());

			// Look for an existing DX texture corresponding to the filepath
			ResourceStore::Access store(rdr());
			res = store.FindRes(desc.m_uri);
			if (res == nullptr)
			{
				// If the texture filepath doesn't exist, use the resolve event
				if (!exists(filepath))
					filepath = rdr().ResolvePath(filepath.string());

				// Load the texture from disk
				auto [images, tdesc] = LoadImageData(filepath, 1, false, 0, &rdr().Features());
				desc.m_tdesc = tdesc;
				desc.m_tdesc.Data = images;

				// Create the texture
				res = CreateResource(desc.m_tdesc, desc.m_name.c_str());

				// Record the uri for reuse
				store.Add(desc.m_uri, res.get());
			}
		}
	
		// Allocate a new texture instance
		Texture2DPtr inst(rdr12::New<Texture2D>(rdr(), res.get(), desc), true);
		assert(rdr().mem_tracker().add(inst.get()));

		// Add a pointer (not ref counted) to the texture instance to the lookup table.
		// The caller owns the texture, when released it will be removed from this lookup.
		{
			ResourceStore::Access store(rdr());
			store.Add(inst.get());
		}

		return inst;
	}
	TextureCubePtr ResourceFactory::CreateTextureCube(std::filesystem::path const& resource_path, TextureDesc const& desc_)
	{
		// Notes:
		//  - A cube map is an array of 6 2D textures.
		//  - DDS image files contain all six faces in the single file. Other image types need to be loaded separately.
		//  - 'resource_path' should contain '??' where the first '?' is the sign (+,-) and the second '?' is the axis (x,y,z)

		if (resource_path.empty())
			throw std::runtime_error("Resource path must be given");
		
		// Create the texture resource
		D3DPtr<ID3D12Resource> res;
		TextureDesc desc = desc_;

		// Create a texture from embedded resources ("@<module>:<res_type>:<res_name>")
		if (resource_path.c_str()[0] == '@')
		{
			desc.m_uri = MakeId(resource_path.c_str());
			if (desc.m_name.empty())
				desc.m_name = To<string32>(str::FindLastOf(resource_path.c_str(), ":"));

			// Look for an existing Dx12 resource corresponding to the uri
			ResourceStore::Access store(rdr());
			res = store.FindRes(desc.m_uri);
			if (res == nullptr)
			{
				// Parse the embedded resource string: "@<module>:<res_type>:<res_name>"
				HMODULE hmodule;
				wstring32 res_type, res_name;
				ParseEmbeddedResourceUri(resource_path.wstring(), hmodule, res_type, res_name);

				// The faces of the cube
				pr::vector<std::span<uint8_t const>> source_images;

				// Read each face of the cube map
				auto idx = res_name.find(L"??");
				if (idx != std::wstring::npos)
				{
					// Get the data for each face of the cube map
					for (auto face : { L"px", L"nx", L"py", L"ny", L"pz", L"nz" })
					{
						res_name[idx + 0] = face[0];
						res_name[idx + 1] = face[1];
						auto emb = resource::Read<uint8_t>(res_name.c_str(), res_type.c_str(), hmodule);
						source_images.push_back(std::span{ emb.m_data, emb.m_len });
					}
				}
				else // Otherwise, the resource is a single file
				{
					auto emb = resource::Read<uint8_t>(res_name.c_str(), res_type.c_str(), hmodule);
					source_images.push_back(std::span{ emb.m_data, emb.m_len });
				}

				// Create the texture data
				auto [images, tdesc] = LoadImageData(source_images, 1, true, 0, &rdr().Features());
				desc.m_tdesc = tdesc;
				desc.m_tdesc.Data = images;

				// Create the texture
				res = CreateResource(desc.m_tdesc, desc.m_name.c_str());

				// Record the uri for reuse
				store.Add(desc.m_uri, res.get());
			}
		}

		// Otherwise, create from a file (or files) on disk
		else
		{
			using namespace std::filesystem;
			auto filepath = resource_path.lexically_normal();

			// Generate an id from the filepath
			desc.m_uri = MakeId(filepath.c_str());
			if (desc.m_name.empty())
				desc.m_name = To<string32>(filepath.filename().c_str());

			// Look for an existing DX texture corresponding to the filepath
			ResourceStore::Access store(rdr());
			res = store.FindRes(desc.m_uri);
			if (res == nullptr)
			{
				auto res_name = filepath.string();

				// The faces of the cube
				pr::vector<path> source_images;

				// If this is a filename pattern rather than a single file, load each face
				auto idx = res_name.find("??");
				if (idx != std::string::npos)
				{
					// Get the data for each face of the cube map
					for (auto face : { "px", "nx", "py", "ny", "pz", "nz" })
					{
						res_name[idx + 0] = face[0];
						res_name[idx + 1] = face[1];
						source_images.push_back(rdr().ResolvePath(res_name));
					}
				}
				else
				{
					// Otherwise, the filename is a single file (expect DDS file containing all six faces)
					source_images.push_back(rdr().ResolvePath(res_name));
				}

				// Load the texture from disk (supports '??' in the filepath)
				auto [images, tdesc] = LoadImageData(source_images, 1, true, 0, &rdr().Features());
				tdesc.DepthOrArraySize = s_cast<UINT16>(images.ssize());
				desc.m_tdesc = tdesc;
				desc.m_tdesc.Data = images;

				// Create the texture
				res = CreateResource(desc.m_tdesc, desc.m_name.c_str());

				// Record the uri for reuse
				store.Add(desc.m_uri, res.get());
			}
		}

		// Allocate a new texture instance
		TextureCubePtr inst(rdr12::New<TextureCube>(rdr(), res.get(), desc), true);
		assert(rdr().mem_tracker().add(inst.get()));

		// Add a pointer (not ref counted) to the texture instance to the lookup table.
		// The caller owns the texture, when released it will be removed from this lookup.
		{
			ResourceStore::Access store(rdr());
			store.Add(inst.get());
		}

		return inst;
	}
	Texture2DPtr ResourceFactory::CreateTexture(EStockTexture id)
	{
		// Note:
		//  - Don't make "auto tdesc" into "auto& tdesc" inspite of the lint warning.
		//    'auto& tdesc' binds to an rvalue which is destroyed at the end of the expression,
		//    so 'tdesc' is invalid in the call to CreateTexture2D.

		switch (id)
		{
			case EStockTexture::Black:
			{
				uint32_t const data[] = { 0xFF000000 };
				auto src = Image(1, 1, data, DXGI_FORMAT_B8G8R8A8_UNORM);
				TextureDesc tdesc = TextureDesc(AutoId, ResDesc::Tex2D(src, 1)).uri(EStockTexture::Black).name("#black");
				return CreateTexture2D(tdesc);
			}
			case EStockTexture::White:
			{
				uint32_t const data[] = { 0xFFFFFFFF };
				auto src = Image(1, 1, data, DXGI_FORMAT_B8G8R8A8_UNORM);
				TextureDesc tdesc = TextureDesc(AutoId, ResDesc::Tex2D(src, 1)).uri(EStockTexture::White).name("#white");
				return CreateTexture2D(tdesc);
			}
			case EStockTexture::Gray:
			{
				uint32_t const data[] = { 0xFF808080 };
				auto src = Image(1, 1, data, DXGI_FORMAT_B8G8R8A8_UNORM);
				TextureDesc tdesc = TextureDesc(AutoId, ResDesc::Tex2D(src, 1)).uri(EStockTexture::Gray).name("#gray");
				return CreateTexture2D(tdesc);
			}
			case EStockTexture::Checker:
			{
				constexpr uint32_t X = 0xFFFFFFFF;
				constexpr uint32_t O = 0x00000000;
				uint32_t const data[] =
				{
					X, X, O, O, X, X, O, O,
					X, X, O, O, X, X, O, O,
					O, O, X, X, O, O, X, X,
					O, O, X, X, O, O, X, X,
					X, X, O, O, X, X, O, O,
					X, X, O, O, X, X, O, O,
					O, O, X, X, O, O, X, X,
					O, O, X, X, O, O, X, X,
				};
				auto src = Image(8, 8, data, DXGI_FORMAT_B8G8R8A8_UNORM);
				TextureDesc tdesc = TextureDesc(AutoId, ResDesc::Tex2D(src, 0)).uri(EStockTexture::Checker).name("#checker");
				return CreateTexture2D(tdesc);
			}
			case EStockTexture::Checker2:
			{
				constexpr uint32_t X = 0xFFFFFFFF;
				constexpr uint32_t O = 0xFFAAAAAA;
				uint32_t const data[] =
				{
					X, X, O, O, X, X, O, O,
					X, X, O, O, X, X, O, O,
					O, O, X, X, O, O, X, X,
					O, O, X, X, O, O, X, X,
					X, X, O, O, X, X, O, O,
					X, X, O, O, X, X, O, O,
					O, O, X, X, O, O, X, X,
					O, O, X, X, O, O, X, X,
				};
				auto src = Image(8, 8, data, DXGI_FORMAT_B8G8R8A8_UNORM);
				TextureDesc tdesc = TextureDesc(AutoId, ResDesc::Tex2D(src, 0)).uri(EStockTexture::Checker2).name("#checker2");
				return CreateTexture2D(tdesc);
			}
			case EStockTexture::Checker3:
			{
				constexpr uint32_t X = 0xFFEEEEEE;
				constexpr uint32_t O = 0xFFFFFFFF;
				uint32_t const data[] =
				{
					X, X, O, O, X, X, O, O,
					X, X, O, O, X, X, O, O,
					O, O, X, X, O, O, X, X,
					O, O, X, X, O, O, X, X,
					X, X, O, O, X, X, O, O,
					X, X, O, O, X, X, O, O,
					O, O, X, X, O, O, X, X,
					O, O, X, X, O, O, X, X,
				};
				auto src = Image(8, 8, data, DXGI_FORMAT_B8G8R8A8_UNORM);
				TextureDesc tdesc = TextureDesc(AutoId, ResDesc::Tex2D(src, 0)).uri(EStockTexture::Checker3).name("#checker3");
				return CreateTexture2D(tdesc);
			}
			case EStockTexture::WhiteDot:
			{
				constexpr int sz = 256;
				constexpr auto radius = (sz - 1.0f) / 2.0f;
				std::vector<uint32_t> data(sz * sz);
				for (int j = 0; j != sz; ++j)
				{
					for (int i = 0; i != sz; ++i)
					{
						auto c = Colour32White;
						auto r = Len(i - radius, j - radius);
						c.a = r <= radius ? 0xff : 0x00;
						data[size_t(j * sz + i)] = c.argb;
					}
				}

				auto src = Image(sz, sz, data.data(), DXGI_FORMAT_B8G8R8A8_UNORM);
				TextureDesc tdesc = TextureDesc(AutoId, ResDesc::Tex2D(src, 0)).uri(EStockTexture::WhiteDot).has_alpha().name("#whitedot");
				return CreateTexture2D(tdesc);
			}
			case EStockTexture::WhiteSpot:
			{
				constexpr int sz = 256;
				constexpr auto radius = (sz - 1.0f) / 2.0f;
				std::vector<uint32_t> data(sz * sz);
				for (int j = 0; j != sz; ++j)
				{
					for (int i = 0; i != sz; ++i)
					{
						auto c = Colour32White;
						auto r = Len(i - radius, j - radius);
						auto t = Frac(0.0f, r, radius);
						t = SmoothStep(0.0f, 1.0f, t);
						c.a = uint8_t(Lerp(0xFF, 0x00, t));
						data[size_t(j * sz + i)] = c.argb;
					}
				}

				auto src = Image(sz, sz, data.data(), DXGI_FORMAT_B8G8R8A8_UNORM);
				TextureDesc tdesc = TextureDesc(AutoId, ResDesc::Tex2D(src, 0)).uri(EStockTexture::WhiteSpot).has_alpha().name("#whitespot");
				return CreateTexture2D(tdesc);
			}
			case EStockTexture::WhiteSpike:
			{
				constexpr int sz = 256;
				constexpr auto radius = (sz - 1.0f) / 2.0f;
				std::vector<uint32_t> data(sz * sz);
				for (int j = 0; j != sz; ++j)
				{
					for (int i = 0; i != sz; ++i)
					{
						auto c = Colour32White;
						auto r = Len(i - radius, j - radius);
						auto t = Frac(0.0f, r, radius);
						if (t > 1.0f)
						{
							data[size_t(j * sz + i)] = 0;
						}
						else
						{
							t = Clamp(1.0f - t, 0.0f, 1.0f);
							t = Clamp(Pow(t, 3.0f) + 0.05f, 0.0f, 1.0f);
							c.a = uint8_t(Lerp(0x00, 0xFF, t));
							data[size_t(j * sz + i)] = c.argb;
						}
					}
				}

				auto src = Image(sz, sz, data.data(), DXGI_FORMAT_B8G8R8A8_UNORM);
				TextureDesc tdesc = TextureDesc(AutoId, ResDesc::Tex2D(src, 0)).uri(EStockTexture::WhiteSpike).has_alpha().name("#whitespike");
				return CreateTexture2D(tdesc);
			}
			case EStockTexture::WhiteSphere:
			{
				constexpr int sz = 256;
				constexpr auto radius = (sz - 1.0f) / 2.0f;
				constexpr auto light = v4(1000, 1000, 1000, 1);
				constexpr auto eye = v4(0, 0, 1000, 1);

				std::vector<uint32_t> data(sz * sz);
				for (int j = 0; j != sz; ++j)
				{
					for (int i = 0; i != sz; ++i)
					{
						auto r = Len(i - radius, j - radius);
						if (r > radius)
						{
							// Note: alpha is 0 or 1 => has_alpha = false for thresholding.
							data[size_t(j * sz + i)] = 0x00000000;
							continue;
						}

						// The point on the sphere
						auto pt = v4(i - radius, j - radius, Sqrt(Sqr(radius) - Sqr(r)), 1);
						auto nm = Normalise(pt.w0());

						// Brightness based on the surface orientation compared to the light
						auto b = Dot(nm, Normalise(light - pt));
						b = Clamp(b, 0.05f, 1.0f);
						data[size_t(j * sz + i)] = Colour(b, b, b, 1).argb().argb;
					}
				}

				auto src = Image(sz, sz, data.data(), DXGI_FORMAT_B8G8R8A8_UNORM);
				TextureDesc tdesc = TextureDesc(AutoId, ResDesc::Tex2D(src, 0)).uri(EStockTexture::WhiteSphere).has_alpha(false).name("#whitesphere");
				return CreateTexture2D(tdesc);
			}
			case EStockTexture::WhiteTriangle:
			{
				constexpr int sz = 64, hsz = sz / 2;
				constexpr auto dx = maths::root3_by_2f / 2.0f;
				constexpr auto dy = 0.75f;
				constexpr auto s = 1.0f / sz;

				// Equilateral triangle, 'pointing' up.
				// (-sqrt(3)/2,0.75)------(sqrt(3)/2,0.75)
				//               \         /
				//                \       /
				//                 \     / 
				//                   0,0
				std::vector<uint32_t> data(sz * sz);
				for (int j = 0; j * 4 <= sz * 3; ++j)
				{
					auto y = j * s; // [0, 0.75]

					// Do the positive half x range and mirror to -x
					for (int i = 0; i != hsz; ++i)
					{
						auto x0 = s * (i + 0);
						auto x1 = s * (i + 1);

						// x*dy == y*dx on the edge
						auto t =
							(x1 * dy < y * dx) ? 0.0f :  // inside the triangle
							(x0 * dy > y * dx) ? 1.0f :  // outside the triangle
							(Frac(x0 * dy, y * dx, x1 * 0.75f)); // Spanning the edge

						auto c = Colour32White;
						c.a = uint8_t(Lerp(0xFF, 0x00, SmoothStep(0.0f, 1.0f, t)));

						data[size_t(j * sz + hsz - i)] = c.argb;
						data[size_t(j * sz + hsz + i)] = c.argb;
					}
				}

				auto src = Image(sz, sz, data.data(), DXGI_FORMAT_B8G8R8A8_UNORM);
				TextureDesc tdesc = TextureDesc(AutoId, ResDesc::Tex2D(src, 0)).uri(EStockTexture::WhiteTriangle).has_alpha().name("#whitetriangle");
				return CreateTexture2D(tdesc);
			}
			case EStockTexture::EnvMapProjection:
			{
				uint32_t const data[] = { 0 };
				auto src = Image(1, 1, data, DXGI_FORMAT_B8G8R8A8_UNORM);
				TextureDesc tdesc = TextureDesc(AutoId, ResDesc::Tex2D(src, 0)).uri(EStockTexture::EnvMapProjection).name("#envmapproj");
				return CreateTexture2D(tdesc);
			}
			default:
			{
				throw std::runtime_error("Unknown stock texture");
			}
		}
	}
	
	// Create (or Get) a new sampler instance.
	SamplerPtr ResourceFactory::CreateSampler(SamplerDesc const& desc)
	{
		// Check whether 'id' already exists, if so, return it.
		// There is no per-instance data in samplers, so they can be shared.
		// So really 'CreateSampler' isn't quite right, it's more like 'GetOrCreateSampler'.
		if (desc.m_id != AutoId)
		{
			ResourceStore::Access store(rdr());
			auto res = store.FindSampler(desc.m_id);
			if (res != nullptr)
				return res;
		}

		// Allocate a new sampler instance
		SamplerPtr inst(rdr12::New<Sampler>(rdr(), desc), true);
		assert(rdr().mem_tracker().add(inst.get()));
		m_keep_alive.Add(inst);

		// Add the sampler instance pointer (not ref counted) to the lookup table.
		// The caller owns the sampler, when released it will be removed from this lookup.
		{
			ResourceStore::Access store(rdr());
			store.Add(inst.get());
		}

		return inst;
	}
	SamplerPtr ResourceFactory::CreateSampler(EStockSampler id)
	{
		switch (id)
		{
			case EStockSampler::PointClamp:
			{
				SamplerDesc sdesc = SamplerDesc(EStockSampler::PointClamp, SamDesc::PointClamp()).name("#pointclamp");
				return CreateSampler(sdesc);
			}
			case EStockSampler::PointWrap:
			{
				SamplerDesc sdesc = SamplerDesc(EStockSampler::PointWrap, SamDesc::PointWrap()).name("#pointwrap");
				return CreateSampler(sdesc);
			}
			case EStockSampler::LinearClamp:
			{
				SamplerDesc sdesc = SamplerDesc(EStockSampler::LinearClamp, SamDesc::LinearClamp()).name("#linearclamp");
				return CreateSampler(sdesc);
			}
			case EStockSampler::LinearWrap:
			{
				SamplerDesc sdesc = SamplerDesc(EStockSampler::LinearWrap, SamDesc::LinearWrap()).name("#linearwrap");
				return CreateSampler(sdesc);
			}
			case EStockSampler::AnisotropicClamp:
			{
				SamplerDesc sdesc = SamplerDesc(EStockSampler::AnisotropicClamp, SamDesc::AnisotropicClamp()).name("#anisotropicclamp");
				return CreateSampler(sdesc);
			}
			case EStockSampler::AnisotropicWrap:
			{
				SamplerDesc sdesc = SamplerDesc(EStockSampler::AnisotropicWrap, SamDesc::AnisotropicWrap()).name("#anisotropicwrap");
				return CreateSampler(sdesc);
			}
			default:
			{
				throw std::runtime_error("Unknown stock sampler type");
			}
		}
	}
	
	// Create a texture that references a shared resource
	Texture2DPtr ResourceFactory::OpenSharedTexture2D(HANDLE shared_handle, TextureDesc const& desc)
	{
		// Check whether 'id' already exists, if so, throw.
		if (desc.m_tdesc.DepthOrArraySize != 1)
			throw std::runtime_error("Expected a 2D texture");

		Texture2DPtr inst(rdr12::New<Texture2D>(rdr(), shared_handle, desc), true);
		assert(rdr().mem_tracker().add(inst.get()));

		// Add the texture instance to the lookup table
		{
			ResourceStore::Access store(rdr());
			store.Add(inst.get());
		}

		return inst;
	}
	
	// Create a shader
	ShaderPtr ResourceFactory::CreateShader(EStockShader id, char const* config)
	{
		// Parse the config string
		mem_istream<char> src(config);
		rdr12::ldraw::TextReader reader(src, {});

		switch (id)
		{
			// Radial fade params:
			//  *Type {Spherical|Cylindrical}
			//  *Radius {min,max}
			//  *Centre {x,y,z} (optional, defaults to camera position)
			//  *Absolute (optional, default false) - True if 'radius' is absolute, false if 'radius' should be scaled by the focus distance
			case EStockShader::FwdRadialFadePS:
			{
				// todo
				throw std::runtime_error("Not implemented");
			}

			// Point sprite params: *PointSize {w,h} *Depth {true|false}
			case EStockShader::PointSpritesGS:
			{
				v2 radius = { 1,1 };
				bool depth = false;
				for (int kw; reader.NextKeyword(kw);) switch (kw)
				{
					case rdr12::ldraw::HashI("PointSize"):
						radius = reader.Vector2f();
						break;
					case rdr12::ldraw::HashI("Depth"):
						depth = reader.Bool();
						break;
				}
				return Shader::Create<shaders::PointSpriteGS>(rdr(), radius, depth);
			}

			// Thick line params: *LineWidth {width}
			case EStockShader::ThickLineListGS:
			{
				auto line_width = 0.0f;
				for (int kw; reader.NextKeyword(kw);) switch (kw)
				{
					case rdr12::ldraw::HashI("LineWidth"):
						line_width = reader.Real<float>();
						break;
				}
				return Shader::Create<shaders::ThickLineListGS>(rdr(), line_width);
			}

			// Thick line params: *LineWidth {width}
			case EStockShader::ThickLineStripGS:
			{
				auto line_width = 0.0f;
				for (int kw; reader.NextKeyword(kw);) switch (kw)
				{
					case rdr12::ldraw::HashI("LineWidth"):
						line_width = reader.Real<float>();
						break;
				}
				return Shader::Create<shaders::ThickLineStripGS>(rdr(), line_width);
			}

			// Arrow params: *Size {size} *Depth {true|false}
			case EStockShader::ArrowHeadGS:
			{
				v2 size = { 1,1 };
				bool depth = false;
				for (int kw; reader.NextKeyword(kw);) switch (kw)
				{
					case rdr12::ldraw::HashI("Size"):
						size.x = reader.IsSectionEnd() ? 1 : reader.Real<float>();
						size.y = reader.IsSectionEnd() ? size.y : reader.Real<float>();
						break;
					case rdr12::ldraw::HashI("Depth"):
						depth = reader.IsSectionEnd() ? true : reader.Bool();
						break;
				}
				return Shader::Create<shaders::ArrowHeadGS>(rdr(), size, depth);
			}

			default:
			{
				throw std::runtime_error("Unsupported stock shader type");
			}
		}
	}
}
