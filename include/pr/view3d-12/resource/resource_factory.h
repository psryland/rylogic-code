//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/resource/stock_resources.h"
#include "pr/view3d-12/resource/gpu_transfer_buffer.h"
#include "pr/view3d-12/resource/mipmap_generator.h"
#include "pr/view3d-12/shaders/stock_shaders.h"
#include "pr/view3d-12/utility/gpu_sync.h"
#include "pr/view3d-12/utility/cmd_alloc.h"
#include "pr/view3d-12/utility/cmd_list.h"
#include "pr/view3d-12/utility/keep_alive.h"

namespace pr::rdr12
{
	struct ResourceFactory
	{
		// Notes:
		// - The resource factory is an instance-able object used to create resources.
		// - Multiple resource factories can exist at any one time.
		// - The resource factory is expected to be used on one thread only.

	private:

		Renderer&       m_rdr;                // The owning renderer instance
		GpuSync         m_gsync;              // Sync with GPU
		KeepAlive       m_keep_alive;         // Keep alive for the resource manager
		GfxCmdAllocPool m_gfx_cmd_alloc_pool; // A pool of command allocators.
		GfxCmdList      m_gfx_cmd_list;       // Command list for resource manager operations.
		GpuUploadBuffer m_upload_buffer;      // Upload memory buffer for initialising resources
		MipMapGenerator m_mipmap_gen;         // Utility class for generating mip maps for a texture
		bool            m_flush_required;     // True if commands have been added to the command list and need sending to the GPU

	public:

		ResourceFactory(Renderer& rdr);
		ResourceFactory(ResourceFactory const&) = delete;
		ResourceFactory& operator = (ResourceFactory const&) = delete;
		~ResourceFactory();

		// Renderer access
		ID3D12Device4* d3d() const;
		Renderer& rdr() const;

		// Access the command list associated with this factory instance
		GfxCmdList& CmdList();

		// Access the upload buffer associated with this factory instance
		GpuUploadBuffer& UploadBuffer();

		// Flush creation commands to the GPU. Returns the sync point for when they've been executed
		uint64_t FlushToGpu(EGpuFlush flush);

		// Wait for the GPU to finish processing the internal command list
		void Wait(uint64_t sync_point) const;
		
		// Create and initialise a resource
		D3DPtr<ID3D12Resource> CreateResource(ResDesc const& desc, std::string_view name);

		// Create a model.
		ModelPtr CreateModel(ModelDesc const& mdesc, D3DPtr<ID3D12Resource> vb, D3DPtr<ID3D12Resource> ib);
		ModelPtr CreateModel(ModelDesc const& desc);
		ModelPtr CreateModel(EStockModel id);

		// Create a new nugget
		NuggetPtr CreateNugget(NuggetDesc const& ndata, Model* model);

		// Create a new texture instance.
		Texture2DPtr CreateTexture2D(TextureDesc const& desc);
		Texture2DPtr CreateTexture2D(std::filesystem::path const& resource_path, TextureDesc const& desc);
		TextureCubePtr CreateTextureCube(std::filesystem::path const& resource_path, TextureDesc const& desc);
		Texture2DPtr CreateTexture(EStockTexture id);

		// Create (or Get) a new sampler instance.
		SamplerPtr CreateSampler(SamplerDesc const& desc);
		SamplerPtr CreateSampler(EStockSampler id);

		// Create a texture that references a shared resource
		Texture2DPtr OpenSharedTexture2D(HANDLE shared_handle, TextureDesc const& desc);

		// Create a shader
		ShaderPtr CreateShader(EStockShader id, char const* config);
	};
}
