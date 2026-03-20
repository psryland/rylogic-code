//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/utility/utility.h"
#include "pr/view3d-12/utility/wrappers.h"
#include "pr/view3d-12/utility/map_resource.h"
#include "pr/view3d-12/utility/cmd_list.h"
#include "pr/view3d-12/utility/pipe_state.h"
#include "pr/view3d-12/main/renderer.h"

namespace pr::rdr12
{
	// Helper for getting the ref count of a COM pointer.
	ULONG RefCount(IUnknown* ptr)
	{
		// Don't inline this function so that it can be used in the Immediate window during debugging
		if (ptr == nullptr) return 0;
		auto count = ptr->AddRef();
		ptr->Release();
		return count - 1;
	}

	// The number of supported quality levels for the given format and sample count
	UINT MultisampleQualityLevels(ID3D12Device* device, DXGI_FORMAT format, UINT sample_count)
	{
		D3D12_FEATURE_DATA_MULTISAMPLE_QUALITY_LEVELS opts = {format, sample_count, D3D12_MULTISAMPLE_QUALITY_LEVELS_FLAG_NONE};
		auto hr = device->CheckFeatureSupport(D3D12_FEATURE_MULTISAMPLE_QUALITY_LEVELS, &opts, sizeof(opts));
		if (hr == E_INVALIDARG)
			return 0;

		Check(hr);
		return opts.NumQualityLevels;
	}

	// Returns the number of primitives implied by an index count and geometry topology
	int64_t PrimCount(int64_t icount, ETopo topo)
	{
		// https://docs.microsoft.com/en-us/windows/win32/direct3d11/d3d10-graphics-programming-guide-primitive-topologies
		switch (topo)
		{
			case ETopo::PointList: return icount;
			case ETopo::LineList:    PR_ASSERT(PR_DBG_RDR, (icount % 2) == 0, "Incomplete primitive implied by i-count"); return icount / 2;
			case ETopo::LineStrip:   PR_ASSERT(PR_DBG_RDR, icount >= 2, "Incomplete primitive implied by i-count"); return icount - 1;
			case ETopo::TriList:     PR_ASSERT(PR_DBG_RDR, (icount % 3) == 0, "Incomplete primitive implied by i-count"); return icount / 3;
			case ETopo::TriStrip:    PR_ASSERT(PR_DBG_RDR, icount >= 3, "Incomplete primitive implied by i-count"); return icount - 2;
			case ETopo::LineListAdj: PR_ASSERT(PR_DBG_RDR, (icount % 4) == 0, "Incomplete primitive implied by i-count"); return icount / 4;
			case ETopo::LineStripAdj:PR_ASSERT(PR_DBG_RDR, icount >= 4, "Incomplete primitive implied by i-count"); return (icount - 2) - 1;
			case ETopo::TriListAdj:  PR_ASSERT(PR_DBG_RDR, (icount % 6) == 0, "Incomplete primitive implied by i-count"); return icount / 6;
			case ETopo::TriStripAdj: PR_ASSERT(PR_DBG_RDR, icount >= 3, "Incomplete primitive implied by i-count"); return (icount - 4) / 2;
			default: throw std::runtime_error("Unknown primitive type");
		}
	}

	// Returns the number of indices implied by a primitive count and geometry topology
	int64_t IndexCount(int64_t pcount, ETopo topo)
	{
		if (pcount == 0) return 0;
		switch (topo)
		{
			case ETopo::PointList:   return pcount;
			case ETopo::LineList:    return pcount * 2;
			case ETopo::LineStrip:   return pcount + 1;
			case ETopo::TriList:     return pcount * 3;
			case ETopo::TriStrip:    return pcount + 2;
			case ETopo::LineListAdj: return pcount * 4;
			case ETopo::LineStripAdj:return (pcount + 1) + 2;
			case ETopo::TriListAdj:  return pcount * 6;
			case ETopo::TriStripAdj: return (pcount * 2) + 4;
			default: throw std::runtime_error("Unknown primitive type");
		}
	}

	// True if 'fmt' is a compression image format
	bool IsCompressed(DXGI_FORMAT fmt)
	{
		switch (fmt)
		{
			case DXGI_FORMAT_BC1_TYPELESS:
			case DXGI_FORMAT_BC1_UNORM:
			case DXGI_FORMAT_BC1_UNORM_SRGB:
			case DXGI_FORMAT_BC2_TYPELESS:
			case DXGI_FORMAT_BC2_UNORM:
			case DXGI_FORMAT_BC2_UNORM_SRGB:
			case DXGI_FORMAT_BC3_TYPELESS:
			case DXGI_FORMAT_BC3_UNORM:
			case DXGI_FORMAT_BC3_UNORM_SRGB:
			case DXGI_FORMAT_BC4_TYPELESS:
			case DXGI_FORMAT_BC4_UNORM:
			case DXGI_FORMAT_BC4_SNORM:
			case DXGI_FORMAT_BC5_TYPELESS:
			case DXGI_FORMAT_BC5_UNORM:
			case DXGI_FORMAT_BC5_SNORM:
			case DXGI_FORMAT_BC6H_TYPELESS:
			case DXGI_FORMAT_BC6H_UF16:
			case DXGI_FORMAT_BC6H_SF16:
			case DXGI_FORMAT_BC7_TYPELESS:
			case DXGI_FORMAT_BC7_UNORM:
			case DXGI_FORMAT_BC7_UNORM_SRGB:
				return true;
			default:
				return false;
		}
	}

	// True if 'fmt' has an alpha channel
	bool HasAlphaChannel(DXGI_FORMAT fmt)
	{
		switch (fmt)
		{
			case DXGI_FORMAT_R32G32B32A32_TYPELESS:
			case DXGI_FORMAT_R32G32B32A32_FLOAT:
			case DXGI_FORMAT_R32G32B32A32_UINT:
			case DXGI_FORMAT_R32G32B32A32_SINT:
			case DXGI_FORMAT_R16G16B16A16_TYPELESS:
			case DXGI_FORMAT_R16G16B16A16_FLOAT:
			case DXGI_FORMAT_R16G16B16A16_UNORM:
			case DXGI_FORMAT_R16G16B16A16_UINT:
			case DXGI_FORMAT_R16G16B16A16_SNORM:
			case DXGI_FORMAT_R16G16B16A16_SINT:
			case DXGI_FORMAT_R10G10B10A2_TYPELESS:
			case DXGI_FORMAT_R10G10B10A2_UNORM:
			case DXGI_FORMAT_R10G10B10A2_UINT:
			case DXGI_FORMAT_R8G8B8A8_TYPELESS:
			case DXGI_FORMAT_R8G8B8A8_UNORM:
			case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
			case DXGI_FORMAT_R8G8B8A8_UINT:
			case DXGI_FORMAT_R8G8B8A8_SNORM:
			case DXGI_FORMAT_R8G8B8A8_SINT:
			case DXGI_FORMAT_BC1_TYPELESS:
			case DXGI_FORMAT_BC1_UNORM:
			case DXGI_FORMAT_BC1_UNORM_SRGB:
			case DXGI_FORMAT_BC2_TYPELESS:
			case DXGI_FORMAT_BC2_UNORM:
			case DXGI_FORMAT_BC2_UNORM_SRGB:
			case DXGI_FORMAT_BC3_TYPELESS:
			case DXGI_FORMAT_BC3_UNORM:
			case DXGI_FORMAT_BC3_UNORM_SRGB:
			case DXGI_FORMAT_B5G5R5A1_UNORM:
			case DXGI_FORMAT_B8G8R8A8_UNORM:
			case DXGI_FORMAT_B8G8R8X8_UNORM:
			case DXGI_FORMAT_R10G10B10_XR_BIAS_A2_UNORM:
			case DXGI_FORMAT_B8G8R8A8_TYPELESS:
			case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
			case DXGI_FORMAT_B8G8R8X8_TYPELESS:
			case DXGI_FORMAT_B8G8R8X8_UNORM_SRGB:
			case DXGI_FORMAT_BC6H_TYPELESS:
			case DXGI_FORMAT_BC7_TYPELESS:
			case DXGI_FORMAT_BC7_UNORM:
			case DXGI_FORMAT_BC7_UNORM_SRGB:
			case DXGI_FORMAT_A8P8:
			case DXGI_FORMAT_B4G4R4A4_UNORM:
				return true;
			default:
				return false;
		}
	}

	// True if 'fmt' is compatible with UA views
	bool IsUAVCompatible(DXGI_FORMAT fmt)
	{
		switch (fmt)
		{
			case DXGI_FORMAT_R32G32B32A32_FLOAT:
			case DXGI_FORMAT_R32G32B32A32_UINT:
			case DXGI_FORMAT_R32G32B32A32_SINT:
			case DXGI_FORMAT_R16G16B16A16_FLOAT:
			case DXGI_FORMAT_R16G16B16A16_UINT:
			case DXGI_FORMAT_R16G16B16A16_SINT:
			case DXGI_FORMAT_R8G8B8A8_UNORM:
			case DXGI_FORMAT_R8G8B8A8_UINT:
			case DXGI_FORMAT_R8G8B8A8_SINT:
			case DXGI_FORMAT_R32_FLOAT:
			case DXGI_FORMAT_R32_UINT:
			case DXGI_FORMAT_R32_SINT:
			case DXGI_FORMAT_R16_FLOAT:
			case DXGI_FORMAT_R16_UINT:
			case DXGI_FORMAT_R16_SINT:
			case DXGI_FORMAT_R8_UNORM:
			case DXGI_FORMAT_R8_UINT:
			case DXGI_FORMAT_R8_SINT:
				return true;
			default:
				return false;
		}
	}

	// True if 'fmt' is an SRGB format
	bool IsSRGB(DXGI_FORMAT fmt)
	{
		switch (fmt)
		{
			case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
			case DXGI_FORMAT_BC1_UNORM_SRGB:
			case DXGI_FORMAT_BC2_UNORM_SRGB:
			case DXGI_FORMAT_BC3_UNORM_SRGB:
			case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
			case DXGI_FORMAT_B8G8R8X8_UNORM_SRGB:
			case DXGI_FORMAT_BC7_UNORM_SRGB:
				return true;
			default:
				return false;
		}
	}

	// True if 'fmt' is a BGR format
	bool IsBGRFormat(DXGI_FORMAT fmt)
	{
		switch (fmt)
		{
			case DXGI_FORMAT_B8G8R8A8_UNORM:
			case DXGI_FORMAT_B8G8R8X8_UNORM:
			case DXGI_FORMAT_B8G8R8A8_TYPELESS:
			case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
			case DXGI_FORMAT_B8G8R8X8_TYPELESS:
			case DXGI_FORMAT_B8G8R8X8_UNORM_SRGB:
				return true;
			default:
				return false;
		}
	}

	// True if 'fmt' is a depth format
	bool IsDepth(DXGI_FORMAT fmt)
	{
		switch (fmt)
		{
			case DXGI_FORMAT_D32_FLOAT_S8X24_UINT:
			case DXGI_FORMAT_D32_FLOAT:
			case DXGI_FORMAT_D24_UNORM_S8_UINT:
			case DXGI_FORMAT_D16_UNORM:
				return true;
			default:
				return false;
		}
	}

	// Convert 'fmt' to a typeless format
	DXGI_FORMAT ToTypeless(DXGI_FORMAT fmt)
	{
		switch (fmt)
		{
			case DXGI_FORMAT_R32G32B32A32_FLOAT:
			case DXGI_FORMAT_R32G32B32A32_UINT:
			case DXGI_FORMAT_R32G32B32A32_SINT:
				return DXGI_FORMAT_R32G32B32A32_TYPELESS;
			case DXGI_FORMAT_R32G32B32_FLOAT:
			case DXGI_FORMAT_R32G32B32_UINT:
			case DXGI_FORMAT_R32G32B32_SINT:
				return DXGI_FORMAT_R32G32B32_TYPELESS;
			case DXGI_FORMAT_R16G16B16A16_FLOAT:
			case DXGI_FORMAT_R16G16B16A16_UNORM:
			case DXGI_FORMAT_R16G16B16A16_UINT:
			case DXGI_FORMAT_R16G16B16A16_SNORM:
			case DXGI_FORMAT_R16G16B16A16_SINT:
				return DXGI_FORMAT_R16G16B16A16_TYPELESS;
			case DXGI_FORMAT_R32G32_FLOAT:
			case DXGI_FORMAT_R32G32_UINT:
			case DXGI_FORMAT_R32G32_SINT:
				return DXGI_FORMAT_R32G32_TYPELESS;
			case DXGI_FORMAT_D32_FLOAT_S8X24_UINT:
				return DXGI_FORMAT_R32G8X24_TYPELESS;
			case DXGI_FORMAT_R10G10B10A2_UNORM:
			case DXGI_FORMAT_R10G10B10A2_UINT:
				return DXGI_FORMAT_R10G10B10A2_TYPELESS;
			case DXGI_FORMAT_R8G8B8A8_UNORM:
			case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
			case DXGI_FORMAT_R8G8B8A8_UINT:
			case DXGI_FORMAT_R8G8B8A8_SNORM:
			case DXGI_FORMAT_R8G8B8A8_SINT:
				return DXGI_FORMAT_R8G8B8A8_TYPELESS;
			case DXGI_FORMAT_R16G16_FLOAT:
			case DXGI_FORMAT_R16G16_UNORM:
			case DXGI_FORMAT_R16G16_UINT:
			case DXGI_FORMAT_R16G16_SNORM:
			case DXGI_FORMAT_R16G16_SINT:
				return DXGI_FORMAT_R16G16_TYPELESS;
			case DXGI_FORMAT_D32_FLOAT:
			case DXGI_FORMAT_R32_FLOAT:
			case DXGI_FORMAT_R32_UINT:
			case DXGI_FORMAT_R32_SINT:
				return DXGI_FORMAT_R32_TYPELESS;
			case DXGI_FORMAT_R8G8_UNORM:
			case DXGI_FORMAT_R8G8_UINT:
			case DXGI_FORMAT_R8G8_SNORM:
			case DXGI_FORMAT_R8G8_SINT:
				return DXGI_FORMAT_R8G8_TYPELESS;
			case DXGI_FORMAT_R16_FLOAT:
			case DXGI_FORMAT_D16_UNORM:
			case DXGI_FORMAT_R16_UNORM:
			case DXGI_FORMAT_R16_UINT:
			case DXGI_FORMAT_R16_SNORM:
			case DXGI_FORMAT_R16_SINT:
				return DXGI_FORMAT_R16_TYPELESS;
			case DXGI_FORMAT_R8_UNORM:
			case DXGI_FORMAT_R8_UINT:
			case DXGI_FORMAT_R8_SNORM:
			case DXGI_FORMAT_R8_SINT:
				return DXGI_FORMAT_R8_TYPELESS;
			case DXGI_FORMAT_BC1_UNORM:
			case DXGI_FORMAT_BC1_UNORM_SRGB:
				return DXGI_FORMAT_BC1_TYPELESS;
			case DXGI_FORMAT_BC2_UNORM:
			case DXGI_FORMAT_BC2_UNORM_SRGB:
				return DXGI_FORMAT_BC2_TYPELESS;
			case DXGI_FORMAT_BC3_UNORM:
			case DXGI_FORMAT_BC3_UNORM_SRGB:
				return DXGI_FORMAT_BC3_TYPELESS;
			case DXGI_FORMAT_BC4_UNORM:
			case DXGI_FORMAT_BC4_SNORM:
				return DXGI_FORMAT_BC4_TYPELESS;
			case DXGI_FORMAT_BC5_UNORM:
			case DXGI_FORMAT_BC5_SNORM:
				return DXGI_FORMAT_BC5_TYPELESS;
			case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
				return DXGI_FORMAT_B8G8R8A8_TYPELESS;
			case DXGI_FORMAT_B8G8R8X8_UNORM_SRGB:
				return DXGI_FORMAT_B8G8R8X8_TYPELESS;
			case DXGI_FORMAT_BC6H_UF16:
			case DXGI_FORMAT_BC6H_SF16:
				return DXGI_FORMAT_BC6H_TYPELESS;
			case DXGI_FORMAT_BC7_UNORM:
			case DXGI_FORMAT_BC7_UNORM_SRGB:
				return DXGI_FORMAT_BC7_TYPELESS;
			default:
				return fmt;
		}
	}

	// Convert 'fmt' to a SRGB format
	DXGI_FORMAT ToSRGB(DXGI_FORMAT fmt)
	{
		switch (fmt)
		{
			case DXGI_FORMAT_R8G8B8A8_TYPELESS:
			case DXGI_FORMAT_R8G8B8A8_UNORM:
				return DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
			case DXGI_FORMAT_BC1_TYPELESS:
			case DXGI_FORMAT_BC1_UNORM:
				return DXGI_FORMAT_BC1_UNORM_SRGB;
			case DXGI_FORMAT_BC2_TYPELESS:
			case DXGI_FORMAT_BC2_UNORM:
				return DXGI_FORMAT_BC2_UNORM_SRGB;
			case DXGI_FORMAT_BC3_TYPELESS:
			case DXGI_FORMAT_BC3_UNORM:
				return DXGI_FORMAT_BC3_UNORM_SRGB;
			case DXGI_FORMAT_B8G8R8A8_TYPELESS:
			case DXGI_FORMAT_B8G8R8A8_UNORM:
				return DXGI_FORMAT_B8G8R8A8_UNORM_SRGB;
			case DXGI_FORMAT_B8G8R8X8_TYPELESS:
			case DXGI_FORMAT_B8G8R8X8_UNORM:
				return DXGI_FORMAT_B8G8R8X8_UNORM_SRGB;
			case DXGI_FORMAT_BC7_TYPELESS:
			case DXGI_FORMAT_BC7_UNORM:
				return DXGI_FORMAT_BC7_UNORM_SRGB;
			default:
				return fmt;
		}
	}

	// Convert 'fmt' to a UAV compatible format
	DXGI_FORMAT ToUAVCompatable(DXGI_FORMAT fmt)
	{
		switch (fmt)
		{
			case DXGI_FORMAT_R8G8B8A8_TYPELESS:
			case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
			case DXGI_FORMAT_B8G8R8A8_UNORM:
			case DXGI_FORMAT_B8G8R8X8_UNORM:
			case DXGI_FORMAT_B8G8R8A8_TYPELESS:
			case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
			case DXGI_FORMAT_B8G8R8X8_TYPELESS:
			case DXGI_FORMAT_B8G8R8X8_UNORM_SRGB:
				return DXGI_FORMAT_R8G8B8A8_UNORM;
			case DXGI_FORMAT_R32_TYPELESS:
			case DXGI_FORMAT_D32_FLOAT:
				return DXGI_FORMAT_R32_FLOAT;
			default:
				return fmt;
		}
	}

	// Returns the expected row pitch for a given image width and format
	iv3 Pitch(iv3 size, DXGI_FORMAT fmt)
	{
		// x = row pitch = number of bytes per row
		// y = slice pitch = number of bytes per 2D image.
		// z = block pitch = number of bytes per 3D image.
		auto width = size.x;
		auto height = size.y;
		auto depth = size.z;

		auto bc = false;
		auto packed = false;
		auto num_bytes_per_block = 0;
		switch (fmt)
		{
			case DXGI_FORMAT_BC1_TYPELESS:
			case DXGI_FORMAT_BC1_UNORM:
			case DXGI_FORMAT_BC1_UNORM_SRGB:
			case DXGI_FORMAT_BC4_TYPELESS:
			case DXGI_FORMAT_BC4_UNORM:
			case DXGI_FORMAT_BC4_SNORM:
			{
				bc = true;
				num_bytes_per_block = 8;
				break;
			}
			case DXGI_FORMAT_BC2_TYPELESS:
			case DXGI_FORMAT_BC2_UNORM:
			case DXGI_FORMAT_BC2_UNORM_SRGB:
			case DXGI_FORMAT_BC3_TYPELESS:
			case DXGI_FORMAT_BC3_UNORM:
			case DXGI_FORMAT_BC3_UNORM_SRGB:
			case DXGI_FORMAT_BC5_TYPELESS:
			case DXGI_FORMAT_BC5_UNORM:
			case DXGI_FORMAT_BC5_SNORM:
			case DXGI_FORMAT_BC6H_TYPELESS:
			case DXGI_FORMAT_BC6H_UF16:
			case DXGI_FORMAT_BC6H_SF16:
			case DXGI_FORMAT_BC7_TYPELESS:
			case DXGI_FORMAT_BC7_UNORM:
			case DXGI_FORMAT_BC7_UNORM_SRGB:
			{
				bc = true;
				num_bytes_per_block = 16;
				break;
			}
			case DXGI_FORMAT_R8G8_B8G8_UNORM:
			case DXGI_FORMAT_G8R8_G8B8_UNORM:
			{
				packed = true;
				break;
			}
		}

		auto num_rows = 0;
		auto row_bytes = 0;
		if (bc)
		{
			auto blocks_wide = width > 0 ? std::max<int>(1, (width + 3) / 4) : 0;
			auto blocks_high = height > 0 ? std::max<int>(1, (height + 3) / 4) : 0;
			row_bytes = blocks_wide * num_bytes_per_block;
			num_rows = blocks_high;
		}
		else if (packed)
		{
			row_bytes = ((width + 1) >> 1) * 4;
			num_rows = height;
		}
		else
		{
			auto bpp = BitsPerPixel(fmt);
			row_bytes = (width * bpp + 7) / 8; // round up to nearest byte
			num_rows = height;
		}
		return iv3(row_bytes, row_bytes * num_rows, row_bytes * num_rows * depth);
	}
	iv2 Pitch(iv2 size, DXGI_FORMAT fmt)
	{
		return Pitch(iv3(size.x, size.y, 1), fmt).xy;
	}
	iv2 Pitch(D3D12_RESOURCE_DESC const& desc)
	{
		return Pitch(iv2(s_cast<int>(desc.Width), s_cast<int>(desc.Height)), desc.Format);
	}

	// Returns the number of expected mip levels for a given width x height texture
	int MipCount(int w, int h)
	{
		int count, largest = std::max(w, h);
		for (count = 1; largest >>= 1; ++count) {}
		return count;
	}
	int MipCount(iv2 size)
	{
		return MipCount(size.x, size.y);
	}

	// Returns the dimensions of a mip level 'levels' lower than the given size
	iv2 MipDimensions(iv2 size, size_t levels)
	{
		PR_ASSERT(PR_DBG_RDR, levels > 0, "A specific mip level must be given");
		PR_ASSERT(PR_DBG_RDR, s_cast<int>(levels) <= MipCount(size), "The number of mip levels provided exceeds the expected number for this texture dimension");
		for (;levels-- != 0;)
		{
			size.x = std::max(size.x/2, 1);
			size.y = std::max(size.y/2, 1);
		}
		return size;
	}

	// Returns the number of pixels needed to contain the data for a mip chain with 'levels' levels
	// If 'levels' is 0, all mips down to 1x1 are assumed
	// Note, size.x should be the pitch rather than width of the texture
	size_t MipChainSize(iv2 size, size_t levels)
	{
		PR_ASSERT(PR_DBG_RDR, s_cast<int>(levels) <= MipCount(size), "Number of mip levels provided exceeds the expected number for this texture dimension");

		if (levels == 0)
			levels = MipCount(size);

		size_t pixel_count = 0;
		for (;levels-- != 0;)
		{
			pixel_count += size.x * size.y;
			size = MipDimensions(size, 1);
		}
		return pixel_count;
	}

	// Get/Set the default state for a resource
	D3D12_RESOURCE_STATES DefaultResState(ID3D12Resource const* res)
	{
		#if PR_DBG
		auto name = DebugName(res);
		(void)name;
		#endif

		UINT size(sizeof(D3D12_RESOURCE_STATES));
		char bytes[sizeof(D3D12_RESOURCE_STATES)];
		auto hr = const_cast<ID3D12Resource*>(res)->GetPrivateData(Guid_DefaultResourceState, &size, &bytes[0]);
		return hr != DXGI_ERROR_NOT_FOUND
			? *reinterpret_cast<D3D12_RESOURCE_STATES*>(&bytes[0])
			: D3D12_RESOURCE_STATE_COMMON;
	}
	void DefaultResState(ID3D12Resource* res, D3D12_RESOURCE_STATES state)
	{
		// Assume 'Common' state and don't store it
		if (state == D3D12_RESOURCE_STATE_COMMON) return;
		Check(res->SetPrivateData(Guid_DefaultResourceState, s_cast<UINT>(sizeof(D3D12_RESOURCE_STATES)), &state));
	}

	// Parse an embedded resource string of the form: "@<hmodule|module_name>:<res_type>:<res_name>"
	void ParseEmbeddedResourceUri(std::wstring const& uri, HMODULE& hmodule, wstring32& res_type, wstring32& res_name)
	{
		if (uri.empty() || uri[0] != '@')
			throw std::runtime_error("Not an embedded resource URI");

		hmodule = nullptr;
		res_type.resize(0);
		res_name.resize(0);

		auto div0 = uri.c_str();
		auto div1 = *div0 != 0 ? str::FindChar(div0 + 1, ':') : div0;
		auto div2 = *div1 != 0 ? str::FindChar(div1 + 1, ':') : div1;
		if (*div2 == 0)
			throw std::runtime_error(FmtS("Embedded resource URI (%S) invalid. Expected format \"@<hmodule|module_name>:<res_type>:<res_name>\"", uri.c_str()));
		
		// Read the HMODULE handle from a string name or 
		auto HModule = [=](wchar_t const* s, wchar_t const* e)
		{
			wstring32 name(s, e);
			if (name.empty())
				return HMODULE();

			if (auto h = GetModuleHandleW(name.c_str()); h != nullptr)
				return h;

			auto end = (wchar_t*)nullptr;
			auto address = std::wcstoll(s, &end, 16);
			if (auto h = reinterpret_cast<HMODULE>((uint8_t*)nullptr + (end == e ? address : 0)); h != nullptr)
				return h;

			throw std::runtime_error(FmtS("Embedded resource URI (%S) not found. HMODULE could not be determined", uri.c_str()));
		};

		res_name.append(div2 + 1);
		res_type.append(div1 + 1, div2);
		hmodule = HModule(div0 + 1, div1);

		// Both name and type are required
		if (res_name.empty() || res_type.empty())
			throw std::runtime_error(FmtS("Embedded resource URI (%S) not found. Resource name and type could not be determined", uri.c_str()));
	}

	// Return an ordered list of filepaths based on 'pattern'
	vector<std::filesystem::path> PatternToPaths(std::filesystem::path const& dir, char8_t const* pattern)
	{
		using namespace std::filesystem;

		vector<path> paths;

		// Assume the pattern is in the filename only
		auto pat = std::regex(char_ptr(pattern), std::regex::flag_type::icase);
		for (auto& entry : directory_iterator(dir))
		{
			if (!std::regex_match(entry.path().filename().string(), pat)) continue;
			paths.push_back(entry.path());
		}

		// Sort the paths lexically
		pr::sort(paths);
		return std::move(paths);
	}
}
