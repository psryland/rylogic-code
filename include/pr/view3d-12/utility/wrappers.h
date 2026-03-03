//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/resource/image.h"
#include "pr/view3d-12/utility/utility.h"
#include "pr/view3d-12/shaders/shader_registers.h"

namespace pr::rdr12
{
	// Descriptor types
	enum class EDescriptorType
	{
		CBV = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV,
		SRV = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV,
		UAV = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV,
		Sampler = D3D12_DESCRIPTOR_HEAP_TYPE_SAMPLER,
		RTV = D3D12_DESCRIPTOR_HEAP_TYPE_RTV,
		DSV = D3D12_DESCRIPTOR_HEAP_TYPE_DSV,
	};

	// Resource usage flags
	enum class EUsage :std::underlying_type_t<D3D12_RESOURCE_FLAGS>
	{
		Default = D3D12_RESOURCE_FLAG_NONE,
		RenderTarget = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET,
		DepthStencil = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL,
		UnorderedAccess = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
		DenyShaderResource = D3D12_RESOURCE_FLAG_DENY_SHADER_RESOURCE,
		CrossAdapter = D3D12_RESOURCE_FLAG_ALLOW_CROSS_ADAPTER,
		SimultaneousAccess = D3D12_RESOURCE_FLAG_ALLOW_SIMULTANEOUS_ACCESS,
		VideoDecodeRefOnly = D3D12_RESOURCE_FLAG_VIDEO_DECODE_REFERENCE_ONLY,
		VideoEncodeRefOnly = D3D12_RESOURCE_FLAG_VIDEO_ENCODE_REFERENCE_ONLY,
		_flags_enum = 0,
	};

	// Root signature flags
	enum class ERootSigFlags :std::underlying_type_t<D3D12_ROOT_SIGNATURE_FLAGS>
	{
		None                              = D3D12_ROOT_SIGNATURE_FLAG_NONE,
		AllowInputAssemblerInputLayout    = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT,
		DenyVertexShaderRootAccess        = D3D12_ROOT_SIGNATURE_FLAG_DENY_VERTEX_SHADER_ROOT_ACCESS,
		DenyHullShaderRootAccess          = D3D12_ROOT_SIGNATURE_FLAG_DENY_HULL_SHADER_ROOT_ACCESS,
		DenyDomainShaderRootAccess        = D3D12_ROOT_SIGNATURE_FLAG_DENY_DOMAIN_SHADER_ROOT_ACCESS,
		DenyGeometryShaderRootAccess      = D3D12_ROOT_SIGNATURE_FLAG_DENY_GEOMETRY_SHADER_ROOT_ACCESS,
		DenyPixelShaderRootAccess         = D3D12_ROOT_SIGNATURE_FLAG_DENY_PIXEL_SHADER_ROOT_ACCESS,
		AllowStreamOutput                 = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_STREAM_OUTPUT,
		LocalRootSignature                = D3D12_ROOT_SIGNATURE_FLAG_LOCAL_ROOT_SIGNATURE,
		DenyAmplificationShaderRootAccess = D3D12_ROOT_SIGNATURE_FLAG_DENY_AMPLIFICATION_SHADER_ROOT_ACCESS,
		DenyMeshShaderRootAccess          = D3D12_ROOT_SIGNATURE_FLAG_DENY_MESH_SHADER_ROOT_ACCESS,
		CbvSrvUavHeapDirectlyIndexed      = D3D12_ROOT_SIGNATURE_FLAG_CBV_SRV_UAV_HEAP_DIRECTLY_INDEXED,
		SamplerHeapDirectlyIndexed        = D3D12_ROOT_SIGNATURE_FLAG_SAMPLER_HEAP_DIRECTLY_INDEXED,

		GraphicsOnly =
			AllowInputAssemblerInputLayout    |
			DenyAmplificationShaderRootAccess |
			DenyMeshShaderRootAccess          |
			None,

		ComputeOnly =
			DenyVertexShaderRootAccess        |
			DenyHullShaderRootAccess          |
			DenyDomainShaderRootAccess        |
			DenyGeometryShaderRootAccess      |
			DenyPixelShaderRootAccess         |
			DenyAmplificationShaderRootAccess |
			DenyMeshShaderRootAccess          |
			None,

		VertGeomPixelOnly =
			AllowInputAssemblerInputLayout    |
			DenyHullShaderRootAccess          |
			DenyDomainShaderRootAccess        |
			AllowStreamOutput                 |
			DenyAmplificationShaderRootAccess |
			DenyMeshShaderRootAccess          |
			None,

		_flags_enum = 0,
	};

	// 32bit data union
	union F32U32
	{
		float f32;
		uint32_t u32;

		F32U32(float f) : f32(f) {}
		F32U32(uint32_t u) :u32(u) {}
	};

	// 64bit data union
	union F64U64
	{
		double f64;
		uint64_t u64;

		F64U64(double f) : f64(f) {}
		F64U64(uint64_t u) :u64(u) {}
	};

	// Bit stuff a size and alignment value
	template <std::unsigned_integral U, int AlignBits, int SizeBits>
	struct SizeAndAlign
	{
		enum
		{
			size_bits_t = SizeBits,
			align_bits_t = AlignBits
		};
		static_assert(size_bits_t + align_bits_t <= 8 * sizeof(U));

		U sa;
		SizeAndAlign() = default;
		SizeAndAlign(int sz, int al)
			: sa()
		{
			size(sz);
			align(al);
		}
		int size() const
		{
			return GrabBits<int>(sa, size_bits_t, 0);
		}
		int align() const
		{
			return GrabBits<int>(sa, align_bits_t + size_bits_t, size_bits_t);
		}
		void size(int sz)
		{
			if (sz > (1U << size_bits_t)) throw std::runtime_error("Size too large");
			sa = PackBits(sa, sz, size_bits_t, 0);
		}
		void align(int al)
		{
			if (al > (1U << align_bits_t)) throw std::runtime_error("Alignment too large");
			sa = PackBits(sa, al, align_bits_t + size_bits_t, size_bits_t);
		}
	};
	using SizeAndAlign16 = SizeAndAlign<uint16_t, 6, 10>;
	using SizeAndAlign32 = SizeAndAlign<uint32_t, 10, 22>;

	// The 3D volume (typically within a resource, relative to mip 0)
	struct Box :D3D12_BOX
	{
		Box()
			:D3D12_BOX()
		{}
		Box(iv3 first, iv3 range)
			:D3D12_BOX({
				.left = s_cast<UINT>(first.x),
				.top = s_cast<UINT>(first.y),
				.front = s_cast<UINT>(first.z),
				.right = s_cast<UINT>(std::clamp<int64_t>(s_cast<int64_t>(first.x) + range.x, first.x, pr::limits<int>::max())),
				.bottom = s_cast<UINT>(std::clamp<int64_t>(s_cast<int64_t>(first.y) + range.y, first.y, pr::limits<int>::max())),
				.back = s_cast<UINT>(std::clamp<int64_t>(s_cast<int64_t>(first.z) + range.z, first.z, pr::limits<int>::max())),
			})
		{}

		iv3 pos(int mip = 0) const
		{
			constexpr auto PosAtMip = [](int pos, int mip) { return pos >> mip; };
			return iv3(
				PosAtMip(s_cast<int>(left), mip),
				PosAtMip(s_cast<int>(top), mip),
				PosAtMip(s_cast<int>(front), mip));
		}
		iv3 size(int mip = 0) const
		{
			constexpr auto SizeAtMip = [](int size, int mip) { return std::max<int>(size >> mip, 1); };
			return iv3(
				SizeAtMip(s_cast<int>(right - left), mip),
				SizeAtMip(s_cast<int>(bottom - top), mip),
				SizeAtMip(s_cast<int>(back - front), mip));
		}
		Box mip(int mip = 0) const
		{
			return Box(pos(mip), size(mip));
		}
		Box& Clip(iv3 first, iv3 range)
		{
			Vec3l<void> last = {
				std::clamp<int64_t>(s_cast<int64_t>(first.x) + range.x, first.x, pr::limits<int>::max()),
				std::clamp<int64_t>(s_cast<int64_t>(first.y) + range.y, first.y, pr::limits<int>::max()),
				std::clamp<int64_t>(s_cast<int64_t>(first.z) + range.z, first.z, pr::limits<int>::max()),
			};

			if (s_cast<int>(left)  < first.x) left   = s_cast<UINT>(first.x);
			if (s_cast<int>(top)   < first.y) top    = s_cast<UINT>(first.y);
			if (s_cast<int>(front) < first.z) front  = s_cast<UINT>(first.z);
			if (s_cast<int>(right)  > last.x) right  = s_cast<UINT>(last.x);
			if (s_cast<int>(bottom) > last.y) bottom = s_cast<UINT>(last.y);
			if (s_cast<int>(back)   > last.z) back   = s_cast<UINT>(last.z);
			return *this;
		}
	};

	// Display mode description
	struct DisplayMode :DXGI_MODE_DESC
	{
		// Notes:
		//  - Credit: https://www.rastertek.com/dx12tut03.html
		//    Before we can initialize the swap chain we have to get the refresh rate from the video card/monitor.
		//    Each computer may be slightly different so we will need to query for that information. We query for
		//    the numerator and denominator values and then pass them to DirectX during the setup and it will calculate
		//    the proper refresh rate. If we don't do this and just set the refresh rate to a default value which may
		//    not exist on all computers then DirectX will respond by performing a buffer copy instead of a buffer flip
		//    which will degrade performance and give us annoying errors in the debug output.
		//  - For gamma-correct rendering to standard 8-bit per channel UNORM formats, you'll want to create the Render
		//    Target using an sRGB format. The new flip modes, however, do not allow you to create a swap chain back buffer
		//    using an sRGB format. In this case, you create one using the non-sRGB format (i.e. DXGI_SWAP_CHAIN_DESC1.Format
		//    = DXGI_FORMAT_B8G8R8A8_UNORM) and use sRGB for the Render Target View (i.e. D3D12_RENDER_TARGET_VIEW_DESC.Format
		//    = DXGI_FORMAT_B8G8R8A8_UNORM_SRGB).
		DisplayMode(int width = 1024, int height = 768, DXGI_FORMAT format = DXGI_FORMAT_B8G8R8A8_UNORM)
			:DXGI_MODE_DESC()
		{
			Width                   = width  ? s_cast<uint32_t>(width) : 16;
			Height                  = height ? s_cast<uint32_t>(height) : 16;
			Format                  = format;
			RefreshRate.Numerator   = 0;
			RefreshRate.Denominator = 1;
			ScanlineOrdering        = DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED;
			Scaling                 = DXGI_MODE_SCALING_UNSPECIFIED;
		}
		DisplayMode(iv2 const& area, DXGI_FORMAT format = DXGI_FORMAT_B8G8R8A8_UNORM)
			:DisplayMode(area.x, area.y, format)
		{}
		DisplayMode& size(int w, int h)
		{
			Width  = w ? s_cast<uint32_t>(w) : 16;
			Height = h ? s_cast<uint32_t>(h) : 16;
			return *this;
		}
		DisplayMode& format(DXGI_FORMAT fmt)
		{
			Format = fmt;
			return *this;
		}
		DisplayMode& refresh_rate(int numerator, int denominator)
		{
			RefreshRate.Numerator   = numerator;
			RefreshRate.Denominator = denominator;
			return *this;
		}
		DisplayMode& default_refresh_rate()
		{
			RefreshRate.Numerator   = 0;
			RefreshRate.Denominator = 0;
			return *this;
		}
		DisplayMode& scaling(DXGI_MODE_SCALING scaling)
		{
			Scaling = scaling;
			return *this;
		}
		DisplayMode& scanline_order(DXGI_MODE_SCANLINE_ORDER scanline_order)
		{
			ScanlineOrdering = scanline_order;
			return *this;
		}
	};

	// Resource clear value
	struct ClearValue :D3D12_CLEAR_VALUE
	{
		constexpr ClearValue()
			:D3D12_CLEAR_VALUE{ .Format = DXGI_FORMAT_UNKNOWN, .Color = { 0.0f, 0.0f, 0.0f, 1.0f } }
		{}
		constexpr ClearValue(DXGI_FORMAT format, Colour col)
			:D3D12_CLEAR_VALUE{ .Format = format, .Color = { col.r, col.g, col.b, col.a } }
		{}
		constexpr ClearValue(DXGI_FORMAT format, Colour32 col)
			:ClearValue(format, Colour(col))
		{}
		constexpr ClearValue(DXGI_FORMAT format, float depth, uint8_t stencil)
			: D3D12_CLEAR_VALUE{ .Format = format, .DepthStencil = {.Depth = depth, .Stencil = stencil } }
		{}
	};

	// Multi sampling description
	struct MultiSamp :DXGI_SAMPLE_DESC
	{
		constexpr MultiSamp()
			:DXGI_SAMPLE_DESC()
		{
			Count   = 1;
			Quality = 0;
		}
		constexpr MultiSamp(uint32_t count, uint32_t quality = ~0U)
			:DXGI_SAMPLE_DESC()
		{
			Count = count;
			Quality = quality;
		}
		MultiSamp& ScaleQualityLevel(ID3D12Device* device, DXGI_FORMAT format)
		{
			uint32_t quality = 0;
			for (; Count > 1 && (quality = MultisampleQualityLevels(device, format, Count)) == 0; Count >>= 1) {}
			if (quality != 0 && Quality >= quality) Quality = quality - 1;
			return *this;
		}
		friend bool operator == (MultiSamp const& lhs, MultiSamp const& rhs)
		{
			return lhs.Count == rhs.Count && lhs.Quality == rhs.Quality;
		}
	};

	// Viewport description
	struct Viewport :D3D12_VIEWPORT
	{
		// Notes:
		//  - Viewports represent an area on the back buffer, *not* the target HWND.
		//  - Viewports are in render target space.
		//    e.g.
		//     x,y          = 0,0 (not -0.5f,-0.5f)
		//     width,height = 800,600 (not 1.0f,1.0f)
		//     depth is normalised from 0.0f -> 1.0f
		//  - Viewports are measured in render target pixels not DIP or window pixels.
		//    i.e. generally, the viewport is not in client space coordinates.
		//  - ScreenW/H should be in DIP
		//  - Dx12 requires scissor rectangles for all viewports so they're combined here
		using ScissorRects = pr::vector<RECT, 4, false>;

		int ScreenW; // The screen width (in DIP) that the render target will be mapped to.
		int ScreenH; // The screen height (in DIP) that the render target will be mapped to.
		ScissorRects m_clip;

		Viewport()
			:Viewport(0.0f, 0.0f, 16.0f, 16.0f, 16, 16, 0.0f, 1.0f)
		{}
		Viewport(iv2 const& area)
			:Viewport(0.0f, 0.0f, float(area.x), float(area.y), area.x, area.y, 0.0f, 1.0f)
		{}
		Viewport(float x, float y, float width, float height, int screen_w, int screen_h, float min_depth, float max_depth)
			:D3D12_VIEWPORT()
			,ScreenW()
			,ScreenH()
			,m_clip()
		{
			Set(x, y, width, height, screen_w, screen_h, min_depth, max_depth);
		}

		// Set the viewport area and clip rectangle
		Viewport& Set(iv2 const& area)
		{
			return Set(0.0f, 0.0f, float(area.x), float(area.y), area.x, area.y, 0.0f, 1.0f);
		}
		Viewport& Set(float x, float y, float width, float height)
		{
			return Set(x, y, width, height, (int)width, (int)height, 0.0f, 1.0f);
		}
		Viewport& Set(float x, float y, float width, float height, int screen_w, int screen_h, float min_depth, float max_depth)
		{
			#if PR_DBG_RDR
			Check(x >= D3D12_VIEWPORT_BOUNDS_MIN && x <= D3D12_VIEWPORT_BOUNDS_MAX , "X value out of range");
			Check(y >= D3D12_VIEWPORT_BOUNDS_MIN && y <= D3D12_VIEWPORT_BOUNDS_MAX , "Y value out of range");
			Check(width >= 0.0f                                                    , "Width value invalid");
			Check(height >= 0.0f                                                   , "Height value invalid");
			Check(x + width  <= D3D12_VIEWPORT_BOUNDS_MAX                          , "Width value out of range");
			Check(y + height <= D3D12_VIEWPORT_BOUNDS_MAX                          , "Height value out of range");
			Check(min_depth >= 0.0f && min_depth <= 1.0f                           , "Min depth value out of range");
			Check(max_depth >= 0.0f && max_depth <= 1.0f                           , "Max depth value out of range");
			Check(min_depth <= max_depth                                           , "Min and max depth values invalid");
			Check(screen_w >= 0                                                    , "Screen Width value invalid");
			Check(screen_h >= 0                                                    , "Screen Height value invalid");
			#endif

			TopLeftX = x;
			TopLeftY = y;
			Width    = width;
			Height   = height;
			MinDepth = min_depth;
			MaxDepth = max_depth;
			ScreenW  = screen_w;
			ScreenH  = screen_h;
			
			ClearClips();
			Clip(static_cast<int>(TopLeftX), static_cast<int>(TopLeftY), static_cast<int>(Width), static_cast<int>(Height));
			return *this;
		}

		// Reset the clip rectangle collection
		Viewport& ClearClips()
		{
			m_clip.resize(0);
			return *this;
		}

		// Add a clip rectangle
		Viewport& Clip(int x, int y, int width, int height)
		{
			return Clip(RECT{x, y, x+width, y+height});
		}
		Viewport& Clip(RECT const& rect)
		{
			m_clip.push_back(rect);
			return *this;
		}

		// The aspect ratio (width/height) of the viewport
		float Aspect() const
		{
			return 1.0f * Width / Height;
		}

		// The viewport rectangle, in render target pixels
		FRect AsFRect() const
		{
			return FRect(TopLeftX, TopLeftY, TopLeftX + Width, TopLeftY + Height);
		}
		IRect AsIRect() const
		{
			return IRect(int(TopLeftX), int(TopLeftY), int(TopLeftX + Width), int(TopLeftY + Height));
		}
		RECT AsRECT() const
		{
			return RECT{LONG(TopLeftX), LONG(TopLeftY), LONG(TopLeftX + Width), LONG(TopLeftY + Height)};
		}

		// Convert lengths in pixels (X and Y) into lengths in normalised screen space
		v2 PixelsToNSS(v2 pixels) const
		{
			// Remember: NSS = [-1, +1] so if pixels == (ScreenW, ScreenH), then the lengths should be (2, 2)
			return v2 {
				ScreenW != 0 ? 2.0f * pixels.x / ScreenW : 0,
				ScreenH != 0 ? 2.0f * pixels.y / ScreenH : 0,
			};
		}

		// Convert a screen space point to normalised screen space
		// 'ss_point' must be in screen pixels, not logical pixels (DIP).
		v2 SSPointToNSSPoint(v2 const& ss_point) const
		{
			return NormalisePoint(IRect(0, 0, ScreenW, ScreenH), ss_point, 1.0f, -1.0f);
		}
		v2 NSSPointToSSPoint(v2 const& nss_point) const
		{
			return ScalePoint(IRect(0, 0, ScreenW, ScreenH), nss_point, 1.0f, -1.0f);
		}
	};

	// Heap properties
	struct HeapProps :D3D12_HEAP_PROPERTIES
	{
		HeapProps() = default;
		explicit HeapProps(D3D12_HEAP_TYPE heap_type, uint32_t creation_node_mask = 1, uint32_t visible_node_mask = 1)
			:HeapProps()
		{
			Type = heap_type;
			CPUPageProperty = D3D12_CPU_PAGE_PROPERTY::D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
			MemoryPoolPreference = D3D12_MEMORY_POOL::D3D12_MEMORY_POOL_UNKNOWN;
			CreationNodeMask = creation_node_mask;
			VisibleNodeMask = visible_node_mask;
		}
		HeapProps(D3D12_HEAP_PROPERTIES const& rhs)
			:D3D12_HEAP_PROPERTIES(rhs)
		{}
		
		// Can the CPU read/write this heap?
		bool IsCPUReadable() const
		{
			return Type == D3D12_HEAP_TYPE_READBACK || (Type == D3D12_HEAP_TYPE_CUSTOM && CPUPageProperty == D3D12_CPU_PAGE_PROPERTY_WRITE_BACK);
		}
		bool IsCPUWriteable() const
		{
			return Type == D3D12_HEAP_TYPE_UPLOAD || (Type == D3D12_HEAP_TYPE_CUSTOM && CPUPageProperty == D3D12_CPU_PAGE_PROPERTY_WRITE_COMBINE);
		}
		bool IsCPUAccessible() const
		{
			return IsCPUWriteable() && IsCPUReadable();
		}

		// Common heap properties
		static HeapProps const& Default()
		{
			static HeapProps props(D3D12_HEAP_TYPE_DEFAULT);
			return props;
		}
		static HeapProps const& Upload()
		{
			static HeapProps props(D3D12_HEAP_TYPE_UPLOAD);
			return props;
		}
	};

	// A resource description
	struct ResDesc :D3D12_RESOURCE_DESC
	{
		// Notes:
		//  - Width/Height/Depth should be in pixels/verts/indices/etc
		//  - ElemStride is used to calculate size in bytes.
		//  - Resources must be allocated with 0, 4096, or 65536 alignment.
		//  - Data within resources can use the 'DATA_PLACEMENT_ALIGNMENT' values.
		//  - Size of resource heap must be at least 64K for single-textures and constant buffers
		using clear_value_t = std::optional<D3D12_CLEAR_VALUE>;
		enum class EMiscFlags
		{
			None = 0,
			PartialInitData = 1 << 0,
			CubeMap = 1 << 1,
			RayTracingStruct = 1 << 2,
			_flags_enum = 0,
		};

		int ElemStride;                     // Element stride
		int DataAlignment;                  // The alignment that initialisation data should have.
		pr::vector<Image> Data;             // The initialisation data for the buffer, texture, or texture array
		HeapProps HeapProps;                // The heap to create this buffer in
		D3D12_HEAP_FLAGS HeapFlags;         // Properties
		clear_value_t ClearValue;           // A clear value for the resource
		D3D12_RESOURCE_STATES DefaultState; // The state the resource should be in between command list executions
		EMiscFlags MiscFlags;               // Other flags

		ResDesc()
			: D3D12_RESOURCE_DESC()
			, ElemStride()
			, DataAlignment()
			, Data()
			, HeapProps(HeapProps::Default())
			, HeapFlags(D3D12_HEAP_FLAG_NONE)
			, ClearValue()
			, DefaultState(D3D12_RESOURCE_STATE_COMMON)
			, MiscFlags(EMiscFlags::None)
		{}
		ResDesc(ResDesc&& rhs) = default;
		ResDesc(ResDesc const& rhs) = default;
		ResDesc(D3D12_RESOURCE_DESC const& rhs)
			: ResDesc()
		{
			*static_cast<D3D12_RESOURCE_DESC*>(this) = rhs;
		}
		ResDesc(D3D12_RESOURCE_DIMENSION dimension, DXGI_FORMAT format, uint64_t width, uint32_t height, uint16_t depth, int element_stride)
			:ResDesc()
		{
			// Note: Dx12 expects 'Width' to be in bytes for buffers
			// However, I'm using 'Width' as the array length (in elements) to be consistent with textures
			// ResourceManager::CreateResource() converts to bytes as needed, but you'll need to convert
			// manually if you don't use 'ResourceManager'.
			Dimension = dimension;
			Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT;
			Width = width;
			Height = height;
			DepthOrArraySize = depth;
			MipLevels = 0;
			Format = format;
			SampleDesc = MultiSamp{};
			Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
			Flags = D3D12_RESOURCE_FLAG_NONE;
			ElemStride = element_stride;
			DataAlignment = 0;
		}
		ResDesc& operator = (ResDesc&& rhs) = default;
		ResDesc& operator = (ResDesc const& rhs) = default;
		ResDesc& operator = (D3D12_RESOURCE_DESC const& rhs)
		{
			if (&rhs == this) return *this;
			*static_cast<D3D12_RESOURCE_DESC*>(this) = rhs;
			return *this;
		}

		// Sanity check resource settings
		bool Check() const
		{
			if (Width < 1 || Height < 1 || DepthOrArraySize < 1)
				return false;

			if (AnySet(Flags, D3D12_RESOURCE_FLAGS::D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET | D3D12_RESOURCE_FLAGS::D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL) && Alignment != D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT)
				return false;

			return true;
		}

		ResDesc& init_data(Image data, bool partial_init = false)
		{
			if (data.m_data.vptr != nullptr)
			{
				Data.push_back(data);
				MiscFlags |= partial_init ? EMiscFlags::PartialInitData : EMiscFlags::None;
			}
			return *this;
		}
		ResDesc& clear(D3D12_CLEAR_VALUE clear)
		{
			assert(Dimension != D3D12_RESOURCE_DIMENSION_BUFFER && "Cannot use a clear value with buffer resources");
			ClearValue = clear;
			return *this;
		}
		ResDesc& clear(DXGI_FORMAT format, Colour32 colour)
		{
			return clear(format, Colour(colour));
		}
		ResDesc& clear(DXGI_FORMAT format, Colour colour)
		{
			return clear(D3D12_CLEAR_VALUE{ .Format = format, .Color = {colour.r, colour.g, colour.b, colour.a} });
		}
		ResDesc& clear(DXGI_FORMAT format, D3DCOLORVALUE colour)
		{
			return clear(D3D12_CLEAR_VALUE{ .Format = format, .Color = {colour.r, colour.g, colour.b, colour.a} });
		}
		ResDesc& clear(DXGI_FORMAT format, D3D12_DEPTH_STENCIL_VALUE depth_stencil)
		{
			return clear(D3D12_CLEAR_VALUE{ .Format = format, .DepthStencil = depth_stencil });
		}
		ResDesc& mips(int mips)
		{
			MipLevels = s_cast<uint16_t>(mips);
			return *this;
		}
		ResDesc& usage(EUsage usage)
		{
			Flags = s_cast<D3D12_RESOURCE_FLAGS>(usage);
			if (AnySet(Flags, D3D12_RESOURCE_FLAGS::D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET | D3D12_RESOURCE_FLAGS::D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL))
				Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT;
			return *this;
		}
		ResDesc& misc_flags(EMiscFlags flags)
		{
			MiscFlags = flags;
			return *this;
		}
		ResDesc& multisamp(MultiSamp sampling)
		{
			SampleDesc = sampling;
			return *this;
		}
		ResDesc& heap_flags(D3D12_HEAP_FLAGS flags, bool add = true)
		{
			HeapFlags = SetBits(HeapFlags, flags, add);
			return *this;
		}
		ResDesc& layout(D3D12_TEXTURE_LAYOUT tex_layout)
		{
			Layout = tex_layout;
			return *this;
		}
		ResDesc& res_alignment(uint64_t alignment)
		{
			Alignment = alignment;
			return *this;
		}
		ResDesc& data_alignment(int alignment)
		{
			DataAlignment = alignment;
			return *this;
		}
		ResDesc& def_state(D3D12_RESOURCE_STATES default_state)
		{
			DefaultState = default_state;
			return *this;
		}

		D3D12_SRV_DIMENSION SrvDimension() const
		{
			switch (Dimension)
			{
				case D3D12_RESOURCE_DIMENSION_BUFFER:
				{
					return AllSet(MiscFlags, EMiscFlags::RayTracingStruct)
						? D3D12_SRV_DIMENSION_RAYTRACING_ACCELERATION_STRUCTURE
						: D3D12_SRV_DIMENSION_BUFFER;
				}
				case D3D12_RESOURCE_DIMENSION_TEXTURE1D:
				{
					return DepthOrArraySize > 1
						? D3D12_SRV_DIMENSION_TEXTURE1DARRAY
						: D3D12_SRV_DIMENSION_TEXTURE1D;
				}
				case D3D12_RESOURCE_DIMENSION_TEXTURE2D:
				{
					return AllSet(MiscFlags, EMiscFlags::CubeMap)
						? D3D12_SRV_DIMENSION_TEXTURECUBE
						: (DepthOrArraySize > 1
							? (SampleDesc.Count == 1 ? D3D12_SRV_DIMENSION_TEXTURE2DARRAY : D3D12_SRV_DIMENSION_TEXTURE2DMSARRAY)
							: (SampleDesc.Count == 1 ? D3D12_SRV_DIMENSION_TEXTURE2D : D3D12_SRV_DIMENSION_TEXTURE2DMS));
				}
				case D3D12_RESOURCE_DIMENSION_TEXTURE3D:
				{
					return AllSet(MiscFlags, EMiscFlags::CubeMap)
						? D3D12_SRV_DIMENSION_TEXTURECUBEARRAY
						: D3D12_SRV_DIMENSION_TEXTURE3D;
				}
				default:
				{
					throw std::runtime_error("Unknown resource dimension");
				}
			}
		}
		D3D12_RTV_DIMENSION RtvDimension() const
		{
			switch (Dimension)
			{
				case D3D12_RESOURCE_DIMENSION_BUFFER:
				{
					return D3D12_RTV_DIMENSION_BUFFER;
				}
				case D3D12_RESOURCE_DIMENSION_TEXTURE1D:
				{
					return DepthOrArraySize > 1
						? D3D12_RTV_DIMENSION_TEXTURE1DARRAY
						: D3D12_RTV_DIMENSION_TEXTURE1D;
				}
				case D3D12_RESOURCE_DIMENSION_TEXTURE2D:
				{
					return DepthOrArraySize > 1
						? (SampleDesc.Count == 1 ? D3D12_RTV_DIMENSION_TEXTURE2DARRAY : D3D12_RTV_DIMENSION_TEXTURE2DMSARRAY)
						: (SampleDesc.Count == 1 ? D3D12_RTV_DIMENSION_TEXTURE2D : D3D12_RTV_DIMENSION_TEXTURE2DMS);
				}
				case D3D12_RESOURCE_DIMENSION_TEXTURE3D:
				{
					return D3D12_RTV_DIMENSION_TEXTURE3D;
				}
				default:
				{
					throw std::runtime_error("Unknown resource dimension");
				}
			}
		}
		D3D12_DSV_DIMENSION DsvDimension() const
		{
			switch (Dimension)
			{
				case D3D12_RESOURCE_DIMENSION_BUFFER:
				{
					throw std::runtime_error("Depth stencils cannot be buffers");
				}
				case D3D12_RESOURCE_DIMENSION_TEXTURE1D:
				{
					return DepthOrArraySize > 1
						? D3D12_DSV_DIMENSION_TEXTURE1DARRAY
						: D3D12_DSV_DIMENSION_TEXTURE1D;
				}
				case D3D12_RESOURCE_DIMENSION_TEXTURE2D:
				{
					return DepthOrArraySize > 1
						? (SampleDesc.Count == 1 ? D3D12_DSV_DIMENSION_TEXTURE2DARRAY : D3D12_DSV_DIMENSION_TEXTURE2DMSARRAY)
						: (SampleDesc.Count == 1 ? D3D12_DSV_DIMENSION_TEXTURE2D : D3D12_DSV_DIMENSION_TEXTURE2DMS);
				}
				case D3D12_RESOURCE_DIMENSION_TEXTURE3D:
				{
					throw std::runtime_error("Depth stencils cannot be 3D textures");
				}
				default:
				{
					throw std::runtime_error("Unknown resource dimension");
				}
			}
		}

		// Generic buffer resource description
		static ResDesc Buf(int64_t count, int element_stride, std::span<std::byte const> init_data, int data_alignment)
		{
			assert((isize(init_data) % element_stride) == 0); // 'init_data' must be [0, count] elements
			return ResDesc(D3D12_RESOURCE_DIMENSION_BUFFER, DXGI_FORMAT_UNKNOWN, s_cast<uint64_t>(count), 1, 1, element_stride)
				.mips(1)
				.res_alignment(D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT)
				.data_alignment(data_alignment)
				.layout(D3D12_TEXTURE_LAYOUT_ROW_MAJOR) // required for DIMENSION_BUFFER types
				.init_data(Image(init_data.data(), init_data.size() / element_stride, element_stride), isize(init_data) < count * element_stride);
		}
		template <typename TElem> static ResDesc Buf(int64_t count, std::span<TElem const> init_data)
		{
			return Buf(count, sizeof(TElem), byte_span(init_data), alignof(TElem));
		}

		// Vertex buffer description
		template <typename TVert> static ResDesc VBuf(int64_t count, std::span<TVert const> init_data)
		{
			count += int64_t(count == 0);
			return Buf(count, sizeof(TVert), byte_span(init_data), alignof(TVert))
				.def_state(D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER);
		}

		// Derive a vertex buffer description from an existing vertex buffer
		template <typename TVert> static ResDesc VBuf(ID3D12Resource* vbuf)
		{
			auto vb = vbuf->GetDesc();
			auto count = int64_t(vb.Width) / sizeof(TVert);
			count += int64_t(count == 0);
			return Buf(count, sizeof(TVert), {}, alignof(TVert))
				.usage(static_cast<EUsage>(vb.Flags))
				.def_state(D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER);
		}

		// Index buffer description
		template <typename TIndx> static ResDesc IBuf(int64_t count, std::span<TIndx const> init_data)
		{
			count += int64_t(count == 0);
			return Buf(count, sizeof(TIndx), byte_span(init_data), alignof(TIndx))
				.def_state(D3D12_RESOURCE_STATE_INDEX_BUFFER);
		}

		// Derive an index buffer description from an existing index buffer
		template <typename TIndx> static ResDesc IBuf(ID3D12Resource* ibuf)
		{
			auto ib = ibuf->GetDesc();
			auto count = int64_t(ib.Width) / sizeof(TIndx);
			count += int64_t(count == 0);
			return Buf(count, sizeof(TIndx), {}, alignof(TIndx))
				.usage(static_cast<EUsage>(ib.Flags))
				.def_state(D3D12_RESOURCE_STATE_INDEX_BUFFER);
		}

		// Index buffer description of arbitrary element size
		static ResDesc IBuf(int64_t count, int element_stride, std::span<std::byte const> data)
		{
			count += int64_t(count == 0);
			return Buf(count, element_stride, data, element_stride)
				.def_state(D3D12_RESOURCE_STATE_INDEX_BUFFER);
		}

		// Constant buffer description
		static ResDesc CBuf(int size)
		{
			size = PadTo<int>(size, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT);
			return ResDesc(D3D12_RESOURCE_DIMENSION_BUFFER, DXGI_FORMAT_UNKNOWN, s_cast<uint64_t>(size), 1, 1, 1)
				.mips(1)
				.res_alignment(0ULL)
				.data_alignment(D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT)
				.def_state(D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER);
		}

		// Texture resource descriptions
		static ResDesc Tex1D(Image data, uint16_t mips = 0, EUsage flags = EUsage::Default)
		{
			return ResDesc(D3D12_RESOURCE_DIMENSION_TEXTURE1D, data.m_format, s_cast<uint64_t>(data.m_dim.x), 1, 1, BytesPerPixel(data.m_format))
				.mips(mips)
				.usage(flags)
				.res_alignment(ResourceAlignment(data, flags))
				.data_alignment(D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT)
				.def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE)
				.init_data(data);
		}
		static ResDesc Tex2D(Image data, uint16_t mips = 0, EUsage flags = EUsage::Default)
		{
			// Notes:
			// - Create textures in the 'D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE' state because they are typically
			//   used for texturing in shaders. Other cases should be set explicitly by the caller.
			return ResDesc(D3D12_RESOURCE_DIMENSION_TEXTURE2D, data.m_format, s_cast<uint64_t>(data.m_dim.x), s_cast<uint32_t>(data.m_dim.y), 1, BytesPerPixel(data.m_format))
				.mips(mips)
				.usage(flags)
				.res_alignment(ResourceAlignment(data, flags))
				.data_alignment(D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT)
				.def_state(!AllSet(flags, EUsage::DenyShaderResource) ? D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE : D3D12_RESOURCE_STATE_COMMON)
				.init_data(data);
		}
		static ResDesc Tex3D(Image data, uint16_t mips = 0, EUsage flags = EUsage::Default)
		{
			return ResDesc(D3D12_RESOURCE_DIMENSION_TEXTURE3D, data.m_format, s_cast<uint64_t>(data.m_dim.x), s_cast<uint32_t>(data.m_dim.y), s_cast<uint16_t>(data.m_dim.z), BytesPerPixel(data.m_format))
				.mips(mips)
				.usage(flags)
				.res_alignment(ResourceAlignment(data, flags))
				.data_alignment(D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT)
				.def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE)
				.init_data(data);
		}
		static ResDesc TexCube(Image data, uint16_t mips = 0, EUsage flags = EUsage::Default)
		{
			return ResDesc(D3D12_RESOURCE_DIMENSION_TEXTURE2D, data.m_format, s_cast<uint64_t>(data.m_dim.x), s_cast<uint32_t>(data.m_dim.y), 6, BytesPerPixel(data.m_format))
				.mips(mips)
				.usage(flags)
				.misc_flags(EMiscFlags::CubeMap)
				.res_alignment(ResourceAlignment(data, flags))
				.data_alignment(D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT)
				.def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE)
				.init_data(data);
		}

	private:

		// Default resource alignment based on init data size
		static uint64_t ResourceAlignment(Image const& data, EUsage flags)
		{
			return
				data.SizeInBytes() <= D3D12_SMALL_RESOURCE_PLACEMENT_ALIGNMENT &&
				!AnySet(flags, EUsage::RenderTarget | EUsage::DepthStencil)
				? D3D12_SMALL_RESOURCE_PLACEMENT_ALIGNMENT
				: D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT;
		}
	};

	// Resource barrier - Use BarrierBatch
	struct ResourceBarrier :D3D12_RESOURCE_BARRIER
	{
		ResourceBarrier() = delete;
	};
	
	// Render target blend state description
	struct RenderTargetBlendDesc :D3D12_RENDER_TARGET_BLEND_DESC
	{
		RenderTargetBlendDesc()
			:D3D12_RENDER_TARGET_BLEND_DESC()
		{
			BlendEnable = FALSE;
			LogicOpEnable = FALSE;
			SrcBlend = D3D12_BLEND_ONE;
			DestBlend = D3D12_BLEND_ONE;
			BlendOp = D3D12_BLEND_OP_MAX;
			SrcBlendAlpha = D3D12_BLEND_ONE;
			DestBlendAlpha = D3D12_BLEND_ONE;
			BlendOpAlpha = D3D12_BLEND_OP_MAX;
			LogicOp = D3D12_LOGIC_OP_NOOP;
			RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
		}
	};

	// Blend state description
	struct BlendStateDesc :D3D12_BLEND_DESC
	{
		BlendStateDesc()
			:D3D12_BLEND_DESC()
		{
			AlphaToCoverageEnable = FALSE;
			IndependentBlendEnable = FALSE;
			RenderTarget[0] = 
				RenderTarget[1] =
				RenderTarget[2] =
				RenderTarget[3] =
				RenderTarget[4] =
				RenderTarget[5] =
				RenderTarget[6] =
				RenderTarget[7] =
				RenderTargetBlendDesc{};
		}
		BlendStateDesc& enable(int idx, bool on = true)
		{
			assert(idx >= 0 && idx < _countof(RenderTarget));
			RenderTarget[idx].BlendEnable = on ? TRUE : FALSE;
			return *this;
		}
		BlendStateDesc& blend(int idx, D3D12_BLEND_OP op, D3D12_BLEND src, D3D12_BLEND dest)
		{
			assert(idx >= 0 && idx < _countof(RenderTarget));
			RenderTarget[idx].BlendOp = op;
			RenderTarget[idx].SrcBlend = src;
			RenderTarget[idx].DestBlend = dest;
			return *this;
		}
		BlendStateDesc& blend_alpha(int idx, D3D12_BLEND_OP op, D3D12_BLEND src, D3D12_BLEND dest)
		{
			assert(idx >= 0 && idx < _countof(RenderTarget));
			RenderTarget[idx].BlendOpAlpha = op;
			RenderTarget[idx].SrcBlendAlpha = src;
			RenderTarget[idx].DestBlendAlpha = dest;
			return *this;
		}

	};

	// Raster state description
	struct RasterStateDesc :D3D12_RASTERIZER_DESC
	{
		RasterStateDesc()
			:D3D12_RASTERIZER_DESC()
		{
			FillMode = D3D12_FILL_MODE_SOLID;
			CullMode = D3D12_CULL_MODE_NONE;//D3D12_CULL_MODE_BACK;
			FrontCounterClockwise = TRUE;
			DepthBias = D3D12_DEFAULT_DEPTH_BIAS;
			DepthBiasClamp = D3D12_DEFAULT_DEPTH_BIAS_CLAMP;
			SlopeScaledDepthBias = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS;
			DepthClipEnable = TRUE;
			MultisampleEnable = FALSE;
			AntialiasedLineEnable = FALSE;
			ForcedSampleCount = 0U;
			ConservativeRaster = D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF;
		}
		RasterStateDesc& Set(D3D12_CULL_MODE mode)
		{
			CullMode = mode;
			return *this;
		}
	};

	// Depth state description
	struct DepthStateDesc :D3D12_DEPTH_STENCIL_DESC
	{
		DepthStateDesc()
			:D3D12_DEPTH_STENCIL_DESC()
		{
			DepthEnable = TRUE;
			DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
			DepthFunc = D3D12_COMPARISON_FUNC_LESS;
			StencilEnable = FALSE;
			StencilReadMask = D3D12_DEFAULT_STENCIL_READ_MASK;
			StencilWriteMask = D3D12_DEFAULT_STENCIL_WRITE_MASK;
			FrontFace = {
				.StencilFailOp = D3D12_STENCIL_OP_KEEP,
				.StencilDepthFailOp = D3D12_STENCIL_OP_KEEP,
				.StencilPassOp = D3D12_STENCIL_OP_KEEP,
				.StencilFunc = D3D12_COMPARISON_FUNC_ALWAYS,
			};
			BackFace = {
				.StencilFailOp = D3D12_STENCIL_OP_KEEP,
				.StencilDepthFailOp = D3D12_STENCIL_OP_KEEP,
				.StencilPassOp = D3D12_STENCIL_OP_KEEP,
				.StencilFunc = D3D12_COMPARISON_FUNC_ALWAYS,
			};
		}
		DepthStateDesc& Enabled(bool enabled)
		{
			DepthEnable = enabled ? TRUE : FALSE;
			return *this;
		}
	};

	// Stream output description
	struct StreamOutputDesc :D3D12_STREAM_OUTPUT_DESC
	{
		vector<D3D12_SO_DECLARATION_ENTRY, 8> m_entries;
		vector<UINT, 1> m_strides;

		StreamOutputDesc()
			:D3D12_STREAM_OUTPUT_DESC()
			,m_entries()
			,m_strides()
		{
			pSODeclaration = nullptr;
			NumEntries = 0U;
			pBufferStrides = nullptr;
			NumStrides = 0U;
			RasterizedStream = 0;
		}

		StreamOutputDesc& add_entry(D3D12_SO_DECLARATION_ENTRY const& entry)
		{
			m_entries.push_back(entry);
			return *this;
		}
		StreamOutputDesc& add_buffer(size_t stride)
		{
			m_strides.push_back(s_cast<UINT>(stride));
			return *this;
		}
		StreamOutputDesc& raster(int stream_index)
		{
			RasterizedStream = stream_index;
			return *this;
		}
		StreamOutputDesc& no_raster()
		{
			return raster(D3D12_SO_NO_RASTERIZED_STREAM);
		}
		D3D12_STREAM_OUTPUT_DESC const& create()
		{
			pSODeclaration = m_entries.data();
			NumEntries = s_cast<UINT>(m_entries.size());
			pBufferStrides = m_strides.data();
			NumStrides = s_cast<UINT>(m_strides.size());
			return *this;
		}
	};

	// Texture sampler description
	struct SamDesc :D3D12_SAMPLER_DESC
	{
		// Notes:
		//  - There isn't a logical "default" sampler choice. Using invalid values in the default
		//    constructor to force instances to set their own values.
		
		SamDesc()
			:SamDesc(s_cast<D3D12_TEXTURE_ADDRESS_MODE>(0), D3D12_FILTER_MIN_MAG_MIP_POINT)
		{}
		SamDesc(D3D12_TEXTURE_ADDRESS_MODE addr, D3D12_FILTER filter)
			:SamDesc(addr, addr, addr, filter)
		{}
		SamDesc(D3D12_TEXTURE_ADDRESS_MODE addrU, D3D12_TEXTURE_ADDRESS_MODE addrV, D3D12_TEXTURE_ADDRESS_MODE addrW, D3D12_FILTER filter)
			:D3D12_SAMPLER_DESC()
		{
			Filter = filter;
			AddressU = addrU;
			AddressV = addrV;
			AddressW = addrW;
			MipLODBias = 0.0f;
			MaxAnisotropy = 1;
			ComparisonFunc = D3D12_COMPARISON_FUNC(0);
			BorderColor[0] = 0.0f;
			BorderColor[1] = 0.0f;
			BorderColor[2] = 0.0f;
			BorderColor[3] = 0.0f;
			MinLOD = 0.0f;
			MaxLOD = D3D12_FLOAT32_MAX;
		}

		// Hash this description to create an Id that can be used to detect duplicate samplers
		RdrId Id() const
		{
			return s_cast<RdrId>(pr::hash::HashBytes64(this, this + 1));
		}

		SamDesc& border(Colour32 colour)
		{
			Colour c(colour);
			BorderColor[0] = c.a;
			BorderColor[1] = c.r;
			BorderColor[2] = c.g;
			BorderColor[3] = c.b;
			return *this;
		}
		SamDesc& addr(D3D12_TEXTURE_ADDRESS_MODE modeUVW)
		{
			return addr(modeUVW, modeUVW, modeUVW);
		}
		SamDesc& addr(D3D12_TEXTURE_ADDRESS_MODE addrU, D3D12_TEXTURE_ADDRESS_MODE addrV)
		{
			return addr(addrU, addrV, D3D12_TEXTURE_ADDRESS_MODE_BORDER);
		}
		SamDesc& addr(D3D12_TEXTURE_ADDRESS_MODE addrU, D3D12_TEXTURE_ADDRESS_MODE addrV, D3D12_TEXTURE_ADDRESS_MODE addrW)
		{
			AddressU = addrU;
			AddressV = addrV;
			AddressW = addrW;
			return *this;
		}
		SamDesc& filter(D3D12_FILTER mode)
		{
			Filter = mode;
			return *this;
		}
		SamDesc& compare(D3D12_COMPARISON_FUNC comp)
		{
			ComparisonFunc = comp;
			return *this;
		}

		// Standard samplers
		static SamDesc const& PointClamp()       { static SamDesc sam(D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_FILTER_MIN_MAG_MIP_POINT); return sam; }
		static SamDesc const& PointWrap()        { static SamDesc sam(D3D12_TEXTURE_ADDRESS_MODE_WRAP , D3D12_FILTER_MIN_MAG_MIP_POINT); return sam; }
		static SamDesc const& LinearClamp()      { static SamDesc sam(D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_FILTER_MIN_MAG_MIP_LINEAR); return sam; }
		static SamDesc const& LinearWrap()       { static SamDesc sam(D3D12_TEXTURE_ADDRESS_MODE_WRAP , D3D12_FILTER_MIN_MAG_MIP_LINEAR); return sam; }
		static SamDesc const& AnisotropicClamp() { static SamDesc sam(D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_FILTER_ANISOTROPIC); return sam; }
		static SamDesc const& AnisotropicWrap()  { static SamDesc sam(D3D12_TEXTURE_ADDRESS_MODE_WRAP , D3D12_FILTER_ANISOTROPIC); return sam; }
	};

	// Static sampler description
	struct SamDescStatic :D3D12_STATIC_SAMPLER_DESC
	{
		constexpr SamDescStatic(ESamReg shader_register)
			:SamDescStatic(shader_register, D3D12_TEXTURE_ADDRESS_MODE_CLAMP, D3D12_FILTER_MIN_MAG_MIP_LINEAR)
		{}
		constexpr SamDescStatic(ESamReg shader_register, D3D12_TEXTURE_ADDRESS_MODE addr, D3D12_FILTER filter)
			:SamDescStatic(shader_register, addr, addr, addr, filter)
		{}
		constexpr SamDescStatic(ESamReg shader_register, D3D12_TEXTURE_ADDRESS_MODE addrU, D3D12_TEXTURE_ADDRESS_MODE addrV, D3D12_TEXTURE_ADDRESS_MODE addrW, D3D12_FILTER filter)
			:D3D12_STATIC_SAMPLER_DESC()
		{
			Filter           = filter;
			AddressU         = addrU;
			AddressV         = addrV;
			AddressW         = addrW;
			MipLODBias       = 0.0f;
			MaxAnisotropy    = 1;
			ComparisonFunc   = D3D12_COMPARISON_FUNC_ALWAYS;
			BorderColor      = D3D12_STATIC_BORDER_COLOR_TRANSPARENT_BLACK;
			MinLOD           = 0.0f;
			MaxLOD           = D3D12_FLOAT32_MAX;
			ShaderRegister   = s_cast<UINT>(shader_register);
			RegisterSpace    = 0U;
			ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
		}
		constexpr SamDescStatic& border(D3D12_STATIC_BORDER_COLOR colour)
		{
			BorderColor = colour;
			return *this;
		}
		constexpr SamDescStatic& shader_vis(D3D12_SHADER_VISIBILITY vis)
		{
			ShaderVisibility = vis;
			return *this;
		}
		constexpr SamDescStatic& addr(D3D12_TEXTURE_ADDRESS_MODE modeUVW)
		{
			return addr(modeUVW, modeUVW, modeUVW);
		}
		constexpr SamDescStatic& addr(D3D12_TEXTURE_ADDRESS_MODE addrU, D3D12_TEXTURE_ADDRESS_MODE addrV)
		{
			return addr(addrU, addrV, D3D12_TEXTURE_ADDRESS_MODE_BORDER);
		}
		constexpr SamDescStatic& addr(D3D12_TEXTURE_ADDRESS_MODE addrU, D3D12_TEXTURE_ADDRESS_MODE addrV, D3D12_TEXTURE_ADDRESS_MODE addrW)
		{
			AddressU = addrU;
			AddressV = addrV;
			AddressW = addrW;
			return *this;
		}
		constexpr SamDescStatic& filter(D3D12_FILTER mode)
		{
			Filter = mode;
			return *this;
		}
		constexpr SamDescStatic& compare(D3D12_COMPARISON_FUNC comp)
		{
			ComparisonFunc = comp;
			return *this;
		}
	};

	// Compiled shader byte code
	struct ByteCode :D3D12_SHADER_BYTECODE
	{
		ByteCode()
			:D3D12_SHADER_BYTECODE()
		{}
		ByteCode(std::span<BYTE const> code)
			: D3D12_SHADER_BYTECODE(code.data(), code.size())
		{}
		template <int Size> ByteCode(BYTE const (&code)[Size])
			: D3D12_SHADER_BYTECODE(&code[0], Size)
		{}
		explicit operator bool() const
		{
			return pShaderBytecode != nullptr;
		}
		operator std::span<BYTE const>() const
		{
			return std::span<BYTE const>(static_cast<BYTE const*>(pShaderBytecode), BytecodeLength);
		}
	};
}
