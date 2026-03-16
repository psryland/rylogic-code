//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	#ifdef NDEBUG
	inline static constexpr bool IsDebug = false;
	#else
	inline static constexpr bool IsDebug = true;
	#endif

	// Turn an r-value into an l-value. Basically 'std::unmove()'
	template <typename T>
	inline T& LValue(T&& x) { return x; }

	// Helper for getting the ref count of a COM pointer.
	ULONG RefCount(IUnknown* ptr);
	template <typename T> ULONG RefCount(RefPtr<T> ptr)
	{
		if (ptr == nullptr) return 0;
		return RegCount(ptr.m_ptr);
	}

	// Helper for allocating and constructing a type using 'alloc_traits'
	template <typename T, typename... Args> requires std::constructible_from<T, Args...>
	[[nodiscard]] inline T* New(Args&&... args)
	{
		Allocator<T> alex;
		auto ptr = alloc_traits<T>::allocate(alex, sizeof(T));
		alloc_traits<T>::construct(alex, ptr, std::forward<Args>(args)...);
		return ptr;
	}
	template <typename T>
	inline void Delete(T* ptr)
	{
		Allocator<T> alex;
		alloc_traits<T>::destroy(alex, ptr);
		alloc_traits<T>::deallocate(alex, ptr, 1);
	}

	// Returns an incrementing id with each call
	inline RdrId MonotonicId()
	{
		static RdrId s_id = 0;
		return ++s_id;
	}

	// Make a RdrId from a pointer.
	inline RdrId MakeId(void const* ptr)
	{
		// Careful, don't make a templated MakeId(T const& obj) function. It will be selected in preference to this function.
		return static_cast<RdrId>(byte_ptr(ptr) - byte_ptr(nullptr));
	}

	// Make an RdrId from a string
	inline RdrId MakeId(wchar_t const* str)
	{
		return hash::Hash(str);
	}
	inline RdrId MakeId(char const* str)
	{
		return hash::Hash(str);
	}

	// Create a 4-byte CC code
	constexpr uint32_t MakeFourCC(uint8_t ch0, uint8_t ch1, uint8_t ch2, uint8_t ch3)
	{
		return static_cast<uint32_t>(ch0) | (static_cast<uint32_t>(ch1) << 8) | (static_cast<uint32_t>(ch2) << 16) | (static_cast<uint32_t>(ch3) << 24);
	}

	// Convert device independent pixels (DIP) to physical pixels using the given 'dpi'
	constexpr float DIPtoPhysical(float dip, float dpi)
	{
		return dip * dpi / 96.0f;
	}
	constexpr float PhysicaltoDIP(float phys, float dpi)
	{
		return phys * 96.0f / dpi;
	}
	constexpr v2 DIPtoPhysical(v2 pt, v2 dpi)
	{
		return v2(
			DIPtoPhysical(pt.x, dpi.x),
			DIPtoPhysical(pt.y, dpi.y));
	}
	constexpr v2 PhysicaltoDIP(v2 pt, v2 dpi)
	{
		return v2(
			PhysicaltoDIP(pt.x, dpi.x),
			PhysicaltoDIP(pt.y, dpi.y));
	}

	// Return the number of bits per pixel for a given dx format
	constexpr int BitsPerPixel(DXGI_FORMAT fmt)
	{
		switch (fmt)
		{
			case DXGI_FORMAT_R32G32B32A32_TYPELESS:
			case DXGI_FORMAT_R32G32B32A32_FLOAT:
			case DXGI_FORMAT_R32G32B32A32_UINT:
			case DXGI_FORMAT_R32G32B32A32_SINT:
				return 128;

			case DXGI_FORMAT_R32G32B32_TYPELESS:
			case DXGI_FORMAT_R32G32B32_FLOAT:
			case DXGI_FORMAT_R32G32B32_UINT:
			case DXGI_FORMAT_R32G32B32_SINT:
				return 96;

			case DXGI_FORMAT_R16G16B16A16_TYPELESS:
			case DXGI_FORMAT_R16G16B16A16_FLOAT:
			case DXGI_FORMAT_R16G16B16A16_UNORM:
			case DXGI_FORMAT_R16G16B16A16_UINT:
			case DXGI_FORMAT_R16G16B16A16_SNORM:
			case DXGI_FORMAT_R16G16B16A16_SINT:
			case DXGI_FORMAT_R32G32_TYPELESS:
			case DXGI_FORMAT_R32G32_FLOAT:
			case DXGI_FORMAT_R32G32_UINT:
			case DXGI_FORMAT_R32G32_SINT:
			case DXGI_FORMAT_R32G8X24_TYPELESS:
			case DXGI_FORMAT_D32_FLOAT_S8X24_UINT:
			case DXGI_FORMAT_R32_FLOAT_X8X24_TYPELESS:
			case DXGI_FORMAT_X32_TYPELESS_G8X24_UINT:
				return 64;

			case DXGI_FORMAT_R10G10B10A2_TYPELESS:
			case DXGI_FORMAT_R10G10B10A2_UNORM:
			case DXGI_FORMAT_R10G10B10A2_UINT:
			case DXGI_FORMAT_R11G11B10_FLOAT:
			case DXGI_FORMAT_R8G8B8A8_TYPELESS:
			case DXGI_FORMAT_R8G8B8A8_UNORM:
			case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
			case DXGI_FORMAT_R8G8B8A8_UINT:
			case DXGI_FORMAT_R8G8B8A8_SNORM:
			case DXGI_FORMAT_R8G8B8A8_SINT:
			case DXGI_FORMAT_R16G16_TYPELESS:
			case DXGI_FORMAT_R16G16_FLOAT:
			case DXGI_FORMAT_R16G16_UNORM:
			case DXGI_FORMAT_R16G16_UINT:
			case DXGI_FORMAT_R16G16_SNORM:
			case DXGI_FORMAT_R16G16_SINT:
			case DXGI_FORMAT_R32_TYPELESS:
			case DXGI_FORMAT_D32_FLOAT:
			case DXGI_FORMAT_R32_FLOAT:
			case DXGI_FORMAT_R32_UINT:
			case DXGI_FORMAT_R32_SINT:
			case DXGI_FORMAT_R24G8_TYPELESS:
			case DXGI_FORMAT_D24_UNORM_S8_UINT:
			case DXGI_FORMAT_R24_UNORM_X8_TYPELESS:
			case DXGI_FORMAT_X24_TYPELESS_G8_UINT:
			case DXGI_FORMAT_R9G9B9E5_SHAREDEXP:
			case DXGI_FORMAT_R8G8_B8G8_UNORM:
			case DXGI_FORMAT_G8R8_G8B8_UNORM:
			case DXGI_FORMAT_B8G8R8A8_UNORM:
			case DXGI_FORMAT_B8G8R8X8_UNORM:
			case DXGI_FORMAT_R10G10B10_XR_BIAS_A2_UNORM:
			case DXGI_FORMAT_B8G8R8A8_TYPELESS:
			case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
			case DXGI_FORMAT_B8G8R8X8_TYPELESS:
			case DXGI_FORMAT_B8G8R8X8_UNORM_SRGB:
				return 32;

			case DXGI_FORMAT_R8G8_TYPELESS:
			case DXGI_FORMAT_R8G8_UNORM:
			case DXGI_FORMAT_R8G8_UINT:
			case DXGI_FORMAT_R8G8_SNORM:
			case DXGI_FORMAT_R8G8_SINT:
			case DXGI_FORMAT_R16_TYPELESS:
			case DXGI_FORMAT_R16_FLOAT:
			case DXGI_FORMAT_D16_UNORM:
			case DXGI_FORMAT_R16_UNORM:
			case DXGI_FORMAT_R16_UINT:
			case DXGI_FORMAT_R16_SNORM:
			case DXGI_FORMAT_R16_SINT:
			case DXGI_FORMAT_B5G6R5_UNORM:
			case DXGI_FORMAT_B5G5R5A1_UNORM:
			case DXGI_FORMAT_B4G4R4A4_UNORM:
				return 16;

			case DXGI_FORMAT_R8_TYPELESS:
			case DXGI_FORMAT_R8_UNORM:
			case DXGI_FORMAT_R8_UINT:
			case DXGI_FORMAT_R8_SNORM:
			case DXGI_FORMAT_R8_SINT:
			case DXGI_FORMAT_A8_UNORM:
				return 8;

			case DXGI_FORMAT_R1_UNORM:
				return 1;

			case DXGI_FORMAT_BC1_TYPELESS:
			case DXGI_FORMAT_BC1_UNORM:
			case DXGI_FORMAT_BC1_UNORM_SRGB:
			case DXGI_FORMAT_BC4_TYPELESS:
			case DXGI_FORMAT_BC4_UNORM:
			case DXGI_FORMAT_BC4_SNORM:
				return 4;

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
				return 8;

			default:
				throw std::runtime_error(FmtS("Unsupported DXGI format: %d", fmt));
		}
	}
	constexpr int BytesPerPixel(DXGI_FORMAT fmt)
	{
		return BitsPerPixel(fmt) >> 3;
	}

	// Compile-time type to 'DXGI_FORMAT' conversion
	using dxgi = struct { DXGI_FORMAT format; int size; };
	template <typename T> constexpr dxgi dx_format_v =
		std::is_same_v<uint8_t , T> ? dxgi{DXGI_FORMAT_R8_UINT           , (int)sizeof(uint8_t )} :
		std::is_same_v<uint16_t, T> ? dxgi{DXGI_FORMAT_R16_UINT          , (int)sizeof(uint16_t)} :
		std::is_same_v<uint32_t, T> ? dxgi{DXGI_FORMAT_R32_UINT          , (int)sizeof(uint32_t)} :
		std::is_same_v<v2      , T> ? dxgi{DXGI_FORMAT_R32G32_FLOAT      , (int)sizeof(v2      )} :
		std::is_same_v<v3      , T> ? dxgi{DXGI_FORMAT_R32G32B32_FLOAT   , (int)sizeof(v3      )} :
		std::is_same_v<v4      , T> ? dxgi{DXGI_FORMAT_R32G32B32A32_FLOAT, (int)sizeof(v4      )} :
		std::is_same_v<Colour  , T> ? dxgi{DXGI_FORMAT_R32G32B32A32_FLOAT, (int)sizeof(Colour  )} :
		std::is_same_v<Colour32, T> ? dxgi{DXGI_FORMAT_B8G8R8A8_UNORM    , (int)sizeof(Colour32)} :
		dxgi{DXGI_FORMAT_UNKNOWN, (int)sizeof(char)};
		static_assert(dx_format_v<uint32_t>.format == DXGI_FORMAT_R32_UINT);
		static_assert(dx_format_v<uint16_t>.format == DXGI_FORMAT_R16_UINT);

	// The number of supported quality levels for the given format and sample count
	UINT MultisampleQualityLevels(ID3D12Device* device, DXGI_FORMAT format, UINT sample_count);

	// Returns the number of primitives implied by an index count and geometry topology
	int64_t PrimCount(int64_t icount, ETopo topo);

	// Returns the number of indices implied by a primitive count and geometry topology
	int64_t IndexCount(int64_t pcount, ETopo topo);
	
	// True if 'fmt' is a compression image format
	bool IsCompressed(DXGI_FORMAT fmt);

	// True if 'fmt' has an alpha channel
	bool HasAlphaChannel(DXGI_FORMAT fmt);

	// True if 'fmt' is compatible with UA views
	bool IsUAVCompatible(DXGI_FORMAT fmt);

	// True if 'fmt' is an SRGB format
	bool IsSRGB(DXGI_FORMAT fmt);

	// True if 'fmt' is a depth format
	bool IsDepth(DXGI_FORMAT fmt);

	// Convert 'fmt' to a typeless format
	DXGI_FORMAT ToTypeless(DXGI_FORMAT fmt);

	// Convert 'fmt' to a SRGB format
	DXGI_FORMAT ToSRGB(DXGI_FORMAT fmt);

	// Convert 'fmt' to a UAV compatible format
	DXGI_FORMAT ToUAVCompatable(DXGI_FORMAT fmt);

	// Returns the expected row, slice, and block pitch for a given image width*height*depth and format
	iv3 Pitch(iv3 size, DXGI_FORMAT fmt);
	iv2 Pitch(iv2 size, DXGI_FORMAT fmt);
	iv2 Pitch(D3D12_RESOURCE_DESC const& tdesc);

	// Returns the number of expected mip levels for a given width x height texture
	int MipCount(int w, int h);
	int MipCount(iv2 size);

	// Returns the dimensions of a mip level 'levels' below the given texture size
	iv2 MipDimensions(iv2 size, size_t levels);

	// Returns the number of pixels needed contain the data for a mip chain with 'levels' levels.
	// If 'levels' is 0, all mips down to 1x1 are assumed. Note, size.x should be the pitch rather than width of the texture
	size_t MipChainSize(iv2 size, size_t levels);

	// Helper for checking values are not overwritten in a lookup table
	template <class Table, typename Key, typename Value>
	inline void AddLookup(Table& table, Key const& key, Value const& value)
	{
		PR_ASSERT(PR_DBG_RDR, table.count(key) == 0, "Overwriting an existing lookup table item");
		table[key] = value;
	}

	// Helper for reading values from an unordered map, returning 'def' if not found
	template <class Map, typename Key, typename Value>
	inline Value const& GetOrDefault(Map const& map, Key const& key, Value const& def = Value())
	{
		auto iter = map.find(key);
		return iter == end(map) ? def : static_cast<Value const&>(iter->second);
	}

	// Helper for reading values from an unordered map, and lazy inserting if not found
	template <class Map, typename Key, typename Value, typename Factory>
	inline Value const& GetOrAdd(Map const& map, Key const& key, Factory factory)
	{
		auto iter = map.find(key);
		if (iter == end(map)) iter = map.emplace(key, factory()).first;
		return static_cast<Value const&>(iter->second);
	}

	// Walk a depth first hierarchy calling 'func' for each item in the hierarchy.
	// The caller decides what is pushed to the stack at each level ('Parent').
	// 'hierarchy' is the level of each element in depth first order.
	// 'Func' is 'Parent Func(int idx, Parent const* parent) {... return current; }' (the current value is the parent for the next call).
	template <typename Parent, typename IntRange, std::invocable<int, Parent const*> Func>
	requires
	(
		std::same_as<std::invoke_result_t<Func, int, Parent const*>, Parent> &&
		std::ranges::range<IntRange> &&
		std::integral<std::ranges::range_value_t<IntRange>>
	)
	void WalkHierarchy(IntRange const& hierarchy, Func func)
	{
		// Tree example:
		//        A
		//      /   \
		//     B     C
		//   / | \   |
		//  D  E  F  G
		//  Hierarchy = [A0 B1 D2 E2 F2 C1 G2]
		//  Children are all nodes to the right with level > the current.
		vector<Parent> ancestors = {};
		for (int idx = 0, iend = isize(hierarchy); idx != iend; ++idx)
		{
			// Pop ancestors until we find the parent
			for (; isize(ancestors) > hierarchy[idx];)
				ancestors.pop_back();

			auto const* parent = !ancestors.empty() ? &ancestors.back() : nullptr;
			ancestors.push_back(func(idx, parent));
		}
	}

	// Concept for interface with private data
	template <typename T>
	concept HasPrivateData = requires(T v, Guid const& guid, UINT* mdata_size, UINT cdata_size, void* mdata, void const* cdata)
	{
		{ v.GetPrivateData(guid, mdata_size, mdata) } -> std::same_as<HRESULT>;
		{ v.SetPrivateData(guid, cdata_size, cdata) } -> std::same_as<HRESULT>;
	};

	// Get/Set the debug name
	template <HasPrivateData T> char const* DebugName(T const* res)
	{
		static thread_local char existing[256];

		UINT size(sizeof(existing) - 1);
		if (const_cast<T*>(res)->GetPrivateData(WKPDID_D3DDebugObjectName, &size, &existing[0]) != DXGI_ERROR_NOT_FOUND)
			existing[size] = 0;
		else
			existing[0] = 0;

		return &existing[0];
	}
	template <HasPrivateData T> void DebugName(T* res, std::string_view name)
	{
		Check(res->SetPrivateData(WKPDID_D3DDebugObjectName, s_cast<UINT>(name.size()), name.data()));
	}
	template <HasPrivateData T> char const* DebugName(D3DPtr<T>& res)
	{
		return DebugName(res.get());
	}
	template <HasPrivateData T> void DebugName(D3DPtr<T>& res, std::string_view name)
	{
		DebugName(res.get(), name);
	}

	// Get/Set the debug colour
	inline static GUID const Guid_DebugColour = { 0x0405DEE4, 0xADF7, 0x4A27, { 0xBF, 0x37, 0x0B, 0x37, 0x28, 0x39, 0x39, 0x17 } };
	template <HasPrivateData T> Colour32 DebugColour(T const* res)
	{
		extern GUID const Guid_DebugColour;

		Colour32 existing = {};
		UINT size = sizeof(existing);
		if (const_cast<T*>(res)->GetPrivateData(Guid_DebugColour, &size, &existing) != DXGI_ERROR_NOT_FOUND)
			return existing;

		return Colour32Zero;
	}
	template <HasPrivateData T> void DebugColour(T* res, Colour32 colour)
	{
		extern GUID const Guid_DebugColour;
		Check(res->SetPrivateData(Guid_DebugColour, sizeof(colour), &colour));
	}
	template <HasPrivateData T> Colour32 DebugColour(D3DPtr<T>& res)
	{
		return DebugColour(res.get());
	}
	template <HasPrivateData T> void DebugColour(D3DPtr<T>& res, Colour32 colour)
	{
		DebugColour(res.get(), colour);
	}

	// Get/Set the default state for a resource
	inline static GUID const Guid_DefaultResourceState = { 0x5DFA5A73, 0xA8A0, 0x466B, { 0xA1, 0x0A, 0x3E, 0x3A, 0x35, 0x87, 0x5B, 0xB3 } };
	D3D12_RESOURCE_STATES DefaultResState(ID3D12Resource const* res);
	void DefaultResState(ID3D12Resource* res, D3D12_RESOURCE_STATES state);

	// Parse an embedded resource string of the form: "@<hmodule|module_name>:<res_type>:<res_name>"
	void ParseEmbeddedResourceUri(std::wstring const& uri, HMODULE& hmodule, wstring32& res_type, wstring32& res_name);

	// Return an ordered list of file paths based on 'pattern'
	vector<std::filesystem::path> PatternToPaths(std::filesystem::path const& dir, char8_t const* pattern);
}