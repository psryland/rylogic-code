//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	namespace vertex_stream
	{
		// Return the semantic id used for an additional texture-coordinate channel.
		inline RdrId TexCoord(int channel)
		{
			if (channel <= 0)
				throw std::runtime_error("Unsupported texture-coordinate stream channel");

			auto seed = hash::HashCT("TexCoord");
			return RdrId(hash::Hash32CT(s_cast<uint32_t>(channel), seed));
		}
	}

	struct VertexStreamDesc
	{
		using InitData = std::span<std::byte const>;

		RdrId m_semantic; // The logical meaning of the stream.
		int64_t m_count;  // Number of elements in the stream.
		int m_stride;     // Size in bytes of one element.
		InitData m_data;  // Optional initialisation data.
		string32 m_name;  // Debugging name for the stream.

		VertexStreamDesc()
			: m_semantic()
			, m_count()
			, m_stride()
			, m_data()
			, m_name()
		{}

		// Set the stream semantic id.
		VertexStreamDesc& semantic(RdrId semantic)
		{
			m_semantic = semantic;
			return *this;
		}

		// Set the stream element count, stride, and optional initialisation data.
		VertexStreamDesc& data(int64_t count, int stride, InitData data)
		{
			m_count = count;
			m_stride = stride;
			m_data = data;
			return *this;
		}

		// Set the debugging name for the stream.
		VertexStreamDesc& name(std::string_view name)
		{
			m_name = name;
			return *this;
		}
	};

	// A model-owned vertex stream for additional data such as texture coordinates, tangents, etc.
	struct VertexStream
	{
		RdrId m_semantic;             // The logical meaning of the stream.
		int64_t m_count;              // Number of elements in the stream.
		int m_stride;                 // Size in bytes of one element.
		D3DPtr<ID3D12Resource> m_res; // GPU buffer containing the stream data.
		Descriptor m_srv;             // Shader-resource view for binding the stream.
		string32 m_name;              // Debugging name for the stream.

		VertexStream()
			: m_semantic()
			, m_count()
			, m_stride()
			, m_res()
			, m_srv()
			, m_name()
		{}
	};
}
