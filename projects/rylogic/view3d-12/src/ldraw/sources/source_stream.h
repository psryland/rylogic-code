//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "view3d-12/src/ldraw/sources/source_base.h"

namespace pr::rdr12::ldraw
{
	struct SourceStream : SourceBase
	{
		using Socket = network::Socket;
		using EMode = enum class EMode { Auto, Text, Binary };

		Renderer*    m_rdr;     // The owning renderer
		Socket       m_socket;  // A non-owning reference to the network connection
		std::jthread m_thread;  // Thread that receives data from the socket
		std::mutex   m_mutex;   // For synchronising access to 'm_output'
		EMode        m_mode;    // The format of the data to expect

		SourceStream(Guid const* context_id, Renderer* rdr, Socket&& socket, sockaddr_in addr);
		SourceStream(SourceStream&& rhs) noexcept;
		SourceStream(SourceStream const&) = delete;
		SourceStream& operator =(SourceStream&& rhs) noexcept;
		SourceStream& operator =(SourceStream const&) = delete;
		~SourceStream();

		// Auto-detect the stream format from the first byte of data
		static EMode DetectMode(byte_data<4> const& buffer, int bytes_read);

		std::tuple<int,int> ConsumeBinary(byte_data<4>& buffer, int& bytes_read);
		std::tuple<int,int> ConsumeText(byte_data<4>& buffer, int& bytes_read);
	};
}
