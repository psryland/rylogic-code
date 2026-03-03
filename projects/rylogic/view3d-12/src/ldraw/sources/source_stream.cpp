//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "view3d-12/src/ldraw/sources/source_stream.h"
#include "pr/view3d-12/ldraw/ldraw_object.h"
#include "pr/view3d-12/ldraw/ldraw_reader_text.h"
#include "pr/view3d-12/ldraw/ldraw_reader_binary.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"

namespace pr::rdr12::ldraw
{
	SourceStream::SourceStream(Guid const* context_id, Renderer* rdr, Socket&& socket, sockaddr_in addr)
		: SourceBase(context_id)
		, m_rdr(rdr)
		, m_socket(std::move(socket))
		, m_thread()
		, m_mutex()
		, m_mode(EMode::Text)
	{
		m_name = std::format("{}:{}", network::GetIPAddress(addr), network::GetPort(addr));

		// Start a thread to receive incoming data
		m_thread = std::jthread([this]()
		{
			threads::SetCurrentThreadName(m_name);
			try
			{
				// Consume data from the socket
				byte_data<4> buffer(65536);
				for (int bytes_read = 0; !m_thread.get_stop_token().stop_requested();)
				{
					// Timeout on select means no more data is available.
					if (!network::SelectToRecv(m_socket, 100))
						continue;

					// Read into '&buffer[bytes_read]'.
					// Reading zero bytes indicates the socket has been closed gracefully.
					int read = ::recv(m_socket, buffer.data<char>() + bytes_read, s_cast<int>(buffer.size<char>() - bytes_read), 0);
					network::Check(read == 0 || read != SOCKET_ERROR);
					if (read == 0)
						break;

					bytes_read += read;

					// Parse the data by batches of sections. Find the range of whole sections to consume.
					auto [consumed, required] =
						m_mode == EMode::Text ? ConsumeText(buffer, bytes_read) :
						m_mode == EMode::Binary ? ConsumeBinary(buffer, bytes_read) :
						throw std::runtime_error("Unsupported format");

					// If sections where consumed, remove the data from 'buffer'
					if (consumed != 0)
					{
						// Move any remaining data to the front
						memmove(buffer.data(), buffer.data() + consumed, s_cast<size_t>(bytes_read - consumed));
						bytes_read -= consumed;
					}

					// Otherwise, if 0 bytes can be consumed, check the buffer is big enough and the partial data is not invalid
					else if (required > isize(buffer))
					{
						// If there's not enough data yet, keep waiting. Make sure the buffer is big enough
						buffer.resize(std::max(buffer.size(), s_cast<size_t>(required)));
					}
				}
			}
			catch (std::exception const& ex)
			{
				// log?
				OutputDebugStringA(ex.what());
			}

			// Make this source as invalid
			m_socket = nullptr;

			// Signal that the connection was lost
			Notify(shared_from_this(), { {}, ENotifyReason::Disconnected, {}, nullptr });
		});
	}
	SourceStream::SourceStream(SourceStream&& rhs) noexcept
		: SourceBase(rhs)
		, m_rdr(rhs.m_rdr)
		, m_socket()
		, m_thread()
		, m_mutex()
		, m_mode(rhs.m_mode)
	{
		std::swap(m_socket, rhs.m_socket);
		std::swap(m_thread, rhs.m_thread);
	}
	SourceStream& SourceStream::operator =(SourceStream&& rhs) noexcept
	{
		if (this == &rhs) return *this;
		SourceBase::operator=(rhs);
		std::swap(m_rdr, rhs.m_rdr);
		std::swap(m_socket, rhs.m_socket);
		std::swap(m_thread, rhs.m_thread);
		std::swap(m_mode, rhs.m_mode);
		return *this;
	}
	SourceStream::~SourceStream()
	{
		// Stop the thread
		m_thread.request_stop();
		if (m_thread.joinable())
			m_thread.join();
	}

	// Receive binary ldraw script. Returns the number of bytes consumed
	std::tuple<int,int> SourceStream::ConsumeBinary(byte_data<4>& buffer, int& bytes_read)
	{
		int consume = 0;
		int required = 0;

		// Find the range of complete sections that can be consumed
		for (; bytes_read - consume >= int(sizeof(ldraw::SectionHeader));)
		{
			auto const& header = buffer.at_byte_ofs<ldraw::SectionHeader>(s_cast<size_t>(consume));

			// If the first 4 bytes are not a keyword, then flush the buffer
			if (!ldraw::EKeyword_::IsValue(s_cast<int>(header.m_keyword)))
			{
				bytes_read = 0;
				consume = 0;
				break;
			}

			// The next 4 bytes should be the section size in bytes (excluding the header)
			auto size = s_cast<int>(header.m_size + sizeof(ldraw::SectionHeader));
			if (consume + size > bytes_read)
			{
				// We can only use up to 'consume' bytes
				break;
			}

			// The next section is complete, include it for consumption
			consume += size;

			// The TextStream command interrupts consuming data
			// because the following data is expected to be in binary mode
			if (header.m_keyword == EKeyword::TextStream)
			{
				m_mode = EMode::Text;
				break;
			}
		}

		// If there are sections to consume, do so
		if (consume != 0)
		{
			mem_istream<char> strm(buffer.data(), consume);
			BinaryReader reader(strm, m_name.c_str(), { this, OnReportError }, { this, OnProgress });
			auto out = ldraw::Parse(*m_rdr, reader, m_context_id);
			if (out)
			{
				auto src = shared_from_this();
				m_rdr->RunOnMainThread([src, out = std::move(out)]() mutable noexcept
				{
					src->Notify(src, { std::move(out), ENotifyReason::LoadComplete, EDataChangeTrigger::NewData, nullptr });
				});
			}
		}

		// Otherwise, if 0 bytes can be consumed, check the buffer is big enough and the partial data is not invalid
		else if (bytes_read >= sizeof(ldraw::SectionHeader))
		{
			auto const& header = buffer.at_byte_ofs<ldraw::SectionHeader>(s_cast<size_t>(0));
			required = header.m_size + sizeof(ldraw::SectionHeader);
		}

		return { consume, required };
	}

	// Receive text ldraw script
	std::tuple<int,int> SourceStream::ConsumeText(byte_data<4>& buffer, int& bytes_read)
	{
		int consume = 0;
		int required = 0;

		// Find the range of complete sections that can be consumed
		for (; bytes_read - consume > 0;)
		{
			script::StringSrc src(std::string_view{ &buffer.at_byte_ofs<char const>(consume), s_cast<size_t>(bytes_read - consume) });
			auto remaining0 = src.size_in_bytes();

			str::AdvanceToNonDelim(src);

			// Expect to start with a keyword, if not then flush the buffer
			string32 id; std::optional<EKeyword> kw;
			if (*src != '*' || !str::ExtractIdentifier(id, ++src) || !(kw = ldraw::EKeyword_::TryParse(id, false)))
			{
				bytes_read = 0;
				consume = 0;
				break;
			}

			// Scan forward to the '{' character then see if a complete section is within the buffer
			StringProxyForLength<wchar_t> section;
			if (!str::Advance(src, [](auto ch) { return ch != '{'; }) || !str::ExtractSection(section, src))
			{
				// We can only use up to 'consume' bytes
				break;
			}

			// The next section is complete, include it for consumption
			consume += s_cast<int>(remaining0 - src.size_in_bytes());

			// The BinaryStream command interrupts consuming data
			// because the following data is expected to be in binary mode
			if (*kw == EKeyword::BinaryStream)
			{
				m_mode = EMode::Binary;
				break;
			}
		}

		// If there are sections to consume, do so
		if (consume != 0)
		{
			mem_istream<char> strm(buffer.data(), consume);
			TextReader reader(strm, m_name.c_str(), EEncoding::utf8, { this, OnReportError }, { this, OnProgress });
			auto out = ldraw::Parse(*m_rdr, reader, m_context_id);
			if (out)
			{
				auto src = shared_from_this();
				m_rdr->RunOnMainThread([src, out = std::move(out)]() mutable noexcept
				{
					src->Notify(src, { std::move(out), ENotifyReason::LoadComplete, EDataChangeTrigger::NewData, nullptr });
				});
			}
		}

		// Otherwise, if 0 bytes can be consumed, check the buffer is big enough and the partial data is not invalid
		else if (bytes_read == isize(buffer))
		{
			required = 2 * isize(buffer);
		}

		return { consume, required };
	}
}
