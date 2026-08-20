//*****************************************
// Email
//	Copyright (c) Rylogic 2019
//*****************************************

#pragma once
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <array>
#include <windows.h>
#include <winsock2.h>
#include "pr/common/fmt.h"
#include "pr/network/winsock.h"

namespace pr::network
{
	class Email
	{
		// Notes:
		//  - This is the old way to send emails. It doesn't really work nowadays because
		//    email servers require SSL and don't allow sending from unknown accounts.
		//  - Library options:
		//      VMine C++ MIME Library (GNU GPL Licence)
		//      libcurl (MIT licence) https://curl.haxx.se/libcurl/
		//      libquickmail https://sourceforge.net/projects/libquickmail/


		Winsock m_winsock;
		std::string m_to_addr;
		std::string m_from_addr;
		std::string m_subject;
		std::string m_body;

	public:
		Email()
			:m_winsock()
			,m_to_addr()
			,m_from_addr()
			,m_subject()
			,m_body()
		{}

		// Email recipient
		Email& To(std::string_view recipient)
		{
			m_to_addr = recipient;
			return *this;
		}

		// Email sender
		Email& From(std::string_view sender)
		{
			m_from_addr = sender;
			return *this;
		}

		// Email subject
		Email& Subject(std::string_view subject)
		{
			m_subject = subject;
			return *this;
		}

		// Email body
		Email& Body(std::string_view body)
		{
			m_body = body;
			return *this;
		}

		// Post this email
		// E.g. Send("smtp.gmail.com", "587")
		void Send(char const* smtp_server_name, char const* port = nullptr) const
		{
			using namespace std::literals;

			// Lookup email server's IP address and port.
			auto addr = GetAddress(smtp_server_name, "mail");
			if (port != nullptr)
				addr.sin_port = (USHORT)atoi(port);

			// Create a TCP/IP socket, no specific protocol
			auto server = Socket(PF_INET, SOCK_STREAM, 0);
			if (server == INVALID_SOCKET)
				throw std::runtime_error("Failed to open socket to mail server");

			int result;

			// Connect the Socket
			result = connect(server, reinterpret_cast<SOCKADDR const*>(&addr), sizeof(addr));
			Check(result != SOCKET_ERROR, "Failed to connect to the email server");

			// Receive initial response from SMTP server
			std::array<char, 4096> buffer;
			result = recv(server, buffer.data(), int(buffer.size()), 0);
			Check(result != SOCKET_ERROR, "'recv' error");

			std::string msg;
			msg.reserve(4096);

			// Send 'HELO server.com'
			msg.clear(); msg.append("HELO ").append(smtp_server_name).append("\r\n");
			result = send(server, msg.data(), int(msg.size()), 0);
			Check(result != SOCKET_ERROR, "'send' HELO error");
			result = recv(server, buffer.data(), int(buffer.size()), 0);
			Check(result != SOCKET_ERROR, "'recv' HELO error");

			// Send MAIL FROM: <sender@mydomain.com>
			msg.clear(); msg.append("MAIL FROM:<").append(m_from_addr).append(">\r\n");
			result = send(server, msg.data(), int(msg.size()), 0);
			Check(result != SOCKET_ERROR, "'send' MAIL FROM error");
			result = recv(server, buffer.data(), int(buffer.size()), 0);
			Check(result != SOCKET_ERROR, "'recv' MAIL FROM error");

			// Send RCPT TO: <receiver@domain.com>
			msg.clear(); msg.append("RCPT TO:<").append(m_to_addr).append(">\r\n");
			result = send(server, msg.data(), int(msg.size()), 0);
			Check(result != SOCKET_ERROR, "'send' RCPT TO error");
			result = recv(server, buffer.data(), int(buffer.size()), 0);
			Check(result != SOCKET_ERROR, "'recv' RCPT TO error");

			// Send DATA
			msg.clear(); msg.append("DATA\r\n");
			result = send(server, msg.data(), int(msg.size()), 0);
			Check(result != SOCKET_ERROR, "'send' DATA error");
			result = recv(server, buffer.data(), int(buffer.size()), 0);
			Check(result != SOCKET_ERROR, "'recv' DATA error");

			// Send all lines of message body
			for (std::stringstream ss(m_body); ss.good();)
			{
				msg.resize(msg.capacity());
				auto len = ss.getline(msg.data(), int(msg.size())-2).gcount();
				msg.resize(static_cast<size_t>(len));
				msg.append("\r\n");

				result = send(server, msg.data(), int(msg.size()), 0);
				Check(result != SOCKET_ERROR, "'send' message-line error");
			}

			// Send blank line and a period
			msg.clear(); msg.append("\r\n.\r\n");
			result = send(server, msg.data(), int(msg.size()), 0);
			Check(result != SOCKET_ERROR, "'send' end-message");
			result = recv(server, buffer.data(), int(buffer.size()), 0);
			Check(result != SOCKET_ERROR, "'recv' end-message");

			// Send QUIT
			msg.clear(); msg.append("QUIT\r\n");
			result = send(server, msg.data(), int(msg.size()), 0);
			Check(result != SOCKET_ERROR, "'send' QUIT error");
			result = recv(server, buffer.data(), int(buffer.size()), 0);
			Check(result != SOCKET_ERROR, "'recv' QUIT error");
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"

namespace pr::network
{
	PRUnitTest(EmailTests, Quick)
	{
		//Email()
		//	.To("psryland@gmail.com")
		//	.From("psryland@yahoo.co.nz")
		//	.Subject("Test Email")
		//	.Body("This is a\r\ntest email")
		//	.Send("smtp.gmail.com", "25");//:587");
	}
}

#endif
