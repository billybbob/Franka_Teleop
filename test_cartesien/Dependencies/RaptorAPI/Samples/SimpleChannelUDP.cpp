/**
 * @file SimpleChannelUDP.hpp
 * This file holds the implementation of the class SimpleChannelInterface for a simple UDP channel.
 * @copyright Haption SA
 * @date 2021
 * @author Jerome Perret
 */

#ifdef WIN32 // From https://docs.microsoft.com/en-us/windows/win32/api/winsock/nf-winsock-wsastartup
#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>

// Need to link with Ws2_32.lib
#pragma comment(lib, "ws2_32.lib")

#define socklen_t int

#else

#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define SOCKET int
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket(s) close(s)
#define sprintf_s(a,b,c,...) sprintf((a),(c),##__VA_ARGS__)
#define strtok_s(a,b,c) strtok((a),(b))
typedef struct sockaddr SOCKADDR;

#endif

#include <iostream>
#include <sstream>
#include "SimpleChannelInterface.hpp"


namespace HAPTION
{
    namespace SimpleChannel
    {
        /// @brief SimpleChannelUDP interface class
        class SimpleChannelUDP final : public SimpleChannelInterface
        {
        public:
            /// @brief Get the last error details as a string
            /// @param[out] oMessage Description of the last error that occurred
            /// @return SCE_NOERROR
			ErrorCode GetLastErrorMessage(std::string& oMessage) noexcept override;

            /// @brief Open the channel
            /// @param[in] iOpenParameters String in the form of assignments "field=value" separated by commas, colons or semicolons <br/>
            /// Example for Ethernet/UDP: "localip=127.0.0.1:localport=12120:remoteip=192.168.100.53:remoteport=5000"
            /// @return SCE_NOERROR in case of success <br/>
            /// SCE_WRONGPARAMETER if one of the parameters is missing or wrong <br/>
            /// SCE_UNAVAILABLE if the communication transport is not available or not responding (e.g. the port is already in use)
            ErrorCode Open(std::string const& iOpenParameters) noexcept override;

            /// @brief Check that the channel is open
            /// @return true if open
            bool IsOpen() noexcept override;

            /// @brief Close the channel
            /// @return SCE_NOERROR in case of success <br/>
            /// SCE_UNAVAILABLE if the channel is not open
            ErrorCode Close() noexcept override;

            /// @brief Send data on the channel
            /// @param[in] iSize Size of the data to be sent, in Bytes
            /// @param[in] iData Buffer holding the data
            /// @return SCE_NOERROR in case of success <br/>
            /// SCE_UNAVAILABLE if the channel is not open <br/>
            /// SCE_DISCONNECTED in case of disconnection <br/>
            /// SCE_SHORTOPERATION if not all the data could be sent <br/>
            /// SCE_OTHER in case of an unidentified error
            ErrorCode Send(size_t const iSize, std::array<Byte8_t, PACKET_SIZE> const& iData) noexcept override;

            /// @brief Wait for data availability on the channel
            /// @note If called with 0U, then returns SCE_NOERROR if data is already available
            /// @param[in] iTimeoutMS Timeout in milliseconds
            /// @return SCE_NOERROR in case of success <br/>
            /// SCE_UNAVAILABLE if the channel is not open <br/>
            /// SCE_DISCONNECTED in case of disconnection <br/>
            /// SCE_UNSUPPORTED in case the transport does not support waiting <br/>
            /// SCE_OTHER in case of an unidentified error
            ErrorCode Wait(size_t iTimeoutMS) noexcept override;

            /// @brief Receive data from the channel
            /// @param[out] iSize Size of the data to be received, in Bytes
            /// @param[out] oSize Size of the data received, in Bytes
            /// @param[out] oData Buffer where the data shall be written (does not need to be set to zeroes)
            /// @param[in] iTimeoutMS Timeout in milliseconds
            /// @return SCE_NOERROR in case of success <br/>
            /// SCE_UNAVAILABLE if the channel is not open <br/>
            /// SCE_DISCONNECTED in case of disconnection <br/>
            /// SCE_OTHER in case of an unidentified error
            ErrorCode Receive(size_t const& iSize, size_t& oSize, std::array<Byte8_t, PACKET_SIZE>& oData, size_t const iTimeoutMS = 0U) noexcept override;

        private:
			/// @brief Last error details
			std::string m_errorMessage;
		
            /// @brief The socket itself
			SOCKET m_socket{ INVALID_SOCKET };

            /// @brief The local socket address and port
			sockaddr_in m_local;

            /// @brief The remote socket address and port
			sockaddr_in m_remote;

            /// @brief Flag for waiting message(s)
            bool m_available{ false };
        };

        /// @brief Get the last error details as a string
        /// @param[out] oMessage Description of the last error that occurred
        /// @return SCE_NOERROR
		ErrorCode SimpleChannelUDP::GetLastErrorMessage(std::string& oMessage) noexcept
		{ 
			oMessage = m_errorMessage;
            return ErrorCode::SCE_NOERROR;
		}

        /// @brief Open the channel
        /// @param[in] iOpenParameters String in the form of assignments "field=value" separated by commas, colons or semicolons <br/>
        /// Example for Ethernet/UDP: "localip=127.0.0.1:localport=12120:remoteip=192.168.100.53:remoteport=5000"
        /// @return SCE_NOERROR in case of success <br/>
        /// SCE_WRONGPARAMETER if one of the parameters is missing or wrong <br/>
        /// SCE_UNAVAILABLE if the communication transport is not available or not responding (e.g. the port is already in use)
		ErrorCode SimpleChannelUDP::Open(std::string const& iOpenParameters) noexcept
		{
			// Create an l-value copy of the parameters
			std::string fields{ iOpenParameters };
			// Set default values in network convention
			uint32_t local_ip { htonl(INADDR_ANY) };
			uint16_t local_port{ htons(0U) };
			uint32_t remote_ip { htonl(INADDR_ANY) };
			uint16_t remote_port { htons(0U) };
			// Parse fields
			while(true)
			{
				// Extract token (until first character "=")
				size_t equal{ fields.find('=') };
				if (equal == std::string::npos)
				{
					break;
				}
				std::string const token{ fields.substr(0U, equal) };
				// Extract value (till next separator or end of line)
				std::string valueString;
				std::string const separators{ ",;:/" };
				size_t const sep{ fields.find_first_of(separators) };
				if (sep != std::string::npos)
				{
					valueString = fields.substr(equal + 1U, sep - equal - 1U);
				}
				else
				{
					valueString = fields.substr(equal + 1U);
				}
				// Test token name
				std::string const localipStr{ "localip" };
				std::string const localportStr{ "localport" };
				std::string const remoteipStr{ "remoteip" };
				std::string const remoteportStr{ "remoteport" };
				if (token.compare(localipStr) == 0)
				{
                    local_ip = inet_addr(valueString.c_str());
				}
				else if (token.compare(localportStr) == 0)
				{
					std::stringstream valueStream{ valueString };
					uint16_t value{ 0U };
					valueStream >> value;
					if (valueStream.fail())
					{
						m_errorMessage = "Error in SimpleChannelUDP::Open() : Field \"localport\" has incompatible value";
						return ErrorCode::SCE_WRONGPARAMETER;
					}
					local_port = htons(value);
				}
				else if (token.compare(remoteipStr) == 0)
				{
                    remote_ip = inet_addr(valueString.c_str());
				}
				else if (token.compare(remoteportStr) == 0)
				{
					std::stringstream valueStream{ valueString };
					uint16_t value{ 0U };
					valueStream >> value;
					if (valueStream.fail())
					{
						m_errorMessage = "Error in SimpleChannelUDP::Open() : Field \"remoteport\" has incompatible value";
						return ErrorCode::SCE_WRONGPARAMETER;
					}
					remote_port = htons(value);
				}
				// Skip to next token
				if (sep == std::string::npos)
				{
					break;
				}
				fields = fields.substr(sep + 1U);
			}

#ifdef WIN32 // From https://docs.microsoft.com/en-us/windows/win32/api/winsock/nf-winsock-wsastartup
            WORD wVersionRequested;
            WSADATA wsaDataVar;
            int32_t err;

            /* Use the MAKEWORD(lowbyte, highbyte) macro declared in Windef.h */
            wVersionRequested = MAKEWORD(2, 2);

            err = WSAStartup(wVersionRequested, &wsaDataVar);
            if (err != 0) {
                /* Tell the user that we could not find a usable */
                /* Winsock DLL.                                  */
                std::cerr << "WSAStartup failed with error: " << err << std::endl;
				m_errorMessage = "Error in SimpleChannelUDP::Open() : WSAStartup failed with error " + std::to_string(err);
                return ErrorCode::SCE_UNAVAILABLE;
            }
#endif
            // Create the socket
			m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			if (m_socket == INVALID_SOCKET)
			{
#ifdef WIN32
                m_errorMessage = "Error in SimpleChannelUDP::Open() : socket failed with "
                        + std::to_string(WSAGetLastError());
#else
                m_errorMessage = "Error in SimpleChannelUDP::Open() : socket failed with "
                        + std::to_string(errno);
#endif
				return ErrorCode::SCE_UNAVAILABLE;
			}

			// Bind the socket locally
            m_local.sin_family = AF_INET;
            m_local.sin_port = local_port;
            m_local.sin_addr.s_addr = local_ip;
			if (bind(m_socket, (sockaddr *)(&m_local), sizeof(struct sockaddr_in)) != 0)
			{
				if(closesocket(m_socket) !=0)
				{ }
				m_socket = 0U;
#ifdef WIN32
                m_errorMessage = "Error in SimpleChannelUDP::Open() : bind failed with "
                        + std::to_string(WSAGetLastError());
#else
                m_errorMessage = "Error in SimpleChannelUDP::Open() : bind failed with "
                        + std::to_string(errno);
#endif
				return ErrorCode::SCE_UNAVAILABLE;
			}

            // Set the remote host
            m_remote.sin_family = AF_INET;
            m_remote.sin_port = remote_port;
            m_remote.sin_addr.s_addr = remote_ip;

            // Reset available flag
            m_available = false;

            return ErrorCode::SCE_NOERROR;
        }

        /// @brief Check that the channel is open
        /// @return true if open
        bool SimpleChannelUDP::IsOpen() noexcept
        {
            return (m_socket != INVALID_SOCKET);
        }

        /// @brief Close the channel
        /// @return SCE_NOERROR in case of success <br/>
        /// SCE_UNAVAILABLE if the channel is not open
        ErrorCode SimpleChannelUDP::Close() noexcept
        {
            if (m_socket == 0U)
            {
				m_errorMessage = "Error in SimpleChannelUDP::Close() : Channel is not open";
                return ErrorCode::SCE_UNAVAILABLE;
            }
            closesocket(m_socket);
            m_socket = INVALID_SOCKET;
            m_available = false;
#ifdef WIN32
            // Note from MSDN:
            // There must be a call to WSACleanup for each successful call to WSAStartup.
            // Only the final WSACleanup function call performs the actual cleanup.
            // The preceding calls simply decrement an internal reference count in the WS2_32.DLL.
			if (WSACleanup() != 0)
			{
			}
#endif
            return ErrorCode::SCE_NOERROR;
        }

        /// @brief Send data on the channel
        /// @param[in] iSize Size of the data to be sent, in Bytes
        /// @param[in] iData Buffer holding the data
        /// @return SCE_NOERROR in case of success <br/>
        /// SCE_UNAVAILABLE if the channel is not open <br/>
        /// SCE_DISCONNECTED in case of disconnection <br/>
        /// SCE_SHORTOPERATION if not all the data could be sent <br/>
        /// SCE_OTHER in case of an unidentified error
        ErrorCode SimpleChannelUDP::Send(size_t const iSize, std::array<Byte8_t, PACKET_SIZE> const& iData) noexcept
        {
            if (m_socket == 0U)
            {
				m_errorMessage = "Error in SimpleChannelUDP::Send() : Channel is not open";
                return ErrorCode::SCE_UNAVAILABLE;
            }
			int32_t const result{ sendto(m_socket, (char*)iData.data(), static_cast<int32_t>(iSize), 0, (sockaddr *)&m_remote, sizeof(sockaddr_in)) };
            if (result == SOCKET_ERROR)
            {
#ifdef WIN32
                m_errorMessage = "Error in SimpleChannelUDP::Send() : WSAGetLastError() returns "
                        + std::to_string(WSAGetLastError());
#else
                m_errorMessage = "Error in SimpleChannelUDP::Send() : errno is "
                        + std::to_string(errno);
#endif
                return ErrorCode::SCE_OTHER;
            }
            if (result != static_cast<int32_t>(iSize))
            {
                return ErrorCode::SCE_SHORTOPERATION;
            }
            return ErrorCode::SCE_NOERROR;
        }

        /// @brief Wait for data availability on the channel
        /// @note If called with 0U, then returns SCE_NOERROR if data is already available
        /// @param[in] iTimeoutMS Timeout in milliseconds
        /// @return SCE_NOERROR in case of success <br/>
        /// SCE_UNAVAILABLE if the channel is not open <br/>
        /// SCE_DISCONNECTED in case of disconnection <br/>
        /// SCE_UNSUPPORTED in case the transport does not support waiting <br/>
        /// SCE_OTHER in case of an unidentified error
        ErrorCode SimpleChannelUDP::Wait(size_t iTimeoutMS) noexcept
        {
            if (m_socket == 0U)
            {
				m_errorMessage = "Error in SimpleChannelUDP::Send() : Channel is not open";
                return ErrorCode::SCE_UNAVAILABLE;
            }
			fd_set readfds;
			timeval timeout;
			FD_ZERO(&readfds);
			FD_SET(m_socket, &readfds);
			timeout.tv_sec = static_cast<int64_t>(iTimeoutMS) / 1000;
			timeout.tv_usec = (static_cast<int64_t>(iTimeoutMS) % 1000U) * 1000U;
			int32_t const res{ select(static_cast<int32_t>(m_socket) + 1, &readfds, nullptr, nullptr, &timeout) };
			if (res == 1)
			{
                m_available = true;
				return ErrorCode::SCE_NOERROR;
			}
			if (res == 0)
			{
                m_available = false;
                return ErrorCode::SCE_TIMEOUT;
			}
#ifdef WIN32
            m_errorMessage = "Error in SimpleChannelUDP::Wait() : WSAGetLastError() returns "
                    + std::to_string(WSAGetLastError());
#else
            m_errorMessage = "Error in SimpleChannelUDP::Wait() : errno is "
                    + std::to_string(errno);
#endif
            return ErrorCode::SCE_OTHER;
        }

        /// @brief Receive data from the channel
        /// @param[out] iSize Size of the data to be received, in Bytes
        /// @param[out] oSize Size of the data received, in Bytes
        /// @param[out] oData Buffer where the data shall be written (does not need to be set to zeroes)
        /// @param[in] iTimeoutMS Timeout in milliseconds
        /// @return SCE_NOERROR in case of success <br/>
        /// SCE_UNAVAILABLE if the channel is not open <br/>
        /// SCE_DISCONNECTED in case of disconnection <br/>
        /// SCE_OTHER in case of an unidentified error
        ErrorCode SimpleChannelUDP::Receive(size_t const& iSize, size_t& oSize, std::array<Byte8_t, PACKET_SIZE>& oData, size_t const iTimeoutMS) noexcept
        {
            if (m_socket == 0U)
            {
                return ErrorCode::SCE_UNAVAILABLE;
            }
            if (!m_available)
            {
                // Wait till data is available
				ErrorCode const error{ Wait(iTimeoutMS) };
                if (error != ErrorCode::SCE_NOERROR)
                {
                    return error;
                }
            }
            sockaddr_in from_addr;
            socklen_t from_len = sizeof(sockaddr_in);
			int32_t const result{ recvfrom(m_socket, (char*)(oData.data()), PACKET_SIZE, 0, (sockaddr *)&from_addr, &from_len) };
            m_available = false;
            if (result > 0)
            {
                oSize = static_cast<size_t>(result);
                return ErrorCode::SCE_NOERROR;
            }
            if (result == 0)
            {
                oSize = 0;
                m_errorMessage = "Error in SimpleChannelUDP::Receive : connection closed by peer";
                return ErrorCode::SCE_DISCONNECTED;
            }
#ifdef WIN32
            m_errorMessage = "Error in SimpleChannelUDP::Receive() : WSAGetLastError() returns "
                    + std::to_string(WSAGetLastError());
#else
             m_errorMessage = "Error in SimpleChannelUDP::Receive() : errno is "
                    + std::to_string(errno);
#endif
            return ErrorCode::SCE_OTHER;
        }
    }
}

// @brief Factory for the ProtocolInterface implementation
// @return The new instance of the ProtocolInterface
#ifndef LINUX
// @brief Factory
extern "C" __declspec(dllexport) HAPTION::SimpleChannel::SimpleChannelInterface* __cdecl ChannelFactory()
#else
// @brief Factory
extern "C" HAPTION::SimpleChannel::SimpleChannelInterface* ChannelFactory()
#endif
{
    return new HAPTION::SimpleChannel::SimpleChannelUDP();
}
