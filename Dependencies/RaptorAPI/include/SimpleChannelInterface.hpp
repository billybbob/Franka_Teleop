/**
 * @file SimpleChannelInterface.hpp
 * This file holds the definition of the interface class SimpleChannelInterface.
 * @copyright Haption SA
 * @date 2021
 * @author Jerome Perret
 */

#ifndef H_SIMPLE_CHANNEL_INTERFACE
#define H_SIMPLE_CHANNEL_INTERFACE

#include <string>
#include <array>

namespace HAPTION
{
    namespace SimpleChannel
    {
        /// @brief Max packet size: the Virtuose protocol is constrained to packets 1 KB in size
        constexpr size_t PACKET_SIZE{ 1024U };

        /// @brief Simple byte type, because std::byte is supported only from C++ 2017
        using Byte8_t = unsigned char;

        /// @brief Error code for the SimpleChannelInterface class methods
        enum class ErrorCode: uint16_t
        {
            /// @brief No error
            SCE_NOERROR,
            /// @brief Function not supported
            SCE_UNSUPPORTED,
            /// @brief Some parameter is missing or has an unexpected value
            SCE_WRONGPARAMETER,
            /// @brief The channel is not available, or the method called is not available
            SCE_UNAVAILABLE,
            /// @brief A timeout occurred in the method called
            SCE_TIMEOUT,
            /// @brief The channel has been disconnected by the peer or the infrastructure
            SCE_DISCONNECTED,
            /// @brief Not all bytes could be received or sent
            SCE_SHORTOPERATION,
            /// @brief Any other error
            SCE_OTHER
        };

        /// @brief SimpleChannel interface class (pure virtual), to be implemented as shared library
        class SimpleChannelInterface
        {
        public:
			
			// @brief Destructor must be virtual (rule [AUTOSAR-A12_4_2-a])
			virtual ~SimpleChannelInterface() = default;

            /// @brief Get the last error details as a string
            /// @param[out] oMessage Description of the last error that occurred
            /// @return SCE_NOERROR
			virtual ErrorCode GetLastErrorMessage(std::string& oMessage) noexcept = 0;
			
            /// @brief Open the channel
            /// @param[in] iOpenParameters String in the form of assignments "field=value" separated by commas, colons or semicolons <br/>
            /// Example for Ethernet/UDP: "localip=127.0.0.1:localport=12120:remoteip=192.168.100.53:remoteport=5000"
            /// @return SCE_NOERROR in case of success <br/>
            /// SCE_WRONGPARAMETER if one of the parameters is missing or wrong <br/>
            /// SCE_UNAVAILABLE if the communication transport is not available or not responding (e.g. the port is already in use)
            virtual ErrorCode Open(std::string const& iOpenParameters) noexcept = 0;

            /// @brief Check that the channel is open
            /// @return true if open
            virtual bool IsOpen() noexcept = 0;

            /// @brief Close the channel
            /// @return SCE_NOERROR in case of success <br/>
            /// SCE_UNAVAILABLE if the channel is not open
            virtual ErrorCode Close() noexcept = 0;

            /// @brief Send data on the channel
            /// @param[in] iSize Size of the data to be sent, in Bytes
            /// @param[in] iData Buffer holding the data
            /// @return SCE_NOERROR in case of success <br/>
            /// SCE_UNAVAILABLE if the channel is not open <br/>
            /// SCE_DISCONNECTED in case of disconnection <br/>
            /// SCE_SHORTOPERATION if not all the data could be sent <br/>
            /// SCE_OTHER in case of an unidentified error
            virtual ErrorCode Send(size_t const iSize, std::array<Byte8_t, PACKET_SIZE> const& iData) noexcept = 0;

            /// @brief Wait for data availability on the channel
            /// @note If called with 0U, then returns SCE_NOERROR if data is already available
            /// @param[in] iTimeoutMS Timeout in milliseconds
            /// @return SCE_NOERROR in case of success <br/>
            /// SCE_UNAVAILABLE if the channel is not open <br/>
            /// SCE_DISCONNECTED in case of disconnection <br/>
            /// SCE_UNSUPPORTED in case the transport does not support waiting <br/>
            /// SCE_OTHER in case of an unidentified error
            virtual ErrorCode Wait(size_t iTimeoutMS) noexcept = 0;

            /// @brief Receive data from the channel
            /// @param[out] iSize Size of the data to be received, in Bytes
            /// @param[out] oSize Size of the data received, in Bytes
            /// @param[out] oData Buffer where the data shall be written (does not need to be set to zeroes)
            /// @param[in] iTimeoutMS Timeout in milliseconds
            /// @return SCE_NOERROR in case of success <br/>
            /// SCE_UNAVAILABLE if the channel is not open <br/>
            /// SCE_DISCONNECTED in case of disconnection <br/>
            /// SCE_OTHER in case of an unidentified error
            virtual ErrorCode Receive(size_t const& iSize, size_t& oSize, std::array<Byte8_t, PACKET_SIZE>& oData, size_t const iTimeoutMS = 0U) noexcept = 0;
        };
    }
}

/// @brief Factory for the ProtocolInterface implementation
/// @return The new instance of the ProtocolInterface
#ifndef LINUX
// @brief Factory for the ProtocolInterface implementation
// @return The new instance of the ProtocolInterface
extern "C" __declspec(dllexport) HAPTION::SimpleChannel::SimpleChannelInterface* __cdecl ChannelFactory();
#else
// @brief Factory for the ProtocolInterface implementation
// @return The new instance of the ProtocolInterface
extern "C" HAPTION::SimpleChannel::SimpleChannelInterface* ChannelFactory();
#endif

#endif // H_SIMPLE_CHANNEL_INTERFACE