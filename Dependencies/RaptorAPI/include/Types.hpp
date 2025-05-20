/*!
* @file Types.hpp
* This file contains basic type definitions.
* @copyright Haption SA
*/

#ifndef BASIC_TYPES_HH_
#define BASIC_TYPES_HH_

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>

#ifndef WIN32
#include <stdint.h>
#endif

namespace HAPTION
{
	#ifndef FLOAT32_T
	#define FLOAT32_T
	/// @brief Single-precision floating-point number stored in 32 bits.
	typedef float float32_t;
	#endif

	/// @brief Double-precision floating-point number stored in 64 bits.
	typedef double float64_t;

#ifdef WIN32
	#ifndef UINT16_T
	#define UINT16_T
	/// @brief Unsigned short integer stored in 16 bits.
	typedef unsigned short uint16_t;
	#endif

	/// @brief Unsigned integer stored in 32 bits.
	typedef unsigned int uint32_t;

	/// @brief Signed integer stored in 32 bits.
	typedef int int32_t;

	/// @brief Signed long integer stored in 64 bits.
	typedef long long int64_t;

	/// @brief Unsigned long integer stored in 64 bits.
	//typedef unsigned long uint64_t;

	/// @brief Platform-neutral data unit stored in 8 bits.
	typedef unsigned char Byte;
#elif defined LINUX
	/// @brief Unsigned short integer stored in 16 bits.
	typedef ::uint16_t uint16_t;

	/// @brief Unsigned integer stored in 32 bits.
	typedef ::uint32_t uint32_t;

	/// @brief Signed integer stored in 32 bits.
	typedef ::int32_t int32_t;

	/// @brief Signed long integer stored in 64 bits.
	typedef ::int64_t int64_t;

	/// @brief Unsigned long integer stored in 64 bits.
	//typedef ::uint64_t uint64_t;

	/// @brief Platform-neutral data unit stored in 8 bits.
	typedef ::uint8_t Byte;
#else
    #ifndef UINT16_T
    #define UINT16_T
    /// @brief Unsigned short integer stored in 16 bits.
    typedef unsigned short uint16_t;
    #endif

    /// @brief Unsigned integer stored in 32 bits.
    typedef unsigned int uint32_t;

    /// @brief Signed integer stored in 32 bits.
    typedef int int32_t;

    /// @brief Signed long integer stored in 64 bits.
    typedef long long int64_t;

    /// @brief Platform-neutral data unit stored in 8 bits.
    typedef unsigned char Byte;

    /// @brief Platform-neutral data unit stored in 8 bits.
    typedef unsigned char BYTE;

    /// @brief Unsigned short integer stored in 16 bits.
    typedef unsigned short WORD;
#endif

	/// @brief Floating-point epsilon used for minimum-value comparisons.
	constexpr float32_t EPSILON{ 0.000001F };
}

#endif /* BASIC_TYPES_H_ */
