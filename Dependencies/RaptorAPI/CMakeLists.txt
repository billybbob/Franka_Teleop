#
# File: CMakeLists.txt
# Author: Jerome Perret
# Copyright: Haption SA, 2021
# Note: Permission granted to copy, modify, redistribute, provided the above copyright is mentioned
#

cmake_minimum_required(VERSION 3.10.2)

# Linux only: Identify version of glibc

if(MSVC)
else()
    execute_process(COMMAND /usr/bin/ldd --version
        COMMAND head -n 1
        COMMAND /usr/bin/awk "{print $NF}"
        OUTPUT_STRIP_TRAILING_WHITESPACE
        OUTPUT_VARIABLE GLIBC)
endif()

# TARGET TestCalibration

add_executable(TestCalibration ${CMAKE_CURRENT_SOURCE_DIR}/Samples/TestCalibration.cpp)
target_include_directories(TestCalibration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
if(MSVC)
    #set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
    #set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")
    target_compile_definitions(TestCalibration PUBLIC WIN32)
    target_include_directories(TestCalibration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/Windows/v${MSVC_TOOLSET_VERSION})
    target_link_libraries(TestCalibration PUBLIC optimized "${CMAKE_CURRENT_SOURCE_DIR}/lib/Windows/v${MSVC_TOOLSET_VERSION}/x64/Release/RaptorAPI.lib"
        debug "${CMAKE_CURRENT_SOURCE_DIR}/lib/Windows/v${MSVC_TOOLSET_VERSION}/x64/Debug/RaptorAPI.lib")
else()
    target_compile_definitions(TestCalibration PUBLIC LINUX)
    target_include_directories(TestCalibration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/Linux/glibc-${GLIBC})
    target_link_libraries(TestCalibration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/bin/Linux/glibc-${GLIBC}/libRaptorAPI.so -lpciaccess -ldl pthread)
endif()

# TARGET TestImpedance

add_executable(TestImpedance ${CMAKE_CURRENT_SOURCE_DIR}/Samples/TestImpedance.cpp)
target_include_directories(TestImpedance PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
if(MSVC)
    target_compile_definitions(TestImpedance PUBLIC WIN32)
    target_include_directories(TestImpedance PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/Windows/v${MSVC_TOOLSET_VERSION})
    target_link_libraries(TestImpedance PUBLIC optimized "${CMAKE_CURRENT_SOURCE_DIR}/lib/Windows/v${MSVC_TOOLSET_VERSION}/x64/Release/RaptorAPI.lib"
        debug "${CMAKE_CURRENT_SOURCE_DIR}/lib/Windows/v${MSVC_TOOLSET_VERSION}/x64/Debug/RaptorAPI.lib")
else()
    target_compile_definitions(TestImpedance PUBLIC LINUX)
    target_include_directories(TestImpedance PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/Linux/glibc-${GLIBC})
    target_link_libraries(TestImpedance PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/bin/Linux/glibc-${GLIBC}/libRaptorAPI.so -lpciaccess -ldl pthread)
endif()

# TARGET TestBlockedMode

add_executable(TestBlockedMode ${CMAKE_CURRENT_SOURCE_DIR}/Samples/TestBlockedMode.cpp)
target_include_directories(TestBlockedMode PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
if(MSVC)
    target_compile_definitions(TestBlockedMode PUBLIC WIN32)
    target_include_directories(TestBlockedMode PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/Windows/v${MSVC_TOOLSET_VERSION})
    target_link_libraries(TestBlockedMode PUBLIC optimized "${CMAKE_CURRENT_SOURCE_DIR}/lib/Windows/v${MSVC_TOOLSET_VERSION}/x64/Release/RaptorAPI.lib"
        debug "${CMAKE_CURRENT_SOURCE_DIR}/lib/Windows/v${MSVC_TOOLSET_VERSION}/x64/Debug/RaptorAPI.lib")
else()
    target_compile_definitions(TestBlockedMode PUBLIC LINUX)
    target_include_directories(TestBlockedMode PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/Linux/glibc-${GLIBC})
    target_link_libraries(TestBlockedMode PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/bin/Linux/glibc-${GLIBC}/libRaptorAPI.so -lpciaccess -ldl pthread)
endif()

# TARGET TestCalibration_TwoDevices

add_executable(TestCalibration_TwoDevices ${CMAKE_CURRENT_SOURCE_DIR}/Samples/TestCalibration_TwoDevices.cpp)
target_include_directories(TestCalibration_TwoDevices PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
if(MSVC)
    target_compile_definitions(TestCalibration_TwoDevices PUBLIC WIN32)
    target_include_directories(TestCalibration_TwoDevices PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/Windows/v${MSVC_TOOLSET_VERSION})
    target_link_libraries(TestCalibration_TwoDevices PUBLIC optimized "${CMAKE_CURRENT_SOURCE_DIR}/lib/Windows/v${MSVC_TOOLSET_VERSION}/x64/Release/RaptorAPI.lib"
        debug "${CMAKE_CURRENT_SOURCE_DIR}/lib/Windows/v${MSVC_TOOLSET_VERSION}/x64/Debug/RaptorAPI.lib")
else()
    target_compile_definitions(TestCalibration_TwoDevices PUBLIC LINUX)
    target_include_directories(TestCalibration_TwoDevices PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/Linux/glibc-${GLIBC})
    target_link_libraries(TestCalibration_TwoDevices PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/bin/Linux/glibc-${GLIBC}/libRaptorAPI.so -lpciaccess -ldl pthread)
endif()

# TARGET TestJointAngles_TwoDevices

add_executable(TestJointAngles_TwoDevices ${CMAKE_CURRENT_SOURCE_DIR}/Samples/TestJointAngles_TwoDevices.cpp)
target_include_directories(TestJointAngles_TwoDevices PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
if(MSVC)
    target_compile_definitions(TestJointAngles_TwoDevices PUBLIC WIN32)
    target_include_directories(TestJointAngles_TwoDevices PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/Windows/v${MSVC_TOOLSET_VERSION})
    target_link_libraries(TestJointAngles_TwoDevices PUBLIC optimized "${CMAKE_CURRENT_SOURCE_DIR}/lib/Windows/v${MSVC_TOOLSET_VERSION}/x64/Release/RaptorAPI.lib"
        debug "${CMAKE_CURRENT_SOURCE_DIR}/lib/Windows/v${MSVC_TOOLSET_VERSION}/x64/Debug/RaptorAPI.lib")
else()
    target_compile_definitions(TestJointAngles_TwoDevices PUBLIC LINUX)
    target_include_directories(TestJointAngles_TwoDevices PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/Linux/glibc-${GLIBC})
    target_link_libraries(TestJointAngles_TwoDevices PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/bin/Linux/glibc-${GLIBC}/libRaptorAPI.so -lpciaccess -ldl pthread)
endif()
