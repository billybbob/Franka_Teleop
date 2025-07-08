cmake_minimum_required(VERSION 3.5)
project(test_cartesien)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(raptor_api_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(TestCalibration src/TestCalibration.cpp)
ament_target_dependencies(TestCalibration rclcpp raptor_api_interfaces)

add_executable(TestPoseCartesian src/TestPoseCartesian.cpp)
ament_target_dependencies(TestPoseCartesian rclcpp std_msgs tf2_msgs raptor_api_interfaces)

add_executable(value_to_speed src/value_to_speed.cpp)
ament_target_dependencies(value_to_speed rclcpp std_msgs geometry_msgs tf2_msgs raptor_api_interfaces)

add_executable(offset_position src/offset_position.cpp)
ament_target_dependencies(offset_position rclcpp std_msgs geometry_msgs tf2_msgs raptor_api_interfaces)
add_executable(offset_position_simu src/offset_position_simu.cpp)
ament_target_dependencies(offset_position_simu rclcpp std_msgs geometry_msgs tf2_msgs raptor_api_interfaces)

#add_dependencies(TestCalibration virtuose_generate_messages_cpp)
target_include_directories(TestCalibration PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${CMAKE_CURRENT_SOURCE_DIR}/../../install/raptor_api_interfaces/include/raptor_api_interfaces
${CMAKE_CURRENT_SOURCE_DIR}/../../install/raptor_api_interfaces/include/
)

#add_dependencies(TestPoseCartesian virtuose_generate_messages_cpp)
target_include_directories(TestPoseCartesian PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${CMAKE_CURRENT_SOURCE_DIR}/../../install/raptor_api_interfaces/include/raptor_api_interfaces
${CMAKE_CURRENT_SOURCE_DIR}/../../install/raptor_api_interfaces/include/
)

#add_dependencies(value_to_speed virtuose_generate_messages_cpp)
target_include_directories(value_to_speed PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${CMAKE_CURRENT_SOURCE_DIR}/../../install/raptor_api_interfaces/include/raptor_api_interfaces
${CMAKE_CURRENT_SOURCE_DIR}/../../install/raptor_api_interfaces/include/
)

#add_dependencies(offset_position virtuose_generate_messages_cpp)
target_include_directories(offset_position PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${CMAKE_CURRENT_SOURCE_DIR}/../../install/raptor_api_interfaces/include/raptor_api_interfaces
${CMAKE_CURRENT_SOURCE_DIR}/../../install/raptor_api_interfaces/include/
)

#add_dependencies(offset_position_simu virtuose_generate_messages_cpp)
target_include_directories(offset_position_simu PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${CMAKE_CURRENT_SOURCE_DIR}/../../install/raptor_api_interfaces/include/raptor_api_interfaces
${CMAKE_CURRENT_SOURCE_DIR}/../../install/raptor_api_interfaces/include/
)

if(MSVC)  
else()
	execute_process(COMMAND /usr/bin/ldd --version
		COMMAND head -n 1
		COMMAND /usr/bin/awk "{print $NF}"
		OUTPUT_STRIP_TRAILING_WHITESPACE
		OUTPUT_VARIABLE GLIBC)
	message("GLIBC=${GLIBC}")
    target_compile_definitions(TestCalibration PUBLIC LINUX _OS_UNIX)
    target_compile_definitions(TestPoseCartesian PUBLIC LINUX _OS_UNIX)
    target_compile_definitions(value_to_speed PUBLIC LINUX _OS_UNIX)
    target_compile_definitions(offset_position PUBLIC LINUX _OS_UNIX)
    target_compile_definitions(offset_position_simu PUBLIC LINUX _OS_UNIX)
    set(CMAKE_C_FLAGS_RELEASE "-Wall")
    set(CMAKE_CXX_FLAGS_RELEASE "-Wall")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

# Link to RaptorAPI
target_include_directories(TestCalibration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/RaptorAPI/include)
target_include_directories(TestPoseCartesian PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/RaptorAPI/include)
target_include_directories(value_to_speed PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/RaptorAPI/include)
target_include_directories(offset_position PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/RaptorAPI/include)
target_include_directories(offset_position_simu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/RaptorAPI/include)

if(MSVC)

else()

    target_compile_definitions(TestCalibration PUBLIC LINUX)
    target_include_directories(TestCalibration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/RaptorAPI/include/Linux/glibc-${GLIBC})
    target_link_libraries(TestCalibration ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/RaptorAPI/bin/Linux/glibc-${GLIBC}/libRaptorAPI.so -lpciaccess -ldl pthread)

    target_compile_definitions(TestPoseCartesian PUBLIC LINUX)
    target_include_directories(TestPoseCartesian PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/RaptorAPI/include/Linux/glibc-${GLIBC})
    target_link_libraries(TestPoseCartesian ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/RaptorAPI/bin/Linux/glibc-${GLIBC}/libRaptorAPI.so -lpciaccess -ldl pthread)

    target_compile_definitions(value_to_speed PUBLIC LINUX)
    target_include_directories(value_to_speed PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/RaptorAPI/include/Linux/glibc-${GLIBC})
    target_link_libraries(value_to_speed ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/RaptorAPI/bin/Linux/glibc-${GLIBC}/libRaptorAPI.so -lpciaccess -ldl pthread)

    target_compile_definitions(offset_position PUBLIC LINUX)
    target_include_directories(offset_position PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/RaptorAPI/include/Linux/glibc-${GLIBC})
    target_link_libraries(offset_position ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/RaptorAPI/bin/Linux/glibc-${GLIBC}/libRaptorAPI.so -lpciaccess -ldl pthread)
    
    target_compile_definitions(offset_position_simu PUBLIC LINUX)
    target_include_directories(offset_position_simu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/RaptorAPI/include/Linux/glibc-${GLIBC})
    target_link_libraries(offset_position_simu ${CMAKE_CURRENT_SOURCE_DIR}/Dependencies/RaptorAPI/bin/Linux/glibc-${GLIBC}/libRaptorAPI.so -lpciaccess -ldl pthread)

endif()

install(TARGETS TestCalibration
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS TestPoseCartesian
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS value_to_speed
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS offset_position
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS offset_position_simu
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
