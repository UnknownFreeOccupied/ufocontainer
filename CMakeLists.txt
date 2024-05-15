cmake_minimum_required(VERSION 3.5...3.16)

project(ufocontainer
	VERSION 2.0.0
	DESCRIPTION "The UFO container library"
	LANGUAGES CXX
)

add_subdirectory(1stparty)

add_library(ufocontainer INTERFACE)
add_library(UFO::Container ALIAS ufocontainer)

set_target_properties(ufocontainer PROPERTIES
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_VERSION_MAJOR}
	CXX_STANDARD 17
	CXX_EXTENSIONS OFF
)

target_link_libraries(ufocontainer INTERFACE UFO::Geometry UFO::Utility)

target_include_directories(ufocontainer INTERFACE
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

target_compile_features(ufocontainer INTERFACE cxx_std_17)

include(GNUInstallDirs)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Only enable tests when we're the root project
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
	# Treat warnings as errors
	# set_target_properties(ufocontainer PROPERTIES COMPILE_WARNING_AS_ERROR ON)

	# # Turn all warnings on
	# if (MSVC)
	# 	target_compile_options(ufocontainer INTERFACE /W4)
	# else()
	# 	target_compile_options(ufocontainer INTERFACE -Wall -Wextra -pedantic)
	# endif()

  add_subdirectory(tests)
endif()

install(TARGETS ufocontainer
	EXPORT ufocontainerTargets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(EXPORT ufocontainerTargets
	FILE "ufocontainerTargets.cmake"
	NAMESPACE UFO::
	DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
	"${PROJECT_SOURCE_DIR}/cmake/ufocontainerConfig.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/ufocontainerConfig.cmake"
	INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/ufocontainerConfigVersion.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

INSTALL(
	FILES
	"${CMAKE_CURRENT_BINARY_DIR}/ufocontainerConfig.cmake"
	"${CMAKE_CURRENT_BINARY_DIR}/ufocontainerConfigVersion.cmake"
	DESTINATION lib/cmake/${PROJECT_NAME}
)

install(
	DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION include
)