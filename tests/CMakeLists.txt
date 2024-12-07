include("${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/catch2/catch2.cmake")

# SET(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage")
# SET(GCC_COVERAGE_LINK_FLAGS    "--coverage")
# SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
# SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )
# # set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

add_executable(ufocontainer_tests
	octree_test.cpp
	octree_map_test.cpp
	octree_set_test.cpp
	# quadtree_set_test.cpp
	# range_test.cpp
	# tree_index_test.cpp
	# tree_node_test.cpp
)

target_link_libraries(ufocontainer_tests PRIVATE UFO::Container Catch2::Catch2WithMain)

target_compile_features(ufocontainer_tests PRIVATE cxx_std_17)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)
catch_discover_tests(ufocontainer_tests)