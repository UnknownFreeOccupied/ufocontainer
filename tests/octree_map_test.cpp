// UFO
#include <ufo/container/octree_map.hpp>

// Catch2
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Tree Index")
{
	ufo::OctreeMap<int> map;
	map.insert({0, 0, 0}, 5, false);
}