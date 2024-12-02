// UFO
#include "test_tree.hpp"

// Catch2
#include <catch2/catch_test_macros.hpp>

// STL
#include <algorithm>
#include <array>
#include <iostream>

using namespace ufo;

using OctreeTest           = TestTree<3>;
using OctreeTestWithCenter = TestTree<3, true>;

TEST_CASE("[Octree] constructor")
{
	SECTION("Default constructor")
	{
		OctreeTest tree(0.1f, 17);
		REQUIRE(tree.size() == 1);
	}
}