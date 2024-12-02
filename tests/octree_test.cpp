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

TEST_CASE("[Octree] comparison")
{
	SECTION("Untouched")
	{
		OctreeTest tree1(0.1f, 17);
		OctreeTest tree2(0.1f, 17);
		REQUIRE(tree1 == tree2);
	}

	SECTION("Equal") {
		OctreeTest tree1(0.1f, 17);
		tree1.create(Vec3f(0, 0, 0));
		tree1.clear();
		OctreeTest tree2(0.1f, 17);
		REQUIRE(tree1 == tree2);
	}

	SECTION("Not equal") {
		OctreeTest tree1(0.1f, 17);
		OctreeTest tree2(0.1f, 17);
		tree2.create(Vec3f(0, 0, 0));
		OctreeTest tree3(0.01f, 17);
		OctreeTest tree4(0.1f, 16);
		REQUIRE(tree1 != tree2);
		REQUIRE(tree1 != tree3);
		REQUIRE(tree1 != tree4);
	}
}