// UFO
#include "test_tree.hpp"

// Catch2
#include <catch2/catch_test_macros.hpp>

// STL
#include <algorithm>
#include <iostream>
#include <random>
#include <utility>

using namespace ufo;

using Octree           = TestTree<3>;
using OctreeWithCenter = TestTree<3, true>;

TEST_CASE("[Octree] constructor")
{
	SECTION("Default constructor")
	{
		Octree tree(0.1f, 17);
		REQUIRE(tree.size() == 1);
	}
}

TEST_CASE("[Octree] comparison")
{
	SECTION("Untouched")
	{
		Octree tree1(0.1f, 17);
		Octree tree2(0.1f, 17);
		REQUIRE(tree1 == tree2);
	}

	SECTION("Equal")
	{
		Octree tree1(0.1f, 17);
		tree1.create(OctCoord(0, 0, 0));
		tree1.clear();
		Octree tree2(0.1f, 17);
		REQUIRE(tree1 == tree2);

		tree1.create(OctCoord(0));
		tree2.create(OctCoord(0));
		REQUIRE(tree1 == tree2);
	}

	SECTION("Not equal")
	{
		Octree tree1(0.1f, 17);
		Octree tree2(0.1f, 17);
		tree2.create(OctCoord(0, 0, 0));
		Octree tree3(0.01f, 17);
		Octree tree4(0.1f, 16);
		REQUIRE(tree1 != tree2);
		REQUIRE(tree1 != tree3);
		REQUIRE(tree1 != tree4);
	}
}

TEST_CASE("[Octree] create")
{
	std::vector<Vec3f> v;
	std::mt19937       gen(42);
	std::uniform_real_distribution<float> dis_x(-20.0f, 20.0f);
	std::uniform_real_distribution<float> dis_y(-20.0f, 20.0f);
	std::uniform_real_distribution<float> dis_z(0.0f, 4.0f);
	for (std::size_t i{}; 10'000 > i; ++i) {
		v.emplace_back(dis_x(gen), dis_y(gen), dis_z(gen));
	}

	Octree tree1(0.1f, 17);
	Octree tree2(0.1f, 17);

	auto const t1 = std::chrono::high_resolution_clock::now();
	auto       n1 = tree1.create(v.begin(), v.end());
	auto const t2 = std::chrono::high_resolution_clock::now();
	auto       n2 = tree2.create(execution::par, v.begin(), v.end());
	auto const t3 = std::chrono::high_resolution_clock::now();

	std::chrono::duration<double, std::milli> const serial_ms   = t2 - t1;
	std::chrono::duration<double, std::milli> const parallel_ms = t3 - t2;
	std::cout << "Serial:   " << serial_ms.count() << " ms\n";
	std::cout << "Parallel: " << parallel_ms.count() << " ms\n";

	std::vector<OctCode> c1;
	std::vector<OctCode> c2;

	std::transform(n1.begin(), n1.end(), std::back_inserter(c1),
	               [&tree1](auto n) { return tree1.code(n); });
	std::transform(n2.begin(), n2.end(), std::back_inserter(c2),
	               [&tree2](auto n) { return tree2.code(n); });

	REQUIRE(c1 == c2);

	SECTION("SERIAL") {}

	SECTION("Parallel") {}
}

TEST_CASE("[Octree] with and without center")
{
	SECTION("Center")
	{
		Octree           tree1(0.1f, 17);
		OctreeWithCenter tree2(0.1f, 17);

		REQUIRE(tree1.center(tree1.index()) == tree2.center(tree2.index()));

		TreeIndex node1 = tree1.create(OctCoord(0, 0, 0));
		TreeIndex node2 = tree2.create(OctCoord(0, 0, 0));

		REQUIRE(tree1.center(node1) == tree2.center(node2));
	}

	SECTION("Center axis")
	{
		Octree           tree1(0.1f, 17);
		OctreeWithCenter tree2(0.1f, 17);

		REQUIRE(tree1.centerAxis(tree1.index(), 2) == tree2.centerAxis(tree2.index(), 2));

		TreeIndex node1 = tree1.create(OctCoord(0, 0, 0));
		TreeIndex node2 = tree2.create(OctCoord(0, 0, 0));

		REQUIRE(tree1.centerAxis(node1, 1) == tree2.centerAxis(node2, 1));
	}
}

TEST_CASE("[Octree] swap")
{
	using std::swap;

	Octree tree1(0.1f, 17);
	tree1.create(OctCoord(0, 0, 0));

	Octree tree2 = tree1;

	Octree tree3(0.1f, 17);

	REQUIRE((tree1 == tree2 && tree1 != tree3));

	swap(tree2, tree3);

	REQUIRE((tree1 != tree2 && tree1 == tree3));
}

TEST_CASE("[Octree] traverse")
{
	SECTION("Index version")
	{
		Octree tree(0.1f, 17);
		tree.traverse([](TreeIndex) { return true; });
		tree.traverse(TreeCoord<3>(OctCoord(0), 15), [](TreeIndex) { return true; });
	}

	SECTION("Node version")
	{
		Octree tree(0.1f, 17);
		tree.traverse([](TreeNode<3> const& node) { return true; });
		tree.traverse([&tree](TreeNode<3> const& node) { return tree.depth(node) > 14; },
		              pred::True(), false);

		tree.traverse(
		    TreeCoord<3>(OctCoord(0), 15), [](TreeNode<3> const& node) { return true; },
		    true);
		tree.traverse(
		    TreeCoord<3>(OctCoord(0), 15),
		    [&tree](TreeNode<3> const& node) { return tree.depth(node) > 14; }, pred::True(),
		    false);
	}
}