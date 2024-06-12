// UFO
#include <ufo/container/octree_set.hpp>
#include <ufo/geometry/fun.hpp>

// Catch2
#include <catch2/catch_test_macros.hpp>

// STL
#include <algorithm>
#include <array>

TEST_CASE("[OctreeSet] constructor")
{
	SECTION("Default constructor")
	{
		ufo::OctreeSet set;
		REQUIRE(set.empty());
	}

	SECTION("Iterator constructor")
	{
		std::array     v = {ufo::Vec3f(1, 2, 3), ufo::Vec3f(4, 5, 6), ufo::Vec3f(0, 0, 0)};
		ufo::OctreeSet set(v.begin(), v.end());
		for (auto e : set) {
			REQUIRE((equal(v[0], e) || equal(v[1], e) || equal(v[2], e)));
		}
	}

	SECTION("Initializer list constructor")
	{
		std::array     v = {ufo::Vec3f(1, 2, 3), ufo::Vec3f(4, 5, 6), ufo::Vec3f(0, 0, 0)};
		ufo::OctreeSet set({ufo::Vec3f(1, 2, 3), ufo::Vec3f(4, 5, 6), ufo::Vec3f(0, 0, 0)});
		for (auto e : set) {
			REQUIRE((equal(v[0], e) || equal(v[1], e) || equal(v[2], e)));
		}
	}

	SECTION("Range constructor")
	{
		std::array     v = {ufo::Vec3f(1, 2, 3), ufo::Vec3f(4, 5, 6), ufo::Vec3f(0, 0, 0)};
		ufo::OctreeSet set(v);
		for (auto e : set) {
			REQUIRE((equal(v[0], e) || equal(v[1], e) || equal(v[2], e)));
		}
	}

	SECTION("Copy constructor")
	{
		std::array     v = {ufo::Vec3f(1, 2, 3), ufo::Vec3f(4, 5, 6), ufo::Vec3f(0, 0, 0)};
		ufo::OctreeSet set1(v);
		ufo::OctreeSet set2(set1);
		// TODO: Implement REQUIRE(set1 == set2);
		REQUIRE(std::equal(set1.begin(), set1.end(), set2.begin(), set2.end(),
		                   [](auto const& a, auto const& b) { return ufo::equal(a, b); }));
	}

	SECTION("Move constructor")
	{
		std::array     v = {ufo::Vec3f(1, 2, 3), ufo::Vec3f(4, 5, 6), ufo::Vec3f(0, 0, 0)};
		ufo::OctreeSet set1(v);
		ufo::OctreeSet set2(std::move(set1));
		for (auto e : set2) {
			REQUIRE((equal(v[0], e) || equal(v[1], e) || equal(v[2], e)));
		}
	}
}

TEST_CASE("[OctreeSet] assignment operator")
{
	SECTION("Copy assignment")
	{
		std::array     v = {ufo::Vec3f(1, 2, 3), ufo::Vec3f(4, 5, 6), ufo::Vec3f(0, 0, 0)};
		ufo::OctreeSet set1(v);
		ufo::OctreeSet set2;
		set2 = set1;
		// TODO: Implement REQUIRE(set1 == set2);
		REQUIRE(std::equal(set1.begin(), set1.end(), set2.begin(), set2.end(),
		                   [](auto const& a, auto const& b) { return ufo::equal(a, b); }));
	}

	SECTION("Move assignment")
	{
		std::array     v = {ufo::Vec3f(1, 2, 3), ufo::Vec3f(4, 5, 6), ufo::Vec3f(0, 0, 0)};
		ufo::OctreeSet set1(v);
		ufo::OctreeSet set2;
		set2 = std::move(set1);
		for (auto e : set2) {
			REQUIRE((equal(v[0], e) || equal(v[1], e) || equal(v[2], e)));
		}
	}
}

// TODO: Iterators

// TODO: Query iterators

// TODO: Nearest iterators

// TODO: Query nearest iterators

TEST_CASE("[OctreeSet] empty")
{
	ufo::OctreeSet set;
	SECTION("Empty") { REQUIRE(set.empty()); }

	SECTION("Not empty")
	{
		set.insert({ufo::Vec3f(1, 2, 3), ufo::Vec3f(4, 5, 6), ufo::Vec3f(0, 0, 0)});
		REQUIRE(!set.empty());
	}
}

TEST_CASE("[OctreeSet] size")
{
	ufo::OctreeSet set;
	SECTION("Empty") { REQUIRE(0 == set.size()); }

	SECTION("Not empty")
	{
		std::array v = {ufo::Vec3f(1, 2, 3), ufo::Vec3f(4, 5, 6), ufo::Vec3f(0, 0, 0)};
		set.insert(v);
		REQUIRE(v.size() == set.size());
		set.erase(v[1]);
		REQUIRE(v.size() - 1 == set.size());
	}
}

TEST_CASE("[OctreeSet] bounds")
{
	ufo::OctreeSet set;

	SECTION("Empty")
	{
		auto b = set.bounds();

		for (std::size_t i{}; 3 > i; ++i) {
			std::cout << min(b)[i] << " and " << max(b)[i] << std::endl;
			REQUIRE(min(b)[i] > max(b)[i]);
		}
	}

	SECTION("Not empty")
	{
		set.insert({0, 0, 35});
		set.insert({{100, -234, 40}, {-102.3, 200, 33}});
		set.insert(ufo::Vec3f(1, 0, 5));

		auto b = set.bounds();

		REQUIRE(equal(ufo::Vec3f(-102.3, -234, 5), min(b)));
		REQUIRE(equal(ufo::Vec3f(100, 200, 40), max(b)));

		auto it = set.cbegin();
		set.erase(it);

		b = set.bounds();

		REQUIRE(equal(ufo::Vec3f(-102.3, -234, 5), min(b)));
		REQUIRE(equal(ufo::Vec3f(100, 200, 40), max(b)));
	}
}

TEST_CASE("[OctreeSet] clear")
{
	ufo::OctreeSet set;

	SECTION("Empty")
	{
		set.clear();
		REQUIRE(set.empty());
	}

	SECTION("Not empty")
	{
		set.insert({ufo::Vec3f(1, 2, 3), ufo::Vec3f(4, 5, 6), ufo::Vec3f(0, 0, 0)});
		REQUIRE(!set.empty());
		set.clear();
		REQUIRE(set.empty());
	}
}

TEST_CASE("[OctreeSet] insert")
{
	ufo::OctreeSet set;

	SECTION("Insert const reference")
	{
		ufo::Vec3f v = {1, 2, 3};
		set.insert(v);
		for (auto e : set) {
			REQUIRE(equal(v, e));
		}
	}

	SECTION("Insert r-value reference")
	{
		ufo::Vec3f v = {1, 2, 3};
		set.insert(std::move(v));
		for (auto e : set) {
			REQUIRE(equal(v, e));
		}
	}

	SECTION("Insert using iterators")
	{
		std::array v = {ufo::Vec3f(1, 2, 3), ufo::Vec3f(4, 5, 6), ufo::Vec3f(0, 0, 0)};
		set.insert(v.begin(), v.end());
		for (auto e : set) {
			REQUIRE((equal(v[0], e) || equal(v[1], e) || equal(v[2], e)));
		}
	}

	SECTION("Insert using initializer list")
	{
		std::array v = {ufo::Vec3f(1, 2, 3), ufo::Vec3f(4, 5, 6), ufo::Vec3f(0, 0, 0)};
		set.insert({ufo::Vec3f(1, 2, 3), ufo::Vec3f(4, 5, 6), ufo::Vec3f(0, 0, 0)});
		for (auto e : set) {
			REQUIRE((equal(v[0], e) || equal(v[1], e) || equal(v[2], e)));
		}
	}

	SECTION("Insert using range")
	{
		std::array v = {ufo::Vec3f(1, 2, 3), ufo::Vec3f(4, 5, 6), ufo::Vec3f(0, 0, 0)};
		set.insert(v);
		for (auto e : set) {
			REQUIRE((equal(v[0], e) || equal(v[1], e) || equal(v[2], e)));
		}
	}
}

// TODO: Erase

// TODO: Swap

// TODO: Count

// TODO: Contains

// TODO: Query

// TODO: Nearest

// TODO: Query nearest

TEST_CASE("Octree Set")
{
	ufo::OctreeSet set;

	auto b = set.bounds();

	for (std::size_t i{}; 3 > i; ++i) {
		std::cout << min(b)[i] << " and " << max(b)[i] << std::endl;
		REQUIRE(min(b)[i] > max(b)[i]);
	}

	set.insert({0, 0, 35});
	set.insert({{100, -234, 40}, {-102.3, 200, 33}});
	set.insert(ufo::Vec3f(1, 0, 5));

	b = set.bounds();

	REQUIRE(equal(ufo::Vec3f(-102.3, -234, 5), min(b)));
	REQUIRE(equal(ufo::Vec3f(100, 200, 40), max(b)));

	std::cout << "Before" << std::endl;
	for (auto x : set) {
		std::cout << x << std::endl;
	}

	auto it = set.cbegin();
	set.erase(it);

	b = set.bounds();

	REQUIRE(equal(ufo::Vec3f(-102.3, -234, 5), min(b)));
	REQUIRE(equal(ufo::Vec3f(100, 200, 40), max(b)));

	std::cout << "After" << std::endl;
	for (auto x : set) {
		std::cout << x << std::endl;
	}

	set.erase(std::next(set.begin()), set.end());

	b = set.bounds();

	REQUIRE(equal(ufo::Vec3f(100, -234, 40), min(b)));
	REQUIRE(equal(ufo::Vec3f(100, -234, 40), max(b)));

	std::cout << "After 2" << std::endl;
	for (auto x : set) {
		std::cout << x << std::endl;
	}

	set.clear();

	b = set.bounds();

	for (std::size_t i{}; 3 > i; ++i) {
		REQUIRE(min(b)[i] > max(b)[i]);
	}

	std::cout << "After 3" << std::endl;
	for (auto x : set) {
		std::cout << x << std::endl;
	}

	set.insert({0, 0, 35});
	set.insert({{100, -234, 40}, {-102.3, 200, 33}});
	set.insert(ufo::Vec3f(1, 0, 5));

	std::cout << "After 4" << std::endl;
	for (auto x : set) {
		std::cout << x << std::endl;
	}

	std::cout << "After 5" << std::endl;
	for (auto [p, d] : set.nearest({0, 0, 0})) {
		std::cout << p << " with distance: " << d << std::endl;
	}

	auto nearest_it = set.beginNearest({0, 0, 0});

	set.erase(nearest_it, std::next(nearest_it, 2));

	std::cout << "After 6" << std::endl;
	for (auto x : set) {
		std::cout << x << std::endl;
	}

	std::cout << "Query iterator" << std::endl;
	for (auto x : set.query(ufo::pred::Satisfies([](auto e) { return 100 != e.x; }))) {
		std::cout << x << std::endl;
	}
}