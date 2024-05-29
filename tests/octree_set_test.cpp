// UFO
#include <ufo/container/octree_set.hpp>

// Catch2
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Octree Set")
{
	ufo::OctreeSet set;

	auto b = set.bounds();

	for (std::size_t i{}; 3 > i; ++i) {
		std::cout << b.min()[i] << " and " << b.max()[i] << std::endl;
		REQUIRE(!std::isfinite(b.min()[i]));
		REQUIRE(!std::isfinite(b.max()[i]));
	}

	set.insert({0, 0, 35});
	set.insert({{100, -234, 40}, {-102.3, 200, 33}});
	set.insert(ufo::Vec3f(1, 0, 5));

	b = set.bounds();

	REQUIRE(ufo::Vec3f(-102.3, -234, 5) == b.min());
	REQUIRE(ufo::Vec3f(100, 200, 40) == b.max());

	std::cout << "Before" << std::endl;
	for (auto x : set) {
		std::cout << x << std::endl;
	}

	auto it = set.cbegin();
	set.erase(it);

	b = set.bounds();

	REQUIRE(ufo::Vec3f(-102.3, 0, 5) == b.min());
	REQUIRE(ufo::Vec3f(1, 200, 35) == b.max());

	std::cout << "After" << std::endl;
	for (auto x : set) {
		std::cout << x << std::endl;
	}

	set.erase(std::next(set.begin()), set.end());

	b = set.bounds();

	REQUIRE(ufo::Vec3f(-102.3, 200, 33) == b.min());
	REQUIRE(ufo::Vec3f(-102.3, 200, 33) == b.max());

	std::cout << "After 2" << std::endl;
	for (auto x : set) {
		std::cout << x << std::endl;
	}

	set.clear();

	b = set.bounds();

	for (std::size_t i{}; 3 > i; ++i) {
		REQUIRE(!std::isfinite(b.min()[i]));
		REQUIRE(!std::isfinite(b.max()[i]));
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
}