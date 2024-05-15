// UFO
#include <ufo/container/tree/tree_node.hpp>

// Catch2
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Tree node")
{
	struct Code {
		unsigned depth() const { return 0; }
	};

	ufo::TreeNode<Code> node;
}