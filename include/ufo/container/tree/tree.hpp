/*!
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_CONTAINER_TREE_TREE_HPP
#define UFO_CONTAINER_TREE_TREE_HPP

// UFO
#include <ufo/container/tree/tree_bounds.hpp>
#include <ufo/container/tree/tree_container.hpp>
#include <ufo/container/tree/tree_coord.hpp>
// #include <ufo/container/tree/tree_file_header.hpp>
#include <ufo/container/tree/tree_index.hpp>
#include <ufo/container/tree/tree_iterator.hpp>
#include <ufo/container/tree/tree_node.hpp>
#include <ufo/container/tree/tree_node_nearest.hpp>
#include <ufo/container/tree/tree_predicate.hpp>
#include <ufo/container/tree/tree_type.hpp>
#include <ufo/container/tree/tree_types.hpp>
#include <ufo/math/util.hpp>
#include <ufo/utility/bit_set.hpp>
#include <ufo/utility/compression.hpp>
#include <ufo/utility/execution.hpp>
#include <ufo/utility/io/buffer.hpp>
#include <ufo/utility/iterator_wrapper.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <deque>
#include <iterator>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace ufo
{

/*!
 * @brief
 *
 * Utilizing curiously recurring template pattern (CRTP)
 *
 * \tparam Derived ...
 * \tparam TreeBlock ...
 * \tparam TT ...
 */
template <class Derived, template <TreeType> class TreeBlock, TreeType TT>
class Tree
{
	//
	// Friends
	//

	template <class Derived2, template <TreeType> class TreeBlock2, TreeType TT2>
	friend class Tree;

 public:
	//
	// Tags
	//

	using length_t = double;
	using depth_t  = unsigned;

	using Code   = typename TreeTypes<TT>::Code;
	using Key    = typename TreeTypes<TT>::Key;
	using Point  = typename TreeTypes<TT>::Point;
	using Bounds = typename TreeTypes<TT>::Bounds;

	using Index       = TreeIndex;
	using Node        = TreeNode<Code>;
	using NodeNearest = TreeNodeNearest<Node>;
	using coord_t     = typename Point::scalar_t;
	using Coord       = TreeCoord<Point, depth_t>;

	using pos_t    = typename TreeIndex::pos_t;
	using offset_t = typename TreeIndex::offset_t;
	using key_t    = typename Key::key_t;
	using code_t   = typename Code::code_t;

	using const_iterator               = TreeIteratorWrapper<Derived, Node>;
	using const_nearest_iterator       = TreeIteratorWrapper<Derived, NodeNearest>;
	using const_query_iterator         = const_iterator;
	using const_nearest_query_iterator = const_nearest_iterator;

	using Query        = IteratorWrapper<const_query_iterator>;
	using QueryNearest = IteratorWrapper<const_nearest_query_iterator>;

	using Block     = TreeBlock<TT>;
	using Container = TreeContainer<Block>;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tree                                         |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Returns the tree type.
	 *
	 * @return The tree type.
	 */
	[[nodiscard]] static constexpr TreeType treeType() noexcept { return TT; }

	/*!
	 * @brief Returns the branching factor of the tree (i.e., 2 = binary tree, 4 = quadtree,
	 * 8 = octree, 16 = hextree).
	 *
	 * @return The branching factor of the tree.
	 */
	[[nodiscard]] static constexpr std::size_t branchingFactor() noexcept
	{
		return ufo::branchingFactor(treeType());
	}

	/*!
	 * @brief Erases all nodes from the tree.
	 */
	void clear()
	{
		blocks_.clear();
		free_blocks_.clear();
		// Create root
		blocks_.emplace_back(code().parent());
		derived().clear();
	}

	//
	// Depth
	//

	/*!
	 * @brief Returns the number of depth levels of the tree, i.e. `depth() + 1`.
	 *
	 * @return The number of depth levels of the tree.
	 */
	[[nodiscard]] constexpr depth_t numDepthLevels() const noexcept
	{
		return num_depth_levels_;
	}

	/*!
	 * @brief Returns the minimum number of depth levels a tree must have.
	 *
	 * @return The minimum number of depth levels a tree must have.
	 */
	[[nodiscard]] static constexpr depth_t minNumDepthLevels() noexcept { return 2; }

	/*!
	 * @brief Returns the maximum number of depth levels a tree can have.
	 *
	 * @return The maximum number of depth levels a tree can have.
	 */
	[[nodiscard]] static constexpr depth_t maxNumDepthLevels() noexcept
	{
		return Code::maxDepth();
	}

	/*!
	 * @brief Returns the depth of the root node, i.e. `numDepthLevels() - 1`.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @return The depth of the root node.
	 */
	[[nodiscard]] depth_t depth() const { return numDepthLevels() - 1; }

	/*!
	 * @brief Returns the depth of the block.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param block the block
	 * @return The depth of the block.
	 */
	[[nodiscard]] depth_t depth(pos_t block) const
	{
		assert(blocks_.size() > block);
		return blocks_[block].depth();
	}

	/*!
	 * @brief Returns the depth of the node.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param node the node
	 * @return The depth of the node.
	 */
	[[nodiscard]] depth_t depth(Index node) const { return depth(node.pos); }

	/*!
	 * @brief Returns the depth of the node.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param node the node
	 * @return The depth of the node.
	 */
	[[nodiscard]] static constexpr depth_t depth(Node node) noexcept
	{
		return node.depth();
	}

	/*!
	 * @brief Returns the depth of the node.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param node the node
	 * @return The depth of the node.
	 */
	[[nodiscard]] static constexpr depth_t depth(Code node) noexcept
	{
		return node.depth();
	}

	/*!
	 * @brief Returns the depth of the node.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param node the node
	 * @return The depth of the node.
	 */
	[[nodiscard]] static constexpr depth_t depth(Key node) noexcept { return node.depth(); }

	/*!
	 * @brief Returns the depth of the node.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param node the node
	 * @return The depth of the node.
	 */
	[[nodiscard]] static constexpr depth_t depth(Coord node) noexcept { return node.depth; }

	//
	// Length
	//

	/*!
	 * @brief Returns the length of the tree (/ root node), i.e. `leaf_node_length *
	 * 2^depth()`.
	 *
	 * @return The length of the tree (/ root node).
	 */
	[[nodiscard]] length_t length() const { return length(depth()); }

	/*!
	 * @brief Returns the length of nodes at `depth`, i.e. `leaf_node_length *
	 * 2^depth`.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param depth the depth
	 * @return The length of nodes at `depth`.
	 */
	[[nodiscard]] length_t length(depth_t depth) const
	{
		assert(numDepthLevels() > depth + 1);
		return node_half_length_[depth + 1];
	}

	/*!
	 * @brief Returns the length of `node`, i.e. `leaf_node_length * 2^depth(node)`.
	 *
	 * @param node the node
	 * @return The length of the node.
	 */
	[[nodiscard]] length_t length(Index node) const { return length(depth(node)); }

	/*!
	 * @brief Returns the length of `node`, i.e. `leaf_node_length * 2^depth(node)`.
	 *
	 * @param node the node
	 * @return The length of the node.
	 */
	[[nodiscard]] length_t length(Node node) const { return length(depth(node)); }

	/*!
	 * @brief Returns the length of `node`, i.e. `leaf_node_length * 2^depth(node)`.
	 *
	 * @param node the node
	 * @return The length of the node.
	 */
	[[nodiscard]] length_t length(Code node) const { return length(depth(node)); }

	/*!
	 * @brief Returns the length of `node`, i.e. `leaf_node_length * 2^depth(node)`.
	 *
	 * @param node the node
	 * @return The length of the node.
	 */
	[[nodiscard]] length_t length(Key node) const { return length(depth(node)); }

	/*!
	 * @brief Returns the length of `node`, i.e. `leaf_node_length * 2^depth(node)`.
	 *
	 * @param node the node
	 * @return The length of the node.
	 */
	[[nodiscard]] length_t length(Coord node) const { return length(depth(node)); }

	/*!
	 * @brief Returns the half length of the tree (/ root node), i.e. `length() / 2`.
	 *
	 * @note The half length is often used, therefore this function exists for improved
	 * performance and precision.
	 *
	 * @return The half length of the tree (/ root node).
	 */
	[[nodiscard]] length_t halfLength() const { return halfLength(depth()); }

	/*!
	 * @brief Returns the half length of nodes at `depth`, i.e. `length(depth) / 2`.
	 *
	 * @note The half length is often used, therefore this function exists for improved
	 * performance and precision.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param depth the depth
	 * @return The half length of nodes at `depth`.
	 */
	[[nodiscard]] length_t halfLength(depth_t depth) const
	{
		assert(numDepthLevels() > depth);
		return node_half_length_[depth];
	}

	/*!
	 * @brief Returns the half length of `node`, i.e. `length(node) / 2`.
	 *
	 * @note The half length is often used, therefore this function exists for improved
	 * performance and precision.
	 *
	 * @param node the node
	 * @return The half length of the node.
	 */
	[[nodiscard]] length_t halfLength(Index node) const { return halfLength(depth(node)); }

	/*!
	 * @brief Returns the half length of `node`, i.e. `length(node) / 2`.
	 *
	 * @note The half length is often used, therefore this function exists for improved
	 * performance and precision.
	 *
	 * @param node the node
	 * @return The half length of the node.
	 */
	[[nodiscard]] length_t halfLength(Node node) const { return halfLength(depth(node)); }

	/*!
	 * @brief Returns the half length of `node`, i.e. `length(node) / 2`.
	 *
	 * @note The half length is often used, therefore this function exists for improved
	 * performance and precision.
	 *
	 * @param node the node
	 * @return The half length of the node.
	 */
	[[nodiscard]] length_t halfLength(Code node) const { return halfLength(depth(node)); }

	/*!
	 * @brief Returns the half length of `node`, i.e. `length(node) / 2`.
	 *
	 * @note The half length is often used, therefore this function exists for improved
	 * performance and precision.
	 *
	 * @param node the node
	 * @return The half length of the node.
	 */
	[[nodiscard]] length_t halfLength(Key node) const { return halfLength(depth(node)); }

	/*!
	 * @brief Returns the half length of `node`, i.e. `length(node) / 2`.
	 *
	 * @note The half length is often used, therefore this function exists for improved
	 * performance and precision.
	 *
	 * @param node the node
	 * @return The half length of the node.
	 */
	[[nodiscard]] length_t halfLength(Coord node) const { return halfLength(depth(node)); }

	/*!
	 * @brief Returns the reciprocal of the length of the tree (/ root node), i.e. `1 /
	 * length()`.
	 *
	 * @note The reciprocal of the length is often used, therefore this function exists for
	 * improved performance and precision.
	 *
	 * @return The reciprocal of the length of the tree (/ root node).
	 */
	[[nodiscard]] length_t lengthReciprocal() const { return lengthReciprocal(depth()); }

	/*!
	 * @brief Returns the reciprocal of the length of nodes at `depth`, i.e. `1 /
	 * length(depth)`.
	 *
	 * @note The reciprocal of the length is often used, therefore this function exists for
	 * improved performance and precision.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param depth the depth
	 * @return The reciprocal of the length of nodes at `depth`.
	 */
	[[nodiscard]] length_t lengthReciprocal(depth_t depth) const
	{
		assert(numDepthLevels() > depth + 1);
		return node_half_length_reciprocal_[depth + 1];
	}

	/*!
	 * @brief Returns the reciprocal of the length of `node`, i.e. `1 / length(node)`.
	 *
	 * @note The reciprocal of the length is often used, therefore this function exists for
	 * improved performance and precision.
	 *
	 * @param node the node
	 * @return The reciprocal of the length of the node.
	 */
	[[nodiscard]] length_t lengthReciprocal(Index node) const
	{
		return lengthReciprocal(depth(node));
	}

	/*!
	 * @brief Returns the reciprocal of the length of `node`, i.e. `1 / length(node)`.
	 *
	 * @note The reciprocal of the length is often used, therefore this function exists for
	 * improved performance and precision.
	 *
	 * @param node the node
	 * @return The reciprocal of the length of the node.
	 */
	[[nodiscard]] length_t lengthReciprocal(Node node) const
	{
		return lengthReciprocal(depth(node));
	}

	/*!
	 * @brief Returns the reciprocal of the length of `node`, i.e. `1 / length(node)`.
	 *
	 * @note The reciprocal of the length is often used, therefore this function exists for
	 * improved performance and precision.
	 *
	 * @param node the node
	 * @return The reciprocal of the length of the node.
	 */
	[[nodiscard]] length_t lengthReciprocal(Code node) const
	{
		return lengthReciprocal(depth(node));
	}

	/*!
	 * @brief Returns the reciprocal of the length of `node`, i.e. `1 / length(node)`.
	 *
	 * @note The reciprocal of the length is often used, therefore this function exists for
	 * improved performance and precision.
	 *
	 * @param node the node
	 * @return The reciprocal of the length of the node.
	 */
	[[nodiscard]] length_t lengthReciprocal(Key node) const
	{
		return lengthReciprocal(depth(node));
	}

	/*!
	 * @brief Returns the reciprocal of the length of `node`, i.e. `1 / length(node)`.
	 *
	 * @note The reciprocal of the length is often used, therefore this function exists for
	 * improved performance and precision.
	 *
	 * @param node the node
	 * @return The reciprocal of the length of the node.
	 */
	[[nodiscard]] length_t lengthReciprocal(Coord node) const
	{
		return lengthReciprocal(depth(node));
	}

	/*!
	 * @brief Returns the reciprocal of the half length of the tree (/ root node), i.e. `1 /
	 * (length() / 2) = 2 / length()`.
	 *
	 * @note The reciprocal of the half length is often used, therefore this function exists
	 * for improved performance and precision.
	 *
	 * @return The reciprocal of the half length of the tree (/ root node).
	 */
	[[nodiscard]] length_t halfLengthReciprocal() const
	{
		return halfLengthReciprocal(depth());
	}

	/*!
	 * @brief Returns the reciprocal of the half length of nodes at `depth`, i.e. `1 /
	 * (length(depth) / 2) = 2 / length(depth)`.
	 *
	 * @note The reciprocal of the half length is often used, therefore this function exists
	 * for improved performance and precision.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param depth the depth
	 * @return The reciprocal of the half length of nodes at `depth`.
	 */
	[[nodiscard]] length_t halfLengthReciprocal(depth_t depth) const
	{
		assert(numDepthLevels() > depth);
		return node_half_length_reciprocal_[depth];
	}

	/*!
	 * @brief Returns the reciprocal of the half length of `node`, i.e. `1 / (length(node) /
	 * 2) = 2 / length(node)`.
	 *
	 * @note The reciprocal of the half length is often used, therefore this function exists
	 * for improved performance and precision.
	 *
	 * @param node the node
	 * @return The reciprocal of the half length of the node.
	 */
	[[nodiscard]] length_t halfLengthReciprocal(Index node) const
	{
		return halfLengthReciprocal(depth(node));
	}

	/*!
	 * @brief Returns the reciprocal of the half length of `node`, i.e. `1 / (length(node) /
	 * 2) = 2 / length(node)`.
	 *
	 * @note The reciprocal of the half length is often used, therefore this function exists
	 * for improved performance and precision.
	 *
	 * @param node the node
	 * @return The reciprocal of the half length of the node.
	 */
	[[nodiscard]] length_t halfLengthReciprocal(Node node) const
	{
		return halfLengthReciprocal(depth(node));
	}

	/*!
	 * @brief Returns the reciprocal of the half length of `node`, i.e. `1 / (length(node) /
	 * 2) = 2 / length(node)`.
	 *
	 * @note The reciprocal of the half length is often used, therefore this function exists
	 * for improved performance and precision.
	 *
	 * @param node the node
	 * @return The reciprocal of the half length of the node.
	 */
	[[nodiscard]] length_t halfLengthReciprocal(Code node) const
	{
		return halfLengthReciprocal(depth(node));
	}

	/*!
	 * @brief Returns the reciprocal of the half length of `node`, i.e. `1 / (length(node) /
	 * 2) = 2 / length(node)`.
	 *
	 * @note The reciprocal of the half length is often used, therefore this function exists
	 * for improved performance and precision.
	 *
	 * @param node the node
	 * @return The reciprocal of the half length of the node.
	 */
	[[nodiscard]] length_t halfLengthReciprocal(Key node) const
	{
		return halfLengthReciprocal(depth(node));
	}

	/*!
	 * @brief Returns the reciprocal of the half length of `node`, i.e. `1 / (length(node) /
	 * 2) = 2 / length(node)`.
	 *
	 * @note The reciprocal of the half length is often used, therefore this function exists
	 * for improved performance and precision.
	 *
	 * @param node the node
	 * @return The reciprocal of the half length of the node.
	 */
	[[nodiscard]] length_t halfLengthReciprocal(Coord node) const
	{
		return halfLengthReciprocal(depth(node));
	}

	//
	// Bounds
	//

	/*!
	 * @brief Returns the bounds of the tree (/ root node).
	 *
	 * @return The bounds of the tree (/ root node).
	 */
	[[nodiscard]] Bounds bounds() const { return {center(), halfLength(depth())}; }

	/*!
	 * @brief Returns the bounds of `node`.
	 *
	 * @param node the node
	 * @return The bounds of the node.
	 */
	[[nodiscard]] Bounds bounds(Index node) const
	{
		return Bounds(center(node), halfLength(node));
	}

	/*!
	 * @brief Returns the bounds of `node`.
	 *
	 * @param node the node
	 * @return The bounds of the node.
	 */
	[[nodiscard]] Bounds bounds(Node node) const { return bounds(index(node)); }

	/*!
	 * @brief Returns the bounds of `node`.
	 *
	 * @param node the node
	 * @return The bounds of the node.
	 */
	[[nodiscard]] Bounds bounds(Code node) const { return bounds(index(node)); }

	/*!
	 * @brief Returns the bounds of `node`.
	 *
	 * @param node the node
	 * @return The bounds of the node.
	 */
	[[nodiscard]] Bounds bounds(Key node) const { return bounds(index(node)); }

	/*!
	 * @brief Returns the bounds of `node`.
	 *
	 * @param node the node
	 * @return The bounds of the node.
	 */
	[[nodiscard]] Bounds bounds(Coord node) const { return bounds(index(node)); }

	//
	// Inside
	//

	/*!
	 * @brief Checks if a coordinate is inside the tree bounds, i.e. inside `bounds()`.
	 *
	 * @param coord the coordinate
	 * @return `true` if the coordinate is inside the bounds, `false` otherwise.
	 */
	[[nodiscard]] bool isInside(Point coord) const
	{
		auto const hl = halfLength(depth());
		for (std::size_t i{}; coord.size() > i; ++i) {
			if (-hl > coord[i] || hl <= coord[i]) {
				return false;
			}
		}
		return true;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Access                                        |
	|                                                                                     |
	**************************************************************************************/

	//
	// Center
	//

	/*!
	 * @brief Returns the center of the tree (/ root node).
	 *
	 * @return The center of the tree (/ root node).
	 */
	[[nodiscard]] Coord center() const { return Coord(Point(), depth()); }

	/*!
	 * @brief Returns the center of `node`.
	 *
	 * @param node the node
	 * @return The center of the node.
	 */
	[[nodiscard]] Coord center(Index node) const
	{
		return center(blocks_[node.pos].parent_code.child(node.offset));
	}

	/*!
	 * @brief Returns the center of `node`.
	 *
	 * @param node the node
	 * @return The center of the node.
	 */
	[[nodiscard]] Coord center(Node node) const { return code(node); }

	/*!
	 * @brief Returns the center of `node`.
	 *
	 * @param node the node
	 * @return The center of the node.
	 */
	[[nodiscard]] Coord center(Code node) const { return center(key(node)); }

	/*!
	 * @brief Returns the center of `node`.
	 *
	 * @param node the node
	 * @return The center of the node.
	 */
	[[nodiscard]] Coord center(Key node) const
	{
		assert(valid(node));

		auto node_depth = depth(node);

		if (depth() == node_depth) {
			return center();
		}

		length_t          l   = length(node_depth);
		std::int_fast64_t hmv = static_cast<std::int_fast64_t>(half_max_value_ >> node_depth);

		Point coord;
		for (std::size_t i{}; node.size() != i; ++i) {
			coord[i] = (static_cast<length_t>(
			                static_cast<std::int_fast64_t>(node[i] >> node_depth) - hmv) +
			            static_cast<length_t>(0.5)) *
			           l;
		}
		return Coord(coord, node_depth);
	}

	/*!
	 * @brief Returns the center of `node`.
	 *
	 * @param node the node
	 * @return The center of the node.
	 */
	[[nodiscard]] Coord center(Coord node) const { return center(key(node)); }

	/*!
	 * @brief Returns the center of `node` if the node is valid, i.e. `valid(node)`.
	 *
	 * @param node the node
	 * @return The center of the node if the node is valid, null otherwise.
	 */
	[[nodiscard]] std::optional<Coord> centerChecked(Code node) const
	{
		return valid(node) ? std::optional<Coord>(center(node)) : std::nullopt;
	}

	/*!
	 * @brief Returns the center of `node` if the node is valid, i.e. `valid(node)`.
	 *
	 * @param node the node
	 * @return The center of the node if the node is valid, null otherwise.
	 */
	[[nodiscard]] std::optional<Coord> centerChecked(Key node) const
	{
		return valid(node) ? std::optional<Coord>(center(node)) : std::nullopt;
	}

	/*!
	 * @brief Returns the center of `node` if the node is valid, i.e. `valid(node)`.
	 *
	 * @param node the node
	 * @return The center of the node if the node is valid, null otherwise.
	 */
	[[nodiscard]] std::optional<Coord> centerChecked(Coord node) const
	{
		return valid(node) ? std::optional<Coord>(center(node)) : std::nullopt;
	}

	//
	// Block
	//

	/*!
	 * @brief Returns the block position of the root node.
	 *
	 * @return The block position of the root node.
	 */
	[[nodiscard]] pos_t block() const { return 0; }

	/*!
	 * @brief Returns the block position of `node`.
	 *
	 * @param node the node
	 * @return The block position of the node.
	 */
	[[nodiscard]] pos_t block(Index node) const { return node.pos; }

	/*!
	 * @brief Returns the block position of `node`.
	 *
	 * @param node the node
	 * @return The block position of the node.
	 */
	[[nodiscard]] pos_t block(Node node) const { return block(index(node)); }

	/*!
	 * @brief Returns the block position of `node`.
	 *
	 * @param node the node
	 * @return The block position of the node.
	 */
	[[nodiscard]] pos_t block(Code node) const { return block(index(node)); }

	/*!
	 * @brief Returns the block position of `node`.
	 *
	 * @param node the node
	 * @return The block position of the node.
	 */
	[[nodiscard]] pos_t block(Key node) const { return block(index(node)); }

	/*!
	 * @brief Returns the block position of `node`.
	 *
	 * @param node the node
	 * @return The block position of the node.
	 */
	[[nodiscard]] pos_t block(Coord node) const { return block(index(node)); }

	//
	// Index
	//

	/*!
	 * @brief Returns the index of the root node.
	 *
	 * @return The index of the root node.
	 */
	[[nodiscard]] Index index() const { return Index(block(), 0); }

	/*!
	 * @brief Returns the index of `node`.
	 *
	 * @param node the node
	 * @return The index of the node.
	 */
	[[nodiscard]] Index index(Node node) const
	{
		if (!valid(node.index()) || depth(node.index()) < depth(node) ||
		    !Code::equalAtDepth(code(node.index()), node.code(), depth(node.index()))) {
			return index(node.code());
		}

		if (code(node.index()) == node.code() || isLeaf(node.index())) {
			return node.index();
		}

		return index(node.code(), node.index(), depth(node.index()));
	}

	/*!
	 * @brief Returns the index of `node`.
	 *
	 * @param node the node
	 * @return The index of the node.
	 */
	[[nodiscard]] Index index(Code node) const { return index(node, index(), depth()); }

	/*!
	 * @brief Returns the index of `node`.
	 *
	 * @param node the node
	 * @return The index of the node.
	 */
	[[nodiscard]] Index index(Key node) const { return index(code(node)); }

	/*!
	 * @brief Returns the index of `node`.
	 *
	 * @param node the node
	 * @return The index of the node.
	 */
	[[nodiscard]] Index index(Coord node) const { return index(code(node)); }

	//
	// Node
	//

	/*!
	 * @brief Returns the root node.
	 *
	 * @return The root node.
	 */
	[[nodiscard]] Node node() const { return Node(code(), index()); }

	/*!
	 * @brief Returns the node corresponding to `node`.
	 *
	 * @param node the node
	 * @return The node.
	 */
	[[nodiscard]] Node node(Index node) const
	{
		assert(valid(node));
		return Node(code(node), node);
	}

	/*!
	 * @brief Returns the node corresponding to `node`.
	 *
	 * @param node the node
	 * @return The node.
	 */
	[[nodiscard]] Node node(Node node) const { return Node(node.code(), index(node)); }

	/*!
	 * @brief Returns the node corresponding to `node`.
	 *
	 * @param node the node
	 * @return The node.
	 */
	[[nodiscard]] Node node(Code node) const { return Node(node, index(node)); }

	/*!
	 * @brief Returns the node corresponding to `node`.
	 *
	 * @param node the node
	 * @return The node.
	 */
	[[nodiscard]] Node node(Key node) const { return Node(code(node)); }

	/*!
	 * @brief Returns the node corresponding to `node`.
	 *
	 * @param node the node
	 * @return The node.
	 */
	[[nodiscard]] Node node(Coord node) const { return node(code(node)); }

	/*!
	 * @brief Returns the node corresponding to `node`.
	 *
	 * @param node the node
	 * @return The node.
	 */
	[[nodiscard]] Node operator[](Index node) const { return this->node(node); }

	/*!
	 * @brief Returns the node corresponding to `node`.
	 *
	 * @param node the node
	 * @return The node.
	 */
	[[nodiscard]] Node operator[](Node node) const { return this->node(node); }

	/*!
	 * @brief Get the node corresponding to a code.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNode' function can be used. The
	 * data inside the nodes returned by this function and 'createNode' will be the same, so
	 * it is only neccessary to use 'createNode' if you intend to alter what the node
	 * stores.
	 *
	 * @param node The node.
	 * @return The node.
	 */
	[[nodiscard]] Node operator[](Code node) const { return this->node(node); }

	/*!
	 * @brief Get the node corresponding to a key.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNode' function can be used. The
	 * data inside the nodes returned by this function and 'createNode' will be the same, so
	 * it is only neccessary to use 'createNode' if you intend to alter what the node
	 * stores.
	 *
	 * @param node The node.
	 * @return The node.
	 */
	[[nodiscard]] Node operator[](Key node) const { return this->node(node); }

	/*!
	 * @brief Get the node corresponding to a coordinate at a specific depth.
	 *
	 * @note The node can be higher up the tree than the specified depth. This happens if
	 * the node at a higher depth has no children. If it is neccessary that the node is at
	 * the specified depth, then the corresponding 'createNode' function can be used. The
	 * data inside the nodes returned by this function and 'createNode' will be the same, so
	 * it is only neccessary to use 'createNode' if you intend to alter what the node
	 * stores.
	 *
	 * @param coord The coordinate.
	 * @param depth The depth.
	 * @return The node.
	 */
	[[nodiscard]] Node operator[](Coord node) const { return this->node(node); }

	//
	// Code
	//

	[[nodiscard]] Code code() const { return Code(static_cast<code_t>(0), depth()); }

	[[nodiscard]] Code code(Index node) const
	{
		assert(valid(node));
		return blocks_[node.pos].parent_code.child(node.offset);
	}

	[[nodiscard]] Code code(Node node) const { return node.code(); }

	[[nodiscard]] Code code(Key node) const { return Code(node); }

	[[nodiscard]] Code code(Coord node) const { return code(key(node)); }

	[[nodiscard]] std::optional<Code> codeChecked(Coord node) const
	{
		return valid(node) ? std::optional<Code>(Code(node)) : std::nullopt;
	}

	//
	// Key
	//

	[[nodiscard]] Key key() const { return Key(0, 0, 0, depth()); }

	[[nodiscard]] Key key(Index node) const
	{
		return blocks_[node.pos].parent_code.child(node.offset);
	}

	[[nodiscard]] Key key(Node node) const { return Key(node.code()); }

	[[nodiscard]] Key key(Code node) const { return Key(node); }

	[[nodiscard]] Key key(Coord node) const
	{
		assert(valid(node));

		auto lr = lengthReciprocal(depth(node));

		Key k;
		for (std::size_t i{}; node.size() != i; ++i) {
			k[i] = static_cast<key_t>(
			           static_cast<std::make_signed_t<key_t>>(std::floor(node[i] * lr))) +
			       half_max_value_;
		}
		k.depth = depth(node);
		return k;
	}

	[[nodiscard]] std::optional<Key> keyChecked(Coord node) const
	{
		return valid(node) ? std::optional<Key>(key(node)) : std::nullopt;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                 Create/erase nodes                                  |
	|                                                                                     |
	**************************************************************************************/

	//
	// Create
	//

	Index create(Node node) { return create(code(node), index(node)); }

	Index create(Code node) { return create(node, index()); }

	Index create(Key node) { return create(code(node)); }

	Index create(Coord node) { return create(code(node)); }

	template <
	    class InputIt,
	    std::enable_if_t<std::is_same_v<Code, std::decay_t<typename std::iterator_traits<
	                                              InputIt>::value_type>>,
	                     bool> = true>
	std::vector<Index> create(InputIt first, InputIt last)
	{
		std::vector<Index> nodes;
		nodes.reserve(std::distance(first, last));

		std::array<Index, maxNumDepthLevels()> node;
		auto                                   cur_depth = depth();
		node[cur_depth]                                  = index();
		Code prev_code                                   = code();
		for (; first != last; ++first) {
			Code code         = *first;
			auto wanted_depth = depth(code);
			cur_depth         = Code::depthWhereEqual(prev_code, code);
			prev_code         = code;

			for (; wanted_depth < cur_depth; --cur_depth) {
				node[cur_depth - 1] = createChild(node[cur_depth], code.offset[cur_depth - 1]);
			}
			nodes.push_back(node[cur_depth]);
		}

		return nodes;
	}

	template <
	    class InputIt,
	    std::enable_if_t<!std::is_same_v<Code, std::decay_t<typename std::iterator_traits<
	                                               InputIt>::value_type>>,
	                     bool> = true>
	std::vector<Index> create(InputIt first, InputIt last)
	{
		std::vector<Code> codes;
		codes.reserve(std::distance(first, last));
		std::transform(first, last, std::back_inserter(codes),
		               [this](auto const& v) { return code(v); });
		return create(std::begin(codes), std::end(codes));
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	std::vector<Index> create(ExecutionPolicy&& /* policy */,
	                          std::vector<Code> const& codes)
	{
		// TODO: Make it possible to use this without it having to be a `std::vector<Code>`
		// specifically (like the two functions above)

		if constexpr (std::is_same_v<execution::sequenced_policy,
		                             std::decay_t<ExecutionPolicy>>) {
			return create(std::begin(codes), std::end(codes));
		}

#if !defined(UFO_TBB) && !defined(UFO_OMP)
		return create(std::begin(codes), std::end(codes));
#else
		// FIXME: Implement, using `createChildThreadSafe`
		// FIXME: Remove when above has been implemented
		return create(std::begin(codes), std::end(codes));
#endif
	}

	template <class Range>
	std::vector<Index> create(Range const& r)
	{
		return create(std::begin(r), std::end(r));
	}

	pos_t createChildren(Index node)
	{
		assert(!isPureLeaf(node));
		if (isParent(node)) {
			return children(node);
		}

		pos_t block;
		if (free_blocks_.empty()) {
			block = createBlock(node);
		} else {
			block = free_blocks_.front();
			free_blocks_.pop_front();
			fillBlock(node, block);
		}

		return block;
	}

	Index createChild(Index node, offset_t child_index)
	{
		assert(0 < depth(node));
		assert(branchingFactor() > child_index);
		return Index(createChildren(node), child_index);
	}

	//
	// Erase
	//

	void eraseChildren(Index node)
	{
		assert(valid(node));
		erase(node, children(node));
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Leaf                                         |
	|                                                                                     |
	**************************************************************************************/

	//
	// Pure leaf
	//

	/*!
	 * @brief Checks if the node is a pure leaf (i.e., can never have children).
	 *
	 * @note Only have to check if the depth of the node is 0.
	 *
	 * @param node the node to check
	 * @return `true` if the node is a pure leaf, `false` otherwise.
	 */
	[[nodiscard]] bool isPureLeaf(Index node) const { return 0 == depth(node); }

	/*!
	 * @brief Checks if the node is a pure leaf (i.e., can never have children).
	 *
	 * @note Only have to check if the depth of the node is 0.
	 *
	 * @param node the node to check
	 * @return `true` if the node is a pure leaf, `false` otherwise.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Node node) noexcept
	{
		return 0 == depth(node);
	}

	/*!
	 * @brief Checks if the node is a pure leaf (i.e., can never have children).
	 *
	 * @note Only have to check if the depth of the node is 0.
	 *
	 * @param node the node to check
	 * @return `true` if the node is a pure leaf, `false` otherwise.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Code node) noexcept
	{
		return 0 == depth(node);
	}

	/*!
	 * @brief Checks if the node is a pure leaf (i.e., can never have children).
	 *
	 * @note Only have to check if the depth of the node is 0.
	 *
	 * @param node the node to check
	 * @return `true` if the node is a pure leaf, `false` otherwise.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Key node) noexcept
	{
		return 0 == depth(node);
	}

	/*!
	 * @brief Checks if the node is a pure leaf (i.e., can never have children).
	 *
	 * @note Only have to check if the depth of the node is 0.
	 *
	 * @param node the node to check
	 * @return `true` if the node is a pure leaf, `false` otherwise.
	 */
	[[nodiscard]] static constexpr bool isPureLeaf(Coord node) noexcept
	{
		return 0 == depth(node);
	}

	//
	// Leaf
	//

	/*!
	 * @brief Checks if the node is a leaf (i.e., has no children).
	 *
	 * @param node the node to check
	 * @return `true` if the node is a leaf, `false` otherwise.
	 */
	[[nodiscard]] bool isLeaf(Index node) const
	{
		return TreeIndex::NULL_POS == children(node);
	}

	/*!
	 * @brief Checks if the node is a leaf (i.e., has no children).
	 *
	 * @param node the node to check
	 * @return `true` if the node is a leaf, `false` otherwise.
	 */
	[[nodiscard]] bool isLeaf(Node node) const { return isLeaf(index(node)); }

	/*!
	 * @brief Checks if the node is a leaf (i.e., has no children).
	 *
	 * @param node the node to check
	 * @return `true` if the node is a leaf, `false` otherwise.
	 */
	[[nodiscard]] bool isLeaf(Code node) const { return isLeaf(index(node)); }

	/*!
	 * @brief Checks if the node is a leaf (i.e., has no children).
	 *
	 * @param node the node to check
	 * @return `true` if the node is a leaf, `false` otherwise.
	 */
	[[nodiscard]] bool isLeaf(Key node) const { return isLeaf(index(node)); }

	/*!
	 * @brief Checks if the node is a leaf (i.e., has no children).
	 *
	 * @param node the node to check
	 * @return `true` if the node is a leaf, `false` otherwise.
	 */
	[[nodiscard]] bool isLeaf(Coord node) const { return isLeaf(index(node)); }

	//
	// Parent
	//

	/*!
	 * @brief Checks if the node is a parent (i.e., has children).
	 *
	 * @param node the node to check
	 * @return `true` if the node is a parent, `false` otherwise.
	 */
	[[nodiscard]] bool isParent(Index node) const { return !isLeaf(node); }

	/*!
	 * @brief Checks if the node is a parent (i.e., has children).
	 *
	 * @param node the node to check
	 * @return `true` if the node is a parent, `false` otherwise.
	 */
	[[nodiscard]] bool isParent(Node node) const { return !isLeaf(node); }

	/*!
	 * @brief Checks if the node is a parent (i.e., has children).
	 *
	 * @param node the node to check
	 * @return `true` if the node is a parent, `false` otherwise.
	 */
	[[nodiscard]] bool isParent(Code node) const { return !isLeaf(node); }

	/*!
	 * @brief Checks if the node is a parent (i.e., has children).
	 *
	 * @param node the node to check
	 * @return `true` if the node is a parent, `false` otherwise.
	 */
	[[nodiscard]] bool isParent(Key node) const { return !isLeaf(node); }

	/*!
	 * @brief Checks if the node is a parent (i.e., has children).
	 *
	 * @param node the node to check
	 * @return `true` if the node is a parent, `false` otherwise.
	 */
	[[nodiscard]] bool isParent(Coord node) const { return !isLeaf(node); }

	/**************************************************************************************
	|                                                                                     |
	|                                        Root                                         |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Checks if the node is the root of the tree.
	 *
	 * @param node the node to check
	 * @return `true` if the node is the root, `false` otherwise.
	 */
	[[nodiscard]] bool isRoot(Index node) const { return index() == node; }

	/*!
	 * @brief Checks if the node is the root of the tree.
	 *
	 * @param node the node to check
	 * @return `true` if the node is the root, `false` otherwise.
	 */
	[[nodiscard]] bool isRoot(Node node) const { return isRoot(node.code()); }

	/*!
	 * @brief Checks if the node is the root of the tree.
	 *
	 * @param node the node to check
	 * @return `true` if the node is the root, `false` otherwise.
	 */
	[[nodiscard]] bool isRoot(Code node) const { return code() == node; }

	/*!
	 * @brief Checks if the node is the root of the tree.
	 *
	 * @param node the node to check
	 * @return `true` if the node is the root, `false` otherwise.
	 */
	[[nodiscard]] bool isRoot(Key node) const { return key() == node; }

	/*!
	 * @brief Checks if the node is the root of the tree.
	 *
	 * @param node the node to check
	 * @return `true` if the node is the root, `false` otherwise.
	 */
	[[nodiscard]] bool isRoot(Coord node) const { return isRoot(key(node)); }

	/**************************************************************************************
	|                                                                                     |
	|                                        Valid                                        |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] bool valid(pos_t block) const { return blocks_.size() > block; }

	[[nodiscard]] bool valid(Index index) const
	{
		return valid(index.pos) && branchingFactor() > index.offset &&
		       blocks_[index.pos].parent_code.valid();
	}

	[[nodiscard]] bool valid(Node node) const { return valid(code(node)); }

	[[nodiscard]] bool valid(Code code) const
	{
		return code.valid() && numDepthLevels() > depth(code);
	}

	[[nodiscard]] bool valid(Key key) const
	{
		// FIXME: This should be checked inside `key.valid()`
		auto const mv = 2 * half_max_value_;
		for (std::size_t i{}; key.size() != i; ++i) {
			if (mv < key[i]) {
				return false;
			}
		}

		return key.valid() && numDepthLevels() > depth(key);
	}

	[[nodiscard]] bool valid(Coord coord) const
	{
		return isInside(coord) && numDepthLevels() > depth(coord);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Exist                                        |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Checks if a node exists.
	 *
	 * @param node the node to check
	 * @return `true` if the node exists, `false` otherwise.
	 */
	[[nodiscard]] bool exists(Node node) const { return code(index(node)) == code(node); }

	/*!
	 * @brief Checks if a node exists.
	 *
	 * @param node the node to check
	 * @return `true` if the node exists, `false` otherwise.
	 */
	[[nodiscard]] bool exists(Code node) const { return code(index(node)) == node; }

	/*!
	 * @brief Checks if a node exists.
	 *
	 * @param node the node to check
	 * @return `true` if the node exists, `false` otherwise.
	 */
	[[nodiscard]] bool exists(Key node) const { return exists(code(node)); }

	/*!
	 * @brief Checks if a node exists.
	 *
	 * @param node the node to check
	 * @return `true` if the node exists, `false` otherwise.
	 */
	[[nodiscard]] bool exists(Coord node) const { return exists(code(node)); }

	/**************************************************************************************
	|                                                                                     |
	|                                      Traverse                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] std::array<pos_t, branchingFactor()> children(pos_t block) const
	{
		assert(valid(block));
		return blocks_[block].children;
	}

	[[nodiscard]] pos_t children(Index node) const
	{
		assert(valid(node));
		assert(isParent(node));
		return children(node.pos)[node.offset];
	}

	[[nodiscard]] Index child(Index node, offset_t child_index) const
	{
		assert(valid(node));
		assert(branchingFactor() > child_index);
		assert(isParent(node));
		return {children(node), child_index};
	}

	/*!
	 * @brief Get a child of a node.
	 *
	 * @param node The node.
	 * @param child_index The index of the child.
	 * @return The child.
	 */
	[[nodiscard]] Node child(Node node, offset_t child_index) const
	{
		assert(0 < depth(node));
		assert(branchingFactor() > child_index);
		return Node(node.code().child(child_index),
		            isParent(node) ? child(node.index(), child_index) : node.index());
	}

	[[nodiscard]] Code child(Code node, offset_t child_index) const
	{
		// TODO: Implement
	}

	[[nodiscard]] Key child(Key node, offset_t child_index) const
	{
		// TODO: Implement
	}

	[[nodiscard]] Coord child(Coord node, offset_t child_index) const
	{
		assert(0 < depth(node));
		return Coord(childCenter(node, child_index), node.depth - 1);
	}

	/*!
	 * @brief Get a child of a node.
	 *
	 * @param node The node.
	 * @param child_index The index of the child.
	 * @return The child.
	 */
	[[nodiscard]] Node childUnsafe(Node node, offset_t child_index) const
	{
		assert(0 < depth(node));
		assert(branchingFactor() > child_index);
		assert(isParent(node));
		return Node(child(node.code(), child_index), child(node.index(), child_index));
	}

	/*!
	 * @brief Get a child of a node with bounds checking.
	 *
	 * @param node The node.
	 * @param child_index The index of the child.
	 * @return The child.
	 */
	[[nodiscard]] Node childChecked(Node node, offset_t child_index) const
	{
		if (isLeaf(node)) {
			throw std::out_of_range("Node has no children");
		} else if (branchingFactor() <= child_index) {
			throw std::out_of_range("child_index out of range");
		} else {
			return child(node, child_index);
		}
	}

	//
	// Sibling
	//

	[[nodiscard]] Index sibling(Index node, offset_t s) const
	{
		assert(branchingFactor() > s);
		return {node.pos, s};
	}

	/*!
	 * @brief Get the sibling of a node.
	 *
	 * @param node The node.
	 * @param sibling_index The index of the sibling.
	 * @return The sibling.
	 */
	[[nodiscard]] Node sibling(Node node, offset_t sibling_index) const
	{
		assert(branchingFactor() > sibling_index);
		return Node(node.code().sibling(sibling_index),
		            exists(node) ? node.index().sibling(sibling_index) : node.index());
	}

	[[nodiscard]] Code sibling(Code node, offset_t sibling_index) const
	{
		// TODO: Implement
	}

	[[nodiscard]] Key sibling(Key node, offset_t sibling_index) const
	{
		// TODO: Implement
	}

	[[nodiscard]] Coord sibling(Coord node, offset_t sibling_index) const
	{
		// TODO: Implement
	}

	/*!
	 * @brief Get the sibling of a node.
	 *
	 * @param node The node.
	 * @param sibling_index The index of the sibling.
	 * @return The sibling.
	 */
	[[nodiscard]] Node siblingUnsafe(Node node, offset_t sibling_index) const
	{
		assert(branchingFactor() > sibling_index);
		return Node(node.code().sibling(sibling_index), node.index().sibling(sibling_index));
	}

	/*!
	 * @brief Get the sibling of a node with bounds checking.
	 *
	 * @param node The node.
	 * @param sibling_index The index of the sibling.
	 * @return The sibling.
	 */
	[[nodiscard]] Node siblingChecked(Node node, offset_t sibling_index) const
	{
		if (!isRoot(node)) {
			throw std::out_of_range("Node has no siblings");
		} else if (branchingFactor() <= sibling_index) {
			throw std::out_of_range("sibling_index out of range");
		}
		return sibling(node, sibling_index);
	}

	//
	// Parent
	//

	[[nodiscard]] Index parent(Index node) const
	{
		assert(!isRoot(node));
		return index(blocks_[node.pos].parent_code);
	}

	/*!
	 * @brief Get the parent of a node.
	 *
	 * @param node The node.
	 * @return The parent.
	 */
	[[nodiscard]] Node parent(Node node) const
	{
		assert(!isRoot(node));
		return this->node(node.code().parent());
	}

	[[nodiscard]] Code parent(Code node) const
	{
		assert(!isRoot(node));
		return node.parent();
	}

	[[nodiscard]] Key parent(Key node) const
	{
		assert(!isRoot(node));
		return node.parent();
	}

	[[nodiscard]] Coord parent(Coord node) const
	{
		assert(!isRoot(node));
		return center(parent(key(node)));
	}

	/*!
	 * @brief Depth first traversal of the tree, starting at the root node. The function
	 * 'f' will be called for each node traverse. If 'f' returns true then the children of
	 * the node will also be traverse, otherwise they will not.
	 *
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class UnaryFun,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryFun, Index>, bool> = true>
	void traverse(UnaryFun f) const
	{
		traverse(index(), f);
	}

	/*!
	 * @brief Depth first traversal of the tree, starting at the root node. The function
	 * 'f' will be called for each node traverse. If 'f' returns true then the children of
	 * the node will also be traverse, otherwise they will not.
	 *
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class UnaryFun,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryFun, Node>, bool> = true>
	void traverse(UnaryFun f, bool only_exists = true) const
	{
		traverse(node(), f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the tree, starting at the node. The function 'f'
	 * will be called for each node traverse. If 'f' returns true then the children of the
	 * node will also be traverse, otherwise they will not.
	 *
	 * @param node The node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class UnaryFun,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryFun, Index>, bool> = true>
	void traverse(Index node, UnaryFun f) const
	{
		if (!f(node) || isLeaf(node)) {
			return;
		}

		std::array<Index, maxNumDepthLevels()> nodes;
		nodes[1] = child(node, 0);
		for (std::size_t i{1}; 0 != i;) {
			node = nodes[i];
			i -= branchingFactor() <= ++nodes[i].offset;
			if (f(node) && isParent(node)) {
				nodes[++i] = child(node, 0);
			}
		}
	}

	template <class UnaryFun,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryFun, Node>, bool> = true>
	void traverse(Index node, UnaryFun f, bool only_exists = true) const
	{
		traverse(this->node(node), f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the tree, starting at the node. The function 'f'
	 * will be called for each node traverse. If 'f' returns true then the children of the
	 * node will also be traverse, otherwise they will not.
	 *
	 * @param node The node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class UnaryFun,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryFun, Node>, bool> = true>
	void traverse(Node node, UnaryFun f, bool only_exists = true) const
	{
		std::array<Node, maxNumDepthLevels()> nodes;
		nodes[0] = node;

		if (only_exists) {
			if (!exists(node)) {
				return;
			}
			for (int depth{}; 0 <= depth;) {
				node        = nodes[depth];
				auto offset = nodes[depth].offset();
				if (branchingFactor() - 1 > offset) {
					nodes[depth] = sibling(nodes[depth], offset + 1);
				} else {
					--depth;
				}
				if (f(node) && isParent(node)) {
					nodes[++depth] = child(node, 0);
				}
			}
		} else {
			for (int depth{}; 0 <= depth;) {
				node        = nodes[depth];
				auto offset = nodes[depth].offset();
				if (branchingFactor() - 1 > offset) {
					nodes[depth] = sibling(nodes[depth], offset + 1);
				} else {
					--depth;
				}
				if (f(node) && !isPureLeaf(node)) {
					nodes[++depth] = child(node, 0);
				}
			}
		}
	}

	/*!
	 * @brief Depth first traversal of the tree, starting at the node corresponding to the
	 * code. The function 'f' will be called for each node traverse. If 'f' returns true
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param node The code to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class UnaryFun,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryFun, Node>, bool> = true>
	void traverse(Code node, UnaryFun f, bool only_exists = true) const
	{
		traverse(this->node(node), f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the tree, starting at the node corresponding to the
	 * key. The function 'f' will be called for each node traverse. If 'f' returns true
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param node The key to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <class UnaryFun,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryFun, Node>, bool> = true>
	void traverse(Key node, UnaryFun f, bool only_exists = true) const
	{
		traverse(this->node(node), f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the tree, starting at the node corresponding to the
	 * coordinate at a specified depth. The function 'f' will be called for each node
	 * traverse. If 'f' returns true then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param coord The coord to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 * @param depth The depth of the node.
	 */
	template <class UnaryFun,
	          std::enable_if_t<std::is_invocable_r_v<bool, UnaryFun, Node>, bool> = true>
	void traverse(Coord node, UnaryFun f, bool only_exists = true) const
	{
		traverse(this->node(node), f, only_exists);
	}

	/*! TODO: Update info for all nearest
	 * @brief Traverse the tree in the orderDepth first traversal of the tree, starting
	 * at the root node. The function 'f' will be called for each node traverse. If 'f'
	 * returns true then the children of the node will also be traverse, otherwise they
	 * will not.
	 *
	 * @param f The callback function to be called for each node traversed.
	 */
	template <
	    class Geometry, class UnaryFun,
	    std::enable_if_t<std::is_invocable_r_v<bool, UnaryFun, NodeNearest>, bool> = true>
	void traverseNearest(Geometry const& g, UnaryFun f, bool only_exists = true) const
	{
		traverseNearest(node(), g, f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the tree, starting at the node. The function 'f'
	 * will be called for each node traverse. If 'f' returns true then the children of the
	 * node will also be traverse, otherwise they will not.
	 *
	 * @param node The node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <
	    class Geometry, class UnaryFun,
	    std::enable_if_t<std::is_invocable_r_v<bool, UnaryFun, NodeNearest>, bool> = true>
	void traverseNearest(Node node, Geometry const& g, UnaryFun f,
	                     bool only_exists = true) const
	{
		// TODO: Implement

		// std::priority_queue<NodeNearest, std::vector<NodeNearest>,
		// std::greater<NodeNearest>>
		//        nodes;
		// NodeBV nbv = toNodeBV(node);
		// nodes.emplace(nbv, squaredDistance(nbv.boundingVolume(), g));

		// if (only_exists) {
		// 	if (!exists(node)) {
		// 		return;
		// 	}
		// 	while (!nodes.empty()) {
		// 		auto n_d = nodes.top();
		// 		nodes.pop();

		// 		if (f(n_d) && isParent(n_d)) {
		// 			for (offset_t i{}; branchingFactor() != i; ++i) {
		// 				auto c = child(n_d, i);
		// 				nodes.emplace(c, squaredDistance(c.boundingVolume(), g));
		// 			}
		// 		}
		// 	}
		// } else {
		// 	while (!nodes.empty()) {
		// 		auto n_d = nodes.top();
		// 		nodes.pop();

		// 		if (f(n_d) && !isPureLeaf(n_d)) {
		// 			for (offset_t i{}; branchingFactor() != i; ++i) {
		// 				auto c = child(n_d, i);
		// 				nodes.emplace(c, squaredDistance(c.boundingVolume(), g));
		// 			}
		// 		}
		// 	}
		// }
	}

	/*!
	 * @brief Depth first traversal of the tree, starting at the node corresponding to the
	 * code. The function 'f' will be called for each node traverse. If 'f' returns true
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param node The code to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <
	    class Geometry, class UnaryFun,
	    std::enable_if_t<std::is_invocable_r_v<bool, UnaryFun, NodeNearest>, bool> = true>
	void traverseNearest(Code node, Geometry const& g, UnaryFun f,
	                     bool only_exists = true) const
	{
		traverseNearest(this->node(node), g, f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the tree, starting at the node corresponding to the
	 * key. The function 'f' will be called for each node traverse. If 'f' returns true
	 * then the children of the node will also be traverse, otherwise they will not.
	 *
	 * @param node The key to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 */
	template <
	    class Geometry, class UnaryFun,
	    std::enable_if_t<std::is_invocable_r_v<bool, UnaryFun, NodeNearest>, bool> = true>
	void traverseNearest(Key node, Geometry const& g, UnaryFun f,
	                     bool only_exists = true) const
	{
		traverseNearest(this->node(node), g, f, only_exists);
	}

	/*!
	 * @brief Depth first traversal of the tree, starting at the node corresponding to the
	 * coordinate at a specified depth. The function 'f' will be called for each node
	 * traverse. If 'f' returns true then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param coord The coord to the node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
	 * @param depth The depth of the node.
	 */
	template <
	    class Geometry, class UnaryFun,
	    std::enable_if_t<std::is_invocable_r_v<bool, UnaryFun, NodeNearest>, bool> = true>
	void traverseNearest(Coord node, Geometry const& g, UnaryFun f,
	                     bool only_exists = true) const
	{
		traverseNearest(this->node(node), g, f, only_exists);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Iterators                                      |
	|                                                                                     |
	**************************************************************************************/

	//
	// Iterator
	//

	[[nodiscard]] const_iterator begin(bool only_leaves = true, bool only_exists = true,
	                                   bool early_stopping = false) const
	{
		return begin(node(), only_leaves, only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator begin(Index node, bool only_leaves = true,
	                                   bool only_exists    = true,
	                                   bool early_stopping = false) const
	{
		return beginQuery(this->node(node), only_leaves, only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator begin(Node node, bool only_leaves = true,
	                                   bool only_exists    = true,
	                                   bool early_stopping = false) const
	{
		return only_leaves ? beginQuery(node, pred::Leaf{}, only_exists, early_stopping)
		                   : beginQuery(node, pred::True{}, only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator begin(Code node, bool only_leaves = true,
	                                   bool only_exists    = true,
	                                   bool early_stopping = false) const
	{
		return begin(this->node(node), only_leaves, only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator begin(Key node, bool only_leaves = true,
	                                   bool only_exists    = true,
	                                   bool early_stopping = false) const
	{
		return begin(this->node(node), only_leaves, only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator begin(Coord node, bool only_leaves = true,
	                                   bool only_exists    = true,
	                                   bool early_stopping = false) const
	{
		return begin(this->node(node), only_leaves, only_exists, early_stopping);
	}

	[[nodiscard]] const_iterator end() const { return endQuery(); }

	//
	// Nearest iterator
	//

	template <class Geometry>  // FIXME: Add something for geometry
	[[nodiscard]] const_nearest_iterator beginNearest(Geometry const& geometry,
	                                                  double          epsilon     = 0.0,
	                                                  bool            only_leaves = true,
	                                                  bool            only_exists = true,
	                                                  bool early_stopping = false) const
	{
		return beginNearest(node(), geometry, epsilon, only_leaves, only_exists,
		                    early_stopping);
	}

	template <class Geometry>  // FIXME: Add something for geometry
	[[nodiscard]] const_nearest_iterator beginNearest(Index node, Geometry const& geometry,
	                                                  double epsilon        = 0.0,
	                                                  bool   only_leaves    = true,
	                                                  bool   only_exists    = true,
	                                                  bool   early_stopping = false) const
	{
		return beginNearest(this->node(node), geometry, epsilon, only_leaves, only_exists,
		                    early_stopping);
	}

	template <class Geometry>  // FIXME: Add something for geometry
	[[nodiscard]] const_nearest_iterator beginNearest(Node node, Geometry const& geometry,
	                                                  double epsilon        = 0.0,
	                                                  bool   only_leaves    = true,
	                                                  bool   only_exists    = true,
	                                                  bool   early_stopping = false) const
	{
		return only_leaves ? beginQueryNearest(node, geometry, pred::Leaf{}, epsilon,
		                                       only_exists, early_stopping)
		                   : beginQueryNearest(node, geometry, pred::True{}, epsilon,
		                                       only_exists, early_stopping);
	}

	template <class Geometry>  // FIXME: Add something for geometry
	[[nodiscard]] const_nearest_iterator beginNearest(Code node, Geometry const& geometry,
	                                                  double epsilon        = 0.0,
	                                                  bool   only_leaves    = true,
	                                                  bool   only_exists    = true,
	                                                  bool   early_stopping = false) const
	{
		return beginNearest(this->node(node), geometry, epsilon, only_leaves, only_exists,
		                    early_stopping);
	}

	template <class Geometry>  // FIXME: Add something for geometry
	[[nodiscard]] const_nearest_iterator beginNearest(Key node, Geometry const& geometry,
	                                                  double epsilon        = 0.0,
	                                                  bool   only_leaves    = true,
	                                                  bool   only_exists    = true,
	                                                  bool   early_stopping = false) const
	{
		return beginNearest(this->node(node), geometry, epsilon, only_leaves, only_exists,
		                    early_stopping);
	}

	template <class Geometry>  // FIXME: Add something for geometry
	[[nodiscard]] const_nearest_iterator beginNearest(Coord node, Geometry const& geometry,
	                                                  double epsilon        = 0.0,
	                                                  bool   only_leaves    = true,
	                                                  bool   only_exists    = true,
	                                                  bool   early_stopping = false) const
	{
		return beginNearest(this->node(node), geometry, epsilon, only_leaves, only_exists,
		                    early_stopping);
	}

	[[nodiscard]] const_nearest_iterator endNearest() const { return endQueryNearest(); }

	//
	// Query iterator
	//

	template <class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] const_query_iterator beginQuery(Predicate const& predicate,
	                                              bool             only_exists = true,
	                                              bool early_stopping = false) const
	{
		return beginQuery(node(), predicate, only_exists, early_stopping);
	}

	template <class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] const_query_iterator beginQuery(Index node, Predicate const& predicate,
	                                              bool only_exists    = true,
	                                              bool early_stopping = false) const
	{
		return beginQuery(this->node(node), predicate, only_exists, early_stopping);
	}

	template <class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] const_query_iterator beginQuery(Node node, Predicate const& predicate,
	                                              bool only_exists    = true,
	                                              bool early_stopping = false) const
	{
		if (only_exists) {
			if (early_stopping) {
				return {new TreeForwardIterator<Derived, Node, Predicate, true, true>(
				    &derived(), node, predicate)};
			} else {
				return {new TreeForwardIterator<Derived, Node, Predicate, true, false>(
				    &derived(), node, predicate)};
			}
		} else {
			if (early_stopping) {
				return {new TreeForwardIterator<Derived, Node, Predicate, false, true>(
				    &derived(), node, predicate)};
			} else {
				return {new TreeForwardIterator<Derived, Node, Predicate, false, false>(
				    &derived(), node, predicate)};
			}
		}
	}

	template <class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] const_query_iterator beginQuery(Code node, Predicate const& predicate,
	                                              bool only_exists    = true,
	                                              bool early_stopping = false) const
	{
		return beginQuery(this->node(node), predicate, only_exists, early_stopping);
	}

	template <class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] const_query_iterator beginQuery(Key node, Predicate const& predicate,
	                                              bool only_exists    = true,
	                                              bool early_stopping = false) const
	{
		return beginQuery(this->node(node), predicate, only_exists, early_stopping);
	}

	template <class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] const_query_iterator beginQuery(Coord node, Predicate const& predicate,
	                                              bool only_exists    = true,
	                                              bool early_stopping = false) const
	{
		return beginQuery(this->node(node), predicate, only_exists, early_stopping);
	}

	[[nodiscard]] const_query_iterator endQuery() const
	{
		return {new TreeForwardIterator<Derived, Node, pred::True, true, true>(&derived())};
	}

	//
	// Query nearest iterator
	//

	// FIXME: Add something for geometry
	template <class Geometry, class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] const_nearest_query_iterator beginQueryNearest(
	    Geometry const& geometry, Predicate const& predicate, double epsilon = 0.0,
	    bool only_exists = true, bool early_stopping = false) const
	{
		return beginQueryNearest(node(), geometry, predicate, epsilon, only_exists,
		                         early_stopping);
	}

	// FIXME: Add something for geometry
	template <class Geometry, class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] const_nearest_query_iterator beginQueryNearest(
	    Index node, Geometry const& geometry, Predicate const& predicate,
	    double epsilon = 0.0, bool only_exists = true, bool early_stopping = false) const
	{
		return beginQueryNearest(this->node(node), geometry, predicate, epsilon, only_exists,
		                         early_stopping);
	}

	// FIXME: Add something for geometry
	template <class Geometry, class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] const_nearest_query_iterator beginQueryNearest(
	    Node node, Geometry const& geometry, Predicate const& predicate,
	    double epsilon = 0.0, bool only_exists = true, bool early_stopping = false) const
	{
		if (only_exists) {
			if (early_stopping) {
				return {new TreeNearestIterator<Derived, Node, Geometry, Predicate, true, true>(
				    &derived(), node, geometry, predicate, epsilon)};
			} else {
				return {new TreeNearestIterator<Derived, Node, Geometry, Predicate, true, false>(
				    &derived(), node, geometry, predicate, epsilon)};
			}
		} else {
			if (early_stopping) {
				return {new TreeNearestIterator<Derived, Node, Geometry, Predicate, false, true>(
				    &derived(), node, geometry, predicate, epsilon)};
			} else {
				return {new TreeNearestIterator<Derived, Node, Geometry, Predicate, false, false>(
				    &derived(), node, geometry, predicate, epsilon)};
			}
		}
	}

	// FIXME: Add something for geometry
	template <class Geometry, class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] const_nearest_query_iterator beginQueryNearest(
	    Code node, Geometry const& geometry, Predicate const& predicate,
	    double epsilon = 0.0, bool only_exists = true, bool early_stopping = false) const
	{
		return beginQueryNearest(this->node(node), geometry, predicate, epsilon, only_exists,
		                         early_stopping);
	}

	// FIXME: Add something for geometry
	template <class Geometry, class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] const_nearest_query_iterator beginQueryNearest(
	    Key node, Geometry const& geometry, Predicate const& predicate,
	    double epsilon = 0.0, bool only_exists = true, bool early_stopping = false) const
	{
		return beginQueryNearest(this->node(node), geometry, predicate, epsilon, only_exists,
		                         early_stopping);
	}

	// FIXME: Add something for geometry
	template <class Geometry, class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] const_nearest_query_iterator beginQueryNearest(
	    Coord node, Geometry const& geometry, Predicate const& predicate,
	    double epsilon = 0.0, bool only_exists = true, bool early_stopping = false) const
	{
		return beginQueryNearest(this->node(node), geometry, predicate, epsilon, only_exists,
		                         early_stopping);
	}

	[[nodiscard]] const_nearest_query_iterator endQueryNearest() const
	{
		return const_nearest_query_iterator(
		    new TreeNearestIterator<Derived, Node, Point, pred::True, true, true>());
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Query                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	//
	// Query
	//

	template <class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] Query query(Predicate const& predicate, bool only_exists = true,
	                          bool early_stopping = false) const
	{
		return query(node(), predicate, only_exists, early_stopping);
	}

	template <class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] Query query(Index node, Predicate const& predicate,
	                          bool only_exists = true, bool early_stopping = false) const
	{
		return query(this->node(node), predicate, only_exists, early_stopping);
	}

	template <class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] Query query(Node node, Predicate const& predicate,
	                          bool only_exists = true, bool early_stopping = false) const
	{
		return Query(beginQuery(node, predicate, only_exists, early_stopping), endQuery());
	}

	template <class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] Query query(Code node, Predicate const& predicate,
	                          bool only_exists = true, bool early_stopping = false) const
	{
		return query(this->node(node), predicate, only_exists, early_stopping);
	}

	template <class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] Query query(Key node, Predicate const& predicate, bool only_exists = true,
	                          bool early_stopping = false) const
	{
		return query(this->node(node), predicate, only_exists, early_stopping);
	}

	template <class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] Query query(Coord node, Predicate const& predicate,
	                          bool only_exists = true, bool early_stopping = false) const
	{
		return query(this->node(node), predicate, only_exists, early_stopping);
	}

	//
	// Query nearest
	//

	// FIXME: Add something for geometry
	template <class Geometry, class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] QueryNearest queryNearest(Geometry const&  geometry,
	                                        Predicate const& predicate = pred::True{},
	                                        double epsilon = 0.0, bool only_exists = true,
	                                        bool early_stopping = false) const
	{
		return queryNearest(node(), geometry, predicate, epsilon, only_exists,
		                    early_stopping);
	}

	// FIXME: Add something for geometry
	template <class Geometry, class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] QueryNearest queryNearest(Index node, Geometry const& geometry,
	                                        Predicate const& predicate = pred::True{},
	                                        double epsilon = 0.0, bool only_exists = true,
	                                        bool early_stopping = false) const
	{
		return queryNearest(this->node(node), geometry, predicate, epsilon, only_exists,
		                    early_stopping);
	}

	// FIXME: Add something for geometry
	template <class Geometry, class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] QueryNearest queryNearest(Node node, Geometry const& geometry,
	                                        Predicate const& predicate = pred::True{},
	                                        double epsilon = 0.0, bool only_exists = true,
	                                        bool early_stopping = false) const
	{
		return QueryNearest(beginQueryNearest(node, geometry, predicate, epsilon, only_exists,
		                                      early_stopping),
		                    endQueryNearest());
	}

	// FIXME: Add something for geometry
	template <class Geometry, class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] QueryNearest queryNearest(Code node, Geometry const& geometry,
	                                        Predicate const& predicate = pred::True{},
	                                        double epsilon = 0.0, bool only_exists = true,
	                                        bool early_stopping = false) const
	{
		return queryNearest(this->node(node), geometry, predicate, epsilon, only_exists,
		                    early_stopping);
	}

	// FIXME: Add something for geometry
	template <class Geometry, class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] QueryNearest queryNearest(Key node, Geometry const& geometry,
	                                        Predicate const& predicate = pred::True{},
	                                        double epsilon = 0.0, bool only_exists = true,
	                                        bool early_stopping = false) const
	{
		return queryNearest(this->node(node), geometry, predicate, epsilon, only_exists,
		                    early_stopping);
	}

	// FIXME: Add something for geometry
	template <class Geometry, class Predicate,
	          std::enable_if_t<pred::is_predicate_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] QueryNearest queryNearest(Coord node, Geometry const& geometry,
	                                        Predicate const& predicate = pred::True{},
	                                        double epsilon = 0.0, bool only_exists = true,
	                                        bool early_stopping = false) const
	{
		return queryNearest(this->node(node), geometry, predicate, epsilon, only_exists,
		                    early_stopping);
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	Tree(length_t leaf_node_length, depth_t num_depth_levels)

	{
		if (minNumDepthLevels() > num_depth_levels ||
		    maxNumDepthLevels() < num_depth_levels) {
			throw std::invalid_argument("'num_depth_levels' has to be in range [" +
			                            std::to_string(+minNumDepthLevels()) + ".." +
			                            std::to_string(+maxNumDepthLevels()) + "], '" +
			                            std::to_string(+num_depth_levels) + "' was supplied.");
		}
		if (static_cast<length_t>(0) >= leaf_node_length ||
		    !std::isfinite(leaf_node_length)) {
			throw std::invalid_argument(
			    "'leaf_node_length' has to be finite and greater than zero, '" +
			    std::to_string(leaf_node_length) + "' was supplied.");
		}
		if (!std::isfinite(std::ldexp(leaf_node_length, num_depth_levels - 1))) {
			throw std::invalid_argument(
			    "'leaf_node_length * 2^(num_depth_levels - 1)' has to be finite, '" +
			    std::to_string(std::ldexp(leaf_node_length, num_depth_levels - 1)) +
			    "' was supplied.");
		}
		if (static_cast<length_t>(0) >=
		    static_cast<length_t>(1) / std::ldexp(leaf_node_length, -1)) {
			throw std::invalid_argument(
			    "The reciprocal of half 'leaf_node_length' (i.e., 1 / (leaf_node_length / 2)) "
			    "has to be a greater than zero, '" +
			    std::to_string(static_cast<length_t>(1) / std::ldexp(leaf_node_length, -1)) +
			    "' was supplied.");
		}

		num_depth_levels_ = num_depth_levels;
		half_max_value_ = static_cast<key_t>(1) << (num_depth_levels - 2);  // TODO: Correct?

		// For increased precision
		for (int i{}; node_half_length_.size() > i; ++i) {
			node_half_length_[i]            = std::ldexp(leaf_node_length, i - 1);
			node_half_length_reciprocal_[i] = static_cast<length_t>(1) / node_half_length_[i];
		}

		// Create root
		blocks_.emplace_back(code().parent());
	}

	Tree(Tree const& other)
	    : num_depth_levels_(other.num_depth_levels_)
	    , half_max_value_(other.half_max_value_)
	    , blocks_(other.blocks_)
	    , free_blocks_(other.free_blocks_)
	    , node_half_length_(other.node_half_length_)
	    , node_half_length_reciprocal_(other.node_half_length_reciprocal_)
	{
	}

	Tree(Tree&& other)
	    : num_depth_levels_(std::move(other.num_depth_levels_))
	    , half_max_value_(std::move(other.half_max_value_))
	    , blocks_(std::move(other.blocks_))
	    , free_blocks_(std::move(other.free_blocks_))
	    , node_half_length_(std::move(other.node_half_length_))
	    , node_half_length_reciprocal_(std::move(other.node_half_length_reciprocal_))
	{
	}

	template <class Derived2>
	Tree(Tree<Derived2, TreeBlock, TT> const& other)
	    : num_depth_levels_(other.num_depth_levels_)
	    , half_max_value_(other.half_max_value_)
	    , blocks_(other.blocks_)
	    , free_blocks_(other.free_blocks_)
	    , node_half_length_(other.node_half_length_)
	    , node_half_length_reciprocal_(other.node_half_length_reciprocal_)
	{
	}

	template <class Derived2>
	Tree(Tree<Derived2, TreeBlock, TT>&& other)
	    : num_depth_levels_(std::move(other.num_depth_levels_))
	    , half_max_value_(std::move(other.half_max_value_))
	    , blocks_(std::move(other.blocks_))
	    , free_blocks_(std::move(other.free_blocks_))
	    , node_half_length_(std::move(other.node_half_length_))
	    , node_half_length_reciprocal_(std::move(other.node_half_length_reciprocal_))
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~Tree() {}

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	Tree& operator=(Tree const& rhs)
	{
		num_depth_levels_            = rhs.num_depth_levels_;
		half_max_value_              = rhs.half_max_value_;
		blocks_                      = rhs.blocks_;
		free_blocks_                 = rhs.free_blocks_;
		node_half_length_            = rhs.node_half_length_;
		node_half_length_reciprocal_ = rhs.node_half_length_reciprocal_;
		return *this;
	}

	Tree& operator=(Tree&& rhs)
	{
		num_depth_levels_            = std::move(rhs.num_depth_levels_);
		half_max_value_              = std::move(rhs.half_max_value_);
		blocks_                      = std::move(rhs.blocks_);
		free_blocks_                 = std::move(rhs.free_blocks_);
		node_half_length_            = std::move(rhs.node_half_length_);
		node_half_length_reciprocal_ = std::move(rhs.node_half_length_reciprocal_);
		return *this;
	}

	template <class Derived2>
	Tree& operator=(Tree<Derived2, TreeBlock, TT> const& rhs)
	{
		num_depth_levels_            = rhs.num_depth_levels_;
		half_max_value_              = rhs.half_max_value_;
		blocks_                      = rhs.blocks_;
		free_blocks_                 = rhs.free_blocks_;
		node_half_length_            = rhs.node_half_length_;
		node_half_length_reciprocal_ = rhs.node_half_length_reciprocal_;
		return *this;
	}

	template <class Derived2>
	Tree& operator=(Tree<Derived2, TreeBlock, TT>&& rhs)
	{
		num_depth_levels_            = std::move(rhs.num_depth_levels_);
		half_max_value_              = std::move(rhs.half_max_value_);
		blocks_                      = std::move(rhs.blocks_);
		free_blocks_                 = std::move(rhs.free_blocks_);
		node_half_length_            = std::move(rhs.node_half_length_);
		node_half_length_reciprocal_ = std::move(rhs.node_half_length_reciprocal_);
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         Swap                                        |
	|                                                                                     |
	**************************************************************************************/

	friend void swap(Tree& lhs, Tree& rhs)
	{
		std::swap(lhs.num_depth_levels_, rhs.num_depth_levels_);
		std::swap(lhs.half_max_value_, rhs.half_max_value_);
		std::swap(lhs.blocks_, rhs.blocks_);
		std::swap(lhs.free_blocks_, rhs.free_blocks_);
		std::swap(lhs.node_half_length_, rhs.node_half_length_);
		std::swap(lhs.node_half_length_reciprocal_, rhs.node_half_length_reciprocal_);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Derived                                       |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Returns a reference of the derived class.
	 *
	 * @return A reference of the derived class.
	 */
	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	/*!
	 * @brief Returns a reference of the derived class.
	 *
	 * @return A reference of the derived class.
	 */
	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Recurs                                       |
	|                                                                                     |
	**************************************************************************************/

	template <class NodeFun,
	          std::enable_if_t<std::is_invocable_v<NodeFun, Index>, bool> = true>
	void recurs(Index node, NodeFun node_f) const
	{
		node_f(node);

		if (isLeaf(node)) {
			return;
		}

		auto c = children(node);
		for (offset_t i{}; branchingFactor() > i; ++i) {
			recurs(Index(c, i), node_f);
		}
	}

	template <class BlockFun,
	          std::enable_if_t<std::is_invocable_v<BlockFun, pos_t>, bool> = true>
	void recurs(pos_t block, BlockFun block_f) const
	{
		block_f(block);

		if (allLeaf(block)) {
			return;
		}

		for (offset_t i{}; branchingFactor() > i; ++i) {
			Index node(block, i);
			if (isLeaf(node)) {
				continue;
			}
			recurs(children(node), block_f);
		}
	}

	template <class NodeFun, class BlockFun,
	          std::enable_if_t<std::is_invocable_v<NodeFun, Index>, bool>  = true,
	          std::enable_if_t<std::is_invocable_v<BlockFun, pos_t>, bool> = true>
	void recurs(Index node, NodeFun node_f, BlockFun block_f) const
	{
		node_f(node);

		if (isLeaf(node)) {
			return;
		}

		recurs(children(node), node_f, block_f);
	}

	template <class NodeFun,
	          std::enable_if_t<std::is_invocable_v<NodeFun, Index>, bool> = true>
	void recursLeaves(Index node, NodeFun node_f) const
	{
		if (isLeaf(node)) {
			node_f(node);
			return;
		}

		auto c = children(node);
		for (offset_t i{}; branchingFactor() > i; ++i) {
			recursLeaves(Index(c, i), node_f);
		}
	}

	template <class NodeFun, class BlockFun,
	          std::enable_if_t<std::is_invocable_v<NodeFun, Index>, bool>  = true,
	          std::enable_if_t<std::is_invocable_v<BlockFun, pos_t>, bool> = true>
	void recursLeaves(Index node, NodeFun node_f, BlockFun block_f) const
	{
		if (isLeaf(node)) {
			node_f(node);
		} else if (allLeaf(children(node))) {
			block_f(children(node));
		} else {
			std::array<Index, maxNumDepthLevels()> nodes;
			nodes[1] = child(node, 0);
			for (std::size_t i{1}; 0 != i;) {
				node = nodes[i];
				i -= branchingFactor() <= ++nodes[i].offset;
				if (isLeaf(node)) {
					node_f(node);
				} else if (allLeaf(children(node))) {
					block_f(children(node));
				} else {
					nodes[++i] = child(node, 0);
				}
			}
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Leaf                                         |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Checks if all nodes of a block are pure leaves.
	 *
	 * @param block the block to check
	 * @return `true` if all nodes of the block are pure leaves, `false` otherwise.
	 */
	[[nodiscard]] bool allPureLeaf(pos_t block) const
	{
		assert(blocks_.size() > block);
		return 0 == blocks_[block].depth();
	}

	/*!
	 * @brief Checks if any node of a block is a pure leaves.
	 *
	 * @param block the block to check
	 * @return `true` if any node of the block is a pure leaf, `false` otherwise.
	 */
	[[nodiscard]] bool anyPureLeaf(pos_t block) const
	{
		assert(blocks_.size() > block);
		return 0 == blocks_[block].depth();
	}

	/*!
	 * @brief Checks if no nodes of a block are pure leaves.
	 *
	 * @param block the block to check
	 * @return `true` if no nodes of the block are pure leaves, `false` otherwise.
	 */
	[[nodiscard]] bool nonePureLeaf(pos_t block) const
	{
		assert(blocks_.size() > block);
		return 0 != blocks_[block].depth();
	}

	/*!
	 * @brief Checks if some nodes of a block are pure leaves, same as
	 * `anyPureLeaf(block) && !allPureLeaf(block)`.
	 *
	 * @param block the block to check
	 * @return `true` if some nodes of the block are pure leaves, `false` otherwise.
	 */
	[[nodiscard]] bool somePureLeaf(pos_t block) const
	{
		assert(blocks_.size() > block);
		return false;
	}

	/*!
	 * @brief Checks if all nodes of a block are leaves.
	 *
	 * @param block the block to check
	 * @return `true` if all nodes of the block are leaves, `false` otherwise.
	 */
	[[nodiscard]] bool allLeaf(pos_t block) const
	{
		assert(blocks_.size() > block);
		return std::all_of(std::begin(blocks_[block].children),
		                   std::end(blocks_[block].children),
		                   [](auto e) { return Index::NULL_POS == e; });
	}

	/*!
	 * @brief Checks if any node of a block is a leaf.
	 *
	 * @param block the block to check
	 * @return `true` if any node of the block is a leaf, `false` otherwise.
	 */
	[[nodiscard]] bool anyLeaf(pos_t block) const
	{
		assert(blocks_.size() > block);
		return std::any_of(std::begin(blocks_[block].children),
		                   std::end(blocks_[block].children),
		                   [](auto e) { return Index::NULL_POS == e; });
	}

	/*!
	 * @brief Checks if no nodes of a block are leaves.
	 *
	 * @param block the block to check
	 * @return `true` if no nodes of the block are leaves, `false` otherwise.
	 */
	[[nodiscard]] bool noneLeaf(pos_t block) const
	{
		assert(blocks_.size() > block);
		return std::none_of(std::begin(blocks_[block].children),
		                    std::end(blocks_[block].children),
		                    [](auto e) { return Index::NULL_POS == e; });
	}

	/*!
	 * @brief Checks if some nodes of a block are leaves, same as `anyLeaf(block) &&
	 * !allLeaf(block)`.
	 *
	 * @param block the block to check
	 * @return `true` if no nodes of the block are leaves, `false` otherwise.
	 */
	[[nodiscard]] bool someLeaf(pos_t block) const
	{
		bool leaf   = false;
		bool parent = false;
		for (auto e : children(block)) {
			leaf   = leaf || Index::NULL_POS == e;
			parent = parent || Index::NULL_POS != e;
		}
		return leaf && parent;
	}

	/*!
	 * @brief Checks if all nodes of a block are parents.
	 *
	 * @param block the block to check
	 * @return `true` if all nodes of the block are parents, `false` otherwise.
	 */
	[[nodiscard]] bool allParent(pos_t block) const { return noneLeaf(block); }

	/*!
	 * @brief Checks if any node of a block is a parent.
	 *
	 * @param block the block to check
	 * @return `true` if any node of the block is a parent, `false` otherwise.
	 */
	[[nodiscard]] bool anyParent(pos_t block) const { return !allLeaf(block); }

	/*!
	 * @brief Checks if no nodes of a block are parents.
	 *
	 * @param block the block to check
	 * @return `true` if no nodes of the block are parents, `false` otherwise.
	 */
	[[nodiscard]] bool noneParent(pos_t block) const { return allLeaf(block); }

	/*!
	 * @brief Checks if some nodes of a block are parents, same as `anyParent(block) &&
	 * !allParent(block)`.
	 *
	 * @param block the block to check
	 * @return `true` if some nodes of the block are parents, `false` otherwise.
	 */
	[[nodiscard]] bool someParent(pos_t block) const { return someLeaf(block); }

	/**************************************************************************************
	|                                                                                     |
	|                                       Access                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	[[nodiscard]] Block& treeBlock(pos_t block)
	{
		assert(valid(block));
		return blocks_[block];
	}

	[[nodiscard]] Block const& treeBlock(pos_t block) const
	{
		assert(valid(block));
		return blocks_[block];
	}

	[[nodiscard]] Block& treeBlock(Index node) { return treeBlock(node.pos); }

	[[nodiscard]] Block const& treeBlock(Index node) const { return treeBlock(node.pos); }

	[[nodiscard]] Index index(Code code, Index node, depth_t depth) const
	{
		depth_t min_depth = this->depth(code);
		while (min_depth < depth && isParent(node)) {
			node = child(node, code.offset(--depth));
		}
		return node;
	}

	[[nodiscard]] std::vector<Index> trail(Code code) const
	{
		// TODO: assert()
		std::vector<Index> result;
		result.reserve(numDepthLevels());
		auto node      = index();
		auto depth     = this->depth();
		auto min_depth = this->depth(code);
		result.push_back(node);
		while (min_depth < depth && isParent(node)) {
			node = child(node, code.offset(--depth));
			result.push_back(node);
		}
		return result;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Center                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add comments

	//
	// Child center
	//

	[[nodiscard]] static constexpr Point childCenter(Point    node_center,
	                                                 length_t node_half_size,
	                                                 offset_t child_index)
	{
		assert(branchingFactor() > child_index);
		length_t child_half_size = node_half_size / static_cast<length_t>(2);
		for (std::size_t i{}; Coord::size() != i; ++i) {
			node_center[i] +=
			    child_index & offset_t(1u << i) ? child_half_size : -child_half_size;
		}
		return node_center;
	}

	[[nodiscard]] Point childCenter(Coord node, offset_t child_index) const
	{
		assert(0 < depth(node));
		return childCenter(static_cast<Point>(node), halfLength(node), child_index);
	}

	//
	// Sibling center
	//

	[[nodiscard]] static constexpr Coord siblingCenter(Coord center, coord_t half_size,
	                                                   offset_t index,
	                                                   offset_t sibling_index)
	{
		assert(branchingFactor() > sibling_index);
		offset_t const temp = index ^ sibling_index;
		coord_t const  size = 2 * half_size;
		for (std::size_t i{}; Coord::size() != i; ++i) {
			center[i] += temp & offset_t(1u << i)
			                 ? (sibling_index & offset_t(1u << i) ? size : -size)
			                 : coord_t{};
		}
		return center;
	}

	//
	// Parent center
	//

	[[nodiscard]] static constexpr Coord parentCenter(Coord    child_center,
	                                                  coord_t  child_half_size,
	                                                  offset_t child_index)
	{
		// FIXME: Make work with quadtree
		assert(branchingFactor() > child_index);
		for (std::size_t i{}; Coord::size() != i; ++i) {
			child_center[i] -=
			    child_index & offset_t(1u << i) ? child_half_size : -child_half_size;
		}
		return child_center;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                 Create/erase nodes                                  |
	|                                                                                     |
	**************************************************************************************/

	pos_t createBlock(Index parent)
	{
		pos_t block                                 = static_cast<pos_t>(blocks_.size());
		blocks_[parent.pos].children[parent.offset] = block;
		blocks_.emplace_back(blocks_[parent.pos], parent.offset);
		derived().createBlock(parent);
		return block;
	}

	void fillBlock(Index parent, pos_t block)
	{
		blocks_[parent.pos].children[parent.offset] = block;
		blocks_[block].fill(blocks_[parent.pos], parent.offset);
		derived().fillBlock(parent, block);
	}

	void pruneBlock(Index parent, pos_t block)
	{
		derived().pruneBlock(parent, block);
		// NOTE: Important that derived is pruned first in case they use parent code
		blocks_[block] = Block();
	}

	//
	// Create
	//

	Index create(Code code, Index cur_node)
	{
		assert(valid(cur_node));
		auto wanted_depth = depth(code);
		auto cur_depth    = depth(cur_node);
		while (wanted_depth < cur_depth) {
			cur_node = createChild(cur_node, code.offset(--cur_depth));
		}
		return cur_node;
	}

	pos_t createChildrenThreadSafe(Index node)
	{
		assert(0 < depth(node));
		if (isParent(node)) {
			return children(node);
		}

		std::lock_guard<std::mutex> const lock(create_mutex_);
		return createChildren(node);
	}

	Index createChildThreadSafe(Index node, offset_t child_index)
	{
		assert(branchingFactor() > child_index);
		return Index(createChildren(node), child_index);
	}

	//
	// Erase
	//

	void eraseBlock(Index parent, pos_t block)
	{
		if (!valid(block)) {
			return;
		}

		auto child_blocks = children(block);
		for (offset_t i{}; child_blocks.size() > i; ++i) {
			eraseBlock(Index(block, i), child_blocks[i]);
		}

		pruneBlock(parent, block);
		free_blocks_.push_back(block);
	}

	void eraseBlockThreadSafe(Index parent, pos_t block)
	{
		// TODO: Implement
	}

 protected:
	// Mutex for creating children concurrently
	std::mutex create_mutex_;

	// The number of depth levels
	depth_t num_depth_levels_;
	// Half the maximum key value the tree can store
	key_t half_max_value_;

	// Blocks
	Container blocks_;
	// Free blocks
	std::deque<pos_t> free_blocks_;

	// Stores the node half length at a given depth, where the index is the depth
	std::array<length_t, maxNumDepthLevels() + 1> node_half_length_;
	// Reciprocal of the node half length at a given depth, where the index is the depth
	std::array<length_t, maxNumDepthLevels() + 1> node_half_length_reciprocal_;
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_TREE_HPP