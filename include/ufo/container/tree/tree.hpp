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
#include <ufo/container/tree/bounds.hpp>
#include <ufo/container/tree/code.hpp>
#include <ufo/container/tree/container.hpp>
#include <ufo/container/tree/coord.hpp>
// #include <ufo/container/tree/file_header.hpp>
#include <ufo/container/tree/index.hpp>
#include <ufo/container/tree/iterator.hpp>
#include <ufo/container/tree/key.hpp>
#include <ufo/container/tree/node.hpp>
#include <ufo/container/tree/node_nearest.hpp>
#include <ufo/container/tree/predicate.hpp>
#include <ufo/container/tree/type.hpp>
#include <ufo/geometry/shape/aabb.hpp>
#include <ufo/math/math.hpp>
#include <ufo/math/vec.hpp>
#include <ufo/utility/bit_set.hpp>
#include <ufo/utility/execution.hpp>
#include <ufo/utility/io/buffer.hpp>
#include <ufo/utility/iterator_wrapper.hpp>
#include <ufo/utility/macros.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace ufo
{
enum class NearestSearchAlgorithm { DEPTH_FIRST, A_STAR };

/*!
 * @brief
 *
 * Utilizing curiously recurring template pattern (CRTP)
 *
 * \tparam Derived ...
 * \tparam Block ...
 */
template <class Derived, class Block>
class Tree
{
 protected:
	//
	// Friends
	//

	template <class Derived2, class Block2>
	friend class Tree;

	static constexpr std::size_t const BF  = ufo::branchingFactor(Block::tree_type);
	static constexpr std::size_t const Dim = ufo::dimensions(Block::tree_type);

 public:
	//
	// Tags
	//

	using length_t = double;
	using depth_t  = unsigned;
	using coord_t  = float;
	using ray_t    = coord_t;

	using Code   = TreeCode<Dim>;
	using Key    = TreeKey<Dim>;
	using Coord  = TreeCoord<Dim, coord_t>;
	using Coord2 = TreeCoord<Dim, double>;
	using Point  = Vec<Dim, coord_t>;
	using Point2 = Vec<Dim, double>;
	using Bounds = AABB<Dim, coord_t>;

	using Index       = TreeIndex;
	using Node        = TreeNode<Code>;
	using NodeNearest = TreeNodeNearest<Node>;

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

	template <class T>
	struct is_node_type
	    : is_one_of<std::decay_t<T>, Index, Node, Code, Key, Coord, Point,
	                // We also add the double versions of Coord and Point
	                Coord2, Point2> {
	};

	template <class T>
	static constexpr inline bool is_node_type_v = is_node_type<T>::value;

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
	[[nodiscard]] static constexpr TreeType treeType() noexcept { return Block::tree_type; }

	/*!
	 * @brief Returns the branching factor of the tree (i.e., 2 = binary tree, 4 = quadtree,
	 * 8 = octree, 16 = hextree).
	 *
	 * @return The branching factor of the tree.
	 */
	[[nodiscard]] static constexpr std::size_t branchingFactor() noexcept { return BF; }

	/*!
	 * @brief Returns the number of dimensions of the tree (i.e., 1 = binary tree, 2 =
	 * quadtree, 3 = octree, 4 = hextree).
	 *
	 * @return The number of dimensions of the tree.
	 */
	[[nodiscard]] static constexpr std::size_t dimensions() noexcept { return Dim; }

	/*!
	 * @brief Returns the number of nodes in the tree.
	 *
	 * @return The number of nodes in the tree.
	 */
	[[nodiscard]] std::size_t size() const
	{
		return (block_.size() - free_block_.size()) * BF;
	}

	/*!
	 * @brief Erases all nodes from the tree.
	 */
	void clear()
	{
		block_.clear();
		free_block_.clear();
		// Create root
		block_.emplace_back(code(), parentCenter(center(), halfLength(), 0), length());
		derived().onClear();
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
		return Code::maxDepth() + 1;
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
		assert(block_.size() > block);
		return block_[block].depth();
	}

	/*!
	 * @brief Returns the depth of the node.
	 *
	 * @note The tree's depth levels are `[0..depth()]`.
	 *
	 * @param node the node
	 * @return The depth of the node.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] constexpr depth_t depth(NodeType node) const
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return depth(node.pos);
		} else if constexpr (std::is_same_v<T, Node>) {
			return node.depth();
		} else if constexpr (std::is_same_v<T, Code>) {
			return node.depth();
		} else if constexpr (std::is_same_v<T, Key>) {
			return node.depth();
		} else if constexpr (is_one_of_v<T, Coord, Coord2>) {
			return node.depth;
		} else if constexpr (is_one_of_v<T, Point, Point2>) {
			return 0;
		} else {
			// FIXME: Look at
			static_assert(is_node_type_v<NodeType>);
		}
	}

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
		assert(numDepthLevels() > depth);
		return node_half_length_[depth + 1];
	}

	/*!
	 * @brief Returns the length of `node`, i.e. `leaf_node_length * 2^depth(node)`.
	 *
	 * @param node the node
	 * @return The length of the node.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] length_t length(NodeType node) const
	{
		return length(depth(node));
	}

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
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] constexpr length_t halfLength(NodeType node) const
	{
		return halfLength(depth(node));
	}

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
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] length_t lengthReciprocal(NodeType node) const
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
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] length_t halfLengthReciprocal(NodeType node) const
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
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Bounds bounds(NodeType node) const
	{
		return Bounds(center(node), halfLength(node));
	}

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
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] constexpr Coord center(NodeType node) const
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return center(block_[node.pos].code(node.offset));
		} else if constexpr (std::is_same_v<T, Node>) {
			// TODO: Not working, only returns root or something???
			return center(key(node));
		} else if constexpr (std::is_same_v<T, Code>) {
			return center(key(node));
		} else if constexpr (std::is_same_v<T, Key>) {
			assert(valid(node));

			auto node_depth = depth(node);

			if (depth() == node_depth) {
				return center();
			}

			// TODO: Check performance, might be a lot faster to have float here and in rest of
			// method
			length_t          l = length(node_depth);
			std::int_fast64_t hmv =
			    static_cast<std::int_fast64_t>(half_max_value_ >> node_depth);

			Point coord = cast<coord_t>((cast<length_t>(cast<std::int_fast64_t>(node) - hmv) +
			                             static_cast<length_t>(0.5)) *
			                            l);

			return Coord(coord, node_depth);
		} else if constexpr (is_one_of_v<T, Coord, Coord2>) {
			return center(key(node));
		} else if constexpr (is_one_of_v<T, Point, Point2>) {
			return center(Coord(node, 0u));
		} else {
			// FIXME: Look at
			static_assert(is_node_type_v<NodeType>);
		}
	}

	/*!
	 * @brief Returns the center of `node` if the node is valid, i.e. `valid(node)`.
	 *
	 * @param node the node
	 * @return The center of the node if the node is valid, null otherwise.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] std::optional<Coord> centerChecked(Code node) const
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

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] pos_t block(NodeType node) const
	{
		if constexpr (std::is_same_v<Index, std::decay_t<NodeType>>) {
			return node.pos;
		} else {
			return block(index(node));
		}
	}

	//
	// Index
	//

	/*!
	 * @brief Returns the index of the root node.
	 *
	 * @return The index of the root node.
	 */
	[[nodiscard]] Index index() const { return Index(block(), 0); }

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] constexpr Index index(NodeType node) const
	{
		assert(valid(node));

		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return node;
		} else if constexpr (std::is_same_v<T, Node>) {
			// TODO: Benchmark if this is actually faster than going down the tree

			if (!valid(node.index()) || depth(node.index()) < depth(node) ||
			    !Code::equalAtDepth(code(node.index()), node.code(), depth(node.index()))) {
				return index(node.code());
			}

			if (code(node.index()) == node.code() || isLeaf(node.index())) {
				return node.index();
			}

			return index(node.code(), node.index(), depth(node.index()));
		} else if constexpr (std::is_same_v<T, Code>) {
			return index(node, index(), depth());
		} else {
			return index(code(node));
		}
	}

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
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Node node(NodeType node) const
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			assert(valid(node));
			return Node(code(node), node);
		} else if constexpr (std::is_same_v<T, Node>) {
			return Node(node.code(), index(node));
		} else if constexpr (std::is_same_v<T, Code>) {
			return Node(node, index(node));
		} else {
			return Node(code(node));
		}
	}

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
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Node operator[](NodeType node) const
	{
		return this->node(node);
	}

	//
	// Code
	//

	[[nodiscard]] Code code() const { return Code(static_cast<code_t>(0), depth()); }

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Code code(NodeType node) const
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			assert(valid(node));
			return block_[node.pos].code(node.offset);
		} else if constexpr (std::is_same_v<T, Node>) {
			return node.code();
		} else if constexpr (std::is_same_v<T, Code>) {
			return node;
		} else if constexpr (std::is_same_v<T, Key>) {
			return Code(node);
		} else {
			return code(key(node));
		}
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] std::optional<Code> codeChecked(NodeType node) const
	{
		return valid(node) ? std::optional<Code>(Code(node)) : std::nullopt;
	}

	//
	// Key
	//

	[[nodiscard]] Key key() const { return Key(Vec<Dim, key_t>(0), depth()); }

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Key key(NodeType node) const
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return key(block_[node.pos].code(node.offset));
		} else if constexpr (std::is_same_v<T, Node>) {
			return key(node.code());
		} else if constexpr (std::is_same_v<T, Code>) {
			return Key(node);
		} else if constexpr (std::is_same_v<T, Key>) {
			return node;
		} else if constexpr (is_one_of_v<T, Coord, Coord2>) {
			assert(valid(node));

			auto  d = depth(node);
			Point p = node;

			// TODO: Check performance, might be a lot faster to have float here
			length_t lr = lengthReciprocal(0);

			auto k =
			    cast<key_t>(cast<std::make_signed_t<key_t>>(floor(cast<length_t>(p) * lr))) +
			    half_max_value_;

			return {k >> d, d};
		} else if constexpr (is_one_of_v<T, Point, Point2>) {
			return key(Coord(node, 0u));
		}
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] std::optional<Key> keyChecked(NodeType node) const
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

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	Index create(NodeType node)
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return node;
		} else if constexpr (std::is_same_v<T, Node>) {
			return create(code(node), index(node));
		} else if constexpr (std::is_same_v<T, Code>) {
			return create(node, index());
		} else {
			return create(code(node));
		}
	}

	template <class InputIt>
	std::vector<Index> create(InputIt first, InputIt last)
	{
		using value_type = std::decay_t<typename std::iterator_traits<InputIt>::value_type>;

		if constexpr (std::is_same_v<Code, value_type>) {
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
		} else {
			std::vector<Code> codes;
			codes.reserve(std::distance(first, last));
			std::transform(first, last, std::back_inserter(codes),
			               [this](auto const& v) { return code(v); });
			return create(codes.begin(), codes.end());
		}
	}

	template <
	    class ExecutionPolicy, class ForwardIt,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	std::vector<Index> create(ExecutionPolicy&& policy, ForwardIt first, ForwardIt last)
	{
		using value_type = std::decay_t<typename std::iterator_traits<ForwardIt>::value_type>;

		if constexpr (std::is_same_v<Code, value_type>) {
			if constexpr (std::is_same_v<execution::sequenced_policy,
			                             std::decay_t<ExecutionPolicy>>) {
				return create(first, last);
			}

#if !defined(UFO_TBB) && !defined(UFO_OMP)
			return create(first, last);
#else
			// FIXME: Implement, using `createChildThreadSafe`
			// FIXME: Remove when above has been implemented
			return create(first, last);
#endif
		} else {
			// TODO: Is this correct?
			std::vector<Code> codes;
			codes.reserve(std::distance(first, last));
			std::transform(policy, first, last, std::back_inserter(codes),
			               [this](auto const& v) { return code(v); });
			return create(std::forward<ExecutionPolicy>(policy), codes.begin(), codes.end());
		}
	}

	template <class Range>
	std::vector<Index> create(Range const& r)
	{
		using std::begin;
		using std::end;
		return create(begin(r), end(r));
	}

	template <
	    class ExecutionPolicy, class Range,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	std::vector<Index> create(ExecutionPolicy&& policy, Range const& r)
	{
		using std::begin;
		using std::end;
		return create(std::forward<ExecutionPolicy>(policy), begin(r), end(r));
	}

	pos_t createChildren(Index node)
	{
		assert(!isPureLeaf(node));
		if (isParent(node)) {
			return this->children(node);
		}

		pos_t children;
		if (free_block_.empty()) {
			// FIXME: Maybe move the content of `createBlock` here?
			children = createBlock(node);
		} else {
			children = free_block_.front();
			free_block_.pop_front();
			fillChildren(node, children);
		}

		return children;
	}

	Index createChild(Index node, offset_t child_index)
	{
		assert(0 < depth(node));
		assert(branchingFactor() > child_index);
		return Index(createChildren(node), child_index);
	}

	//
	// Create trail
	//

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	std::array<Index, maxNumDepthLevels()> createTrail(NodeType node)
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			std::array<Index, maxNumDepthLevels()> tail;
			tail[depth(node)] = node;
			return tail;
		} else if constexpr (std::is_same_v<T, Node>) {
			return createTrail(code(node), index(node));
		} else if constexpr (std::is_same_v<T, Code>) {
			return createTrail(node, index());
		} else {
			return createTrail(code(node));
		}
	}

	//
	// Erase
	//

	void eraseChildren() { eraseChildren(index()); }

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	void eraseChildren(NodeType node)
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			assert(valid(node));
			eraseChildren(node, children(node));
		} else if constexpr (std::is_same_v<T, Node>) {
			Index n = index(node);
			if (code(n) != code(node)) {
				// The node does not even exist
				return;
			}

			eraseChildren(n);
		} else if constexpr (std::is_same_v<T, Code>) {
			Index n = index(node);
			if (code(n) != node) {
				// The node does not even exist
				return;
			}

			eraseChildren(n);
		} else {
			eraseChildren(code(node));
		}
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
	 * @brief Checks if the block is pure leaf (i.e., can never have children).
	 *
	 * @note Only have to check if the depth of the block is 0.
	 *
	 * @param block the block to check
	 * @return `true` if the block is pure leaf, `false` otherwise.
	 */
	[[nodiscard]] bool isPureLeaf(pos_t block) const { return 0 == depth(block); }

	/*!
	 * @brief Checks if the node is a pure leaf (i.e., can never have children).
	 *
	 * @note Only have to check if the depth of the node is 0.
	 *
	 * @param node the node to check
	 * @return `true` if the node is a pure leaf, `false` otherwise.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] constexpr bool isPureLeaf(NodeType node) const
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
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] constexpr bool isLeaf(NodeType node) const
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return TreeIndex::NULL_POS == children(node);
		} else {
			return isLeaf(index(node));
		}
	}

	//
	// Parent
	//

	/*!
	 * @brief Checks if the node is a parent (i.e., has children).
	 *
	 * @param node the node to check
	 * @return `true` if the node is a parent, `false` otherwise.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] constexpr bool isParent(NodeType node) const
	{
		return !isLeaf(node);
	}

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
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] constexpr bool isRoot(NodeType node) const
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return index() == node;
		} else if constexpr (std::is_same_v<T, Node>) {
			return isRoot(node.code());
		} else if constexpr (std::is_same_v<T, Code>) {
			return code() == node;
		} else if constexpr (std::is_same_v<T, Key>) {
			return key() == node;
		} else {
			return isRoot(key(node));
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Valid                                        |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Checks if a block is valid.
	 *
	 * @param block the block to check
	 * @return `true` if the block is valid, `false` otherwise.
	 */
	[[nodiscard]] bool valid(pos_t block) const { return block_.size() > block; }

	/*!
	 * @brief Checks if an index is valid.
	 *
	 * @param index the index to check
	 * @return `true` if the index is valid, `false` otherwise.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool valid(NodeType node) const
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return valid(node.pos) && branchingFactor() > node.offset &&
			       block_[node.pos].valid();
		} else if constexpr (std::is_same_v<T, Node>) {
			return valid(code(node));
		} else if constexpr (std::is_same_v<T, Code>) {
			return node.valid() && numDepthLevels() > depth(node);
		} else if constexpr (std::is_same_v<T, Key>) {
			auto const mv = (2 * half_max_value_) >> depth(node);
			for (std::size_t i{}; node.size() != i; ++i) {
				if (mv < node[i]) {
					return false;
				}
			}

			return node.valid() && numDepthLevels() > depth(node);
		} else {
			return isInside(node) && numDepthLevels() > depth(node);
		}
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
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] bool exists(NodeType node) const
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return valid(node);
		} else if constexpr (std::is_same_v<T, Node>) {
			return code(index(node)) == code(node);
		} else if constexpr (std::is_same_v<T, Code>) {
			return code(index(node)) == node;
		} else {
			return exists(code(node));
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Traverse                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] std::array<pos_t, branchingFactor()> children(pos_t block) const
	{
		assert(valid(block));
		return block_[block].children;
	}

	[[nodiscard]] pos_t children(Index node) const
	{
		assert(valid(node));
		// assert(isParent(node));
		return children(node.pos)[node.offset];
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] constexpr NodeType child(NodeType node, offset_t child_index) const
	{
		assert(0 < depth(node));
		assert(branchingFactor() > child_index);

		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			assert(valid(node));
			return {children(node), child_index};
		} else if constexpr (std::is_same_v<T, Node>) {
			return Node(child(node.code(), child_index), child(node.index(), child_index));
		} else if constexpr (std::is_same_v<T, Code>) {
			return node.child(child_index);
		} else if constexpr (std::is_same_v<T, Key>) {
			return node.child(child_index);
		} else {
			return {childCenter(static_cast<Point>(node), halfLength(node), child_index),
			        node.depth - static_cast<depth_t>(1)};
		}
	}

	/*!
	 * @brief Get a child of a node with bounds checking.
	 *
	 * @param node The node.
	 * @param child_index The index of the child.
	 * @return The child.
	 */
	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] NodeType childChecked(NodeType node, offset_t child_index) const
	{
		if (isLeaf(node)) {
			throw std::out_of_range("Node has no children");
		} else if (branchingFactor() <= child_index) {
			throw std::out_of_range("child_index out of range");
		}
		return child(node, child_index);
	}

	//
	// Sibling
	//

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] NodeType sibling(NodeType node, offset_t sibling_index) const
	{
		assert(!isRoot(node));
		assert(branchingFactor() > sibling_index);

		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return {node.pos, sibling_index};
		} else if constexpr (std::is_same_v<T, Node>) {
			return {sibling(node.code(), sibling_index), sibling(node.index(), sibling_index)};
		} else if constexpr (std::is_same_v<T, Code>) {
			return node.sibling(sibling_index);
		} else if constexpr (std::is_same_v<T, Key>) {
			return node.sibling(sibling_index);
		} else {
			return center(sibling(key(node), sibling_index));
		}
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] NodeType siblingChecked(NodeType node, offset_t sibling_index) const
	{
		if (!isRoot(node)) {
			throw std::out_of_range("Root node has no siblings");
		} else if (branchingFactor() <= sibling_index) {
			throw std::out_of_range("sibling_index out of range");
		}
		return sibling(node, sibling_index);
	}

	//
	// Parent
	//

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] NodeType parent(NodeType node) const
	{
		assert(!isRoot(node));

		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Index>) {
			return index(block_[node.pos].parentCode());
		} else if constexpr (std::is_same_v<T, Node>) {
			return this->node(parent(node.code()));
		} else if constexpr (std::is_same_v<T, Code>) {
			return node.parent();
		} else if constexpr (std::is_same_v<T, Key>) {
			return node.parent();
		} else {
			return center(parent(key(node)));
		}
	}

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] NodeType parentChecked(NodeType node) const
	{
		if (!isRoot(node)) {
			throw std::out_of_range("Root node has no parent");
		}
		return parent(node);
	}

	// TODO: Look at functions below

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
		// nodes.emplace(nbv, distanceSquared(nbv.boundingVolume(), g));

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
		// 				nodes.emplace(c, distanceSquared(c.boundingVolume(), g));
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
		// 				nodes.emplace(c, distanceSquared(c.boundingVolume(), g));
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
	 * @brief Depth first traversal of the tree.
	 *
	 * Depth first traversal of the tree, starting at the node corresponding to the
	 * coordinate. The function 'f' will be called for each node
	 * traverse. If 'f' returns true then the children of the node will also be traverse,
	 * otherwise they will not.
	 *
	 * @param node The node where to start the traversal.
	 * @param f The callback function to be called for each node traversed.
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

	template <class NodeType, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] const_iterator begin(NodeType node, bool only_leaves = true,
	                                   bool only_exists    = true,
	                                   bool early_stopping = false) const
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Node>) {
			return only_leaves ? beginQuery(node, pred::Leaf{}, only_exists, early_stopping)
			                   : beginQuery(node, pred::True{}, only_exists, early_stopping);
		} else {
			return begin(this->node(node), only_leaves, only_exists, early_stopping);
		}
	}

	[[nodiscard]] const_iterator end() const { return endQuery(); }

	//
	// Nearest iterator
	//

	template <class Geometry>
	[[nodiscard]] const_nearest_iterator beginNearest(Geometry const& geometry,
	                                                  double          epsilon     = 0.0,
	                                                  bool            only_leaves = true,
	                                                  bool            only_exists = true,
	                                                  bool early_stopping = false) const
	{
		return beginNearest(node(), geometry, epsilon, only_leaves, only_exists,
		                    early_stopping);
	}

	template <class NodeType, class Geometry,
	          std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] const_nearest_iterator beginNearest(
	    NodeType node, Geometry const& geometry, double epsilon = 0.0,
	    bool only_leaves = true, bool only_exists = true, bool early_stopping = false) const
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Node>) {
			return only_leaves ? beginQueryNearest(node, geometry, pred::Leaf{}, epsilon,
			                                       only_exists, early_stopping)
			                   : beginQueryNearest(node, geometry, pred::True{}, epsilon,
			                                       only_exists, early_stopping);
		} else {
			return beginNearest(this->node(node), geometry, epsilon, only_leaves, only_exists,
			                    early_stopping);
		}
	}

	[[nodiscard]] const_nearest_iterator endNearest() const { return endQueryNearest(); }

	//
	// Query iterator
	//

	template <class Predicate,
	          std::enable_if_t<pred::is_pred_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] const_query_iterator beginQuery(Predicate const& predicate,
	                                              bool             only_exists = true,
	                                              bool early_stopping = false) const
	{
		return beginQuery(node(), predicate, only_exists, early_stopping);
	}

	template <class NodeType, class Predicate,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>                  = true,
	          std::enable_if_t<pred::is_pred_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] const_query_iterator beginQuery(NodeType node, Predicate const& predicate,
	                                              bool only_exists    = true,
	                                              bool early_stopping = false) const
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Node>) {
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
		} else {
			return beginQuery(this->node(node), predicate, only_exists, early_stopping);
		}
	}

	[[nodiscard]] const_query_iterator endQuery() const
	{
		return {new TreeForwardIterator<Derived, Node, pred::True, true, true>(&derived())};
	}

	//
	// Query nearest iterator
	//

	template <class Geometry, class Predicate,
	          std::enable_if_t<pred::is_pred_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] const_nearest_query_iterator beginQueryNearest(
	    Geometry const& geometry, Predicate const& predicate, double epsilon = 0.0,
	    bool only_exists = true, bool early_stopping = false) const
	{
		return beginQueryNearest(node(), geometry, predicate, epsilon, only_exists,
		                         early_stopping);
	}

	template <class NodeType, class Geometry, class Predicate,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>                  = true,
	          std::enable_if_t<pred::is_pred_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] const_nearest_query_iterator beginQueryNearest(
	    NodeType node, Geometry const& geometry, Predicate const& predicate,
	    double epsilon = 0.0, bool only_exists = true, bool early_stopping = false) const
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Node>) {
			if (only_exists) {
				if (early_stopping) {
					return {new TreeNearestIterator<Derived, Node, Geometry, Predicate, true, true>(
					    &derived(), node, geometry, predicate, epsilon)};
				} else {
					return {
					    new TreeNearestIterator<Derived, Node, Geometry, Predicate, true, false>(
					        &derived(), node, geometry, predicate, epsilon)};
				}
			} else {
				if (early_stopping) {
					return {
					    new TreeNearestIterator<Derived, Node, Geometry, Predicate, false, true>(
					        &derived(), node, geometry, predicate, epsilon)};
				} else {
					return {
					    new TreeNearestIterator<Derived, Node, Geometry, Predicate, false, false>(
					        &derived(), node, geometry, predicate, epsilon)};
				}
			}
		} else {
			return beginQueryNearest(this->node(node), geometry, predicate, epsilon,
			                         only_exists, early_stopping);
		}
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

	//
	// Query
	//

	template <class Predicate,
	          std::enable_if_t<pred::is_pred_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] Query query(Predicate const& predicate, bool only_exists = true,
	                          bool early_stopping = false) const
	{
		return query(node(), predicate, only_exists, early_stopping);
	}

	template <class NodeType, class Predicate,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>                  = true,
	          std::enable_if_t<pred::is_pred_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] Query query(NodeType node, Predicate const& predicate,
	                          bool only_exists = true, bool early_stopping = false) const
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Node>) {
			return Query(beginQuery(node, predicate, only_exists, early_stopping), endQuery());
		} else {
			return query(this->node(node), predicate, only_exists, early_stopping);
		}
	}

	//
	// Query nearest
	//

	template <class Geometry, class Predicate,
	          std::enable_if_t<pred::is_pred_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] QueryNearest queryNearest(Geometry const&  geometry,
	                                        Predicate const& predicate = pred::True{},
	                                        double epsilon = 0.0, bool only_exists = true,
	                                        bool early_stopping = false) const
	{
		return queryNearest(node(), geometry, predicate, epsilon, only_exists,
		                    early_stopping);
	}

	template <class NodeType, class Geometry, class Predicate,
	          std::enable_if_t<is_node_type_v<NodeType>, bool>                  = true,
	          std::enable_if_t<pred::is_pred_v<Predicate, Derived, Node>, bool> = true>
	[[nodiscard]] QueryNearest queryNearest(NodeType node, Geometry const& geometry,
	                                        Predicate const& predicate = pred::True{},
	                                        double epsilon = 0.0, bool only_exists = true,
	                                        bool early_stopping = false) const
	{
		using T = std::decay_t<NodeType>;
		if constexpr (std::is_same_v<T, Node>) {
			return QueryNearest(beginQueryNearest(node, geometry, predicate, epsilon,
			                                      only_exists, early_stopping),
			                    endQueryNearest());
		} else {
			return queryNearest(this->node(node), geometry, predicate, epsilon, only_exists,
			                    early_stopping);
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Trace                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Implement the predicate once as well

	template <class InnerFun, class HitFun, class T>
	[[nodiscard]] T trace(Ray<Dim, ray_t> const& ray, InnerFun inner_f, HitFun hit_f,
	                      T const& miss) const
	{
		return trace(index(), ray, inner_f, hit_f, miss);
	}

	template <class InputIt, class OutputIt, class InnerFun, class HitFun, class T>
	OutputIt trace(InputIt first, InputIt last, OutputIt d_first, InnerFun inner_f,
	               HitFun hit_f, T const& miss) const
	{
		return trace(index(), first, last, d_first, inner_f, hit_f, miss);
	}

	template <class InputIt, class InnerFun, class HitFun, class T>
	[[nodiscard]] std::vector<T> trace(InputIt first, InputIt last, InnerFun inner_f,
	                                   HitFun hit_f, T const& miss) const
	{
		return trace(index(), first, last, inner_f, hit_f, miss);
	}

	template <class NodeType, class InnerFun, class HitFun, class T,
	          std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] T trace(NodeType node, Ray<Dim, ray_t> const& ray, InnerFun inner_f,
	                      HitFun hit_f, T const& miss) const
	{
		if constexpr (!std::is_same_v<Index, std::decay_t<NodeType>>) {
			// Unless NodeType is Index, we need to check that the node actually exists
			if (!exists(node)) {
				return miss;
			}
		}

		Index n = index(node);

		auto wrapped_inner_f = [&ray, inner_f](Index node, float distance) {
			return inner_f(node, ray, distance);
		};

		auto wrapped_hit_f = [&ray, hit_f](Index node, float distance) {
			return hit_f(node, ray, distance);
		};

		auto params = traceInit(n, ray);
		return trace(n, params, wrapped_inner_f, wrapped_hit_f, miss);
	}

	template <class NodeType, class InputIt, class OutputIt, class InnerFun, class HitFun,
	          class T, std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	OutputIt trace(NodeType node, InputIt first, InputIt last, OutputIt d_first,
	               InnerFun inner_f, HitFun hit_f, T const& miss) const
	{
		if constexpr (!std::is_same_v<Index, std::decay_t<NodeType>>) {
			// Unless NodeType is Index, we need to check that the node actually exists
			if (!exists(node)) {
				return miss;
			}
		}

		Index n = index(node);

		auto center      = this->center(n);
		auto half_length = halfLength(n);

		return std::transform(first, last, d_first, [&](auto const& ray) {
			auto wrapped_inner_f = [&ray, inner_f](Index node, float distance) {
				return inner_f(node, ray, distance);
			};

			auto wrapped_hit_f = [&ray, hit_f](Index node, float distance) {
				return hit_f(node, ray, distance);
			};

			auto params = traceInit(ray, center, half_length);
			return trace(n, params, wrapped_inner_f, wrapped_hit_f, miss);
		});
	}

	template <class NodeType, class InputIt, class InnerFun, class HitFun, class T,
	          std::enable_if_t<is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] std::vector<T> trace(NodeType node, InputIt first, InputIt last,
	                                   InnerFun inner_f, HitFun hit_f, T const& miss) const
	{
		std::vector<T> nodes(std::distance(first, last));
		trace(node, first, last, nodes.begin(), inner_f, hit_f, miss);
		return nodes;
	}

	template <
	    class ExecutionPolicy, class RandomIt1, class RandomIt2, class InnerFun,
	    class HitFun, class T,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	RandomIt2 trace(ExecutionPolicy&& policy, RandomIt1 first, RandomIt1 last,
	                RandomIt2 d_first, InnerFun inner_f, HitFun hit_f, T const& miss) const
	{
		return trace(std::forward<ExecutionPolicy>(policy), index(), first, last, d_first,
		             inner_f, hit_f, miss);
	}

	template <
	    class ExecutionPolicy, class RandomIt, class InnerFun, class HitFun, class T,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	[[nodiscard]] std::vector<T> trace(ExecutionPolicy&& policy, RandomIt first,
	                                   RandomIt last, InnerFun inner_f, HitFun hit_f,
	                                   T const& miss) const
	{
		return trace(std::forward<ExecutionPolicy>(policy), index(), first, last, inner_f,
		             hit_f, miss);
	}

	template <class ExecutionPolicy, class NodeType, class RandomIt1, class RandomIt2,
	          class InnerFun, class HitFun, class T>
	RandomIt2 trace(ExecutionPolicy&& policy, NodeType node, RandomIt1 first,
	                RandomIt1 last, RandomIt2 d_first, InnerFun inner_f, HitFun hit_f,
	                T const& miss) const
	{
		if constexpr (std::is_same_v<execution::sequenced_policy,
		                             std::decay_t<ExecutionPolicy>>) {
			return trace(node, first, last, d_first, inner_f, hit_f, miss);
		}

#if !defined(UFO_TBB) && !defined(UFO_OMP)
		return trace(node, first, last, d_first, inner_f, hit_f, miss);
#endif

		if constexpr (!std::is_same_v<Index, std::decay_t<NodeType>>) {
			// Unless NodeType is Index, we need to check that the node actually exists
			if (!exists(node)) {
				return miss;
			}
		}

		Index n = index(node);

		auto center      = this->center(n);
		auto half_length = halfLength(n);

#if defined(UFO_TBB)
		return std::transform(
		    std::forward<ExecutionPolicy>(policy), first, last, d_first,
		    [&](auto const& ray) {
			    auto wrapped_inner_f = [&ray, inner_f](Index node, float distance) {
				    return inner_f(node, ray, distance);
			    };

			    auto wrapped_hit_f = [&ray, hit_f](Index node, float distance) {
				    return hit_f(node, ray, distance);
			    };

			    auto params = traceInit(ray, center, half_length);
			    return trace(n, params, wrapped_inner_f, wrapped_hit_f, miss);
		    });
#elif defined(UFO_OMP)
		std::size_t size = std::distance(first, last);

#pragma omp parallel for
		for (std::size_t i = 0; i != size; ++i) {
			auto const& ray = first[i];

			auto wrapped_inner_f = [&ray, inner_f](Index node, float distance) {
				return inner_f(node, ray, distance);
			};

			auto wrapped_hit_f = [&ray, hit_f](Index node, float distance) {
				return hit_f(node, ray, distance);
			};
			auto params = traceInit(ray, center, half_length);
			d_first[i]  = trace(n, params, ray, wrapped_inner_f, wrapped_hit_f, miss);
		}

		return std::next(d_first, size);
#endif
	}

	template <class ExecutionPolicy, class NodeType, class RandomIt, class InnerFun,
	          class HitFun, class T>
	[[nodiscard]] std::vector<T> trace(ExecutionPolicy&& policy, NodeType node,
	                                   RandomIt first, RandomIt last, InnerFun inner_f,
	                                   HitFun hit_f, T const& miss) const
	{
		std::vector<T> nodes(std::distance(first, last));
		trace(std::forward<ExecutionPolicy>(policy), node, first, last, nodes.begin(),
		      inner_f, hit_f, miss);
		return nodes;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         I/O                                         |
	|                                                                                     |
	**************************************************************************************/

	// [[nodiscard]] static bool canRead(std::filesystem::path const& file)
	// {
	// 	std::ifstream f;
	// 	openFileIn(file, f);
	// 	return canRead(f);
	// }

	// [[nodiscard]] static bool canRead(std::istream& in)
	// {
	// 	auto g = in.tellg();

	// 	// TODO: Implement
	// 	// std::string line;
	// 	// std::getline(in, line);

	// 	in.seekg(g);

	// 	return true;
	// }

	// [[nodiscard]] static bool canRead(ReadBuffer& in)
	// {
	// 	// TODO: Implement

	// 	return true;
	// }

	// void read(std::filesystem::path const& file)
	// {
	// 	std::ifstream f;
	// 	openFileIn(file, f);
	// 	read(f);
	// }

	// void read(std::istream& in) { readData(in, readHeader(in)); }

	// void read(ReadBuffer& in) { readData(in, readHeader(in)); }

	// [[nodiscard]] TreeFileHeader readHeader(std::filesystem::path const& file) const
	// {
	// 	return {file};
	// }

	// [[nodiscard]] TreeFileHeader readHeader(std::istream& in) const { return {in}; }

	// [[nodiscard]] TreeFileHeader readHeader(ReadBuffer& in) const { return {in}; }

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

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
		if (!std::isnormal(std::ldexp(leaf_node_length, num_depth_levels - 1))) {
			throw std::invalid_argument(
			    "'leaf_node_length * 2^(num_depth_levels - 1)' has to be finite and greater "
			    "than zero, '" +
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
		half_max_value_   = static_cast<key_t>(1) << (num_depth_levels - 2);

		// For increased precision
		for (int i{}; node_half_length_.size() > i; ++i) {
			node_half_length_[i]            = std::ldexp(leaf_node_length, i - 1);
			node_half_length_reciprocal_[i] = static_cast<length_t>(1) / node_half_length_[i];
		}

		// Create root
		block_.emplace_back(code(), parentCenter(center(), halfLength(), 0), length());
	}

	Tree(Tree const& other)
	    : num_depth_levels_(other.num_depth_levels_)
	    , half_max_value_(other.half_max_value_)
	    , block_(other.block_)
	    , free_block_(other.free_block_)
	    , node_half_length_(other.node_half_length_)
	    , node_half_length_reciprocal_(other.node_half_length_reciprocal_)
	{
	}

	Tree(Tree&& other)
	    : num_depth_levels_(std::move(other.num_depth_levels_))
	    , half_max_value_(std::move(other.half_max_value_))
	    , block_(std::move(other.block_))
	    , free_block_(std::move(other.free_block_))
	    , node_half_length_(std::move(other.node_half_length_))
	    , node_half_length_reciprocal_(std::move(other.node_half_length_reciprocal_))
	{
	}

	template <class Derived2>
	Tree(Tree<Derived2, Block> const& other)
	    : num_depth_levels_(other.num_depth_levels_)
	    , half_max_value_(other.half_max_value_)
	    , block_(other.block_)
	    , free_block_(other.free_block_)
	    , node_half_length_(other.node_half_length_)
	    , node_half_length_reciprocal_(other.node_half_length_reciprocal_)
	{
	}

	template <class Derived2>
	Tree(Tree<Derived2, Block>&& other)
	    : num_depth_levels_(std::move(other.num_depth_levels_))
	    , half_max_value_(std::move(other.half_max_value_))
	    , block_(std::move(other.block_))
	    , free_block_(std::move(other.free_block_))
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

	Tree& operator=(Tree const& rhs)
	{
		num_depth_levels_            = rhs.num_depth_levels_;
		half_max_value_              = rhs.half_max_value_;
		block_                       = rhs.block_;
		free_block_                  = rhs.free_block_;
		node_half_length_            = rhs.node_half_length_;
		node_half_length_reciprocal_ = rhs.node_half_length_reciprocal_;
		return *this;
	}

	Tree& operator=(Tree&& rhs)
	{
		num_depth_levels_            = std::move(rhs.num_depth_levels_);
		half_max_value_              = std::move(rhs.half_max_value_);
		block_                       = std::move(rhs.block_);
		free_block_                  = std::move(rhs.free_block_);
		node_half_length_            = std::move(rhs.node_half_length_);
		node_half_length_reciprocal_ = std::move(rhs.node_half_length_reciprocal_);
		return *this;
	}

	template <class Derived2>
	Tree& operator=(Tree<Derived2, Block> const& rhs)
	{
		num_depth_levels_            = rhs.num_depth_levels_;
		half_max_value_              = rhs.half_max_value_;
		block_                       = rhs.block_;
		free_block_                  = rhs.free_block_;
		node_half_length_            = rhs.node_half_length_;
		node_half_length_reciprocal_ = rhs.node_half_length_reciprocal_;
		return *this;
	}

	template <class Derived2>
	Tree& operator=(Tree<Derived2, Block>&& rhs)
	{
		num_depth_levels_            = std::move(rhs.num_depth_levels_);
		half_max_value_              = std::move(rhs.half_max_value_);
		block_                       = std::move(rhs.block_);
		free_block_                  = std::move(rhs.free_block_);
		node_half_length_            = std::move(rhs.node_half_length_);
		node_half_length_reciprocal_ = std::move(rhs.node_half_length_reciprocal_);
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         Swap                                        |
	|                                                                                     |
	**************************************************************************************/

	void swap(Tree& other)
	{
		std::swap(num_depth_levels_, other.num_depth_levels_);
		std::swap(half_max_value_, other.half_max_value_);
		std::swap(block_, other.block_);
		std::swap(free_block_, other.free_block_);
		std::swap(node_half_length_, other.node_half_length_);
		std::swap(node_half_length_reciprocal_, other.node_half_length_reciprocal_);
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

	template <
	    class NodeFun, class BlockFun, class UpdateFun, class StopFun,
	    std::enable_if_t<std::is_invocable_r_v<void, NodeFun, Index>, bool>          = true,
	    std::enable_if_t<std::is_invocable_r_v<void, BlockFun, pos_t>, bool>         = true,
	    std::enable_if_t<std::is_invocable_r_v<void, UpdateFun, Index, pos_t>, bool> = true,
	    std::enable_if_t<std::is_invocable_r_v<bool, StopFun, Index>, bool>          = true>
	void recursLeaves(Index node, NodeFun node_f, BlockFun block_f, UpdateFun update_f,
	                  StopFun stop_f) const
	{
		assert(valid(node));

		if (isLeaf(node)) {
			node_f(node);
			return;
		}

		if (stop_f(node)) {
			return;
		}

		auto c = children(node);

		if (allLeaf(c)) {
			block_f(c);
		} else {
			for (std::size_t i{}; BF > i; ++i) {
				recursLeaves(Index(c, i), node_f, block_f, update_f, stop_f);
			}
		}

		update_f(node, c);
	}

	template <
	    class NodeFun, class BlockFun, class UpdateFun,
	    std::enable_if_t<std::is_invocable_r_v<void, NodeFun, Index>, bool>          = true,
	    std::enable_if_t<std::is_invocable_r_v<void, BlockFun, pos_t>, bool>         = true,
	    std::enable_if_t<std::is_invocable_r_v<void, UpdateFun, Index, pos_t>, bool> = true>
	void recursLeaves(Index node, NodeFun node_f, BlockFun block_f,
	                  UpdateFun update_f) const
	{
		recursLeaves(node, node_f, block_f, update_f,
		             [](Index /* node*/) -> bool { return false; });
	}

	template <
	    class NodeFun, class UpdateFun,
	    std::enable_if_t<std::is_invocable_r_v<void, NodeFun, Index>, bool>          = true,
	    std::enable_if_t<std::is_invocable_r_v<void, UpdateFun, Index, pos_t>, bool> = true>
	void recursLeaves(Index node, NodeFun node_f, UpdateFun update_f) const
	{
		recursLeaves(
		    node, node_f, [](pos_t /* pos */) -> void {}, update_f,
		    [](Index /* node*/) -> bool { return false; });
	}

	template <class NodeFun, class BlockFun,
	          std::enable_if_t<std::is_invocable_r_v<void, NodeFun, Index>, bool>  = true,
	          std::enable_if_t<std::is_invocable_r_v<void, BlockFun, pos_t>, bool> = true>
	void recursLeaves(Index node, NodeFun node_f, BlockFun block_f) const
	{
		recursLeaves(
		    node, node_f, block_f, [](Index /* node */, pos_t /* children */) -> void {},
		    [](Index /* node */) -> bool { return false; });
	}

	template <class NodeFun,
	          std::enable_if_t<std::is_invocable_r_v<void, NodeFun, Index>, bool> = true>
	void recursLeaves(Index node, NodeFun node_f) const
	{
		recursLeaves(
		    node, node_f,
		    [node_f](pos_t pos) -> void {
			    for (std::size_t i{}; BF > i; ++i) {
				    node_f(Index(pos, i));
			    }
		    },
		    [](Index /* node */, pos_t /* children */) -> void {},
		    [](Index /* node */) -> bool { return false; });
	}

	template <
	    class NodeFun, class UpdateFun, class StopFun,
	    std::enable_if_t<std::is_invocable_r_v<void, NodeFun, Index>, bool>          = true,
	    std::enable_if_t<std::is_invocable_r_v<void, UpdateFun, Index, pos_t>, bool> = true,
	    std::enable_if_t<std::is_invocable_r_v<bool, StopFun, Index>, bool>          = true>
	void recursLeaves(Index node, NodeFun node_f, UpdateFun update_f, StopFun stop_f) const
	{
		recursLeaves(node, node_f, [](pos_t /* pos */) -> void {}, update_f, stop_f);
	}

	template <class NodeFun, class BlockFun, class StopFun,
	          std::enable_if_t<std::is_invocable_r_v<void, NodeFun, Index>, bool>  = true,
	          std::enable_if_t<std::is_invocable_r_v<void, BlockFun, pos_t>, bool> = true,
	          std::enable_if_t<std::is_invocable_r_v<bool, StopFun, Index>, bool>  = true>
	void recursLeaves(Index node, NodeFun node_f, BlockFun block_f, StopFun stop_f) const
	{
		recursLeaves(
		    node, node_f, block_f, [](Index /* node */, pos_t /* children */) -> void {},
		    stop_f);
	}

	template <class NodeFun, class StopFun,
	          std::enable_if_t<std::is_invocable_r_v<void, NodeFun, Index>, bool> = true,
	          std::enable_if_t<std::is_invocable_r_v<bool, StopFun, Index>, bool> = true>
	void recursLeaves(Index node, NodeFun node_f, StopFun stop_f) const
	{
		recursLeaves(
		    node, node_f,
		    [node_f](pos_t pos) -> void {
			    for (std::size_t i{}; BF > i; ++i) {
				    node_f(Index(pos, i));
			    }
		    },
		    [](Index /* node */, pos_t /* children */) -> void {}, stop_f);
	}

	template <class NodeFun, class StopFun,
	          std::enable_if_t<std::is_invocable_v<NodeFun, Index>, bool>         = true,
	          std::enable_if_t<std::is_invocable_r_v<bool, StopFun, Index>, bool> = true>
	void recursChildrenFirst(Index node, NodeFun node_f, StopFun stop_f)
	{
		assert(valid(node));

		if (isParent(node) && !stop_f(node)) {
			auto c = children(node);
			for (std::size_t i{}; BF > i; ++i) {
				recursChildrenFirst(Index(c, i), node_f, stop_f);
			}
		}

		node_f(node);
	}

	template <class NodeFun,
	          std::enable_if_t<std::is_invocable_v<NodeFun, Index>, bool> = true>
	void recursChildrenFirst(Index node, NodeFun node_f)
	{
		recursChildrenFirst(node, node_f, [](auto) { return false; });
	}

	template <class NodeFun, class BlockFun, class StopFun,
	          std::enable_if_t<std::is_invocable_v<NodeFun, Index>, bool>         = true,
	          std::enable_if_t<std::is_invocable_v<BlockFun, pos_t>, bool>        = true,
	          std::enable_if_t<std::is_invocable_r_v<bool, StopFun, Index>, bool> = true>
	void recursChildrenFirst(Index node, NodeFun node_f, BlockFun block_f, StopFun stop_f)
	{
		assert(valid(node));

		if (isParent(node) && !stop_f(node)) {
			recursChildrenFirst(children(node), block_f, stop_f);
		}

		node_f(node);
	}

	template <class NodeFun, class BlockFun,
	          std::enable_if_t<std::is_invocable_v<NodeFun, Index>, bool>  = true,
	          std::enable_if_t<std::is_invocable_v<BlockFun, pos_t>, bool> = true>
	void recursChildrenFirst(Index node, NodeFun node_f, BlockFun block_f)
	{
		recursChildrenFirst(node, node_f, block_f, [](auto) { return false; });
	}

	template <class BlockFun, class StopFun,
	          std::enable_if_t<std::is_invocable_v<BlockFun, pos_t>, bool>        = true,
	          std::enable_if_t<std::is_invocable_r_v<bool, StopFun, Index>, bool> = true>
	void recursChildrenFirst(pos_t block, BlockFun block_f, StopFun stop_f)
	{
		assert(valid(block));

		for (std::size_t i{}; BF > i; ++i) {
			Index node(block, i);
			if (isParent(node) && !stop_f(node)) {
				recursChildrenFirst(children(node), block_f, stop_f);
			}
		}

		block_f(block);
	}

	template <class BlockFun,
	          std::enable_if_t<std::is_invocable_v<BlockFun, pos_t>, bool> = true>
	void recursChildrenFirst(pos_t block, BlockFun block_f)
	{
		recursChildrenFirst(block, block_f, [](auto) { return false; });
	}

	template <class NodeFun, class StopFun,
	          std::enable_if_t<std::is_invocable_v<NodeFun, Index>, bool>         = true,
	          std::enable_if_t<std::is_invocable_r_v<bool, StopFun, Index>, bool> = true>
	void recursParentFirst(Index node, NodeFun node_f, StopFun stop_f)
	{
		assert(valid(node));

		node_f(node);

		if (isLeaf(node) || stop_f(node)) {
			return;
		}

		auto c = children(node);
		for (std::size_t i{}; BF > i; ++i) {
			recursParentFirst(Index(c, i), node_f, stop_f);
		}
	}

	template <class NodeFun,
	          std::enable_if_t<std::is_invocable_v<NodeFun, Index>, bool> = true>
	void recursParentFirst(Index node, NodeFun node_f)
	{
		recursParentFirst(node, node_f, [](auto) { return false; });
	}

	template <class NodeFun, class BlockFun, class StopFun,
	          std::enable_if_t<std::is_invocable_v<NodeFun, Index>, bool>         = true,
	          std::enable_if_t<std::is_invocable_v<BlockFun, pos_t>, bool>        = true,
	          std::enable_if_t<std::is_invocable_r_v<bool, StopFun, Index>, bool> = true>
	void recursParentFirst(Index node, NodeFun node_f, BlockFun block_f, StopFun stop_f)
	{
		assert(valid(node));

		node_f(node);

		if (isLeaf(node) || stop_f(node)) {
			return;
		}

		recursParentFirst(children(node), block_f, stop_f);
	}

	template <class NodeFun, class BlockFun,
	          std::enable_if_t<std::is_invocable_v<NodeFun, Index>, bool>  = true,
	          std::enable_if_t<std::is_invocable_v<BlockFun, pos_t>, bool> = true>
	void recursParentFirst(Index node, NodeFun node_f, BlockFun block_f)
	{
		recursParentFirst(node, node_f, block_f, [](auto) { return false; });
	}

	template <class BlockFun, class StopFun,
	          std::enable_if_t<std::is_invocable_v<BlockFun, pos_t>, bool>        = true,
	          std::enable_if_t<std::is_invocable_r_v<bool, StopFun, Index>, bool> = true>
	void recursParentFirst(pos_t block, BlockFun block_f, StopFun stop_f)
	{
		assert(valid(block));

		block_f(block);

		if (allLeaf(block)) {
			return;
		}

		for (std::size_t i{}; BF > i; ++i) {
			Index node(block, i);
			if (isParent(node) && !stop_f(node)) {
				recursParentFirst(children(node), block_f, stop_f);
			}
		}
	}

	template <class BlockFun,
	          std::enable_if_t<std::is_invocable_v<BlockFun, pos_t>, bool> = true>
	void recursParentFirst(pos_t block, BlockFun block_f)
	{
		recursParentFirst(block, block_f, [](auto) { return false; });
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
		assert(block_.size() > block);
		return 0 == block_[block].depth();
	}

	/*!
	 * @brief Checks if any node of a block is a pure leaves.
	 *
	 * @param block the block to check
	 * @return `true` if any node of the block is a pure leaf, `false` otherwise.
	 */
	[[nodiscard]] bool anyPureLeaf(pos_t block) const
	{
		assert(block_.size() > block);
		return 0 == block_[block].depth();
	}

	/*!
	 * @brief Checks if no nodes of a block are pure leaves.
	 *
	 * @param block the block to check
	 * @return `true` if no nodes of the block are pure leaves, `false` otherwise.
	 */
	[[nodiscard]] bool nonePureLeaf(pos_t block) const
	{
		assert(block_.size() > block);
		return 0 != block_[block].depth();
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
		assert(block_.size() > block);
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
		assert(block_.size() > block);
		return std::all_of(block_[block].children.begin(), block_[block].children.end(),
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
		assert(block_.size() > block);
		return std::any_of(block_[block].children.begin(), block_[block].children.end(),
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
		assert(block_.size() > block);
		return std::none_of(block_[block].children.begin(), block_[block].children.end(),
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

	[[nodiscard]] Block& treeBlock(pos_t block)
	{
		assert(valid(block));
		return block_[block];
	}

	[[nodiscard]] Block const& treeBlock(pos_t block) const
	{
		assert(valid(block));
		return block_[block];
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

	/**************************************************************************************
	|                                                                                     |
	|                                       Center                                        |
	|                                                                                     |
	**************************************************************************************/

	//
	// Child center
	//

	/*!
	 * @brief Returns the center of the `child_index`th child.
	 *
	 * @param center the center of the parent
	 * @param half_length the half length of the parent
	 * @param child_index the index of the child
	 * @return The center of the `child_index`th child.
	 */
	[[nodiscard]] static constexpr Point childCenter(Point center, length_t half_length,
	                                                 offset_t child_index)
	{
		assert(branchingFactor() > child_index);
		half_length /= static_cast<length_t>(2);
		for (std::size_t i{}; Point::size() > i; ++i) {
			center[i] += (child_index & offset_t(1u << i)) ? half_length : -half_length;
		}
		return center;
	}

	//
	// Parent center
	//

	/*!
	 * @brief Returns the center of the parent of the node.
	 *
	 * @param center the center of the child
	 * @param half_length the half length of the child
	 * @param index the index of the child
	 * @return The center of the parent.
	 */
	[[nodiscard]] static constexpr Point parentCenter(Point center, length_t half_length,
	                                                  offset_t index)
	{
		assert(branchingFactor() > index);
		for (std::size_t i{}; Point::size() > i; ++i) {
			center[i] += (index & offset_t(1u << i)) ? -half_length : half_length;
		}
		return center;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                 Create/erase nodes                                  |
	|                                                                                     |
	**************************************************************************************/

	pos_t createBlock(Index node)
	{
		pos_t children                         = static_cast<pos_t>(block_.size());
		block_[node.pos].children[node.offset] = children;
		block_.emplace_back(block_[node.pos], node.offset, halfLength(node));
		derived().onCreateChildren(node);
		return children;
	}

	void fillChildren(Index node, pos_t children)
	{
		block_[node.pos].children[node.offset] = children;
		block_[children].fill(block_[node.pos], node.offset, halfLength(node));
		derived().onFillChildren(node, children);
	}

	void pruneChildren(Index node, pos_t children)
	{
		block_[node.pos].children[node.offset] = Index::NULL_POS;
		derived().onPruneChildren(node, children);
		// Important that derived is pruned first in case they use parent code
		block_[children] = Block();
	}

	//
	// Create
	//

	Index create(Code code, Index cur_node)
	{
		assert(valid(code));
		assert(valid(cur_node));
		auto wanted_depth = depth(code);
		auto cur_depth    = depth(cur_node);
		while (wanted_depth < cur_depth) {
			cur_node = createChild(cur_node, code.offset(--cur_depth));
		}
		return cur_node;
	}

	std::array<Index, maxNumDepthLevels()> createTrail(Code code, Index cur_node)
	{
		std::array<Index, maxNumDepthLevels()> trail{};
		auto                                   wanted_depth = depth(code);
		auto                                   cur_depth    = depth(cur_node);
		trail[cur_depth]                                    = cur_node;
		while (wanted_depth < cur_depth) {
			cur_node         = createChild(cur_node, code.offset(--cur_depth));
			trail[cur_depth] = cur_node;
		}
		return trail;
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

	void eraseChildren(Index node, pos_t children)
	{
		if (!valid(children)) {
			return;
		}

		auto child_blocks = this->children(children);
		for (offset_t i{}; child_blocks.size() > i; ++i) {
			eraseChildren(Index(children, i), child_blocks[i]);
		}

		pruneChildren(node, children);
		free_block_.push_back(children);
	}

	void eraseChildrenThreadSafe(Index node, pos_t children)
	{
		// TODO: Implement
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Nearest                                       |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Benchmark against only returning the distance

	template <bool OnlyDistance, bool FastAsSonic, class ValueFun, class InnerFun>
	[[nodiscard]] std::conditional_t<OnlyDistance, float, std::pair<float, Index>> nearest(
	    Index node, NearestSearchAlgorithm search_alg, ValueFun value_f, InnerFun inner_f,
	    float max_dist, float epsilon) const
	{
		assert(std::isfinite(max_dist));
		assert(std::isfinite(epsilon));

		

		std::conditional_t<OnlyDistance, float, std::pair<float, Index>> closest{};
		if constexpr (OnlyDistance) {
			closest = max_dist;
		} else {
			closest.first = max_dist;
		}

		if (isParent(node) && max_dist >= inner_f(node)) {
			auto cb = children(node);
			auto cd = depth(cb);

			// if (0.0f < epsilon) {
			// switch (search_alg) {
			// 	case NearestSearchAlgorithm::DEPTH_FIRST:
			closest = nearestDepthFirst<OnlyDistance, FastAsSonic>(cb, cd, max_dist, epsilon,
			                                                       value_f, inner_f);
			// 		case NearestSearchAlgorithm::A_STAR:
			// 			closest = nearestAStar(cb, cd, max_dist, epsilon, value_f, inner_f);
			// 	}
			// } else {
			// 	switch (search_alg) {
			// 		case NearestSearchAlgorithm::DEPTH_FIRST:
			// 			closest = nearestDepthFirst(cb, cd, max_dist, value_f, inner_f);
			// 		case NearestSearchAlgorithm::A_STAR:
			// 			closest = nearestAStar(cb, cd, max_dist, value_f, inner_f);
			// 	}
			// }
		}

		if constexpr (!FastAsSonic) {
			max_dist = value_f(node);
		}
		assert(!std::isnan(dist));
		if constexpr (OnlyDistance) {
			return UFO_MIN(closest, max_dist);
		} else {
			return closest.first < max_dist ? closest : std::pair{max_dist, node};
		}
	}

	template <class Pred, class ValueFun, class InnerFun,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool> = true>
	[[nodiscard]] std::pair<float, Index> nearest(Index node, Pred pred,
	                                              NearestSearchAlgorithm search_alg,
	                                              ValueFun value_f, InnerFun inner_f,
	                                              float max_dist, float epsilon) const
	{
		using Filter = pred::Filter<Pred>;

		Filter::init(pred);

		auto wrapped_value_f = [value_f, &pred](Index node) -> float {
			return Filter::returnable(pred) ? value_f(node)
			                                : std::numeric_limits<float>::infinity();
		};

		auto wrapped_inner_f = [inner_f, &pred](Index node) -> float {
			return Filter::traversable(pred) ? inner_f(node)
			                                 : std::numeric_limits<float>::infinity();
		};

		return nearest(node, search_alg, wrapped_value_f, wrapped_inner_f, max_dist,
	epsilon);
	}

	template <bool OnlyDistance, bool FastAsSonic, class ValueFun, class InnerFun>
	[[nodiscard]] std::conditional_t<OnlyDistance, float, std::pair<float, Index>>
	nearestDepthFirst(pos_t block, depth_t depth, float c_dist, float epsilon,
	                  ValueFun value_f, InnerFun inner_f) const
	{
		struct StackElement {
			using Container = std::array<std::pair<float, pos_t>, BF>;
			using Iterator  = typename Container::iterator;

			Container container;
			Iterator  it;

			[[nodiscard]] constexpr float& distance() { return it->first; }

			[[nodiscard]] constexpr float const& distance() const { return it->first; }

			[[nodiscard]] constexpr pos_t& block() { return it->second; }

			[[nodiscard]] constexpr pos_t const& block() const { return it->second; }

			constexpr void start() { it = container.begin(); }

			[[nodiscard]] constexpr bool empty() { return container.end() == it; }

			[[nodiscard]] constexpr bool empty() const { return container.end() == it; }

			StackElement& operator++()
			{
				++it;
				return *this;
			}

			constexpr void sort()
			{
				if constexpr (2 == BF) {
					UFO_SORT_ASCENDING_PAIR_FIRST_2(container);
				} else if constexpr (4 == BF) {
					UFO_SORT_ASCENDING_PAIR_FIRST_4(container);
				} else if constexpr (8 == BF) {
					UFO_SORT_ASCENDING_PAIR_FIRST_8(container);
				} else if constexpr (16 == BF) {
					UFO_SORT_ASCENDING_PAIR_FIRST_16(container);
				} else {
					std::sort(container.begin(), container.end(),
					          [](auto a, auto b) { return a.first < b.first; });
				}
			}
		};

		using Stack = std::array<StackElement, maxNumDepthLevels() - 1>;

		Stack stack;
		// Since we only have one block in the beginning we set the index to `BF - 1u` (the
		// last index)
		stack[depth].it = std::prev(stack[depth].container.end());
		// The first block to go through
		stack[depth].block() = block;
		// This distance does not matter as long as it is less than `c_dist - epsilon`, since
		// we only have one block in the beginning
		stack[depth].distance() = 0.0f;

		std::conditional_t<OnlyDistance, bool, Index> c_node;

		std::array<std::conditional_t<OnlyDistance, float, std::pair<float, offset_t>>, BF> d;

		for (depth_t max_depth = depth + 1; max_depth > depth;) {
			StackElement& se = stack[depth];

			if (se.empty() || c_dist - epsilon <= se.distance()) {
				++depth;
				continue;
			}

			block = se.block();
			++se;

			StackElement& cur = stack[depth - 1u];

			cur.start();

			for (std::size_t i{}; BF > i; ++i) {
				Index node(block, i);
				cur.container[i].first = inner_f(node);
				assert(!std::isnan(cur.container[i].first));
				cur.container[i].second = children(node);

				if constexpr (!FastAsSonic) {
					if constexpr (OnlyDistance) {
						d[i] = value_f(node);
						assert(!std::isnan(d[i]));
					} else {
						d[i].first = value_f(node);
						assert(!std::isnan(d[i].first));
						d[i].second = i;
					}
				}
			}

			if constexpr (!FastAsSonic) {
				if constexpr (OnlyDistance) {
					if constexpr (2 == BF) {
						UFO_MIN_2(d);
					} else if constexpr (4 == BF) {
						UFO_MIN_4(d);
					} else if constexpr (8 == BF) {
						UFO_MIN_8(d);
					} else if constexpr (16 == BF) {
						UFO_MIN_16(d);
					} else {
						for (std::size_t i = 1; BF > i; ++i) {
							d[0] = UFO_MIN(d[0], d[i]);
						}
					}

					c_dist = c_dist <= d[0] ? c_dist : d[0];
				} else {
					if constexpr (2 == BF) {
						UFO_MIN_PAIR_FIRST_2(d);
					} else if constexpr (4 == BF) {
						UFO_MIN_PAIR_FIRST_4(d);
					} else if constexpr (8 == BF) {
						UFO_MIN_PAIR_FIRST_8(d);
					} else if constexpr (16 == BF) {
						UFO_MIN_PAIR_FIRST_16(d);
					} else {
						for (std::size_t i = 1; BF > i; ++i) {
							d[0] = UFO_MIN_PAIR_FIRST(d[0], d[i]);
						}
					}

					c_node = c_dist <= d[0].first ? c_node : Index{block, d[0].second};
					c_dist = c_dist <= d[0].first ? c_dist : d[0].first;
				}
			}

			if (1u == depth) {
				for (auto [dist, child_block] : cur.container) {
					if (c_dist <= dist + epsilon) {
						continue;
					}

					if constexpr (OnlyDistance) {
						for (offset_t i{}; BF > i; ++i) {
							d[i] = value_f(Index(child_block, i));
							assert(!std::isnan(d[i]));
						}

						if constexpr (2 == BF) {
							UFO_MIN_2(d);
						} else if constexpr (4 == BF) {
							UFO_MIN_4(d);
						} else if constexpr (8 == BF) {
							UFO_MIN_8(d);
						} else if constexpr (16 == BF) {
							UFO_MIN_16(d);
						} else {
							for (std::size_t i = 1; BF > i; ++i) {
								d[0] = UFO_MIN(d[0], d[i]);
							}
						}

						c_dist = c_dist <= d[0] ? c_dist : d[0];
					} else {
						for (offset_t i{}; BF > i; ++i) {
							d[i].first = value_f(Index(child_block, i));
							assert(!std::isnan(d[i].first));
							d[i].second = i;
						}

						if constexpr (2 == BF) {
							UFO_MIN_PAIR_FIRST_2(d);
						} else if constexpr (4 == BF) {
							UFO_MIN_PAIR_FIRST_4(d);
						} else if constexpr (8 == BF) {
							UFO_MIN_PAIR_FIRST_8(d);
						} else if constexpr (16 == BF) {
							UFO_MIN_PAIR_FIRST_16(d);
						} else {
							for (std::size_t i = 1; BF > i; ++i) {
								d[0] = UFO_MIN_PAIR_FIRST(d[0], d[i]);
							}
						}

						c_node = c_dist <= d[0].first ? c_node : Index{child_block, d[0].second};
						c_dist = c_dist <= d[0].first ? c_dist : d[0].first;
					}
				}
			} else {
				cur.sort();
				--depth;
			}
		}

		if constexpr (OnlyDistance) {
			return c_dist;
		} else {
			return {c_dist, c_node};
		}
	}

	// template <class ValueFun, class InnerFun>
	// [[nodiscard]] std::pair<float, Index> nearestDepthFirst(pos_t block, depth_t depth,
	//                                                         float c_dist, ValueFun
	//                                                         value_f, InnerFun inner_f)
	//                                                         const
	// {
	// 	using Stack =
	// 	    std::array<std::pair<std::size_t, std::array<std::pair<float, pos_t>, BF>>,
	// 	               maxNumDepthLevels() - 1>;

	// 	Stack stack;
	// 	stack[depth].first                 = BF - 1u;
	// 	stack[depth].second[BF - 1].first  = 0.0f;
	// 	stack[depth].second[BF - 1].second = block;

	// 	Index c_node;

	// 	for (depth_t max_depth = depth + 1; max_depth > depth;) {
	// 		auto& [idx, c] = stack[depth];

	// 		if (BF <= idx || c_dist <= c[idx].first) {
	// 			++depth;
	// 			continue;
	// 		}

	// 		block = c[idx].second;
	// 		++idx;

	// 		stack[depth - 1].first = 0;
	// 		auto& candidates       = stack[depth - 1].second;

	// 		for (std::size_t i{}; BF > i; ++i) {
	// 			Index node(block, i);
	// 			candidates[i].first = inner_f(node);
	// 			assert(!std::isnan(candidates[i].first));
	// 			candidates[i].second = children(node);
	// 		}

	// 		if (1u == depth) {
	// 			std::array<std::pair<float, offset_t>, BF> d;
	// 			for (auto [dist, child_block] : candidates) {
	// 				if (c_dist <= dist) {
	// 					continue;
	// 				}

	// 				for (offset_t i{}; BF > i; ++i) {
	// 					d[i].first = value_f(Index(child_block, i));
	// 					assert(!std::isnan(d[i].first));
	// 					d[i].second = i;
	// 				}

	// 				if constexpr (2 == BF) {
	// 					UFO_MIN_PAIR_FIRST_2(d);
	// 				} else if constexpr (4 == BF) {
	// 					UFO_MIN_PAIR_FIRST_4(d);
	// 				} else if constexpr (8 == BF) {
	// 					UFO_MIN_PAIR_FIRST_8(d);
	// 				} else if constexpr (16 == BF) {
	// 					UFO_MIN_PAIR_FIRST_16(d);
	// 				} else {
	// 					for (std::size_t i = 1; BF > i; ++i) {
	// 						d[0] = UFO_MIN_PAIR_FIRST(d[0], d[i]);
	// 					}
	// 				}

	// 				c_node = c_dist <= d[0].first ? c_node : Index{child_block, d[0].second};
	// 				c_dist = c_dist <= d[0].first ? c_dist : d[0].first;
	// 			}
	// 		} else {
	// 			if constexpr (2 == BF) {
	// 				UFO_SORT_ASCENDING_PAIR_FIRST_2(candidates);
	// 			} else if constexpr (4 == BF) {
	// 				UFO_SORT_ASCENDING_PAIR_FIRST_4(candidates);
	// 			} else if constexpr (8 == BF) {
	// 				UFO_SORT_ASCENDING_PAIR_FIRST_8(candidates);
	// 			} else if constexpr (16 == BF) {
	// 				UFO_SORT_ASCENDING_PAIR_FIRST_16(candidates);
	// 			} else {
	// 				std::sort(candidates.begin(), candidates.end(),
	// 				          [](auto a, auto b) { return a.first < b.first; });
	// 			}
	// 			--depth;
	// 		}
	// 	}

	// 	return {c_dist, c_node};
	// }

	// template <class ValueFun, class InnerFun>
	// [[nodiscard]] std::pair<float, Index> nearestAStar(pos_t block, depth_t depth,
	//                                                    float c_dist, float epsilon,
	//                                                    ValueFun value_f,
	//                                                    InnerFun inner_f) const
	// {
	// 	struct S {
	// 		float   dist;
	// 		pos_t   block;
	// 		depth_t depth;

	// 		S(float dist, pos_t block, depth_t depth) noexcept
	// 		    : dist(dist), block(block), depth(depth)
	// 		{
	// 		}

	// 		bool operator>(S rhs) const noexcept
	// 		{
	// 			// return dist > rhs.dist;
	// 			return dist + (depth << 2) > rhs.dist + (rhs.depth << 2);
	// 		}
	// 	};

	// 	using Queue = std::priority_queue<S, std::vector<S>, std::greater<S>>;

	// 	std::vector<S> container;
	// 	container.reserve(1024);
	// 	Queue queue(std::greater<S>{}, std::move(container));
	// 	queue.emplace(0.0f, block, depth);

	// 	auto max_size = depth << 2;

	// 	Index c_node;

	// 	while (!queue.empty()) {
	// 		auto cur = queue.top();

	// 		if (c_dist + max_size - (cur.depth << 2) <= cur.dist + epsilon) {
	// 			return {c_dist, c_node};
	// 		}

	// 		if (c_dist <= cur.dist + epsilon) {
	// 			queue.pop();
	// 			continue;
	// 		}

	// 		queue.pop();

	// 		block = cur.block;
	// 		depth = cur.depth;

	// 		std::array<std::pair<float, pos_t>, BF> candidates;
	// 		for (std::size_t i{}; BF > i; ++i) {
	// 			Index node(block, i);
	// 			candidates[i].first = inner_f(node);
	// 			assert(!std::isnan(candidates[i].first));
	// 			candidates[i].second = children(node);
	// 		}

	// 		if (1u == depth) {
	// 			std::array<std::pair<float, offset_t>, BF> d;
	// 			for (auto [dist, child_block] : candidates) {
	// 				if (c_dist <= dist + epsilon) {
	// 					continue;
	// 				}

	// 				for (offset_t i{}; BF > i; ++i) {
	// 					d[i].first = value_f(Index(child_block, i));
	// 					assert(!std::isnan(d[i].first));
	// 					d[i].second = i;
	// 				}

	// 				if constexpr (2 == BF) {
	// 					UFO_MIN_PAIR_FIRST_2(d);
	// 				} else if constexpr (4 == BF) {
	// 					UFO_MIN_PAIR_FIRST_4(d);
	// 				} else if constexpr (8 == BF) {
	// 					UFO_MIN_PAIR_FIRST_8(d);
	// 				} else if constexpr (16 == BF) {
	// 					UFO_MIN_PAIR_FIRST_16(d);
	// 				} else {
	// 					for (std::size_t i = 1; BF > i; ++i) {
	// 						d[0] = UFO_MIN_PAIR_FIRST(d[0], d[i]);
	// 					}
	// 				}

	// 				c_node = c_dist <= d[0].first ? c_node : Index{child_block, d[0].second};
	// 				c_dist = c_dist <= d[0].first ? c_dist : d[0].first;
	// 			}
	// 		} else {
	// 			for (auto [dist, child_block] : candidates) {
	// 				if (c_dist <= dist + epsilon) {
	// 					continue;
	// 				}
	// 				queue.emplace(dist, child_block, depth - 1);
	// 			}
	// 		}
	// 	}

	// 	return {c_dist, c_node};
	// }

	// template <class ValueFun, class InnerFun>
	// [[nodiscard]] std::pair<float, Index> nearestAStar(pos_t block, depth_t depth,
	//                                                    float c_dist, ValueFun value_f,
	//                                                    InnerFun inner_f) const
	// {
	// 	struct S {
	// 		float   dist;
	// 		pos_t   block;
	// 		depth_t depth;

	// 		S(float dist, pos_t block, depth_t depth) noexcept
	// 		    : dist(dist), block(block), depth(depth)
	// 		{
	// 		}

	// 		bool operator>(S rhs) const noexcept
	// 		{
	// 			// return dist > rhs.dist;
	// 			return dist + (depth << 2) > rhs.dist + (rhs.depth << 2);
	// 		}
	// 	};

	// 	using Queue = std::priority_queue<S, std::vector<S>, std::greater<S>>;

	// 	std::vector<S> container;
	// 	container.reserve(1024);
	// 	Queue queue(std::greater<S>{}, std::move(container));
	// 	queue.emplace(0.0f, block, depth);

	// 	auto max_size = depth << 2;

	// 	Index c_node;

	// 	while (!queue.empty()) {
	// 		auto cur = queue.top();

	// 		if (c_dist + max_size - (cur.depth << 2) <= cur.dist) {
	// 			return {c_dist, c_node};
	// 		}

	// 		if (c_dist <= cur.dist) {
	// 			queue.pop();
	// 			continue;
	// 		}

	// 		queue.pop();

	// 		block = cur.block;
	// 		depth = cur.depth;

	// 		std::array<std::pair<float, pos_t>, BF> candidates;
	// 		for (std::size_t i{}; BF > i; ++i) {
	// 			Index node(block, i);
	// 			candidates[i].first = inner_f(node);
	// 			assert(!std::isnan(candidates[i].first));
	// 			candidates[i].second = children(node);
	// 		}

	// 		if (1u == depth) {
	// 			std::array<std::pair<float, offset_t>, BF> d;
	// 			for (auto [dist, child_block] : candidates) {
	// 				if (c_dist <= dist) {
	// 					continue;
	// 				}

	// 				for (offset_t i{}; BF > i; ++i) {
	// 					d[i].first = value_f(Index(child_block, i));
	// 					assert(!std::isnan(d[i].first));
	// 					d[i].second = i;
	// 				}

	// 				if constexpr (2 == BF) {
	// 					UFO_MIN_PAIR_FIRST_2(d);
	// 				} else if constexpr (4 == BF) {
	// 					UFO_MIN_PAIR_FIRST_4(d);
	// 				} else if constexpr (8 == BF) {
	// 					UFO_MIN_PAIR_FIRST_8(d);
	// 				} else if constexpr (16 == BF) {
	// 					UFO_MIN_PAIR_FIRST_16(d);
	// 				} else {
	// 					for (std::size_t i = 1; BF > i; ++i) {
	// 						d[0] = UFO_MIN_PAIR_FIRST(d[0], d[i]);
	// 					}
	// 				}

	// 				c_node = c_dist <= d[0].first ? c_node : Index{child_block, d[0].second};
	// 				c_dist = c_dist <= d[0].first ? c_dist : d[0].first;
	// 			}
	// 		} else {
	// 			for (auto [dist, child_block] : candidates) {
	// 				if (c_dist <= dist) {
	// 					continue;
	// 				}
	// 				queue.emplace(dist, child_block, depth - 1);
	// 			}
	// 		}
	// 	}

	// 	return {c_dist, c_node};
	// }

	/**************************************************************************************
	|                                                                                     |
	|                                        Trace                                        |
	|                                                                                     |
	**************************************************************************************/

	struct TraceParams {
		Point    t0;
		Point    t1;
		unsigned a{};
	};

	struct TraceStackElement {
		Point    t0;
		Point    t1;
		Point    tm;
		unsigned cur_node;
		Index    node;

		TraceStackElement() = default;

		constexpr TraceStackElement(Index node, unsigned cur_node, Point const& t0,
		                            Point const& t1, Point const& tm)
		    : node(node), cur_node(cur_node), t0(t0), t1(t1), tm(tm)
		{
		}
	};

	[[nodiscard]] TraceParams traceInit(Index node, Ray<Dim, ray_t> const& ray) const
	{
		return traceInit(ray, center(node), halfLength(node));
	}

	[[nodiscard]] TraceParams traceInit(Node node, Ray<Dim, ray_t> const& ray) const
	{
		return traceInit(ray, center(node), halfLength(node));
	}

	[[nodiscard]] static constexpr inline TraceParams traceInit(Ray<Dim, ray_t> const& ray,
	                                                            Point const& center,
	                                                            float half_length) noexcept
	{
		TraceParams params;

		for (std::size_t i{}; Dim > i; ++i) {
			float origin = 0 > ray.direction[i] ? center[i] * 2 - ray.origin[i] : ray.origin[i];

			auto a = center[i] - half_length - origin;
			auto b = center[i] + half_length - origin;

			// TODO: Look at
			params.t0[i] = 0 == ray.direction[i] ? 1e+25 * a : a / std::abs(ray.direction[i]);
			params.t1[i] = 0 == ray.direction[i] ? 1e+25 * b : b / std::abs(ray.direction[i]);

			params.a |= unsigned(0 > ray.direction[i]) << i;
		}

		return params;
	}

	[[nodiscard]] static constexpr inline unsigned firstNode(Point const& t0,
	                                                         Point const& tm) noexcept
	{
		unsigned max_comp = maxIndex(t0);
		unsigned node     = static_cast<unsigned>(tm[0] < t0[max_comp]);
		for (unsigned i = 1; Dim > i; ++i) {
			node |= static_cast<unsigned>(tm[i] < t0[max_comp]) << i;
		}
		return node;
	}

	[[nodiscard]] static constexpr inline unsigned newNode(unsigned cur,
	                                                       unsigned dim) noexcept
	{
		// You are at cur, you want to move along dim in positive direction
		unsigned x = 1u << dim;
		return ((cur & x) << Dim) | cur | x;
	}

	template <class InnerFun, class HitFun, class T>
	[[nodiscard]] constexpr T trace(Index node, TraceParams const& params, InnerFun inner_f,
	                                HitFun hit_f, T const& miss) const
	{
		constexpr auto const new_node_lut = []() {
			std::array<std::array<unsigned, Dim>, BF> lut{};
			for (unsigned cur{}; BF != cur; ++cur) {
				for (unsigned dim{}; Dim != dim; ++dim) {
					unsigned x    = 1u << dim;
					lut[cur][dim] = ((cur & x) << Dim) | cur | x;
				}
			}
			return lut;
		}();

		auto t0 = params.t0;
		auto t1 = params.t1;
		auto a  = params.a;

		if (max(t0) >= min(t1)) {
			return miss;
		}

		if (0.0f > min(t1)) {
			return miss;
		}

		float distance{};

		if (auto const& [hit, value] = hit_f(node, distance); hit) {
			return value;
		}

		if (isLeaf(node) || !inner_f(node, distance)) {
			return miss;
		}

		auto tm = 0.5f * (t0 + t1);

		unsigned cur_node = firstNode(t0, tm);

		std::array<TraceStackElement, maxNumDepthLevels()> stack;
		stack[0] = {node, cur_node, t0, t1, tm};

		for (int idx{}; 0 <= idx;) {
			node     = stack[idx].node;
			cur_node = stack[idx].cur_node;
			t0       = stack[idx].t0;
			t1       = stack[idx].t1;
			tm       = stack[idx].tm;

			node = child(node, cur_node ^ a);

			for (unsigned i{}; Dim > i; ++i) {
				t0[i] = (cur_node & (1u << i)) ? tm[i] : t0[i];
				t1[i] = (cur_node & (1u << i)) ? t1[i] : tm[i];
			}

			stack[idx].cur_node = new_node_lut[cur_node][minIndex(t1)];
			idx -= BF <= stack[idx].cur_node;

			if (0.0f > min(t1)) {
				continue;
			}

			distance = UFO_MAX(0.0f, max(t0));

			if (auto [hit, value] = hit_f(node, distance); hit) {
				return value;
			}

			if (isLeaf(node) || !inner_f(node, distance)) {
				continue;
			}

			tm = 0.5f * (t0 + t1);

			cur_node = firstNode(t0, tm);

			stack[++idx] = {node, cur_node, t0, t1, tm};
		}

		return miss;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         I/O                                         |
	|                                                                                     |
	**************************************************************************************/

	static void openFileIn(std::filesystem::path const& file, std::ifstream& ifs)
	{
		ifs.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		ifs.imbue(std::locale());
		ifs.open(file, std::ios::in | std::ios::binary);
	}

	static void openFileOut(std::filesystem::path const& file, std::ofstream& ofs)
	{
		ofs.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		ofs.imbue(std::locale());
		ofs.open(file, std::ios::out | std::ios::binary);
	}

 protected:
	// Mutex for creating children concurrently
	std::mutex create_mutex_;

	// The number of depth levels
	depth_t num_depth_levels_;
	// Half the maximum key value the tree can store
	key_t half_max_value_;

	// Blocks
	TreeContainer<Block> block_;
	// Free blocks
	std::deque<pos_t> free_block_;

	// Stores the node half length at a given depth, where the index is the depth
	std::array<length_t, maxNumDepthLevels() + 1> node_half_length_;
	// Reciprocal of the node half length at a given depth, where the index is the depth
	std::array<length_t, maxNumDepthLevels() + 1> node_half_length_reciprocal_;
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_TREE_HPP