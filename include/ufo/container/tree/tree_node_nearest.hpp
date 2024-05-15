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

#ifndef UFO_CONTAINER_TREE_TREE_NODE_NEAREST_HPP
#define UFO_CONTAINER_TREE_TREE_NODE_NEAREST_HPP

// UFO
#include <ufo/container/tree/tree_node.hpp>

// STL
#include <functional>

namespace ufo
{
template <class Code>
struct TreeNodeNearest : public TreeNode<Code> {
	double squared_distance;

	constexpr TreeNodeNearest(TreeNode<Code> const& node, double squared_distance = 0.0)
	    : TreeNode<Code>(node), squared_distance(squared_distance)
	{
	}

	[[nodiscard]] friend constexpr bool operator==(TreeNodeNearest const& a,
	                                               TreeNodeNearest const& b)
	{
		return static_cast<TreeNode<Code> const&>(a) == static_cast<TreeNode<Code> const&>(b);
	}

	[[nodiscard]] friend constexpr bool operator!=(TreeNodeNearest const& a,
	                                               TreeNodeNearest const& b)
	{
		return !(a == b);
	}

	[[nodiscard]] friend constexpr bool operator<(TreeNodeNearest const& a,
	                                              TreeNodeNearest const& b)
	{
		return a.squared_distance < b.squared_distance;
	}

	[[nodiscard]] friend constexpr bool operator<=(TreeNodeNearest const& a,
	                                               TreeNodeNearest const& b)
	{
		return a.squared_distance <= b.squared_distance;
	}

	[[nodiscard]] friend constexpr bool operator>(TreeNodeNearest const& a,
	                                              TreeNodeNearest const& b)
	{
		return a.squared_distance > b.squared_distance;
	}

	[[nodiscard]] friend constexpr bool operator>=(TreeNodeNearest const& a,
	                                               TreeNodeNearest const& b)
	{
		return a.squared_distance >= b.squared_distance;
	}
};
}  // namespace ufo

template <class Code>
struct std::hash<ufo::TreeNodeNearest<Code>> {
	std::size_t operator()(ufo::TreeNodeNearest<Code> const& node) const
	{
		return hash<Code>()(node.code());
	}
};

#endif  // UFO_CONTAINER_TREE_TREE_NODE_NEAREST_HPP