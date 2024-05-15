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

#ifndef UFO_CONTAINER_TREE_NODE_HPP
#define UFO_CONTAINER_TREE_NODE_HPP

// UFO
#include <ufo/container/tree/tree_index.hpp>

// STL
#include <functional>

namespace ufo
{
/*!
 * @brief A wrapper around a UFOMap inner/leaf node.
 *
 */
template <class Code>
struct TreeNode {
 public:
	//
	// Constructor
	//

	constexpr TreeNode() = default;

	friend void swap(TreeNode& lhs, TreeNode& rhs) noexcept
	{
		std::swap(lhs.code_, rhs.code_);
		std::swap(lhs.index_, rhs.index_);
	}

	/*!
	 * @brief Compare nodes.
	 *
	 * @param lhs,rhs The nodes to compare.
	 * @return Whether the two nodes are equal.
	 */
	friend constexpr bool operator==(TreeNode lhs, TreeNode rhs) noexcept
	{
		return lhs.code_ == rhs.code_ && lhs.index_ == rhs.index_;
	}

	/*!
	 * @brief Compare nodes.
	 *
	 * @param lhs,rhs The nodes to compare.
	 * @return Whether the two nodes are different.
	 */
	friend constexpr bool operator!=(TreeNode lhs, TreeNode rhs) noexcept
	{
		return !(lhs == rhs);
	}

	friend constexpr bool operator<(TreeNode lhs, TreeNode rhs) noexcept
	{
		return lhs.code_ < rhs.code_;
	}

	friend constexpr bool operator<=(TreeNode lhs, TreeNode rhs) noexcept
	{
		return lhs.code_ <= rhs.code_;
	}

	friend constexpr bool operator>(TreeNode lhs, TreeNode rhs) noexcept
	{
		return lhs.code_ > rhs.code_;
	}

	friend constexpr bool operator>=(TreeNode lhs, TreeNode rhs) noexcept
	{
		return lhs.code_ >= rhs.code_;
	}

	operator Code() const noexcept { return code(); }

	/*!
	 * @brief Get the code for the node.
	 *
	 * @return The code for the node.
	 */
	[[nodiscard]] constexpr Code code() const noexcept { return code_; }

	/*!
	 * @brief Get the depth of the node.
	 *
	 * @return The depth of the node.
	 */
	[[nodiscard]] constexpr auto depth() const noexcept { return code_.depth(); }

	operator TreeIndex() const noexcept { return index(); }

	/*!
	 * @brief Get the corresponding index.
	 *
	 * @note Use the octree that generated the node to read the data.
	 *
	 * @return The corresponding data.
	 */
	[[nodiscard]] constexpr TreeIndex index() const noexcept { return index_; }

	[[nodiscard]] constexpr bool valid() const noexcept { return index_.valid(); }

 protected:
	constexpr TreeNode(Code code, TreeIndex index) noexcept : code_(code), index_(index) {}

	[[nodiscard]] constexpr auto pos() const noexcept { return index_.pos; }

	/*!
	 * @brief Get the offset of the node (i.e., the child from the parent's
	 * perspective).
	 *
	 * @return The offset of the node.
	 */
	[[nodiscard]] constexpr auto offset() const noexcept { return index_.offset; }

	[[nodiscard]] constexpr auto offset(unsigned depth) const noexcept
	{
		return code_.offset(depth);
	}

 protected:
	Code      code_;
	TreeIndex index_;
};

}  // namespace ufo

namespace std
{
template <class Code>
struct hash<ufo::TreeNode<Code>> {
	std::size_t operator()(ufo::TreeNode<Code> node) const
	{
		return hash<Code>()(node.code());
	}
};
}  // namespace std

#endif  // UFO_CONTAINER_TREE_NODE_HPP