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

#ifndef UFO_CONTAINER_OCTREE_CODE_HPP
#define UFO_CONTAINER_OCTREE_CODE_HPP

// UFO
#include <ufo/container/octree/octree_key.hpp>
#include <ufo/utility/morton.hpp>

// STL
#include <cassert>
#include <cstdint>
#include <functional>
#include <limits>
#include <ostream>
#include <utility>

namespace ufo
{
/*!
 * @brief An OctCode is a single value and depth for indexing a specific node in an octree
 * at a specific depth
 *
 * @details Morton codes are used in UFO to increase performance when
 * accessing the tree structures
 *
 */
class OctCode
{
	friend class std::hash<OctCode>;

 public:
	using code_t  = std::uint64_t;
	using key_t   = std::uint32_t;
	using depth_t = code_t;

	constexpr OctCode() = default;

	constexpr explicit OctCode(code_t code, depth_t depth) : code_(code), depth_(depth)
	{
		assert(maxDepth() >= depth);
		assert(0 == (code >> 63));
		assert(maxDepth() == depth ? code == 0
		                           : code == ((code >> (3 * depth)) << (3 * depth)));
	}

	OctCode(key_t x, key_t y, key_t z, depth_t depth)
	    : OctCode(mortonEncode3(x, y, z), depth)
	{
	}

	OctCode(OctKey key) : OctCode(key.x, key.y, key.z, key.depth()) {}

	/*!
	 * @brief Get the corresponding key to code
	 *
	 * @return The corresponding key to code
	 */
	[[nodiscard]] constexpr operator OctKey() const noexcept
	{
		auto [x, y, z] = mortonDecode3(code_);
		return OctKey(x, y, z, depth_);
	}

	friend void swap(OctCode& lhs, OctCode& rhs) noexcept
	{
		std::swap(lhs.code_, rhs.code_);
		std::swap(lhs.depth_, rhs.depth_);
	}

	friend std::ostream& operator<<(std::ostream& os, OctCode code)
	{
		return os << "code: " << code.code_ << " depth: " << code.depth_;
	}

	[[nodiscard]] friend constexpr bool operator==(OctCode lhs, OctCode rhs) noexcept
	{
		return lhs.code_ == rhs.code_ && lhs.depth_ == rhs.depth_;
	}

	[[nodiscard]] friend constexpr bool operator!=(OctCode lhs, OctCode rhs) noexcept
	{
		return !(lhs == rhs);
	}

	[[nodiscard]] friend constexpr bool operator<(OctCode lhs, OctCode rhs) noexcept
	{
		return lhs.code_ < rhs.code_;
	}

	[[nodiscard]] friend constexpr bool operator<=(OctCode lhs, OctCode rhs) noexcept
	{
		return lhs.code_ <= rhs.code_;
	}

	[[nodiscard]] friend constexpr bool operator>(OctCode lhs, OctCode rhs) noexcept
	{
		return lhs.code_ > rhs.code_;
	}

	[[nodiscard]] friend constexpr bool operator>=(OctCode lhs, OctCode rhs) noexcept
	{
		return lhs.code_ >= rhs.code_;
	}

	[[nodiscard]] static constexpr depth_t maxDepth() noexcept
	{
		// +1 because root does not need any bits (since it only has one child)
		return (std::numeric_limits<code_t>::digits / size()) + 1;
	}

	[[nodiscard]] static constexpr std::size_t size() noexcept { return 3; }

	[[nodiscard]] constexpr bool valid() const noexcept { return maxDepth() >= depth_; }

	[[nodiscard]] static constexpr bool equalAtDepth(OctCode lhs, OctCode rhs,
	                                                 depth_t depth) noexcept
	{
		assert(maxDepth() >= depth);
		assert(lhs.valid() && rhs.valid());
		return maxDepth() == depth ||
		       (lhs.code_ >> (3 * depth)) == (rhs.code_ >> (3 * depth));
	}

	[[nodiscard]] static constexpr depth_t depthWhereEqual(OctCode lhs,
	                                                       OctCode rhs) noexcept
	{
		assert(lhs.valid() && rhs.valid());
		auto   depth = std::max(lhs.depth(), rhs.depth());
		code_t code  = (lhs.code_ ^ rhs.code_) >> (3 * depth);
		for (; code; code >>= 3, ++depth) {
		}
		return depth;
	}

	[[nodiscard]] constexpr OctCode toDepth(depth_t depth) const
	{
		assert(maxDepth() >= depth);
		return OctCode(maxDepth() == depth ? static_cast<code_t>(0)
		                                   : (code_ >> (3 * depth)) << (3 * depth),
		               depth);
	}

	constexpr void setDepth(depth_t depth)
	{
		assert(maxDepth() >= depth);
		code_  = maxDepth() == depth ? static_cast<code_t>(0)
		                             : (code_ >> (3 * depth)) << (3 * depth);
		depth_ = depth;
	}

	[[nodiscard]] constexpr key_t key(std::size_t idx) const
	{
		assert(size() > idx);
		return mortonCompact3(code_ >> idx);
	}

	/*!
	 * @brief Get the offset at a specific depth for this code.
	 *
	 * @param depth The depth the index is requested for.
	 * @return The offset at the specified depth.
	 */
	[[nodiscard]] constexpr code_t offset(depth_t depth) const
	{
		assert(maxDepth() >= depth);
		return maxDepth() == depth ? static_cast<code_t>(0) : (code_ >> (3 * depth)) & 0b111;
	}

	[[nodiscard]] constexpr OctCode parent() const { return toDepth(depth_ + 1); }

	/*!
	 * @brief Get the code of a specific child to this code
	 *
	 * @param idx The index of the child
	 * @return OctCode The child code
	 */
	[[nodiscard]] constexpr OctCode child(std::size_t idx) const
	{
		assert(0 < depth_);
		assert(size() > idx);
		depth_t depth = depth_ - 1;
		return OctCode(code_ | (static_cast<code_t>(idx) << (3 * depth)), depth);
	}

	/*!
	 * @brief Get the code of a specific sibling to this code
	 *
	 * @param idx The index of the sibling
	 * @return OctCode The sibling code
	 */
	[[nodiscard]] inline OctCode sibling(std::size_t idx) const
	{
		assert(maxDepth() > depth_);
		assert(size() > idx);
		return OctCode(((code_ >> (3 * depth_)) << (3 * depth_)) |
		                   (static_cast<code_t>(idx) << (3 * depth_)),
		               depth_);
	}

	/*!
	 * @return depth_t The depth this code is specified at
	 */
	[[nodiscard]] constexpr depth_t depth() const noexcept { return depth_; }

 private:
	code_t  code_  = 0;
	depth_t depth_ = maxDepth() + 1;
};
}  // namespace ufo

namespace std
{
template <>
struct hash<ufo::OctCode> {
	std::size_t operator()(ufo::OctCode code) const { return code.code_; }
};
}  // namespace std

#endif  // UFO_CONTAINER_OCTREE_CODE_HPP