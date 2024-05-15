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

#ifndef UFO_CONTAINER_OCTREE_KEY_HPP
#define UFO_CONTAINER_OCTREE_KEY_HPP

// UFO
#include <ufo/utility/morton.hpp>

// STL
#include <cassert>
#include <cstddef>
#include <functional>
#include <ostream>

namespace ufo
{
/*!
 * @brief A key represent an octree index at a specified depth
 *
 */
class OctKey
{
 public:
	using key_t   = std::uint32_t;
	using depth_t = key_t;

	// X component of key
	key_t x = 0;
	// Y component of key
	key_t y = 0;
	// Z component of key
	key_t z = 0;

 private:
	// The depth of the key
	depth_t depth_ = 0;

 public:
	constexpr OctKey() = default;

	constexpr OctKey(key_t x, key_t y, key_t z, depth_t depth = 0)
	    : x(x), y(y), z(z), depth_(depth)
	{
	}

	friend void swap(OctKey& lhs, OctKey& rhs) noexcept
	{
		std::swap(lhs.x, rhs.x);
		std::swap(lhs.y, rhs.y);
		std::swap(lhs.z, rhs.z);
		std::swap(lhs.depth_, rhs.depth_);
	}

	friend std::ostream& operator<<(std::ostream& os, OctKey key)
	{
		os << "x: " << key.x << " y: " << key.y << " z: " << key.z << " d: " << key.depth_;
		return os;
	}

	[[nodiscard]] friend constexpr bool operator==(OctKey lhs, OctKey rhs) noexcept
	{
		return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.depth_ == rhs.depth_;
	}

	[[nodiscard]] friend constexpr bool operator!=(OctKey lhs, OctKey rhs) noexcept
	{
		return !(lhs == rhs);
	}

	[[nodiscard]] friend constexpr bool operator<(OctKey lhs, OctKey rhs) noexcept
	{
		return lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y) ||
		       (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z < rhs.z);
	}

	[[nodiscard]] friend constexpr bool operator<=(OctKey lhs, OctKey rhs) noexcept
	{
		return lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y) ||
		       (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z <= rhs.z);
	}

	[[nodiscard]] friend constexpr bool operator>(OctKey lhs, OctKey rhs) noexcept
	{
		return lhs.x > rhs.x || (lhs.x == rhs.x && lhs.y > rhs.y) ||
		       (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z > rhs.z);
	}

	[[nodiscard]] friend constexpr bool operator>=(OctKey lhs, OctKey rhs) noexcept
	{
		return lhs.x > rhs.x || (lhs.x == rhs.x && lhs.y > rhs.y) ||
		       (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z >= rhs.z);
	}

	[[nodiscard]] static constexpr depth_t maxDepth() noexcept { return 21; }

	[[nodiscard]] static constexpr std::size_t size() noexcept { return 3; }

	[[nodiscard]] friend constexpr OctKey operator+(OctKey lhs, OctKey rhs) noexcept
	{
		lhs.x += rhs.x;
		lhs.y += rhs.y;
		lhs.z += rhs.z;
		lhs.depth_ = std::min(lhs.depth_, rhs.depth_);
		return lhs;
	}

	constexpr OctKey& operator+=(OctKey rhs) noexcept
	{
		assert(depth_ <= rhs.depth_);

		x += rhs.x;
		y += rhs.y;
		z += rhs.z;

		return *this;
	}

	[[nodiscard]] friend constexpr OctKey operator-(OctKey lhs, OctKey rhs) noexcept
	{
		lhs.x -= rhs.x;
		lhs.y -= rhs.y;
		lhs.z -= rhs.z;
		lhs.depth_ = std::min(lhs.depth_, rhs.depth_);
		return lhs;
	}

	constexpr OctKey& operator-=(OctKey rhs) noexcept
	{
		assert(depth_ <= rhs.depth_);

		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;

		return *this;
	}

	[[nodiscard]] constexpr key_t offset() const noexcept { return offset(depth_); }

	[[nodiscard]] constexpr key_t offset(depth_t depth) const noexcept
	{
		return ((x >> depth) & key_t(1)) | (((y >> depth) & key_t(1)) << 1) |
		       (((z >> depth) & key_t(1)) << 2);
	}

	[[nodiscard]] constexpr depth_t depth() const noexcept { return depth_; }

	/*!
	 * @brief Change the depth of the key.
	 *
	 * @note This will change the x, y, z components of the key.
	 */
	constexpr void setDepth(depth_t depth) noexcept
	{
		x            = (x >> depth) << depth;
		y            = (y >> depth) << depth;
		z            = (z >> depth) << depth;
		this->depth_ = depth;
	}

	[[nodiscard]] constexpr key_t operator[](std::size_t idx) const
	{
		assert(3 >= idx);
		return *(&x + idx);
	}

	[[nodiscard]] constexpr key_t& operator[](std::size_t idx)
	{
		assert(3 >= idx);
		return *(&x + idx);
	}

	[[nodiscard]] constexpr key_t step() const noexcept { return step(depth_); }

	[[nodiscard]] static constexpr key_t step(depth_t depth) noexcept
	{
		return key_t(1) << depth;
	}

 private:
	friend class std::hash<OctKey>;
};
}  // namespace ufo

namespace std
{
template <>
struct hash<ufo::OctKey> {
	std::size_t operator()(ufo::OctKey key) const
	{
		return ufo::mortonEncode3(key.x, key.y, key.z);
	}
};
}  // namespace std

#endif  // UFO_CONTAINER_OCTREE_KEY_HPP