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

#ifndef UFO_CONTAINER_TREE_KEY_HPP
#define UFO_CONTAINER_TREE_KEY_HPP

// UFO
#include <ufo/math/vec.hpp>
#include <ufo/morton/morton.hpp>

// STL
#include <cstddef>
#include <cstdint>
#include <functional>
#include <ostream>

namespace ufo
{
template <std::size_t Dim>
class TreeKey
{
 public:
	using key_t     = std::uint32_t;
	using depth_t   = key_t;
	using size_type = std::size_t;

	Vec<Dim, key_t> key;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	constexpr TreeKey() noexcept               = default;
	constexpr TreeKey(TreeKey const&) noexcept = default;

	constexpr TreeKey(Vec<Dim, key_t> key, depth_t depth = 0) : key(key), depth_(depth) {}

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	constexpr TreeKey& operator=(TreeKey const&) noexcept = default;

	/**************************************************************************************
	|                                                                                     |
	|                                   Element access                                    |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr key_t& operator[](size_type pos) noexcept
	{
		assert(size() > pos);
		return key[pos];
	}

	[[nodiscard]] constexpr key_t const& operator[](size_type pos) const noexcept
	{
		assert(size() > pos);
		return key[pos];
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Capacity                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] static constexpr depth_t maxDepth() noexcept
	{
		return static_cast<depth_t>(64 / size());
	}

	[[nodiscard]] static constexpr size_type size() noexcept { return Dim; }

	/**************************************************************************************
	|                                                                                     |
	|                                     Operations                                      |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr depth_t depth() const noexcept { return depth_; }

	/*!
	 * @brief Change the depth of the key.
	 *
	 * @note This will change the x, y, z components of the key.
	 */
	constexpr void setDepth(depth_t depth) noexcept
	{
		assert(maxDepth() >= depth);

		if (depth_ > depth) {
			key <<= depth_ - depth;
		} else {
			key >>= depth - depth_;
		}
		this->depth_ = depth;
	}

	[[nodiscard]] constexpr key_t offset(depth_t depth) const noexcept
	{
		assert(maxDepth() >= depth);
		assert(depth_ <= depth);

		auto  v   = (key >> (depth - depth_)) & key_t(1);
		key_t ret = v[0];
		for (std::size_t i = 1; Dim > i; ++i) {
			ret |= v[i] << i;
		}
		return ret;
	}

	void swap(TreeKey& other) noexcept
	{
		using std::swap;
		swap(key, other.key);
		swap(depth_, other.depth_);
	}

 private:
	depth_t depth_{};
};

using BinaryKey = TreeKey<1>;
using QuadKey   = TreeKey<2>;
using OctKey    = TreeKey<3>;
using HexKey    = TreeKey<4>;

template <std::size_t Dim>
std::ostream& operator<<(std::ostream& out, TreeKey<Dim> const& key)
{
	out << "x: " << key.x;
	if constexpr (1 < Dim) {
		out << " y: " << key.y;
	}
	if constexpr (2 < Dim) {
		out << " z: " << key.z;
	}
	if constexpr (3 < Dim) {
		out << " w: " << key.w;
	}
	out << " d: " << key.depth();
	return out;
}

template <std::size_t Dim>
void swap(TreeKey<Dim>& lhs, TreeKey<Dim>& rhs) noexcept
{
	lhs.swap(rhs);
}

/**************************************************************************************
|                                                                                     |
|                                       Compare                                       |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim>
[[nodiscard]] constexpr bool operator==(TreeKey<Dim> const& lhs,
                                        TreeKey<Dim> const& rhs) noexcept
{
	return all(lhs.key == rhs.key) && lhs.depth() == rhs.depth();
}

template <std::size_t Dim>
[[nodiscard]] constexpr bool operator!=(TreeKey<Dim> const& lhs,
                                        TreeKey<Dim> const& rhs) noexcept
{
	return !(lhs == rhs);
}

template <std::size_t Dim>
[[nodiscard]] constexpr bool operator<(TreeKey<Dim> const& lhs,
                                       TreeKey<Dim> const& rhs) noexcept
{
	for (std::size_t i{}; Dim > i; ++i) {
		if (lhs[i] < rhs[i]) {
			return true;
		} else if (lhs[i] > rhs[i]) {
			return false;
		}
	}
	return false;
}

template <std::size_t Dim>
[[nodiscard]] constexpr bool operator<=(TreeKey<Dim> const& lhs,
                                        TreeKey<Dim> const& rhs) noexcept
{
	for (std::size_t i{}; Dim > i; ++i) {
		if (lhs[i] < rhs[i]) {
			return true;
		} else if (lhs[i] > rhs[i]) {
			return false;
		}
	}
	return true;
}

template <std::size_t Dim>
[[nodiscard]] constexpr bool operator>(TreeKey<Dim> const& lhs,
                                       TreeKey<Dim> const& rhs) noexcept
{
	for (std::size_t i{}; Dim > i; ++i) {
		if (lhs[i] > rhs[i]) {
			return true;
		} else if (lhs[i] < rhs[i]) {
			return false;
		}
	}
	return false;
}

template <std::size_t Dim>
[[nodiscard]] constexpr bool operator>=(TreeKey<Dim> const& lhs,
                                        TreeKey<Dim> const& rhs) noexcept
{
	for (std::size_t i{}; Dim > i; ++i) {
		if (lhs[i] > rhs[i]) {
			return true;
		} else if (lhs[i] < rhs[i]) {
			return false;
		}
	}
	return true;
}
}  // namespace ufo

namespace std
{
template <std::size_t Dim>
struct hash<ufo::TreeKey<Dim>> {
	std::size_t operator()(ufo::TreeKey<Dim> const& key) const
	{
		return static_cast<std::size_t>(ufo::Morton<Dim>::encode(key.key));
	}
};
}  // namespace std

#endif  // UFO_CONTAINER_TREE_KEY_HPP