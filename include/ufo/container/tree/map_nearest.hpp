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

#ifndef UFO_CONTAINER_TREE_MAP_NEAREST_HPP
#define UFO_CONTAINER_TREE_MAP_NEAREST_HPP

// UFO
#include <ufo/container/tree/set_or_map_nearest.hpp>

// STL
#include <type_traits>
#include <utility>

namespace ufo
{

template <class Point, class T>
struct TreeSetOrMapNearest<std::pair<Point const, T>> {
	using value_type  = std::pair<Point const, T>;
	using mapped_type = T;

	float distance;

	TreeSetOrMapNearest() = default;

	TreeSetOrMapNearest(value_type& value, float distance)
	    : distance(distance), value_(&value)
	{
	}

	[[nodiscard]] value_type& value() const { return *value_; }

	[[nodiscard]] Point const& point() const { return value_->first; }

	[[nodiscard]] mapped_type& mapped() const { return value_->second; }

	bool operator==(TreeSetOrMapNearest rhs) const noexcept
	{
		return distance == rhs.distance && value_ == rhs.value_;
	}

	bool operator!=(TreeSetOrMapNearest rhs) const noexcept
	{
		return distance != rhs.distance || value_ != rhs.value_;
	}

	bool operator<(TreeSetOrMapNearest rhs) const noexcept
	{
		return distance < rhs.distance;
	}

	bool operator<=(TreeSetOrMapNearest rhs) const noexcept
	{
		return distance <= rhs.distance;
	}

	bool operator>(TreeSetOrMapNearest rhs) const noexcept
	{
		return distance > rhs.distance;
	}

	bool operator>=(TreeSetOrMapNearest rhs) const noexcept
	{
		return distance >= rhs.distance;
	}

 private:
	value_type* value_;
};

template <class Point, class T>
struct TreeSetOrMapNearest<std::pair<Point const, T> const> {
	using value_type  = std::pair<Point const, T> const;
	using mapped_type = T const;

	float distance;

	TreeSetOrMapNearest() = default;

	TreeSetOrMapNearest(value_type& value, float distance)
	    : distance(distance), value_(&value)
	{
	}

	[[nodiscard]] value_type& value() const { return *value_; }

	[[nodiscard]] Point const& point() const { return value_->first; }

	[[nodiscard]] mapped_type& mapped() const { return value_->second; }

	bool operator==(TreeSetOrMapNearest rhs) const noexcept
	{
		return distance == rhs.distance && value_ == rhs.value_;
	}

	bool operator!=(TreeSetOrMapNearest rhs) const noexcept
	{
		return distance != rhs.distance || value_ != rhs.value_;
	}

	bool operator<(TreeSetOrMapNearest rhs) const noexcept
	{
		return distance < rhs.distance;
	}

	bool operator<=(TreeSetOrMapNearest rhs) const noexcept
	{
		return distance <= rhs.distance;
	}

	bool operator>(TreeSetOrMapNearest rhs) const noexcept
	{
		return distance > rhs.distance;
	}

	bool operator>=(TreeSetOrMapNearest rhs) const noexcept
	{
		return distance >= rhs.distance;
	}

 private:
	value_type* value_;
};

template <std::size_t i, class Point, class T>
auto const& get(TreeSetOrMapNearest<std::pair<Point const, T>> const& n)
{
	if constexpr (i == 0) {
		return n.key();
	} else if constexpr (i == 1) {
		return n.mapped();
	} else if constexpr (i == 2) {
		return n.distance;
	} else {
		// Error
	}
}

template <std::size_t i, class Point, class T>
auto& get(TreeSetOrMapNearest<std::pair<Point const, T>>& n)
{
	if constexpr (i == 0) {
		return n.key();
	} else if constexpr (i == 1) {
		return n.mapped();
	} else if constexpr (i == 2) {
		return n.distance;
	} else {
		// Error
	}
}

template <std::size_t i, class Point, class T>
auto const& get(TreeSetOrMapNearest<std::pair<Point const, T> const> const& n)
{
	if constexpr (i == 0) {
		return n.key();
	} else if constexpr (i == 1) {
		return n.mapped();
	} else if constexpr (i == 2) {
		return n.distance;
	} else {
		// Error
	}
}

template <std::size_t i, class Point, class T>
auto& get(TreeSetOrMapNearest<std::pair<Point const, T> const>& n)
{
	if constexpr (i == 0) {
		return n.key();
	} else if constexpr (i == 1) {
		return n.mapped();
	} else if constexpr (i == 2) {
		return n.distance;
	} else {
		// Error
	}
}
}  // namespace ufo

// Tuple-like binding protocol definition
namespace std
{
template <class Point, class T>
struct tuple_size<ufo::TreeSetOrMapNearest<std::pair<Point const, T>>>
    : std::integral_constant<std::size_t, 3> {
};

// Define the number of decomposable (accessible) elements
// by specializing the 'tuple_size' template class
template <class Point, class T>
struct tuple_size<ufo::TreeSetOrMapNearest<std::pair<Point const, T> const>>
    : std::integral_constant<std::size_t, 3> {
};

// Define the types of decomposable elements by specializing
// the 'tuple_element' template class (indices are 0-based)
template <class Point, class T>
struct tuple_element<0, ufo::TreeSetOrMapNearest<std::pair<Point const, T>>> {
	using type = Point const;
};
template <class Point, class T>
struct tuple_element<1, ufo::TreeSetOrMapNearest<std::pair<Point const, T>>> {
	using type = T;
};
template <class Point, class T>
struct tuple_element<2, ufo::TreeSetOrMapNearest<std::pair<Point const, T>>> {
	using type = float;
};

template <class Point, class T>
struct tuple_element<0, ufo::TreeSetOrMapNearest<std::pair<Point const, T> const>> {
	using type = Point const;
};
template <class Point, class T>
struct tuple_element<1, ufo::TreeSetOrMapNearest<std::pair<Point const, T> const>> {
	using type = T;
};
template <class Point, class T>
struct tuple_element<2, ufo::TreeSetOrMapNearest<std::pair<Point const, T> const>> {
	using type = float;
};
}  // namespace std

#endif  // UFO_CONTAINER_TREE_MAP_NEAREST_HPP