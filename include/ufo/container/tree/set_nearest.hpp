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

#ifndef UFO_CONTAINER_TREE_SET_NEAREST_HPP
#define UFO_CONTAINER_TREE_SET_NEAREST_HPP

// STL
#include <iterator>
#include <limits>
#include <type_traits>
#include <utility>

namespace ufo
{

template <class Iterator>
struct TreeSetNearest {
	template <class>
	friend class TreeSetOrMapNearestIterator;

	template <class>
	friend class TreeSetOrMapQueryNearestIterator;

	using value_type = typename std::iterator_traits<Iterator>::value_type;
	using Point      = value_type;

	float distance = std::numeric_limits<float>::quiet_NaN();

	TreeSetNearest() = default;

	TreeSetNearest(Iterator it, float distance) : distance(distance), it_(it) {}

	[[nodiscard]] Point const& point() const { return *it_; }

	bool operator==(TreeSetNearest rhs) const noexcept
	{
		return distance == rhs.distance && it_ == rhs.it_;
	}

	bool operator!=(TreeSetNearest rhs) const noexcept
	{
		return distance != rhs.distance || it_ != rhs.it_;
	}

	bool operator<(TreeSetNearest rhs) const noexcept { return distance < rhs.distance; }

	bool operator<=(TreeSetNearest rhs) const noexcept { return distance <= rhs.distance; }

	bool operator>(TreeSetNearest rhs) const noexcept { return distance > rhs.distance; }

	bool operator>=(TreeSetNearest rhs) const noexcept { return distance >= rhs.distance; }

 private:
	Iterator it_;
};

template <std::size_t i, class Iterator>
auto get(TreeSetNearest<Iterator> const& n)
{
	if constexpr (i == 0) {
		return n.point();
	} else if constexpr (i == 1) {
		return n.distance;
	} else {
		// Error
	}
}

template <std::size_t i, class Iterator>
auto& get(TreeSetNearest<Iterator>& n)
{
	if constexpr (i == 0) {
		return n.point();
	} else if constexpr (i == 1) {
		return n.distance;
	} else {
		// Error
	}
}
}  // namespace ufo

// Tuple-like binding protocol definition
namespace std
{
template <class Iterator>
struct tuple_size<ufo::TreeSetNearest<Iterator>>
    : std::integral_constant<std::size_t, 2> {
};

// Define the types of decomposable elements by specializing
// the 'tuple_element' template class (indices are 0-based)
template <class Iterator>
struct tuple_element<0, ufo::TreeSetNearest<Iterator>> {
	using type = typename ufo::TreeSetNearest<Iterator>::Point;
};
template <class Iterator>
struct tuple_element<1, ufo::TreeSetNearest<Iterator>> {
	using type = float;
};
}  // namespace std

#endif  // UFO_CONTAINER_TREE_SET_NEAREST_HPP