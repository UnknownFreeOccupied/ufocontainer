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

// STL
#include <iterator>
#include <limits>
#include <type_traits>
#include <utility>

namespace ufo
{

template <class Iterator>
struct TreeMapNearest {
	using value_type  = typename std::iterator_traits<Iterator>::value_type;
	using Point       = typename value_type::first_type;
	using mapped_type = typename value_type::second_type;

	float distance = std::numeric_limits<float>::quiet_NaN();

	TreeMapNearest() = default;

	TreeMapNearest(Iterator it, float distance) : distance(distance), it_(it) {}

	[[nodiscard]] value_type& value() const { return *it_; }

	[[nodiscard]] Point const& point() const { return it_->first; }

	[[nodiscard]] mapped_type const& mapped() const { return it_->second; }

	bool operator==(TreeMapNearest rhs) const noexcept
	{
		return distance == rhs.distance && it_ == rhs.it_;
	}

	bool operator!=(TreeMapNearest rhs) const noexcept
	{
		return distance != rhs.distance || it_ != rhs.it_;
	}

	bool operator<(TreeMapNearest rhs) const noexcept { return distance < rhs.distance; }

	bool operator<=(TreeMapNearest rhs) const noexcept { return distance <= rhs.distance; }

	bool operator>(TreeMapNearest rhs) const noexcept { return distance > rhs.distance; }

	bool operator>=(TreeMapNearest rhs) const noexcept { return distance >= rhs.distance; }

	// TODO: Make private
 public:
	Iterator it_;
};

template <std::size_t i, class Iterator>
auto const& get(TreeMapNearest<Iterator> const& n)
{
	if constexpr (i == 0) {
		return n.point();
	} else if constexpr (i == 1) {
		return n.mapped();
	} else if constexpr (i == 2) {
		return n.distance;
	} else {
		// Error
	}
}

template <std::size_t i, class Iterator>
auto& get(TreeMapNearest<Iterator>& n)
{
	if constexpr (i == 0) {
		return n.point();
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
template <class Iterator>
struct tuple_size<ufo::TreeMapNearest<Iterator>>
    : std::integral_constant<std::size_t, 3> {
};

// Define the types of decomposable elements by specializing
// the 'tuple_element' template class (indices are 0-based)
template <class Iterator>
struct tuple_element<0, ufo::TreeMapNearest<Iterator>> {
	using type = typename ufo::TreeMapNearest<Iterator>::Point;
};
template <class Iterator>
struct tuple_element<1, ufo::TreeMapNearest<Iterator>> {
	using type = typename ufo::TreeMapNearest<Iterator>::mapped_type;
};
template <class Iterator>
struct tuple_element<2, ufo::TreeMapNearest<Iterator>> {
	using type = float;
};
}  // namespace std

#endif  // UFO_CONTAINER_TREE_MAP_NEAREST_HPP