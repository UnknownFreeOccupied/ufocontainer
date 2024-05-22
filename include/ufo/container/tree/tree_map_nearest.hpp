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
#include <utility>

namespace ufo
{
template <class T>
struct TreeMapNearest {
	//
	// Tags
	//

	using value_type = T;
	using reference  = T&;
	using pointer    = T*;

	pointer value_ptr;
	float   distance;

	TreeMapNearest()                      = default;
	TreeMapNearest(TreeMapNearest const&) = default;
	TreeMapNearest(TreeMapNearest&&)      = default;

	TreeMapNearest(T& value, float distance) : value_ptr(&value), distance(distance) {}

	TreeMapNearest(T* value_ptr, float distance) : value_ptr(value_ptr), distance(distance)
	{
	}

	TreeMapNearest& operator=(TreeMapNearest const&) = default;
	TreeMapNearest& operator=(TreeMapNearest&&)      = default;

	void swap(TreeMapNearest& other) noexcept
	{
		std::swap(value_ptr, other.value_ptr);
		std::swap(distance, other.distance);
	}

	operator reference() const { return *value_ptr; }

	operator pointer() const { return value_ptr; }

	[[nodiscard]] reference operator*() const { return *value_ptr; }

	[[nodiscard]] pointer operator->() const { return value_ptr; }

	bool operator==(value_type rhs) const noexcept
	{
		return distance == rhs.distance && value_ptr == rhs.value_ptr;
	}

	bool operator!=(value_type rhs) const noexcept
	{
		return distance != rhs.distance || value_ptr != rhs.value_ptr;
	}

	bool operator<(value_type rhs) const noexcept { return distance < rhs.distance; }

	bool operator<=(value_type rhs) const noexcept { return distance <= rhs.distance; }

	bool operator>(value_type rhs) const noexcept { return distance > rhs.distance; }

	bool operator>=(value_type rhs) const noexcept { return distance >= rhs.distance; }
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_MAP_NEAREST_HPP