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

#ifndef UFO_CONTAINER_TREE_COORD_HPP
#define UFO_CONTAINER_TREE_COORD_HPP

// UFO
#include <ufo/math/detail/vec.hpp>

// STL
#include <cstddef>
#include <type_traits>

namespace ufo
{
template <std::size_t Dim, class T = float>
struct TreeCoord : public Vec<Dim, T> {
	using Point   = Vec<Dim, T>;
	using coord_t = typename Point::value_type;
	using depth_t = unsigned;

	depth_t depth{};

	constexpr TreeCoord() = default;

	constexpr TreeCoord(Point coord) : Point(coord) {}

	constexpr TreeCoord(Point coord, depth_t depth) : Point(coord), depth(depth) {}

	template <class... Args,
	          std::enable_if_t<Point::size() == sizeof...(Args) || 1 == sizeof...(Args),
	                           bool> = true>
	constexpr TreeCoord(Args const&... args) : Point(args...)
	{
	}

	template <class... Args,
	          std::enable_if_t<Point::size() + 1 == sizeof...(Args), bool> = true>
	constexpr TreeCoord(Args const&... args)
	    : TreeCoord(std::integral_constant<std::size_t, Point::size()>{}, args...)
	{
	}

 private:
	template <std::size_t NumTimes, class First, class... Rest>
	constexpr TreeCoord(std::integral_constant<std::size_t, NumTimes>, First const& first,
	                    Rest const&... rest)
	    : TreeCoord(std::integral_constant<std::size_t, NumTimes - 1>{}, rest..., first)
	{
	}

	template <class Depth, class... PointArgs>
	constexpr TreeCoord(std::integral_constant<std::size_t, 0>, Depth const& depth,
	                    PointArgs const&... args)
	    : Point(args...), depth(depth)
	{
	}
};

template <class T = float>
using Coord1 = TreeCoord<1, T>;
template <class T = float>
using Coord2 = TreeCoord<2, T>;
template <class T = float>
using Coord3 = TreeCoord<3, T>;
template <class T = float>
using Coord4 = TreeCoord<4, T>;

template <class T = float>
using BinaryCoord = Coord1<T>;
template <class T = float>
using QuadCoord = Coord2<T>;
template <class T = float>
using OctCoord = Coord3<T>;
template <class T = float>
using HexCoord = Coord4<T>;

using Coord1f      = Coord1<float>;
using Coord1d      = Coord1<double>;
using BinaryCoordf = Coord1f;
using BinaryCoordd = Coord1d;

using Coord2f    = Coord2<float>;
using Coord2d    = Coord2<double>;
using QuadCoordf = Coord2f;
using QuadCoordd = Coord2d;

using Coord3f   = Coord3<float>;
using Coord3d   = Coord3<double>;
using OctCoordf = Coord3f;
using OctCoordd = Coord3d;

using Coord4f   = Coord4<float>;
using Coord4d   = Coord4<double>;
using HexCoordf = Coord4f;
using HexCoordd = Coord4d;
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_COORD_HPP