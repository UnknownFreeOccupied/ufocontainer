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

#ifndef UFO_CONTAINER_TREE_PREDICATE_SPATIAL_HPP
#define UFO_CONTAINER_TREE_PREDICATE_SPATIAL_HPP

// UFO
#include <ufo/container/tree/predicate/predicate.hpp>
#include <ufo/geometry/contains.hpp>
#include <ufo/geometry/intersects.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <string_view>
#include <type_traits>

namespace ufo::pred
{
//
// Spatial tag
//

enum class SpatialTag {
	CONTAINS,    // Geometry has to be inside node/T
	DISJOINT,    // Geometry and node/T are disjoint
	INTERSECTS,  // Geometry and node/T intersects
	INSIDE       // Node/T has to be inside geometry
};

template <SpatialTag Tag>
[[nodiscard]] constexpr std::string_view enumToString()
{
	using namespace std::literals;
	if constexpr (SpatialTag::CONTAINS == Tag) {
		return "contains"sv;
	} else if constexpr (SpatialTag::DISJOINT == Tag) {
		return "disjoint"sv;
	} else if constexpr (SpatialTag::INTERSECTS == Tag) {
		return "intersects"sv;
	} else if constexpr (SpatialTag::INSIDE == Tag) {
		return "inside"sv;
	} else {
		// Error
	}
}

[[nodiscard]] constexpr std::string_view enumToString(SpatialTag tag)
{
	switch (tag) {
		case SpatialTag::CONTAINS: return enumToString<SpatialTag::CONTAINS>();
		case SpatialTag::DISJOINT: return enumToString<SpatialTag::DISJOINT>();
		case SpatialTag::INTERSECTS: return enumToString<SpatialTag::INTERSECTS>();
		case SpatialTag::INSIDE: return enumToString<SpatialTag::INSIDE>();
	}
}

//
// Spatial
//

template <class Geometry, SpatialTag Tag, bool Negated = false>
struct Spatial {
	constexpr Spatial() {}

	constexpr Spatial(Geometry geometry) : geometry{geometry} {}

	Geometry geometry;
};

template <class Geometry>
using Contains = Spatial<Geometry, SpatialTag::CONTAINS, false>;

template <class Geometry>
using Disjoint = Spatial<Geometry, SpatialTag::DISJOINT, false>;

template <class Geometry>
using Intersects = Spatial<Geometry, SpatialTag::INTERSECTS, false>;

template <class Geometry>
using Inside = Spatial<Geometry, SpatialTag::INSIDE, false>;

//
// Spatial negate
//

template <class Geometry, SpatialTag Tag, bool Negated>
constexpr Spatial<Geometry, Tag, !Negated> operator!(
    Spatial<Geometry, Tag, Negated> const& p)
{
	return {p.geometry};
}

//
// Spatial check
//

template <SpatialTag Tag, class G1, class G2>
[[nodiscard]] constexpr bool spatialCheck(G1 const& g1, G2 const& g2)
{
	if constexpr (SpatialTag::CONTAINS == Tag) {
		return contains(g1, g2);
	} else if constexpr (SpatialTag::DISJOINT == Tag) {
		return !intersects(g1, g2);
	} else if constexpr (SpatialTag::INTERSECTS == Tag) {
		return intersects(g1, g2);
	} else if constexpr (SpatialTag::INSIDE == Tag) {
		return contains(g2, g1);
	}
}

//
// Predicate value check
//

template <class Geometry, SpatialTag Tag, bool Negated, class Value>
[[nodiscard]] constexpr bool valueCheck(Spatial<Geometry, Tag, Negated> const& p,
                                        Value const&                           v)
{
	if constexpr (is_pair_v<Value>) {
		if constexpr (Negated) {
			return !spatialCheck<Tag>(v.first, p.geometry);
		} else {
			return spatialCheck<Tag>(v.first, p.geometry);
		}
	} else {
		if constexpr (Negated) {
			return !spatialCheck<Tag>(v, p.geometry);
		} else {
			return spatialCheck<Tag>(v, p.geometry);
		}
	}
}

template <class Geometry, SpatialTag Tag, bool Negated, class Tree, class Node>
[[nodiscard]] constexpr bool valueCheck(Spatial<Geometry, Tag, Negated> const& p,
                                        Tree const& t, Node n)
{
	if constexpr (Negated) {
		return !spatialCheck<Tag>(t.bounds(n), p.geometry);
	} else {
		return spatialCheck<Tag>(t.bounds(n), p.geometry);
	}
}

//
// Predicate inner check
//

template <class Geometry, SpatialTag Tag, bool Negated, class Tree, class Node>
[[nodiscard]] constexpr bool innerCheck(Spatial<Geometry, Tag, Negated> const& p,
                                        Tree const& t, Node n)
{
	if constexpr (Negated) {
		if constexpr (SpatialTag::CONTAINS == Tag) {
			return true;
		} else if constexpr (SpatialTag::DISJOINT == Tag) {
			return spatialCheck<SpatialTag::INTERSECTS>(t.bounds(n), p.geometry);
		} else {
			return !spatialCheck<SpatialTag::INSIDE>(t.bounds(n), p.geometry);
		}
	} else {
		if constexpr (SpatialTag::CONTAINS == Tag) {
			return spatialCheck<SpatialTag::CONTAINS>(t.bounds(n), p.geometry);
		} else if constexpr (SpatialTag::DISJOINT == Tag) {
			return spatialCheck<SpatialTag::INSIDE>(t.bounds(n), p.geometry);
		} else {
			return spatialCheck<SpatialTag::INTERSECTS>(t.bounds(n), p.geometry);
		}
	}
}

//
// Is spatial predicate
//

namespace detail
{
template <class>
struct is_spatial_pred : std::false_type {
};

template <class Geometry, SpatialTag Tag, bool Negated>
struct is_spatial_pred<Spatial<Geometry, Tag, Negated>> : std::true_type {
};
}  // namespace detail

template <class T>
struct is_spatial_pred : detail::is_spatial_pred<std::decay_t<T>> {
};

// Helper variable template
template <class T>
inline constexpr bool is_spatial_pred_v = is_spatial_pred<T>::value;

//
// Contains spatial predicate
//

namespace detail
{
template <class>
struct contains_spatial_pred : std::false_type {
};

template <class Geometry, SpatialTag Tag, bool Negated>
struct contains_spatial_pred<Spatial<Geometry, Tag, Negated>> : std::true_type {
};

template <class... Ts>
struct contains_spatial_pred<std::tuple<Ts...>>
    : std::disjunction<contains_spatial_pred<Ts>...> {
};

template <class L, class R>
struct contains_spatial_pred<OR<L, R>>
    : std::disjunction<contains_spatial_pred<L>, contains_spatial_pred<R>> {
};

template <class L, class R>
struct contains_spatial_pred<THEN<L, R>>
    : std::disjunction<contains_spatial_pred<L>, contains_spatial_pred<R>> {
};

template <class L, class R>
struct contains_spatial_pred<IFF<L, R>>
    : std::disjunction<contains_spatial_pred<L>, contains_spatial_pred<R>> {
};

}  // namespace detail

template <class T>
using contains_spatial_pred = detail::contains_spatial_pred<std::decay_t<T>>;

template <class T>
inline constexpr bool contains_spatial_pred_v = contains_spatial_pred<T>::value;

//
// Contains always spatial predicate
//

namespace detail
{
template <class>
struct contains_always_spatial_pred : std::false_type {
};

template <class Geometry, SpatialTag Tag, bool Negated>
struct contains_always_spatial_pred<Spatial<Geometry, Tag, Negated>> : std::true_type {
};

template <class... Ts>
struct contains_always_spatial_pred<std::tuple<Ts...>>
    : std::disjunction<contains_always_spatial_pred<Ts>...> {
};

template <class L, class R>
struct contains_always_spatial_pred<OR<L, R>>
    : std::conjunction<contains_always_spatial_pred<L>, contains_always_spatial_pred<R>> {
};

template <class L, class R>
struct contains_always_spatial_pred<THEN<L, R>> : std::false_type {
};

template <class L, class R>
struct contains_always_spatial_pred<IFF<L, R>> : std::false_type {
};
}  // namespace detail

template <class T>
using contains_always_spatial_pred = detail::contains_always_spatial_pred<T>;

template <class T>
inline constexpr bool contains_always_spatial_pred_v =
    contains_always_spatial_pred<T>::value;

}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_SPATIAL_HPP