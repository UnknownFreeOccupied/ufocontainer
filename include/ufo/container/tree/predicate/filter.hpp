/*!
 * UFOTree: An Efficient Probabilistic 3D Treeping Framework That Embraces the Unknown
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

#ifndef UFO_CONTAINER_TREE_PREDICATE_FILTER_HPP
#define UFO_CONTAINER_TREE_PREDICATE_FILTER_HPP

// UFO
#include <ufo/container/tree/node.hpp>
#include <ufo/utility/enum.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <type_traits>

namespace ufo::pred
{
template <class Predicate>
struct Filter {
	static_assert(dependent_false_v<Predicate>, "Predicate not implemented correctly.");
};

enum class FilterType : std::size_t {
	None  = 0u,
	Init  = 1u << 0u,
	Value = 1u << 1u,
	Node  = 1u << 2u,
	Ray   = 1u << 3u,
};

constexpr FilterType operator|(FilterType lhs, FilterType rhs)
{
	return static_cast<FilterType>(to_underlying(lhs) | to_underlying(rhs));
}

constexpr FilterType operator&(FilterType lhs, FilterType rhs)
{
	return static_cast<FilterType>(to_underlying(lhs) & to_underlying(rhs));
}

constexpr FilterType operator^(FilterType lhs, FilterType rhs)
{
	return static_cast<FilterType>(to_underlying(lhs) ^ to_underlying(rhs));
}

namespace detail
{
template <class Predicate, bool B>
struct FilterInit {
};

template <class Predicate>
struct FilterInit<Predicate, false> {
	template <class Tree>
	static constexpr void init(Predicate&, Tree const&)
	{
	}
};

template <class Predicate, bool B>
struct FilterValue {
};

template <class Predicate>
struct FilterValue<Predicate, false> {
	template <class Value>
	[[nodiscard]] static constexpr bool returnableValue(Predicate const&, Value const&)
	{
		return true;
	}
};

template <class Predicate, bool B>
struct FilterNode {
};

template <class Predicate>
struct FilterNode<Predicate, false> {
	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Predicate const&, Tree const&,
	                                               typename Tree::Node const&)
	{
		return true;
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Predicate const&, Tree const&,
	                                                typename Tree::Node const&)
	{
		return true;
	}
};

template <class Predicate, bool B>
struct FilterRay {
};

template <class Predicate>
struct FilterRay<Predicate, false> {
	template <class Tree>
	[[nodiscard]] static constexpr bool returnableRay(Predicate const& p, Tree const& t,
	                                                  typename Tree::Node const& n,
	                                                  typename Tree::Ray const&)
	{
		return Filter<Predicate>::returnable(p, t, n);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversableRay(Predicate const& p, Tree const& t,
	                                                   typename Tree::Node const& n,
	                                                   typename Tree::Ray const&)
	{
		return Filter<Predicate>::traversable(p, t, n);
	}
};
}  // namespace detail

template <class Predicate, FilterType Type = FilterType::None>
struct FilterBase
    : detail::FilterInit<Predicate, FilterType::Init == (FilterType::Init & Type)>
    , detail::FilterValue<Predicate, FilterType::Value == (FilterType::Value & Type)>
    , detail::FilterNode<Predicate, FilterType::Node == (FilterType::Node & Type)>
    , detail::FilterRay<Predicate, FilterType::Ray == (FilterType::Ray & Type)> {
};

//
// Contains predicate
//

namespace detail
{
template <class, class>
struct contains_pred : std::false_type {
};

template <class T>
struct contains_pred<T, T> : std::true_type {
};

template <class, class>
struct contains_always_pred : std::false_type {
};

template <class T>
struct contains_always_pred<T, T> : std::true_type {
};
}  // namespace detail

template <class Pred, class Preds>
using contains_pred = detail::contains_pred<Pred, Preds>;

template <class Pred, class Preds>
constexpr inline bool contains_pred_v = contains_pred<Pred, Preds>::value;

template <class Pred, class Preds>
using contains_always_pred = detail::contains_always_pred<Pred, Preds>;

template <class Pred, class Preds>
constexpr inline bool contains_always_pred_v = contains_always_pred<Pred, Preds>::value;

//
// Is predicate
//

template <class, class = void>
struct is_pred : std::false_type {
};

namespace detail
{
template <class Predicate, class Derived>
struct is_pred {
	using type = decltype(test(std::declval<Derived const*>()));

 private:
	template <FilterType Type>
	static constexpr std::true_type test(FilterBase<Predicate, Type> const*);

	static constexpr std::false_type test(...);
};
}  // namespace detail

template <class Predicate>
struct is_pred<Predicate, std::void_t<detail::is_pred<Predicate, Filter<Predicate>>>>
    : std::true_type {
};

template <class Predicate>
constexpr inline bool is_pred_v = is_pred<Predicate>::value;
}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_FILTER_HPP