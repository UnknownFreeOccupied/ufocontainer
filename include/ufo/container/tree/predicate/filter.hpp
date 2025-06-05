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
#include <ufo/utility/type_traits.hpp>

// STL
#include <tuple>
#include <type_traits>

namespace ufo::pred
{
//
// AND (&&)
//

template <class... Preds>
struct And {
	And(Preds&&... preds) : preds(std::forward<Preds>(preds)...) {}

	std::tuple<Preds...> preds;
};

template <class PredLeft, class PredRight,
          std::enable_if_t<!is_specialization_of_v<And, PredLeft> &&
                               !is_specialization_of_v<And, PredRight>,
                           bool> = true>
constexpr And<remove_cvref_t<PredLeft>, remove_cvref_t<PredRight>> operator&&(
    PredLeft&& p1, PredRight&& p2)
{
	return {std::forward<PredLeft>(p1), std::forward<PredRight>(p2)};
}

template <class... PredsLeft, class... PredsRight>
constexpr And<PredsLeft..., PredsRight...> operator&&(And<PredsLeft...>&&  p1,
                                                      And<PredsRight...>&& p2)
{
	return {std::tuple_cat(std::move(p1.preds), std::move(p2.preds))};
}

template <class... PredsLeft, class... PredsRight>
constexpr And<PredsLeft..., PredsRight...> operator&&(And<PredsLeft...> const&  p1,
                                                      And<PredsRight...> const& p2)
{
	return {std::tuple_cat(p1.preds, p2.preds)};
}

template <class... PredsLeft, class PredRight>
constexpr And<PredsLeft..., remove_cvref_t<PredRight>> operator&&(And<PredsLeft...>&& p1,
                                                                  PredRight&&         p2)
{
	return {
	    std::tuple_cat(std::move(p1.preds), std::make_tuple(std::forward<PredRight>(p2)))};
}

template <class... PredsLeft, class PredRight>
constexpr And<PredsLeft..., remove_cvref_t<PredRight>> operator&&(
    And<PredsLeft...> const& p1, PredRight&& p2)
{
	return {std::tuple_cat(p1.preds, std::make_tuple(std::forward<PredRight>(p2)))};
}

template <class PredLeft, class... PredsRight>
constexpr And<remove_cvref_t<PredLeft>, PredsRight...> operator&&(PredLeft&&           p1,
                                                                  And<PredsRight...>&& p2)
{
	return {
	    std::tuple_cat(std::make_tuple(std::forward<PredLeft>(p1)), std::move(p2.preds))};
}

template <class PredLeft, class... PredsRight>
constexpr And<remove_cvref_t<PredLeft>, PredsRight...> operator&&(
    PredLeft&& p1, And<PredsRight...> const& p2)
{
	return {std::tuple_cat(std::make_tuple(std::forward<PredLeft>(p1)), p2.preds)};
}

//
// Or (||)
//

template <class... Preds>
struct Or {
	Or(Preds&&... preds) : preds(std::forward<Preds>(preds)...) {}

	std::tuple<Preds...> preds;
};

template <class PredLeft, class PredRight,
          std::enable_if_t<!is_specialization_of_v<Or, PredLeft> &&
                               !is_specialization_of_v<Or, PredRight>,
                           bool> = true>
constexpr Or<remove_cvref_t<PredLeft>, remove_cvref_t<PredRight>> operator||(
    PredLeft&& p1, PredRight&& p2)
{
	return {std::forward<PredLeft>(p1), std::forward<PredRight>(p2)};
}

template <class... PredsLeft, class... PredsRight>
constexpr Or<PredsLeft..., PredsRight...> operator||(Or<PredsLeft...>&&  p1,
                                                     Or<PredsRight...>&& p2)
{
	return {std::tuple_cat(std::move(p1.preds), std::move(p2.preds))};
}

template <class... PredsLeft, class... PredsRight>
constexpr Or<PredsLeft..., PredsRight...> operator||(Or<PredsLeft...> const&  p1,
                                                     Or<PredsRight...> const& p2)
{
	return {std::tuple_cat(p1.preds, p2.preds)};
}

template <class... PredsLeft, class PredRight>
constexpr Or<PredsLeft..., remove_cvref_t<PredRight>> operator||(Or<PredsLeft...>&& p1,
                                                                 PredRight&&        p2)
{
	return {
	    std::tuple_cat(std::move(p1.preds), std::make_tuple(std::forward<PredRight>(p2)))};
}

template <class... PredsLeft, class PredRight>
constexpr Or<PredsLeft..., remove_cvref_t<PredRight>> operator||(
    Or<PredsLeft...> const& p1, PredRight&& p2)
{
	return {std::tuple_cat(p1.preds, std::make_tuple(std::forward<PredRight>(p2)))};
}

template <class PredLeft, class... PredsRight>
constexpr Or<remove_cvref_t<PredLeft>, PredsRight...> operator||(PredLeft&&          p1,
                                                                 Or<PredsRight...>&& p2)
{
	return {
	    std::tuple_cat(std::make_tuple(std::forward<PredLeft>(p1)), std::move(p2.preds))};
}

template <class PredLeft, class... PredsRight>
constexpr Or<remove_cvref_t<PredLeft>, PredsRight...> operator||(
    PredLeft&& p1, Or<PredsRight...> const& p2)
{
	return {std::tuple_cat(std::make_tuple(std::forward<PredLeft>(p1)), p2.preds)};
}

//
// Then
//

template <class PredPre, class PredPost>
struct Then {
	Then(PredPre const& pre, PredPost const& post) : pre(pre), post(post) {}

	PredPre  pre;
	PredPost post;
};

//
// If and only if (Iff)
//

template <class PredLeft, class PredRight>
struct Iff {
	Iff(PredLeft const& left, PredRight const& right) : left(left), right(right) {}

	PredLeft  left;
	PredRight right;
};

//
// True
//

struct True {
};

//
// False
//

struct False {
};

//
// Filter
//

template <class Derived>
struct FilterBase {
	using Pred = typename Derived::Pred;

	template <class Tree>
	static constexpr void init(Pred&, Tree const&)
	{
	}

	template <class Value>
	[[nodiscard]] static constexpr bool returnable(Pred const&, Value const&)
	{
		return true;
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const&, Tree const&,
	                                               typename Tree::Node const&)
	{
		return true;
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               typename Tree::Node const& n,
	                                               typename Tree::Ray const&)
	{
		return Derived::returnable(p, t, n);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const&, Tree const&,
	                                                typename Tree::Node const&)
	{
		return true;
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                typename Tree::Node const& n,
	                                                typename Tree::Ray const&)
	{
		return Derived::traversable(p, t, n);
	}
};

template <class Pred>
struct Filter;
// {
// 	static_assert(dependent_false_v<Pred>, "Predicate not implemented correctly.");
// };

template <class... Preds>
struct Filter<And<Preds...>> {
	using Pred = And<Preds...>;

	template <class Tree>
	static constexpr void init(Pred& p, Tree const& t)
	{
		std::apply(
		    [&t](auto&... p) { ((Filter<remove_cvref_t<decltype(p)>>::init(p, t)), ...); },
		    p.preds);
	}

	template <class Value>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Value const& v)
	{
		return std::apply(
		    [&v](auto const&... p) {
			    return ((Filter<remove_cvref_t<decltype(p)>>::returnable(p, v)) && ...);
		    },
		    p.preds);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               typename Tree::Node const& n)
	{
		return std::apply(
		    [&t, &n](auto const&... p) {
			    return ((Filter<remove_cvref_t<decltype(p)>>::returnable(p, t, n)) && ...);
		    },
		    p.preds);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                typename Tree::Node const& n)
	{
		return std::apply(
		    [&t, &n](auto const&... p) {
			    return ((Filter<remove_cvref_t<decltype(p)>>::traversable(p, t, n)) && ...);
		    },
		    p.preds);
	}
};

template <class... Preds>
struct Filter<Or<Preds...>> {
	using Pred = Or<Preds...>;

	template <class Tree>
	static constexpr void init(Pred& p, Tree const& t)
	{
		std::apply(
		    [&t](auto&... p) { ((Filter<remove_cvref_t<decltype(p)>>::init(p, t)), ...); },
		    p.preds);
	}

	template <class Value>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Value const& v)
	{
		return std::apply(
		    [&v](auto const&... p) {
			    return ((Filter<remove_cvref_t<decltype(p)>>::returnable(p, v)) || ...);
		    },
		    p.preds);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               typename Tree::Node const& n)
	{
		return std::apply(
		    [&t, &n](auto const&... p) {
			    return ((Filter<remove_cvref_t<decltype(p)>>::returnable(p, t, n)) || ...);
		    },
		    p.preds);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                typename Tree::Node const& n)
	{
		return std::apply(
		    [&t, &n](auto const&... p) {
			    return ((Filter<remove_cvref_t<decltype(p)>>::traversable(p, t, n)) || ...);
		    },
		    p.preds);
	}
};

template <class PredPre, class PredPost>
struct Filter<Then<PredPre, PredPost>> {
	using Pred = Then<PredPre, PredPost>;

	template <class Tree>
	static constexpr void init(Pred& p, Tree const& t)
	{
		Filter<PredPre>::init(p.pre, t);
		Filter<PredPost>::init(p.post, t);
	}

	template <class Value>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Value const& v)
	{
		return !Filter<PredPre>::returnable(p.left, v) ||
		       Filter<PredPost>::returnable(p.right, v);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               typename Tree::Node const& n)
	{
		return !Filter<PredPre>::returnable(p.left, t, n) ||
		       Filter<PredPost>::returnable(p.right, t, n);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                typename Tree::Node const& n)
	{
		return !Filter<PredPre>::traversable(p.left, t, n) ||
		       Filter<PredPost>::traversable(p.right, t, n);
	}
};

template <class PredLeft, class PredRight>
struct Filter<Iff<PredLeft, PredRight>> {
	using Pred = Iff<PredLeft, PredRight>;

	template <class Tree>
	static constexpr void init(Pred& p, Tree const& t)
	{
		Filter<PredLeft>::init(p.left, t);
		Filter<PredRight>::init(p.right, t);
	}

	template <class Value>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Value const& v)
	{
		return Filter<PredLeft>::returnable(p.left, v) ==
		       Filter<PredRight>::returnable(p.right, v);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               typename Tree::Node const& n)
	{
		return Filter<PredLeft>::returnable(p.left, t, n) ==
		       Filter<PredRight>::returnable(p.right, t, n);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                typename Tree::Node const& n)
	{
		return Filter<PredLeft>::traversable(p.left, t, n) ==
		       Filter<PredRight>::traversable(p.right, t, n);
	}
};

template <>
struct Filter<True> {
	using Pred = True;

	template <class Tree>
	static constexpr void init(Pred&, Tree const&)
	{
	}

	template <class Value>
	[[nodiscard]] static constexpr bool returnable(Pred const&, Value const&)
	{
		return true;
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const&, Tree const&,
	                                               typename Tree::Node const&)
	{
		return true;
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const&, Tree const&,
	                                                typename Tree::Node const&)
	{
		return true;
	}
};

template <>
struct Filter<False> {
	using Pred = False;

	template <class Tree>
	static constexpr void init(Pred&, Tree const&)
	{
	}

	template <class Value>
	[[nodiscard]] static constexpr bool returnable(Pred const&, Value const&)
	{
		return false;
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const&, Tree const&,
	                                               typename Tree::Node const&)
	{
		return false;
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const&, Tree const&,
	                                                typename Tree::Node const&)
	{
		return false;
	}
};

template <>
struct Filter<bool> {
	using Pred = bool;

	template <class Tree>
	static constexpr void init(Pred&, Tree const&)
	{
	}

	template <class Value>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Value const&)
	{
		return p;
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const&,
	                                               typename Tree::Node const&)
	{
		return p;
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const&,
	                                                typename Tree::Node const&)
	{
		return p;
	}
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

template <class T, class... Ts>
struct contains_pred<T, And<Ts...>> : std::disjunction<contains_pred<T, Ts>...> {
};

template <class T, class... Ts>
struct contains_pred<T, Or<Ts...>> : std::disjunction<contains_pred<T, Ts>...> {
};

template <class T, class L, class R>
struct contains_pred<T, Then<L, R>>
    : std::disjunction<contains_pred<T, L>, contains_pred<T, R>> {
};

template <class T, class L, class R>
struct contains_pred<T, Iff<L, R>>
    : std::disjunction<contains_pred<T, L>, contains_pred<T, R>> {
};
}  // namespace detail

template <class Pred, class Preds>
using contains_pred = detail::contains_pred<Pred, Preds>;

template <class Pred, class Preds>
constexpr inline bool contains_pred_v = contains_pred<Pred, Preds>::value;

//
// Contains always predicate
//

namespace detail
{
template <class, class>
struct contains_always_pred : std::false_type {
};

template <class T>
struct contains_always_pred<T, T> : std::true_type {
};

template <class T, class... Ts>
struct contains_always_pred<T, And<Ts...>>
    : std::disjunction<contains_always_pred<T, Ts>...> {
};

template <class T, class... Ts>
struct contains_always_pred<T, Or<Ts...>>
    : std::conjunction<contains_always_pred<T, Ts>...> {
};

template <class T, class L, class R>
struct contains_always_pred<T, Then<L, R>> : std::false_type {
};

template <class T, class L, class R>
struct contains_always_pred<T, Iff<L, R>> : std::false_type {
};
}  // namespace detail

template <class Pred, class Preds>
using contains_always_pred = detail::contains_always_pred<Pred, Preds>;

template <class Pred, class Preds>
constexpr inline bool contains_always_pred_v = contains_always_pred<Pred, Preds>::value;

//
// Is predicate
//

template <class, class, class = void>
struct is_pred : std::false_type {
};

template <class Pred, class Tree>
struct is_pred<
    Pred, Tree,
    std::void_t<
        decltype(Filter<Pred>::init(std::declval<Pred&>(), std::declval<Tree>())),
        decltype(Filter<Pred>::returnable(std::declval<Pred>(), std::declval<Tree>(),
                                          std::declval<typename Tree::Node>())),
        decltype(Filter<Pred>::traversable(std::declval<Pred>(), std::declval<Tree>(),
                                           std::declval<typename Tree::Node>()))>>
    : std::true_type {
};

template <class Pred, class Tree>
constexpr inline bool is_pred_v = is_pred<Pred, Tree>::value;

//
// Is value predicate
//

template <class, class, class, class = void>
struct is_value_pred : std::false_type {
};

template <class Pred, class Tree, class Value>
struct is_value_pred<
    Pred, Tree, Value,
    std::void_t<
        decltype(Filter<Pred>::init(std::declval<Pred&>(), std::declval<Tree>())),
        decltype(Filter<Pred>::returnable(std::declval<Pred>(), std::declval<Value>())),
        decltype(Filter<Pred>::traversable(std::declval<Pred>(), std::declval<Tree>(),
                                           std::declval<typename Tree::Node>()))>>
    : std::true_type {
};

template <class Pred, class Tree, class Value>
constexpr inline bool is_value_pred_v = is_value_pred<Pred, Tree, Value>::value;
}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_FILTER_HPP