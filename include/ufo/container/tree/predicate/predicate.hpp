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

#ifndef UFO_CONTAINER_TREE_PREDICATE_PREDICATE_HPP
#define UFO_CONTAINER_TREE_PREDICATE_PREDICATE_HPP

// UFO
#include <ufo/container/tree/index.hpp>

// STL
#include <tuple>
#include <type_traits>

namespace ufo::pred
{
//
// Predicate types
//

enum class PredicateType { VALUE = 1, INNER, VALUE_AND_INNER };

//
// AND (&&)
//

template <class Pred1, class Pred2>
constexpr std::tuple<Pred1, Pred2> operator&&(Pred1&& p1, Pred2&& p2)
{
	return {std::forward<Pred1>(p1), std::forward<Pred2>(p2)};
}

template <class... Preds1, class... Preds2>
constexpr std::tuple<Preds1..., Preds2...> operator&&(std::tuple<Preds1...> const& t1,
                                                      std::tuple<Preds2...> const& t2)
{
	return std::tuple_cat(t1, t2);
}

template <class... Preds, class Pred>
constexpr std::tuple<Preds..., Pred> operator&&(std::tuple<Preds...> const& t, Pred&& p)
{
	return std::tuple_cat(t, std::make_tuple(std::forward<Pred>(p)));
}

template <class Pred, class... Preds>
constexpr std::tuple<Pred, Preds...> operator&&(Pred&& p, std::tuple<Preds...> const& t)
{
	return std::tuple_cat(std::make_tuple(std::forward<Pred>(p)), t);
}

//
// OR (||)
//

template <class PredLeft, class PredRight>
struct OR {
	OR(PredLeft const& left, PredRight const& right) : left(left), right(right) {}

	PredLeft  left;
	PredRight right;
};

template <class PredLeft, class PredRight>
constexpr OR<PredLeft, PredRight> operator||(PredLeft&& p1, PredRight&& p2)
{
	return {std::forward<PredLeft>(p1), std::forward<PredRight>(p2)};
}

//
// THEN
//

template <class PredPre, class PredPost>
struct THEN {
	THEN(PredPre const& pre, PredPost const& post) : pre(pre), post(post) {}

	PredPre  pre;
	PredPost post;
};

//
// If and only if (IFF)
//

template <class PredLeft, class PredRight>
struct IFF {
	IFF(PredLeft const& left, PredRight const& right) : left(left), right(right) {}

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
// Static assert check
//

template <bool Check, class... Ts>
struct static_assert_check : std::bool_constant<Check> {
};

// Helper variable template
template <bool Check, class... Ts>
inline constexpr bool static_assert_check_v = static_assert_check<Check, Ts...>::value;

//
// Predicate init
//

template <class Pred>
struct Init {
	template <class Tree>
	static constexpr void apply(Pred&, Tree const&) {};
};

template <class... Preds>
struct Init<std::tuple<Preds...>> {
	template <class Tree>
	static constexpr void apply(std::tuple<Preds...>& p, Tree const& t)
	{
		std::apply(
		    [&t](auto&... p) { ((Init<std::decay_t<decltype(p)>>::apply(p, t)), ...); }, p);
	}
};

template <class PredLeft, class PredRight>
struct Init<OR<PredLeft, PredRight>> {
	template <class Tree>
	static constexpr void apply(OR<PredLeft, PredRight>& p, Tree const& t)
	{
		Init<PredLeft>::apply(p.left, t);
		Init<PredRight>::apply(p.right, t);
	}
};

template <class PredPre, class PredPost>
struct Init<THEN<PredPre, PredPost>> {
	template <class Tree>
	static constexpr void apply(THEN<PredPre, PredPost>& p, Tree const& t)
	{
		Init<PredPre>::apply(p.pre, t);
		Init<PredPost>::apply(p.post, t);
	}
};

template <class PredLeft, class PredRight>
struct Init<IFF<PredLeft, PredRight>> {
	template <class Tree>
	static constexpr void apply(IFF<PredLeft, PredRight>& p, Tree const& t)
	{
		Init<PredLeft>::apply(p.left, t);
		Init<PredRight>::apply(p.right, t);
	}
};

//
// Predicate value check
//

template <class Pred>
struct ValueCheck {
	static_assert(static_assert_check_v<false, Pred>,
	              "Not implemented for this predicate.");
};

template <class... Preds>
struct ValueCheck<std::tuple<Preds...>> {
	template <class Value>
	[[nodiscard]] static constexpr bool apply(std::tuple<Preds...> const& p, Value const& v)
	{
		return std::apply(
		    [&v](auto const&... p) {
			    return ((ValueCheck<std::decay_t<decltype(p)>>::apply(p, v)) && ...);
		    },
		    p);
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool apply(std::tuple<Preds...> const& p, Tree const& t,
	                                          Node const& n)
	{
		return std::apply(
		    [&t, &n](auto const&... p) {
			    return ((ValueCheck<std::decay_t<decltype(p)>>::apply(p, t, n)) && ...);
		    },
		    p);
	}
};

template <class PredLeft, class PredRight>
struct ValueCheck<OR<PredLeft, PredRight>> {
	template <class Value>
	[[nodiscard]] static constexpr bool apply(OR<PredLeft, PredRight> const& p,
	                                          Value const&                   v)
	{
		return ValueCheck<PredLeft>::apply(p.left, v) ||
		       ValueCheck<PredRight>::apply(p.right, v);
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool apply(OR<PredLeft, PredRight> const& p,
	                                          Tree const& t, Node const& n)
	{
		return ValueCheck<PredLeft>::apply(p.left, t, n) ||
		       ValueCheck<PredRight>::apply(p.right, t, n);
	}
};

template <class PredPre, class PredPost>
struct ValueCheck<THEN<PredPre, PredPost>> {
	template <class Value>
	[[nodiscard]] static constexpr bool apply(THEN<PredPre, PredPost> const& p,
	                                          Value const&                   v)
	{
		return !ValueCheck<PredPre>::apply(p.left, v) ||
		       ValueCheck<PredPost>::apply(p.right, v);
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool apply(THEN<PredPre, PredPost> const& p,
	                                          Tree const& t, Node const& n)
	{
		return !ValueCheck<PredPre>::apply(p.left, t, n) ||
		       ValueCheck<PredPost>::apply(p.right, t, n);
	}
};

template <class PredLeft, class PredRight>
struct ValueCheck<IFF<PredLeft, PredRight>> {
	template <class Value>
	[[nodiscard]] static constexpr bool apply(IFF<PredLeft, PredRight> const& p,
	                                          Value const&                    v)
	{
		return ValueCheck<PredLeft>::apply(p.left, v) ==
		       ValueCheck<PredRight>::apply(p.right, v);
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool apply(IFF<PredLeft, PredRight> const& p,
	                                          Tree const& t, Node const& n)
	{
		return ValueCheck<PredLeft>::apply(p.left, t, n) ==
		       ValueCheck<PredRight>::apply(p.right, t, n);
	}
};

template <>
struct ValueCheck<True> {
	template <class Value>
	[[nodiscard]] static constexpr bool apply(True, Value const&)
	{
		return true;
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool apply(True, Tree const&, Node const&)
	{
		return true;
	}
};

template <>
struct ValueCheck<False> {
	template <class Value>
	[[nodiscard]] static constexpr bool apply(False, Value const&)
	{
		return false;
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool apply(False, Tree const&, Node const&)
	{
		return false;
	}
};

template <>
struct ValueCheck<bool> {
	template <class Value>
	[[nodiscard]] static constexpr bool apply(bool p, Value const&)
	{
		return p;
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool apply(bool p, Tree const&, Node const&)
	{
		return p;
	}
};

//
// Predicate inner check
//

template <class Pred>
struct InnerCheck {
	static_assert(static_assert_check_v<false, Pred>,
	              "Not implemented for this predicate.");
};

template <class... Preds>
struct InnerCheck<std::tuple<Preds...>> {
	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool apply(std::tuple<Preds...> const& p, Tree const& t,
	                                          Node const& n)
	{
		return std::apply(
		    [&t, &n](auto const&... p) {
			    return ((InnerCheck<std::decay_t<decltype(p)>>::apply(p, t, n)) && ...);
		    },
		    p);
	}
};

template <class PredLeft, class PredRight>
struct InnerCheck<OR<PredLeft, PredRight>> {
	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool apply(OR<PredLeft, PredRight> const& p,
	                                          Tree const& t, Node const& n)
	{
		return InnerCheck<PredLeft>::apply(p.left, t, n) ||
		       InnerCheck<PredRight>::apply(p.right, t, n);
	}
};

template <class PredPre, class PredPost>
struct InnerCheck<THEN<PredPre, PredPost>> {
	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool apply(THEN<PredPre, PredPost> const& p,
	                                          Tree const& t, Node const& n)
	{
		return !InnerCheck<PredPre>::apply(p.left, t, n) ||
		       InnerCheck<PredPost>::apply(p.right, t, n);
	}
};

template <class PredLeft, class PredRight>
struct InnerCheck<IFF<PredLeft, PredRight>> {
	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool apply(IFF<PredLeft, PredRight> const& p,
	                                          Tree const& t, Node const& n)
	{
		return InnerCheck<PredLeft>::apply(p.left, t, n) ==
		       InnerCheck<PredRight>::apply(p.right, t, n);
	}
};

template <>
struct InnerCheck<True> {
	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool apply(True, Tree const&, Node const&)
	{
		return true;
	}
};

template <>
struct InnerCheck<False> {
	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool apply(False, Tree const&, Node const&)
	{
		return false;
	}
};

template <>
struct InnerCheck<bool> {
	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool apply(bool p, Tree const&, Node const&)
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
struct contains_pred<T, std::tuple<Ts...>> : std::disjunction<contains_pred<T, Ts>...> {
};

template <class T, class L, class R>
struct contains_pred<T, OR<L, R>>
    : std::disjunction<contains_pred<T, L>, contains_pred<T, R>> {
};

template <class T, class L, class R>
struct contains_pred<T, THEN<L, R>>
    : std::disjunction<contains_pred<T, L>, contains_pred<T, R>> {
};

template <class T, class L, class R>
struct contains_pred<T, IFF<L, R>>
    : std::disjunction<contains_pred<T, L>, contains_pred<T, R>> {
};
}  // namespace detail

template <class Pred, class Preds>
using contains_pred = detail::contains_pred<Pred, Preds>;

template <class Pred, class Preds>
inline constexpr bool contains_pred_v = contains_pred<Pred, Preds>::value;

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
struct contains_always_pred<T, std::tuple<Ts...>>
    : std::disjunction<contains_always_pred<T, Ts>...> {
};

template <class T, class L, class R>
struct contains_always_pred<T, OR<L, R>>
    : std::conjunction<contains_always_pred<T, L>, contains_always_pred<T, R>> {
};

template <class T, class L, class R>
struct contains_always_pred<T, THEN<L, R>> : std::false_type {
};

template <class T, class L, class R>
struct contains_always_pred<T, IFF<L, R>> : std::false_type {
};
}  // namespace detail

template <class Pred, class Preds>
using contains_always_pred = detail::contains_always_pred<Pred, Preds>;

template <class Pred, class Preds>
inline constexpr bool contains_always_pred_v = contains_always_pred<Pred, Preds>::value;

//
// Is predicate
//

template <class, class, class, class = void>
struct is_pred : std::false_type {
};

template <class Pred, class Tree, class Node>
struct is_pred<
    Pred, Tree, Node,
    std::void_t<decltype(ValueCheck<Pred>::apply(
                    std::declval<Pred>(), std::declval<Tree>(), std::declval<Node>())),
                decltype(InnerCheck<Pred>::apply(
                    std::declval<Pred>(), std::declval<Tree>(), std::declval<Node>()))>>
    : std::true_type {
};

template <class Pred, class Tree, class Node>
inline constexpr bool is_pred_v = is_pred<Pred, Tree, Node>::value;

//
// Is point predicate
//

template <class, class, class, class, class = void>
struct is_value_pred : std::false_type {
};

template <class Pred, class Tree, class Node, class Value>
struct is_value_pred<
    Pred, Tree, Node, Value,
    std::void_t<decltype(ValueCheck<Pred>::apply(std::declval<Pred>(),
                                                 std::declval<Value>())),
                decltype(InnerCheck<Pred>::apply(
                    std::declval<Pred>(), std::declval<Tree>(), std::declval<Node>()))>>
    : std::true_type {
};

template <class Pred, class Tree, class Node, class Value>
inline constexpr bool is_value_pred_v = is_value_pred<Pred, Tree, Node, Value>::value;
}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_PREDICATE_HPP