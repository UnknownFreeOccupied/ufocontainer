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

// STL
#include <sstream>
#include <string>
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

template <class Pred, class Tree>
constexpr void init(Pred&, Tree const&)
{
}

template <class... Preds, class Tree>
constexpr void init(std::tuple<Preds...>& p, Tree const& t)
{
	std::apply([&t](auto&... p) { (init(p, t), ...); }, p);
}

template <class PredLeft, class PredRight, class Tree>
constexpr void init(OR<PredLeft, PredRight>& p, Tree const& t)
{
	init(p.left, t);
	init(p.right, t);
}

template <class PredPre, class PredPost, class Tree>
constexpr void init(THEN<PredPre, PredPost>& p, Tree const& t)
{
	init(p.pre, t);
	init(p.post, t);
}

template <class PredLeft, class PredRight, class Tree>
constexpr void init(IFF<PredLeft, PredRight>& p, Tree const& t)
{
	init(p.left, t);
	init(p.right, t);
}

//
// Predicate value check
//

template <class Pred, class Value>
[[nodiscard]] constexpr bool valueCheck(Pred const&, Value const&) = delete;

template <class Pred, class Tree, class Node>
[[nodiscard]] constexpr bool valueCheck(Pred const&, Tree const&, Node) = delete;

template <class... Preds, class Value>
[[nodiscard]] constexpr bool valueCheck(std::tuple<Preds...> const& p, Value const v)
{
	return std::apply([&v](auto const&... p) { return (valueCheck(p, v) && ...); }, p);
}

template <class... Preds, class Tree, class Node>
[[nodiscard]] constexpr bool valueCheck(std::tuple<Preds...> const& p, Tree const& t,
                                        Node n)
{
	return std::apply([&t, n](auto const&... p) { return (valueCheck(p, t, n) && ...); },
	                  p);
}

template <class PredLeft, class PredRight, class Value>
[[nodiscard]] constexpr bool valueCheck(OR<PredLeft, PredRight> const& p, Value const& v)
{
	return valueCheck(p.left, v) || valueCheck(p.right, v);
}

template <class PredLeft, class PredRight, class Tree, class Node>
[[nodiscard]] constexpr bool valueCheck(OR<PredLeft, PredRight> const& p, Tree const& t,
                                        Node n)
{
	return valueCheck(p.left, t, n) || valueCheck(p.right, t, n);
}

template <class PredPre, class PredPost, class Value>
[[nodiscard]] constexpr bool valueCheck(THEN<PredPre, PredPost> const& p, Value const& v)
{
	return !valueCheck(p.pre, v) || valueCheck(p.post, v);
}

template <class PredPre, class PredPost, class Tree, class Node>
[[nodiscard]] constexpr bool valueCheck(THEN<PredPre, PredPost> const& p, Tree const& t,
                                        Node n)
{
	return !valueCheck(p.pre, t, n) || valueCheck(p.post, t, n);
}

template <class PredLeft, class PredRight, class Value>
[[nodiscard]] constexpr bool valueCheck(IFF<PredLeft, PredRight> const& p, Value const& v)
{
	return valueCheck(p.left, v) == valueCheck(p.right, v);
}

template <class PredLeft, class PredRight, class Tree, class Node>
[[nodiscard]] constexpr bool valueCheck(IFF<PredLeft, PredRight> const& p, Tree const& t,
                                        Node n)
{
	return valueCheck(p.left, t, n) == valueCheck(p.right, t, n);
}

template <class Value>
[[nodiscard]] constexpr bool valueCheck(True, Value const&)
{
	return true;
}

template <class Tree, class Node>
[[nodiscard]] constexpr bool valueCheck(True, Tree const&, Node)
{
	return true;
}

template <class Value>
[[nodiscard]] constexpr bool valueCheck(False, Value const&)
{
	return false;
}

template <class Tree, class Node>
[[nodiscard]] constexpr bool valueCheck(False, Tree const&, Node)
{
	return false;
}

template <class Value>
[[nodiscard]] constexpr bool valueCheck(bool p, Value const&)
{
	return p;
}

template <class Tree, class Node>
[[nodiscard]] constexpr bool valueCheck(bool p, Tree const&, Node)
{
	return p;
}

//
// Predicate inner check
//

template <class Pred, class Tree, class Node>
[[nodiscard]] constexpr bool innerCheck(Pred const&, Tree const&, Node) = delete;

template <class... Preds, class Tree, class Node>
[[nodiscard]] constexpr bool innerCheck(std::tuple<Preds...> const& p, Tree const& t,
                                        Node n)
{
	return std::apply([&t, n](auto const&... p) { return (innerCheck(p, t, n) && ...); },
	                  p);
}

template <class PredLeft, class PredRight, class Tree, class Node>
[[nodiscard]] constexpr bool innerCheck(OR<PredLeft, PredRight> const& p, Tree const& t,
                                        Node n)
{
	return innerCheck(p.left, t, n) || innerCheck(p.right, t, n);
}

template <class PredPre, class PredPost, class Tree, class Node>
[[nodiscard]] constexpr bool innerCheck(THEN<PredPre, PredPost> const& p, Tree const& t,
                                        Node n)
{
	return !innerCheck(p.pre, t, n) || innerCheck(p.post, t, n);
}

template <class PredLeft, class PredRight, class Tree, class Node>
[[nodiscard]] constexpr bool innerCheck(IFF<PredLeft, PredRight> const& p, Tree const& t,
                                        Node n)
{
	return innerCheck(p.left, t, n) == innerCheck(p.right, t, n);
}

template <class Tree, class Node>
[[nodiscard]] constexpr bool innerCheck(True, Tree const&, Node)
{
	return true;
}

template <class Tree, class Node>
[[nodiscard]] constexpr bool innerCheck(False, Tree const&, Node)
{
	return false;
}

template <class Tree, class Node>
[[nodiscard]] constexpr bool innerCheck(bool p, Tree const&, Node)
{
	return p;
}

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
struct is_pred<Pred, Tree, Node,
               std::void_t<decltype(valueCheck(std::declval<Pred>(), std::declval<Tree>(),
                                               std::declval<Node>())),
                           decltype(innerCheck(std::declval<Pred>(), std::declval<Tree>(),
                                               std::declval<Node>()))>> : std::true_type {
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
    std::void_t<decltype(valueCheck(std::declval<Pred>(), std::declval<Value>())),
                decltype(innerCheck(std::declval<Pred>(), std::declval<Tree>(),
                                    std::declval<Node>()))>> : std::true_type {
};

template <class Pred, class Tree, class Node, class Value>
inline constexpr bool is_value_pred_v = is_value_pred<Pred, Tree, Node, Value>::value;
}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_PREDICATE_HPP