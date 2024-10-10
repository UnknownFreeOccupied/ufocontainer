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
#include <ufo/utility/type_traits.hpp>

// STL
#include <tuple>
#include <type_traits>

namespace ufo::pred
{
//
// AND (&&)
//

template <
    class Pred1, class Pred2,
    std::enable_if_t<!is_tuple_v<std::decay_t<Pred1>> && !is_tuple_v<std::decay_t<Pred2>>,
                     bool> = true>
constexpr std::tuple<Pred1, Pred2> operator&&(Pred1&& p1, Pred2&& p2)
{
	return std::make_tuple(std::forward<Pred1>(p1), std::forward<Pred2>(p2));
}

template <class... Preds1, class... Preds2>
constexpr std::tuple<Preds1..., Preds2...> operator&&(std::tuple<Preds1...>&& t1,
                                                      std::tuple<Preds2...>&& t2)
{
	return std::tuple_cat(std::move(t1), std::move(t2));
}

template <class... Preds1, class... Preds2>
constexpr std::tuple<Preds1..., Preds2...> operator&&(std::tuple<Preds1...> const& t1,
                                                      std::tuple<Preds2...> const& t2)
{
	return std::tuple_cat(t1, t2);
}

template <class... Preds, class Pred>
constexpr std::tuple<Preds..., std::decay_t<Pred>> operator&&(std::tuple<Preds...>&& t,
                                                              Pred&&                 p)
{
	return std::tuple_cat(std::move(t), std::make_tuple(std::forward<Pred>(p)));
}

template <class... Preds, class Pred>
constexpr std::tuple<Preds..., std::decay_t<Pred>> operator&&(
    std::tuple<Preds...> const& t, Pred&& p)
{
	return std::tuple_cat(t, std::make_tuple(std::forward<Pred>(p)));
}

template <class Pred, class... Preds>
constexpr std::tuple<std::decay_t<Pred>, Preds...> operator&&(Pred&&                 p,
                                                              std::tuple<Preds...>&& t)
{
	return std::tuple_cat(std::make_tuple(std::forward<Pred>(p)), std::move(t));
}

template <class Pred, class... Preds>
constexpr std::tuple<std::decay_t<Pred>, Preds...> operator&&(
    Pred&& p, std::tuple<Preds...> const& t)
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

namespace detail
{
template <class T>
constexpr inline bool False = false;
};  // namespace detail

//
// Filter
//

template <class Pred>
struct Filter {
	static_assert(detail::False<Pred>, "Predicate not implemented correctly.");
};

template <class... Preds>
struct Filter<std::tuple<Preds...>> {
	using Pred = std::tuple<Preds...>;

	template <class Tree>
	static constexpr void init(Pred& p, Tree const& t)
	{
		std::apply(
		    [&t](auto&... p) { ((Filter<std::decay_t<decltype(p)>>::init(p, t)), ...); }, p);
	}

	template <class Value>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Value const& v)
	{
		return std::apply(
		    [&v](auto const&... p) {
			    return ((Filter<std::decay_t<decltype(p)>>::returnable(p, v)) && ...);
		    },
		    p);
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               Node const& n)
	{
		return std::apply(
		    [&t, &n](auto const&... p) {
			    return ((Filter<std::decay_t<decltype(p)>>::returnable(p, t, n)) && ...);
		    },
		    p);
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                Node const& n)
	{
		return std::apply(
		    [&t, &n](auto const&... p) {
			    return ((Filter<std::decay_t<decltype(p)>>::traversable(p, t, n)) && ...);
		    },
		    p);
	}
};

template <class PredLeft, class PredRight>
struct Filter<OR<PredLeft, PredRight>> {
	using Pred = OR<PredLeft, PredRight>;

	template <class Tree>
	static constexpr void init(Pred& p, Tree const& t)
	{
		Filter<PredLeft>::init(p.left, t);
		Filter<PredRight>::init(p.right, t);
	}

	template <class Value>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Value const& v)
	{
		return Filter<PredLeft>::returnable(p.left, v) ||
		       Filter<PredRight>::returnable(p.right, v);
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               Node const& n)
	{
		return Filter<PredLeft>::returnable(p.left, t, n) ||
		       Filter<PredRight>::returnable(p.right, t, n);
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                Node const& n)
	{
		return Filter<PredLeft>::traversable(p.left, t, n) ||
		       Filter<PredRight>::traversable(p.right, t, n);
	}
};

template <class PredPre, class PredPost>
struct Filter<THEN<PredPre, PredPost>> {
	using Pred = THEN<PredPre, PredPost>;

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

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               Node const& n)
	{
		return !Filter<PredPre>::returnable(p.left, t, n) ||
		       Filter<PredPost>::returnable(p.right, t, n);
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                Node const& n)
	{
		return !Filter<PredPre>::traversable(p.left, t, n) ||
		       Filter<PredPost>::traversable(p.right, t, n);
	}
};

template <class PredLeft, class PredRight>
struct Filter<IFF<PredLeft, PredRight>> {
	using Pred = IFF<PredLeft, PredRight>;

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

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               Node const& n)
	{
		return Filter<PredLeft>::returnable(p.left, t, n) ==
		       Filter<PredRight>::returnable(p.right, t, n);
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                Node const& n)
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

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool returnable(Pred const&, Tree const&, Node const&)
	{
		return true;
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool traversable(Pred const&, Tree const&, Node const&)
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

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool returnable(Pred const&, Tree const&, Node const&)
	{
		return false;
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool traversable(Pred const&, Tree const&, Node const&)
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

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const&, Node const&)
	{
		return p;
	}

	template <class Tree, class Node>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const&, Node const&)
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
constexpr inline bool contains_always_pred_v = contains_always_pred<Pred, Preds>::value;

//
// Is predicate
//

template <class, class, class, class = void>
struct is_pred : std::false_type {
};

template <class Pred, class Tree, class Node>
struct is_pred<
    Pred, Tree, Node,
    std::void_t<decltype(Filter<Pred>::init(std::declval<Pred>(), std::declval<Tree>())),
                decltype(Filter<Pred>::returnable(
                    std::declval<Pred>(), std::declval<Tree>(), std::declval<Node>())),
                decltype(Filter<Pred>::traversable(
                    std::declval<Pred>(), std::declval<Tree>(), std::declval<Node>()))>>
    : std::true_type {
};

template <class Pred, class Tree, class Node>
constexpr inline bool is_pred_v = is_pred<Pred, Tree, Node>::value;

//
// Is value predicate
//

template <class, class, class, class, class = void>
struct is_value_pred : std::false_type {
};

template <class Pred, class Tree, class Node, class Value>
struct is_value_pred<
    Pred, Tree, Node, Value,
    std::void_t<decltype(Filter<Pred>::init(std::declval<Pred>(), std::declval<Tree>())),
                decltype(Filter<Pred>::returnable(std::declval<Pred>(),
                                                  std::declval<Value>())),
                decltype(Filter<Pred>::traversable(
                    std::declval<Pred>(), std::declval<Tree>(), std::declval<Node>()))>>
    : std::true_type {
};

template <class Pred, class Tree, class Node, class Value>
constexpr inline bool is_value_pred_v = is_value_pred<Pred, Tree, Node, Value>::value;
}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_PREDICATE_HPP