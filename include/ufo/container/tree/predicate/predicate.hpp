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

enum class PredicateType { RETURN = 1, INNER, RETURN_AND_INNER };

//
// AND (&&)
//

template <class Pred1, class Pred2>
constexpr std::tuple<Pred1, Pred2> operator&&(Pred1 const& p1, Pred2 const& p2)
{
	return std::tuple<Pred1, Pred2>(p1, p2);
}

template <class... Preds1, class... Preds2>
constexpr std::tuple<Preds1..., Preds2...> operator&&(std::tuple<Preds1...> const& t1,
                                                      std::tuple<Preds2...> const& t2)
{
	return std::tuple_cat(t1, t2);
}

template <class... Preds, class Pred>
constexpr std::tuple<Preds..., Pred> operator&&(std::tuple<Preds...> const& t,
                                                Pred const&                 p)
{
	return std::tuple_cat(t, std::tie(p));
}

template <class Pred, class... Preds>
constexpr std::tuple<Pred, Preds...> operator&&(Pred const&                 p,
                                                std::tuple<Preds...> const& t)
{
	return std::tuple_cat(std::tie(p), t);
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
constexpr OR<PredLeft, PredRight> operator||(PredLeft const& p1, PredRight const& p2)
{
	return OR<PredLeft, PredRight>{p1, p2};
}

// template <class... PredsLeft, class... PredsRight>
// constexpr OR<std::tuple<PredsLeft...>, std::tuple<PredsRight...>> operator||(
//     std::tuple<PredsLeft...> const& t1, std::tuple<PredsRight...> const& t2)
// {
// 	return OR<std::tuple<PredsLeft...>, std::tuple<PredsRight...>>{t1, t2};
// }

// template <class... PredsLeft, class PredRight>
// constexpr OR<std::tuple<PredsLeft...>, PredRight> operator||(
//     std::tuple<PredsLeft...> const& t, PredRight const& p)
// {
// 	return OR<std::tuple<PredsLeft...>, PredRight>{t, p};
// }

// template <class PredLeft, class... PredsRight>
// constexpr OR<PredLeft, std::tuple<PredsRight...>> operator||(
//     PredLeft const& p, std::tuple<PredsRight...> const& t)
// {
// 	return OR<PredLeft, std::tuple<PredsRight...>>{p, t};
// }

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

template <class Predicate>
struct Init {
	using Pred = Predicate;

	template <class Map>
	static constexpr void apply(Pred const&, Map const&)
	{
	}
};

template <class... Preds>
struct Init<std::tuple<Preds...>> {
	using Pred = std::tuple<Preds...>;

	template <class Map>
	static constexpr void apply(Pred const& p, Map const& m)
	{
		std::apply(
		    [&m](auto&... p) { ((Init<std::decay_t<decltype(p)>>::apply(p, m)), ...); }, p);
	}
};

template <class PredLeft, class PredRight>
struct Init<OR<PredLeft, PredRight>> {
	using Pred = OR<PredLeft, PredRight>;

	template <class Map>
	static constexpr void apply(Pred const& p, Map const& m)
	{
		Init<PredLeft>::apply(p.left, m);
		Init<PredRight>::apply(p.right, m);
	}
};

template <class PredPre, class PredPost>
struct Init<THEN<PredPre, PredPost>> {
	using Pred = THEN<PredPre, PredPost>;

	template <class Map>
	static constexpr void apply(Pred const& p, Map const& m)
	{
		Init<PredPre>::apply(p.pre, m);
		Init<PredPost>::apply(p.post, m);
	}
};

template <class PredLeft, class PredRight>
struct Init<IFF<PredLeft, PredRight>> {
	using Pred = IFF<PredLeft, PredRight>;

	template <class Map>
	static constexpr void apply(Pred const& p, Map const& m)
	{
		Init<PredLeft>::apply(p.pre, m);
		Init<PredRight>::apply(p.post, m);
	}
};

//
// Predicate value/return check
//

template <class Predicate>
struct ValueCheck {
	// REVIEW: Make better
	// static_assert(static_assert_check_v<false, Predicate>,
	//               "Not implemented for this Predicate.");
};

template <class... Preds>
struct ValueCheck<std::tuple<Preds...>> {
	using Pred = std::tuple<Preds...>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return std::apply(
		    [&m, &n](auto const&... p) {
			    return ((ValueCheck<std::decay_t<decltype(p)>>::apply(p, m, n)) && ...);
		    },
		    p);
	}
};

template <class PredLeft, class PredRight>
struct ValueCheck<OR<PredLeft, PredRight>> {
	using Pred = OR<PredLeft, PredRight>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return ValueCheck<PredLeft>::apply(p.left, m, n) ||
		       ValueCheck<PredRight>::apply(p.right, m, n);
	}
};

template <class PredPre, class PredPost>
struct ValueCheck<THEN<PredPre, PredPost>> {
	using Pred = THEN<PredPre, PredPost>;

	template <class Map, class Node>
	static constexpr auto apply(Pred const& p, Map const& m, Node const& n)
	    -> decltype(ValueCheck<PredPre>::apply(p.pre, m, n), bool())
	{
		return !ValueCheck<PredPre>::apply(p.pre, m, n) ||
		       ValueCheck<PredPost>::apply(p.post, m, n);
	}

	static constexpr bool apply(...) { return true; }
};

template <class PredLeft, class PredRight>
struct ValueCheck<IFF<PredLeft, PredRight>> {
	using Pred = IFF<PredLeft, PredRight>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		bool left  = ValueCheck<PredLeft>::apply(p.left, m, n);
		bool right = ValueCheck<PredRight>::apply(p.right, m, n);
		return (left && right) || (!left && !right);
	}
};

template <>
struct ValueCheck<True> {
	using Pred = True;

	template <class Map, class Node>
	static constexpr bool apply(Pred, Map const&, Node const&)
	{
		return true;
	}
};

template <>
struct ValueCheck<False> {
	using Pred = False;

	template <class Map, class Node>
	static constexpr bool apply(Pred, Map const&, Node const&)
	{
		return false;
	}
};

template <>
struct ValueCheck<bool> {
	using Pred = bool;

	template <class Map, class Node>
	static constexpr bool apply(Pred p, Map const&, Node const&)
	{
		return p;
	}
};

//
// Predicate inner check
//

template <class Predicate>
struct InnerCheck {
	// REVIEW: Make better
	// static_assert(static_assert_check_v<false, Predicate>,
	//               "Not implemented for this Predicate.");
};

template <class... Preds>
struct InnerCheck<std::tuple<Preds...>> {
	using Pred = std::tuple<Preds...>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return std::apply(
		    [&m, &n](auto const&... p) {
			    return ((InnerCheck<std::decay_t<decltype(p)>>::apply(p, m, n)) && ...);
		    },
		    p);
	}
};

template <class PredLeft, class PredRight>
struct InnerCheck<OR<PredLeft, PredRight>> {
	using Pred = OR<PredLeft, PredRight>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		return InnerCheck<PredLeft>::apply(p.left, m, n) ||
		       InnerCheck<PredRight>::apply(p.right, m, n);
	}
};

template <class PredPre, class PredPost>
struct InnerCheck<THEN<PredPre, PredPost>> {
	using Pred = THEN<PredPre, PredPost>;

	template <class Map, class Node>
	static constexpr auto apply(Pred const& p, Map const& m, Node const& n)
	    -> decltype(InnerCheck<PredPre>::apply(p.pre, m, n), bool())
	{
		return !InnerCheck<PredPre>::apply(p.pre, m, n) ||
		       InnerCheck<PredPost>::apply(p.post, m, n);
	}

	static constexpr bool apply(...) { return true; }
};

template <class PredLeft, class PredRight>
struct InnerCheck<IFF<PredLeft, PredRight>> {
	using Pred = IFF<PredLeft, PredRight>;

	template <class Map, class Node>
	static constexpr bool apply(Pred const& p, Map const& m, Node const& n)
	{
		bool left  = InnerCheck<PredLeft>::apply(p.left, m, n);
		bool right = InnerCheck<PredRight>::apply(p.right, m, n);
		return (left && right) || (!left && !right);
	}
};

template <>
struct InnerCheck<True> {
	using Pred = True;

	template <class Map, class Node>
	static constexpr bool apply(Pred, Map const&, Node const&)
	{
		return true;
	}
};

template <>
struct InnerCheck<False> {
	using Pred = False;

	template <class Map, class Node>
	static constexpr bool apply(Pred, Map const&, Node const&)
	{
		return false;
	}
};

template <>
struct InnerCheck<bool> {
	using Pred = bool;

	template <class Map, class Node>
	static constexpr bool apply(Pred p, Map const&, Node const&)
	{
		return p;
	}
};

//
// Shader value check
//

template <class Predicate>
struct ShaderValueCheck {
	// REVIEW: Make better
};

template <class... Preds>
struct ShaderValueCheck<std::tuple<Preds...>> {
	using Pred = std::tuple<Preds...>;

	template <class Map>
	static std::string apply(Pred const& p, Map const& m)
	{
		std::ostringstream stream;
		std::apply(
		    [&m, &stream](auto const& first, auto const&... rest) {
			    stream << '('
			           << ShaderValueCheck<std::decay_t<decltype(first)>>::apply(first, m)
			           << ')';
			    ((stream << " && ("
			             << ShaderValueCheck<std::decay_t<decltype(rest)>>::apply(rest, m)
			             << ')'),
			     ...);
		    },
		    p);
		return stream.str();
	}
};

template <class PredLeft, class PredRight>
struct ShaderValueCheck<OR<PredLeft, PredRight>> {
	using Pred = OR<PredLeft, PredRight>;

	template <class Map>
	static std::string apply(Pred const& p, Map const& m)
	{
		return "((" + ShaderValueCheck<PredLeft>::apply(p.left, m) + ") || (" +
		       ShaderValueCheck<PredRight>::apply(p.right, m) + "))";
	}
};

template <class PredPre, class PredPost>
struct ShaderValueCheck<THEN<PredPre, PredPost>> {
	using Pred = THEN<PredPre, PredPost>;

	template <class Map>
	static auto apply(Pred const& p,
	                  Map const&  m) -> decltype(ShaderValueCheck<PredPre>::apply(p.pre, m),
	                                            std::string())
	{
		return "(!(" + ShaderValueCheck<PredPre>::apply(p.pre, m) + ") || (" +
		       ShaderValueCheck<PredPost>::apply(p.post, m) + "))";
	}

	static std::string apply(...) { return ""; }
};

// TODO: Implement
// template <class PredLeft, class PredRight>
// struct ShaderValueCheck<IFF<PredLeft, PredRight>> {
// 	using Pred = IFF<PredLeft, PredRight>;

// 	template <class Map>
// 	static std::string apply(Pred const& p, Map const& m)
// 	{
// 		return "((" + ShaderValueCheck<PredLeft>::apply(p.left, m) + ") || (" +
// 		       ShaderValueCheck<PredRight>::apply(p.right, m) + "))";
// 	}
// };

template <>
struct ShaderValueCheck<True> {
	using Pred = True;

	template <class Map>
	static std::string apply(Pred, Map const&)
	{
		return "true";
	}
};

template <>
struct ShaderValueCheck<False> {
	using Pred = False;

	template <class Map>
	static std::string apply(Pred, Map const&)
	{
		return "false";
	}
};

template <>
struct ShaderValueCheck<bool> {
	using Pred = bool;

	template <class Map>
	static std::string apply(Pred p, Map const&)
	{
		return p ? "true" : "false";
	}
};

//
// Shader inner check
//

template <class Predicate>
struct ShaderInnerCheck {
	// REVIEW: Make better
};

template <class... Preds>
struct ShaderInnerCheck<std::tuple<Preds...>> {
	using Pred = std::tuple<Preds...>;

	template <class Map>
	static std::string apply(Pred const& p, Map const& m)
	{
		std::ostringstream stream;
		std::apply(
		    [&m, &stream](auto const& first, auto const&... rest) {
			    stream << '('
			           << ShaderInnerCheck<std::decay_t<decltype(first)>>::apply(first, m)
			           << ')';
			    ((stream << " && ("
			             << ShaderInnerCheck<std::decay_t<decltype(rest)>>::apply(rest, m)
			             << ')'),
			     ...);
		    },
		    p);
		return stream.str();
	}
};

template <class PredLeft, class PredRight>
struct ShaderInnerCheck<OR<PredLeft, PredRight>> {
	using Pred = OR<PredLeft, PredRight>;

	template <class Map>
	static std::string apply(Pred const& p, Map const& m)
	{
		return "((" + ShaderInnerCheck<PredLeft>::apply(p.left, m) + ") || (" +
		       ShaderInnerCheck<PredRight>::apply(p.right, m) + "))";
	}
};

template <class PredPre, class PredPost>
struct ShaderInnerCheck<THEN<PredPre, PredPost>> {
	using Pred = THEN<PredPre, PredPost>;

	template <class Map>
	static auto apply(Pred const& p,
	                  Map const&  m) -> decltype(ShaderInnerCheck<PredPre>::apply(p.pre, m),
	                                            std::string())
	{
		return "(!(" + ShaderInnerCheck<PredPre>::apply(p.pre, m) + ") || (" +
		       ShaderInnerCheck<PredPost>::apply(p.post, m) + "))";
	}

	static std::string apply(...) { return ""; }
};

// TODO: Implement
// template <class PredLeft, class PredRight>
// struct ShaderInnerCheck<IFF<PredLeft, PredRight>> {
// 	using Pred = IFF<PredLeft, PredRight>;

// 	template <class Map>
// 	static std::string apply(Pred const& p, Map const& m)
// 	{
// 		return "((" + ShaderInnerCheck<PredLeft>::apply(p.left, m) + ") || (" +
// 		       ShaderInnerCheck<PredRight>::apply(p.right, m) + "))";
// 	}
// };

template <>
struct ShaderInnerCheck<True> {
	using Pred = True;

	template <class Map>
	static std::string apply(Pred, Map const&)
	{
		return "true";
	}
};

template <>
struct ShaderInnerCheck<False> {
	using Pred = False;

	template <class Map>
	static std::string apply(Pred, Map const&)
	{
		return "false";
	}
};

template <>
struct ShaderInnerCheck<bool> {
	using Pred = bool;

	template <class Map>
	static std::string apply(Pred p, Map const&)
	{
		return p ? "true" : "false";
	}
};

//
// Contains predicate
//

template <typename T, typename T2>
struct has_type : std::false_type {
};

template <typename T>
struct has_type<T, std::tuple<>> : std::false_type {
};

template <typename T, typename L>
struct has_type<T, OR<L, T>> : std::true_type {
};

template <typename T, typename R>
struct has_type<T, OR<T, R>> : std::true_type {
};

template <typename T, typename L, typename R>
struct has_type<T, OR<L, R>> : std::false_type {
};

template <typename T, typename L>
struct has_type<T, THEN<L, T>> : std::true_type {
};

template <typename T, typename R>
struct has_type<T, THEN<T, R>> : std::true_type {
};

template <typename T, typename L, typename R>
struct has_type<T, THEN<L, R>> : std::false_type {
};

template <typename T, typename L>
struct has_type<T, IFF<L, T>> : std::true_type {
};

template <typename T, typename R>
struct has_type<T, IFF<T, R>> : std::true_type {
};

template <typename T, typename L, typename R>
struct has_type<T, IFF<L, R>> : std::false_type {
};

template <typename T, typename U, typename... Ts>
struct has_type<T, std::tuple<U, Ts...>> : has_type<T, std::tuple<Ts...>> {
};

template <typename T, typename... Ts>
struct has_type<T, std::tuple<T, Ts...>> : std::true_type {
};

template <typename T>
struct has_type<T, T> : std::true_type {
};

template <typename Predicate, typename Predicates>
using contains_predicate = has_type<Predicate, Predicates>;

template <typename Predicate, typename Predicates>
inline constexpr bool contains_predicate_v =
    contains_predicate<Predicate, Predicates>::value;

//
// Contains always predicate
//

template <typename T, typename T2>
struct has_always_type : std::false_type {
};

template <typename T>
struct has_always_type<T, std::tuple<>> : std::false_type {
};

template <typename T, typename L, typename R>
struct has_always_type<T, OR<L, R>> : std::false_type {
};

// FIXME: Should this be false if L is T?
template <typename T, typename L, typename R>
struct has_always_type<T, THEN<L, R>> : std::false_type {
};

// FIXME: Should this be false if L is T?
template <typename T, typename L, typename R>
struct has_always_type<T, IFF<L, R>> : std::false_type {
};

template <typename T, typename U, typename... Ts>
struct has_always_type<T, std::tuple<U, Ts...>> : has_always_type<T, std::tuple<Ts...>> {
};

template <typename T, typename... Ts>
struct has_always_type<T, std::tuple<T, Ts...>> : std::true_type {
};

template <typename T>
struct has_always_type<T, T> : std::true_type {
};

template <typename Predicate, typename Predicates>
using contains_always_predicate = has_always_type<Predicate, Predicates>;

template <typename Predicate, typename Predicates>
inline constexpr bool contains_always_predicate_v =
    contains_always_predicate<Predicate, Predicates>::value;

//
// Type traits
//

template <typename P, class Map, class Node, class = void>
struct is_predicate : std::false_type {
};

template <typename P, class Map, class Node>
struct is_predicate<
    P, Map, Node,
    std::void_t<decltype(ufo::pred::ValueCheck<P>::apply(
                    std::declval<P>(), std::declval<Map>(), std::declval<Node>())),
                decltype(ufo::pred::InnerCheck<P>::apply(
                    std::declval<P>(), std::declval<Map>(), std::declval<Node>()))>>
    : std::true_type {
};

template <typename P, class Map, class Node>
inline constexpr bool is_predicate_v = is_predicate<P, Map, Node>::value;
}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_PREDICATE_HPP