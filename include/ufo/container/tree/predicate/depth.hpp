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

#ifndef UFO_CONTAINER_TREE_PREDICATE_DEPTH_HPP
#define UFO_CONTAINER_TREE_PREDICATE_DEPTH_HPP

// UFO
#include <ufo/container/tree/predicate/predicate.hpp>
#include <ufo/container/tree/predicate/predicate_compare.hpp>

namespace ufo::pred
{
template <PredicateCompare PC = PredicateCompare::EQUAL,
          PredicateType    PT = PredicateType::VALUE_AND_INNER>
struct Depth {
	using depth_t = std::uint32_t;

	depth_t depth{};

	constexpr Depth() = default;

	constexpr Depth(depth_t depth) noexcept : depth(depth) {}
};

using DepthE  = Depth<PredicateCompare::EQUAL>;
using DepthLE = Depth<PredicateCompare::LESS_EQUAL>;
using DepthGE = Depth<PredicateCompare::GREATER_EQUAL>;
using DepthL  = Depth<PredicateCompare::LESS>;
using DepthG  = Depth<PredicateCompare::GREATER>;

using DepthMin = DepthGE;
using DepthMax = DepthLE;

template <PredicateCompare PC, PredicateType PT, class Tree, class Node>
[[nodiscard]] constexpr bool valueCheck(Depth<PC, PT> p, Tree const& t, Node n)
{
	if constexpr (PredicateType::INNER == PT) {
		return true;
	} else if constexpr (PredicateCompare::EQUAL == PC) {
		return t.depth(n) == p.depth;
	} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
		return t.depth(n) <= p.depth;
	} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
		return t.depth(n) >= p.depth;
	} else if constexpr (PredicateCompare::LESS == PC) {
		return t.depth(n) < p.depth;
	} else if constexpr (PredicateCompare::GREATER == PC) {
		return t.depth(n) > p.depth;
	}
}

template <PredicateCompare PC, PredicateType PT, class Tree, class Node>
[[nodiscard]] constexpr bool innerCheck(Depth<PC, PT> p, Tree const& t, Node n)
{
	if constexpr (PredicateType::VALUE == PT) {
		return true;
	} else if constexpr (PredicateCompare::EQUAL == PC) {
		return t.depth(n) > p.depth;
	} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
		return true;
	} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
		return t.depth(n) > p.depth;
	} else if constexpr (PredicateCompare::LESS == PC) {
		return true;
	} else if constexpr (PredicateCompare::GREATER == PC) {
		return t.depth(n) > (p.depth + 1U);
	}
}
}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_DEPTH_HPP