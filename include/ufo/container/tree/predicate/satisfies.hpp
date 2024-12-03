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

#ifndef UFO_CONTAINER_TREE_PREDICATE_SATISFIES_HPP
#define UFO_CONTAINER_TREE_PREDICATE_SATISFIES_HPP

// UFO
#include <ufo/container/tree/predicate/filter.hpp>

namespace ufo::pred
{
template <class Fun, bool Negated = false>
struct Satisfies {
	Satisfies(Fun fun) : fun(fun) {}

	Fun fun;
};

template <class Fun, bool Negated>
constexpr Satisfies<Fun, !Negated> operator!(Satisfies<Fun, Negated> const& p)
{
	return Satisfies<Fun, !Negated>(p.fun);
}

template <class Fun, bool Negated>
struct Filter<Satisfies<Fun, Negated>> {
	using Pred = Satisfies<Fun, Negated>;

	template <class Tree>
	static constexpr void init(Pred&, Tree const&)
	{
	}

	template <class Value>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Value const& v)
	{
		if constexpr (Negated) {
			return !p.fun(v);
		} else {
			return p.fun(v);
		}
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               typename Tree::Node const& n)
	{
		if constexpr (Negated) {
			return !p.fun(n);
		} else {
			return p.fun(n);
		}
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const&, Tree const&,
	                                                typename Tree::Node const&)
	{
		return true;
	}
};
}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_SATISFIES_HPP