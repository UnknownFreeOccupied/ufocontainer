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

// #ifndef UFO_CONTAINER_TREE_PREDICATE_SIZE_HPP
// #define UFO_CONTAINER_TREE_PREDICATE_SIZE_HPP

// // UFO
// #include <ufo/map/predicate/predicate.hpp>
// #include <ufo/map/predicate/predicate_compare.hpp>
// #include <ufo/map/types.hpp>

// namespace ufo::pred
// {

// template <PredicateCompare PC = PredicateCompare::EQUAL,
//           PredicateType    PT = PredicateType::RETURN_AND_INNER>
// struct Size {
// 	node_size_t size;

// 	Size() = default;

// 	constexpr Size(node_size_t size) noexcept : size(size) {}

//  private:
// 	mutable depth_t depth_;

// 	friend struct Init<Size<PC, PT>>;
// 	friend struct ValueCheck<Size<PC, PT>>;
// 	friend struct InnerCheck<Size<PC, PT>>;
// };

// using SizeE  = Size<PredicateCompare::EQUAL>;
// using SizeLE = Size<PredicateCompare::LESS_EQUAL>;
// using SizeGE = Size<PredicateCompare::GREATER_EQUAL>;
// using SizeL  = Size<PredicateCompare::LESS>;
// using SizeG  = Size<PredicateCompare::GREATER>;

// using SizeMin = SizeGE;
// using SizeMax = SizeLE;

// template <PredicateCompare PC = PredicateCompare::EQUAL,
//           PredicateType    PT = PredicateType::RETURN_AND_INNER>
// struct Init<Size<PC, PT>> {
// 	using Pred = Size<PC, PT>;

// 	template <class Map>
// 	static constexpr void apply(Pred& p, Map const& m)
// 	{
// 		auto dl = m.depthLevels();
// 		if constexpr (PredicateCompare::EQUAL == PC || PredicateCompare::NOT_EQUAL == PC) {
// 			for (depth_t d = 0; dl > d; ++d) {
// 				if (p.size == m.size(d)) {
// 					p.depth_ = d;
// 					return;
// 				}
// 			}
// 			p.depth_ = std::numeric_limits<depth_t>::max();
// 		} else if constexpr (PredicateCompare::LESS_EQUAL == PC) {
// 			for (depth_t d = 0; dl > d; ++d) {
// 				if (p.size <= m.size(d)) {
// 					p.depth_ = d;
// 				}
// 			}
// 		} else if constexpr (PredicateCompare::LESS == PC) {
// 		} else if constexpr (PredicateCompare::GREATER_EQUAL == PC) {
// 		} else if constexpr (PredicateCompare::GREATER == PC) {
// 		}

// 		auto temp = std::max(0.0, (p.size / m.size()) + 1.0);
// 		if constexpr (PredicateCompare::EQUAL == PC ||
// 		              PredicateCompare::GREATER_EQUAL == PC || PredicateCompare::GREATER) {
// 			p.depth_ = std::floor(temp);
// 		} else if constexpr () {
// 			p.depth_ = std::ceil(temp);
// 		}
// 	}
// };

// template <PredicateCompare PC = PredicateCompare::EQUAL,
//           PredicateType    PT = PredicateType::RETURN_AND_INNER>
// struct InnerCheck<Size<PC, PT>> {
// 	using Pred = Size<PC, PT>;

// 	template <class Map, class Node>
// 	static constexpr bool apply(Pred, Map const&, Node const&) noexcept
// 	{
// 		// TODO: Implement
// 	}
// };

// template <PredicateCompare PC = PredicateCompare::EQUAL,
//           PredicateType    PT = PredicateType::RETURN_AND_INNER>
// struct ValueCheck<Size<PC, PT>> {
// 	using Pred = Size<PC, PT>;

// 	template <class Map, class Node>
// 	static constexpr bool apply(Pred, Map const& m, Node const& n)
// 	{
// 		// TODO: Implement
// 	}
// };
// }  // namespace ufo::pred

// #endif  // UFO_CONTAINER_TREE_PREDICATE_SIZE_HPP