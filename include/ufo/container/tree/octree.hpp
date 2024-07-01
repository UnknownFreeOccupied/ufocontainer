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

#ifndef UFO_CONTAINER_OCTREE_HPP
#define UFO_CONTAINER_OCTREE_HPP

// UFO
#include <ufo/container/tree/tree.hpp>
#include <ufo/container/tree/type.hpp>

namespace ufo
{
template <class Derived, template <TreeType> class Block>
class Octree : public Tree<Derived, Block<TreeType::OCT>>
{
	using Base = Tree<Derived, Block<TreeType::OCT>>;

	//
	// Friends
	//

	friend Base;

 public:
	//
	// Tags
	//

	using length_t = typename Base::length_t;
	using depth_t  = typename Base::depth_t;
	using pos_t    = typename Base::pos_t;
	using offset_t = typename Base::offset_t;
	using key_t    = typename Base::key_t;
	using code_t   = typename Base::code_t;

	using Index       = typename Base::Index;
	using Node        = typename Base::Node;
	using NodeNearest = typename Base::NodeNearest;
	using Code        = typename Base::Code;
	using Key         = typename Base::Key;
	using Point       = typename Base::Point;
	using Coord       = typename Base::Coord;
	using Bounds      = typename Base::Bounds;

	// 	/**************************************************************************************
	// 	| | 	|                                        Trace | 	| |
	// 	**************************************************************************************/

	// 	template <class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true> 	Node trace(Ray ray, Predicate const& predicate, bool only_exists = true) const
	// 	{
	// 		return trace(Base::node(), ray, predicate, only_exists);
	// 	}

	// 	template <class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true> 	auto trace(Ray ray, Predicate const& predicate, BinaryFun binary_f, bool
	// only_exists = true) const
	// 	{
	// 		return trace(Base::node(), ray, predicate, binary_f, only_exists);
	// 	}

	// 	template <class InputIt, class OutputIt, class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true> 	OutputIt trace(InputIt first, InputIt last, OutputIt d_first, Predicate const&
	// predicate, bool only_exists = true) const
	// 	{
	// 		return trace(Base::node(), first, last, d_first, predicate, only_exists);
	// 	}

	// 	template <class InputIt, class OutputIt, class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true> 	OutputIt trace(InputIt first, InputIt last, OutputIt d_first, Predicate const&
	// predicate, BinaryFun binary_f, 	               bool only_exists = true) const
	// 	{
	// 		return trace(Base::node(), first, last, d_first, predicate, binary_f,
	// only_exists);
	// 	}

	// 	template <class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true> 	Node trace(Node node, Ray ray, Predicate const& predicate, 	           bool
	// only_exists = true) const
	// 	{
	// 		return trace(node, ray, predicate, [](Node, Ray) -> void {}, only_exists);
	// 	}

	// 	template <class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true> 	auto trace(Node node, Ray ray, Predicate const& predicate, BinaryFun binary_f,
	// 	           bool only_exists = true) const
	// 	{
	// 		pred::Init<Predicate>::apply(predicate, *this);

	// 		auto [t0, t1, a] = traceInit(node, ray);

	// 		if constexpr (pred::contains_spatial_predicate_v<std::decay_t<Predicate>>) {
	// 			if (only_exists) {
	// 				return trace<true>(Base::toNodeBV(node), t0, t1, a, ray, predicate, binary_f);
	// 			} else {
	// 				return trace<false>(Base::toNodeBV(node), t0, t1, a, ray, predicate,
	// binary_f);
	// 			}
	// 		} else {
	// 			if (only_exists) {
	// 				return trace<true>(node, t0, t1, a, ray, predicate, binary_f);
	// 			} else {
	// 				return trace<false>(node, t0, t1, a, ray, predicate, binary_f);
	// 			}
	// 		}
	// 	}

	// 	template <class InputIt, class OutputIt, class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true> 	OutputIt trace(Node node, InputIt first, InputIt last, OutputIt d_first,
	// 	               Predicate const& predicate, bool only_exists = true) const
	// 	{
	// 		return trace(
	// 		    node, first, last, d_first, predicate, [](Node, Ray) -> void {}, only_exists);
	// 	}

	// 	template <class InputIt, class OutputIt, class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true> 	OutputIt trace(Node node, InputIt first, InputIt last, OutputIt d_first,
	// 	               Predicate const& predicate, BinaryFun binary_f,
	// 	               bool only_exists = true) const
	// 	{
	// 		pred::Init<Predicate>::apply(predicate, *this);

	// 		for (; first != last; ++first) {
	// 			Ray ray = *first;

	// 			auto [t0, t1, a] = traceInit(node, ray);

	// 			if constexpr (pred::contains_spatial_predicate_v<std::decay_t<Predicate>>) {
	// 				if (only_exists) {
	// 					*d_first++ =
	// 					    trace<true>(Base::toNodeBV(node), t0, t1, a, ray, predicate, binary_f);
	// 				} else {
	// 					*d_first++ =
	// 					    trace<false>(Base::toNodeBV(node), t0, t1, a, ray, predicate, binary_f);
	// 				}
	// 			} else {
	// 				if (only_exists) {
	// 					*d_first++ = trace<true>(node, t0, t1, a, ray, predicate, binary_f);
	// 				} else {
	// 					*d_first++ = trace<false>(node, t0, t1, a, ray, predicate, binary_f);
	// 				}
	// 			}
	// 		}
	// 		return d_first;
	// 	}

	// 	template <class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true> 	Node trace(Code code, Ray ray, Predicate const& predicate, 	           bool
	// only_exists = true) const
	// 	{
	// 		return trace(Base::operator()(code), ray, predicate, only_exists);
	// 	}

	// 	template <class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true> 	auto trace(Code code, Ray ray, Predicate const& predicate, BinaryFun binary_f,
	// 	           bool only_exists = true) const
	// 	{
	// 		return trace(Base::operator()(code), ray, predicate, binary_f, only_exists);
	// 	}

	// 	template <class InputIt, class OutputIt, class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true> 	OutputIt trace(Code code, InputIt first, InputIt last, OutputIt d_first,
	// 	               Predicate const& predicate, bool only_exists = true) const
	// 	{
	// 		return trace(Base::operator()(code), first, last, d_first, predicate,
	// only_exists);
	// 	}

	// 	template <class InputIt, class OutputIt, class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true> 	OutputIt trace(Code code, InputIt first, InputIt last, OutputIt d_first,
	// 	               Predicate const& predicate, BinaryFun binary_f,
	// 	               bool only_exists = true) const
	// 	{
	// 		return trace(Base::operator()(code), first, last, d_first, predicate, binary_f,
	// 		             only_exists);
	// 	}

	// 	template <class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true> 	Node trace(Key key, Ray ray, Predicate const& predicate, bool only_exists =
	// true) const
	// 	{
	// 		return trace(Base::operator()(key), ray, predicate, only_exists);
	// 	}

	// 	template <class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true> 	auto trace(Key key, Ray ray, Predicate const& predicate, BinaryFun binary_f,
	// 	           bool only_exists = true) const
	// 	{
	// 		return trace(Base::operator()(key), ray, predicate, binary_f, only_exists);
	// 	}

	// 	template <class InputIt, class OutputIt, class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true> 	OutputIt trace(Key key, InputIt first, InputIt last, OutputIt d_first,
	// 	               Predicate const& predicate, bool only_exists = true) const
	// 	{
	// 		return trace(Base::operator()(key), first, last, d_first, predicate, only_exists);
	// 	}

	// 	template <class InputIt, class OutputIt, class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true> 	OutputIt trace(Key key, InputIt first, InputIt last, OutputIt d_first,
	// 	               Predicate const& predicate, BinaryFun binary_f,
	// 	               bool only_exists = true) const
	// 	{
	// 		return trace(Base::operator()(key), first, last, d_first, predicate, binary_f,
	// 		             only_exists);
	// 	}

	// 	template <class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true> 	Node trace(Coord coord, Ray ray, Predicate const& predicate, bool only_exists
	// = true, 	           depth_t depth = 0) const
	// 	{
	// 		return trace(Base::operator()(coord, depth), ray, predicate, only_exists);
	// 	}

	// 	template <class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true> 	auto trace(Coord coord, Ray ray, Predicate const& predicate, BinaryFun
	// binary_f, 	           bool only_exists = true, depth_t depth = 0) const
	// 	{
	// 		return trace(Base::operator()(coord, depth), ray, predicate, binary_f,
	// only_exists);
	// 	}

	// 	template <class InputIt, class OutputIt, class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true> 	OutputIt trace(Coord coord, InputIt first, InputIt last, OutputIt d_first,
	// 	               Predicate const& predicate, bool only_exists = true,
	// 	               depth_t depth = 0) const
	// 	{
	// 		return trace(Base::operator()(coord, depth), first, last, d_first, predicate,
	// 		             only_exists);
	// 	}

	// 	template <class InputIt, class OutputIt, class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true> 	OutputIt trace(Coord coord, InputIt first, InputIt last, OutputIt d_first,
	// 	               Predicate const& predicate, BinaryFun binary_f, bool only_exists =
	// true, 	               depth_t depth = 0) const
	// 	{
	// 		return trace(Base::operator()(coord, depth), first, last, d_first, predicate,
	// 		             binary_f, only_exists);
	// 	}

	// #ifdef UFO_TBB
	// 	template <class ExecutionPolicy, class ForwardIt1, class ForwardIt2, class
	// Predicate, 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree,
	// Node>, bool> = true,
	// std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> =
	// true> 	ForwardIt2 trace(ExecutionPolicy&& policy, ForwardIt1 first, ForwardIt1 last,
	// 	                 ForwardIt2 d_first, Predicate const& predicate,
	// 	                 bool only_exists = true) const
	// 	{
	// 		return trace(std::forward<ExecutionPolicy>(policy), Base::node(), first, last,
	// 		             d_first, predicate, only_exists);
	// 	}

	// 	template <class ExecutionPolicy, class ForwardIt1, class ForwardIt2, class
	// Predicate, 	          class BinaryFun,
	// std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> = true,
	// std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool>     = true,
	// 	          std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	ForwardIt2 trace(ExecutionPolicy&& policy, ForwardIt1 first, ForwardIt1 last,
	// 	                 ForwardIt2 d_first, Predicate const& predicate, BinaryFun binary_f,
	// 	                 bool only_exists = true) const
	// 	{
	// 		return trace(std::forward<ExecutionPolicy>(policy), Base::node(), first, last,
	// 		             d_first, predicate, binary_f, only_exists);
	// 	}

	// 	template <class ExecutionPolicy, class ForwardIt1, class ForwardIt2, class
	// Predicate, 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree,
	// Node>, bool> = true,
	// std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> =
	// true> 	ForwardIt2 trace(ExecutionPolicy&& policy, Node node, ForwardIt1 first,
	// ForwardIt1 last, 	                 ForwardIt2 d_first, Predicate const& predicate,
	// bool only_exists = true) const
	// 	{
	// 		return trace(
	// 		    std::forward<ExecutionPolicy>(policy), node, first, last, d_first, predicate,
	// 		    [](Node, Ray) -> void {}, only_exists);
	// 	}

	// 	template <class ExecutionPolicy, class ForwardIt1, class ForwardIt2, class
	// Predicate, 	          class BinaryFun,
	// std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> = true,
	// std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool>     = true,
	// 	          std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	ForwardIt2 trace(ExecutionPolicy&& policy, Node node, ForwardIt1 first,
	// ForwardIt1 last, 	                 ForwardIt2 d_first, Predicate const& predicate,
	// BinaryFun binary_f, 	                 bool only_exists = true) const
	// 	{
	// 		pred::Init<Predicate>::apply(predicate, *this);

	// 		return std::transform(
	// 		    std::forward<ExecutionPolicy>(policy), first, last, d_first,
	// 		    [this, node, &predicate, binary_f, only_exists](auto ray) {
	// 			    auto [t0, t1, a] = traceInit(node, ray);

	// 			    if constexpr (pred::contains_spatial_predicate_v<std::decay_t<Predicate>>) {
	// 				    if (only_exists) {
	// 					    return trace<true>(Base::toNodeBV(node), t0, t1, a, ray, predicate,
	// 					                       binary_f);
	// 				    } else {
	// 					    return trace<false>(Base::toNodeBV(node), t0, t1, a, ray, predicate,
	// 					                        binary_f);
	// 				    }
	// 			    } else {
	// 				    if (only_exists) {
	// 					    return trace<true>(node, t0, t1, a, ray, predicate, binary_f);
	// 				    } else {
	// 					    return trace<false>(node, t0, t1, a, ray, predicate, binary_f);
	// 				    }
	// 			    }
	// 		    });
	// 	}

	// 	template <class ExecutionPolicy, class ForwardIt1, class ForwardIt2, class
	// Predicate, 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree,
	// Node>, bool> = true,
	// std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> =
	// true> 	ForwardIt2 trace(ExecutionPolicy&& policy, Code code, ForwardIt1 first,
	// ForwardIt1 last, 	                 ForwardIt2 d_first, Predicate const& predicate,
	// bool only_exists = true) const
	// 	{
	// 		return trace(std::forward<ExecutionPolicy>(policy), Base::operator()(code), first,
	// 		             last, d_first, predicate, only_exists);
	// 	}

	// 	template <class ExecutionPolicy, class ForwardIt1, class ForwardIt2, class
	// Predicate, 	          class BinaryFun,
	// std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> = true,
	// std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool>     = true,
	// 	          std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	ForwardIt2 trace(ExecutionPolicy&& policy, Code code, ForwardIt1 first,
	// ForwardIt1 last, 	                 ForwardIt2 d_first, Predicate const& predicate,
	// BinaryFun binary_f, 	                 bool only_exists = true) const
	// 	{
	// 		return trace(std::forward<ExecutionPolicy>(policy), Base::operator()(code), first,
	// 		             last, d_first, predicate, binary_f, only_exists);
	// 	}

	// 	template <class ExecutionPolicy, class ForwardIt1, class ForwardIt2, class
	// Predicate, 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree,
	// Node>, bool> = true,
	// std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> =
	// true> 	ForwardIt2 trace(ExecutionPolicy&& policy, Key key, ForwardIt1 first,
	// ForwardIt1 last, 	                 ForwardIt2 d_first, Predicate const& predicate,
	// bool only_exists = true) const
	// 	{
	// 		return trace(std::forward<ExecutionPolicy>(policy), Base::operator()(key), first,
	// 		             last, d_first, predicate, only_exists);
	// 	}

	// 	template <class ExecutionPolicy, class ForwardIt1, class ForwardIt2, class
	// Predicate, 	          class BinaryFun,
	// std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> = true,
	// std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool>     = true,
	// 	          std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	ForwardIt2 trace(ExecutionPolicy&& policy, Key key, ForwardIt1 first,
	// ForwardIt1 last, 	                 ForwardIt2 d_first, Predicate const& predicate,
	// BinaryFun binary_f, 	                 bool only_exists = true) const
	// 	{
	// 		return trace(std::forward<ExecutionPolicy>(policy), Base::operator()(key), first,
	// 		             last, d_first, predicate, binary_f, only_exists);
	// 	}

	// 	template <class ExecutionPolicy, class ForwardIt1, class ForwardIt2, class
	// Predicate, 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree,
	// Node>, bool> = true,
	// std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> =
	// true> 	ForwardIt2 trace(ExecutionPolicy&& policy, Coord coord, ForwardIt1 first,
	// 	                 ForwardIt1 last, ForwardIt2 d_first, Predicate const& predicate,
	// 	                 bool only_exists = true, depth_t depth = 0) const
	// 	{
	// 		return trace(std::forward<ExecutionPolicy>(policy), Base::operator()(coord,
	// depth), 		             first, last, d_first, predicate, only_exists);
	// 	}

	// 	template <class ExecutionPolicy, class ForwardIt1, class ForwardIt2, class
	// Predicate, 	          class BinaryFun,
	// std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> = true,
	// std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool>     = true,
	// 	          std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	ForwardIt2 trace(ExecutionPolicy&& policy, Coord coord, ForwardIt1 first,
	// 	                 ForwardIt1 last, ForwardIt2 d_first, Predicate const& predicate,
	// 	                 BinaryFun binary_f, bool only_exists = true, depth_t depth = 0)
	// const
	// 	{
	// 		return trace(std::forward<ExecutionPolicy>(policy), Base::operator()(coord,
	// depth), 		             first, last, d_first, predicate, binary_f, only_exists);
	// 	}
	// #endif

	// 	/**************************************************************************************
	// 	| | 	|                                     Trace index | 	| |
	// 	**************************************************************************************/

	// 	template <class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true> 	auto traceIndex(Ray ray, InnerBinaryFun inner_f, RetBinaryFun ret_f)
	// const
	// 	{
	// 		return traceIndex(Base::node(), ray, inner_f, ret_f);
	// 	}

	// 	template <class InputIt, class OutputIt, class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true> 	OutputIt traceIndex(InputIt first, InputIt last, OutputIt d_first,
	// 	                    InnerBinaryFun inner_f, RetBinaryFun ret_f) const
	// 	{
	// 		return traceIndex(Base::node(), first, last, d_first, inner_f, ret_f);
	// 	}

	// 	template <class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true> 	auto traceIndex(Node node, Ray ray, InnerBinaryFun inner_f,
	// RetBinaryFun ret_f) const
	// 	{
	// 		// TODO: Do we need to check node first?

	// 		auto [t0, t1, a] = traceInit(node, ray);
	// 		return traceIndex(node.index(), t0, t1, a, ray, inner_f, ret_f);
	// 	}

	// 	template <class InputIt, class OutputIt, class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true> 	OutputIt traceIndex(Node node, InputIt first, InputIt last, OutputIt
	// d_first, 	                    InnerBinaryFun inner_f, RetBinaryFun ret_f) const
	// 	{
	// 		// TODO: Do we need to check node first?

	// 		for (; first != last; ++first) {
	// 			Ray ray          = *first;
	// 			auto [t0, t1, a] = traceInit(node, ray);
	// 			*d_first++       = traceIndex(node.index(), t0, t1, a, ray, inner_f, ret_f);
	// 		}
	// 		return d_first;
	// 	}

	// 	template <class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true> 	auto traceIndex(Code code, Ray ray, InnerBinaryFun inner_f,
	// RetBinaryFun ret_f) const
	// 	{
	// 		return traceIndex(Base::operator()(code), ray, inner_f, ret_f);
	// 	}

	// 	template <class InputIt, class OutputIt, class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true> 	OutputIt traceIndex(Code code, InputIt first, InputIt last, OutputIt
	// d_first, 	                    InnerBinaryFun inner_f, RetBinaryFun ret_f) const
	// 	{
	// 		return traceIndex(Base::operator()(code), first, last, d_first, inner_f, ret_f);
	// 	}

	// 	template <class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true> 	auto traceIndex(Key key, Ray ray, InnerBinaryFun inner_f, RetBinaryFun
	// ret_f) const
	// 	{
	// 		return traceIndex(Base::operator()(key), ray, inner_f, ret_f);
	// 	}

	// 	template <class InputIt, class OutputIt, class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true> 	OutputIt traceIndex(Key key, InputIt first, InputIt last, OutputIt
	// d_first, 	                    InnerBinaryFun inner_f, RetBinaryFun ret_f) const
	// 	{
	// 		return traceIndex(Base::operator()(key), first, last, d_first, inner_f, ret_f);
	// 	}

	// 	template <class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true> 	auto traceIndex(Coord coord, Ray ray, InnerBinaryFun inner_f,
	// RetBinaryFun ret_f, 	                depth_t depth = 0) const
	// 	{
	// 		return traceIndex(Base::operator()(coord, depth), ray, inner_f, ret_f);
	// 	}

	// 	template <class InputIt, class OutputIt, class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true> 	OutputIt traceIndex(Coord coord, InputIt first, InputIt last, OutputIt
	// d_first, 	                    InnerBinaryFun inner_f, RetBinaryFun ret_f, depth_t
	// depth = 0) const
	// 	{
	// 		return traceIndex(Base::operator()(coord, depth), first, last, d_first, inner_f,
	// 		                  ret_f);
	// 	}

	// #ifdef UFO_TBB

	// 	template <class ExecutionPolicy, class ForwardIt1, class ForwardIt2,
	// 	          class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true,
	// 	          std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	ForwardIt2 traceIndex(ExecutionPolicy&& policy, ForwardIt1 first, ForwardIt1
	// last, 	                      ForwardIt2 d_first, InnerBinaryFun inner_f,
	// RetBinaryFun ret_f) const
	// 	{
	// 		return traceIndex(std::forward<ExecutionPolicy>(policy), Base::node(), first,
	// last, 		                  d_first, inner_f, ret_f);
	// 	}

	// 	template <class ExecutionPolicy, class ForwardIt1, class ForwardIt2,
	// 	          class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true,
	// 	          std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	ForwardIt2 traceIndex(ExecutionPolicy&& policy, Node node, ForwardIt1 first,
	// 	                      ForwardIt1 last, ForwardIt2 d_first, InnerBinaryFun inner_f,
	// 	                      RetBinaryFun ret_f) const
	// 	{
	// 		// TODO: Do we need to check node first?

	// 		return std::transform(std::forward<ExecutionPolicy>(policy), first, last, d_first,
	// 		                      [this, node, inner_f, ret_f](auto ray) {
	// 			                      auto [t0, t1, a] = traceInit(node, ray);
	// 			                      return traceIndex(node.index(), t0, t1, a, ray, inner_f,
	// 			                                        ret_f);
	// 		                      });
	// 	}

	// 	template <class ExecutionPolicy, class ForwardIt1, class ForwardIt2,
	// 	          class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true,
	// 	          std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	ForwardIt2 traceIndex(ExecutionPolicy&& policy, Code code, ForwardIt1 first,
	// 	                      ForwardIt1 last, ForwardIt2 d_first, InnerBinaryFun inner_f,
	// 	                      RetBinaryFun ret_f) const
	// 	{
	// 		return traceIndex(std::forward<ExecutionPolicy>(policy), Base::operator()(code),
	// 		                  first, last, d_first, inner_f, ret_f);
	// 	}

	// 	template <class ExecutionPolicy, class ForwardIt1, class ForwardIt2,
	// 	          class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true,
	// 	          std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	ForwardIt2 traceIndex(ExecutionPolicy&& policy, Key key, ForwardIt1 first,
	// 	                      ForwardIt1 last, ForwardIt2 d_first, InnerBinaryFun inner_f,
	// 	                      RetBinaryFun ret_f) const
	// 	{
	// 		return traceIndex(std::forward<ExecutionPolicy>(policy), Base::operator()(key),
	// first, 		                  last, d_first, inner_f, ret_f);
	// 	}

	// 	template <class ExecutionPolicy, class ForwardIt1, class ForwardIt2,
	// 	          class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true,
	// 	          std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	ForwardIt2 traceIndex(ExecutionPolicy&& policy, Coord coord, ForwardIt1 first,
	// 	                      ForwardIt1 last, ForwardIt2 d_first, InnerBinaryFun inner_f,
	// 	                      RetBinaryFun ret_f, depth_t depth = 0) const
	// 	{
	// 		return traceIndex(std::forward<ExecutionPolicy>(policy),
	// 		                  Base::operator()(coord, depth), first, last, d_first, inner_f,
	// 		                  ret_f);
	// 	}
	// #endif

	// 	/**************************************************************************************
	// 	| | 	|                                        Image | 	| |
	// 	**************************************************************************************/

	// 	template <class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true> 	Image<Node> image(Predicate const& predicate, Pose6f const& pose, std::size_t
	// rows, 	                  std::size_t cols, float vertical_fov, float near_clip, float
	// far_clip, 	                  Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0}, Vec3f
	// forward = {1, 0, 0}, bool only_exists = true) const
	// 	{
	// 		return image(Base::node(), predicate, pose, rows, cols, vertical_fov, near_clip,
	// 		             far_clip, up, right, forward, only_exists);
	// 	}

	// 	template <class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true> 	auto image(Predicate const& predicate, BinaryFun binary_f, Pose6f const& pose,
	// 	           std::size_t rows, std::size_t cols, float vertical_fov, float near_clip,
	// 	           float far_clip, Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0},
	// 	           Vec3f forward = {1, 0, 0}, bool only_exists = true) const
	// 	{
	// 		return image(Base::node(), predicate, binary_f, pose, rows, cols, vertical_fov,
	// 		             near_clip, far_clip, up, right, forward, only_exists);
	// 	}

	// 	template <class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true> 	Image<Node> image(Node node, Predicate const& predicate, Pose6f const& pose,
	// 	                  std::size_t rows, std::size_t cols, float vertical_fov,
	// 	                  float near_clip, float far_clip, Vec3f up = {0, 0, 1},
	// 	                  Vec3f right = {0, -1, 0}, Vec3f forward = {1, 0, 0},
	// 	                  bool only_exists = true) const
	// 	{
	// 		return image(
	// 		    node, predicate, [](Node, Ray) -> void {}, pose, rows, cols, vertical_fov,
	// 		    near_clip, far_clip, up, right, forward, only_exists);
	// 	}

	// 	template <class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true> 	auto image(Node node, Predicate const& predicate, BinaryFun binary_f, Pose6f
	// const& pose, std::size_t rows, std::size_t cols, float vertical_fov,
	// float near_clip, float far_clip, Vec3f up = {0, 0, 1}, 	           Vec3f right = {0,
	// -1, 0}, Vec3f forward = {1, 0, 0}, 	           bool only_exists = true) const
	// 	{
	// 		float aspect_ratio = cols / static_cast<float>(rows);

	// 		float e = far_clip * std::tan(vertical_fov / 2);

	// 		up *= e;
	// 		right *= e * aspect_ratio;
	// 		forward *= far_clip;

	// 		Vec3f tl = up - right + forward;
	// 		Vec3f tr = up + right + forward;
	// 		Vec3f bl = -up - right + forward;
	// 		pose.transformInPlace(tl);
	// 		pose.transformInPlace(tr);
	// 		pose.transformInPlace(bl);
	// 		Vec3f right_dir  = tr - tl;
	// 		Vec3f bottom_dir = bl - tl;

	// 		Image<Ray> rays(rows, cols, Ray(pose.translation, Coord()));
	// 		for (std::size_t row{}; row < rows; ++row) {
	// 			auto r = (row + 0.5f) / rows;
	// 			for (std::size_t col{}; col < cols; ++col) {
	// 				auto c                   = (col + 0.5f) / cols;
	// 				auto end                 = tl + right_dir * c + bottom_dir * r;
	// 				rays(row, col).direction = (end - pose.translation).normalized();
	// 			}
	// 		};

	// 		if constexpr (std::is_void_v<std::invoke_result_t<BinaryFun, Node, Ray>> ||
	// 		              std::is_same_v<std::invoke_result_t<BinaryFun, Node, Ray>, bool>) {
	// 			Image<Node> image(rows, cols);

	// 			trace(node, std::cbegin(rays), std::cend(rays), std::begin(image), predicate,
	// 			      binary_f, only_exists);

	// 			return image;
	// 		} else {
	// 			Image<std::pair<Node,
	// 			                typename std::invoke_result_t<BinaryFun, Node,
	// Ray>::second_type>> 			    image(rows, cols);

	// 			trace(node, std::cbegin(rays), std::cend(rays), std::begin(image), predicate,
	// 			      binary_f, only_exists);

	// 			return image;
	// 		}
	// 	}

	// 	template <class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true> 	Image<Node> image(Code code, Predicate const& predicate, Pose6f const& pose,
	// 	                  std::size_t rows, std::size_t cols, float vertical_fov,
	// 	                  float near_clip, float far_clip, Vec3f up = {0, 0, 1},
	// 	                  Vec3f right = {0, -1, 0}, Vec3f forward = {1, 0, 0},
	// 	                  bool only_exists = true) const
	// 	{
	// 		return image(Base::operator()(code), predicate, pose, rows, cols, vertical_fov,
	// 		             near_clip, far_clip, up, right, forward, only_exists);
	// 	}

	// 	template <class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true> 	auto image(Code code, Predicate const& predicate, BinaryFun binary_f, Pose6f
	// const& pose, std::size_t rows, std::size_t cols, float vertical_fov,
	// float near_clip, float far_clip, Vec3f up = {0, 0, 1}, 	           Vec3f right = {0,
	// -1, 0}, Vec3f forward = {1, 0, 0}, 	           bool only_exists = true) const
	// 	{
	// 		return image(Base::operator()(code), predicate, binary_f, pose, rows, cols,
	// 		             vertical_fov, near_clip, far_clip, up, right, forward, only_exists);
	// 	}

	// 	template <class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true> 	Image<Node> image(Key key, Predicate const& predicate, Pose6f const& pose,
	// 	                  std::size_t rows, std::size_t cols, float vertical_fov,
	// 	                  float near_clip, float far_clip, Vec3f up = {0, 0, 1},
	// 	                  Vec3f right = {0, -1, 0}, Vec3f forward = {1, 0, 0},
	// 	                  bool only_exists = true) const
	// 	{
	// 		return image(Base::operator()(key), predicate, pose, rows, cols, vertical_fov,
	// 		             near_clip, far_clip, up, right, forward, only_exists);
	// 	}

	// 	template <class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true> 	auto image(Key key, Predicate const& predicate, BinaryFun binary_f, Pose6f
	// const& pose, 	           std::size_t rows, std::size_t cols, float vertical_fov,
	// float near_clip, 	           float far_clip, Vec3f up = {0, 0, 1}, Vec3f right = {0,
	// -1, 0}, 	           Vec3f forward = {1, 0, 0}, bool only_exists = true) const
	// 	{
	// 		return image(Base::operator()(key), predicate, binary_f, pose, rows, cols,
	// 		             vertical_fov, near_clip, far_clip, up, right, forward, only_exists);
	// 	}

	// 	template <class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true> 	Image<Node> image(Coord coord, Predicate const& predicate, Pose6f const& pose,
	// 	                  std::size_t rows, std::size_t cols, float vertical_fov,
	// 	                  float near_clip, float far_clip, Vec3f up = {0, 0, 1},
	// 	                  Vec3f right = {0, -1, 0}, Vec3f forward = {1, 0, 0},
	// 	                  bool only_exists = true, depth_t depth = 0) const
	// 	{
	// 		return image(Base::operator()(coord, depth), predicate, pose, rows, cols,
	// 		             vertical_fov, near_clip, far_clip, up, right, forward, only_exists);
	// 	}

	// 	template <class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true> 	auto image(Coord coord, Predicate const& predicate, BinaryFun binary_f,
	// Pose6f const& pose, std::size_t rows, std::size_t cols, float vertical_fov, float
	// near_clip, float far_clip, Vec3f up = {0, 0, 1}, 	           Vec3f right = {0, -1,
	// 0}, Vec3f forward = {1, 0, 0}, bool only_exists = true, 	           depth_t depth =
	// 0) const
	// 	{
	// 		return image(Base::operator()(coord, depth), predicate, binary_f, pose, rows,
	// cols, 		             vertical_fov, near_clip, far_clip, up, right, forward,
	// only_exists);
	// 	}

	// #ifdef UFO_TBB
	// 	template <class ExecutionPolicy, class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	Image<Node> image(ExecutionPolicy&& policy, Predicate const& predicate,
	// Pose6f const& pose, std::size_t rows, std::size_t cols, 	                  float
	// vertical_fov, float near_clip, float far_clip, 	                  Vec3f up = {0, 0,
	// 1}, Vec3f right = {0, -1, 0}, 	                  Vec3f forward = {1, 0, 0}, bool
	// only_exists = true) const
	// 	{
	// 		return image(std::forward<ExecutionPolicy>(policy), Base::node(), predicate, pose,
	// 		             rows, cols, vertical_fov, near_clip, far_clip, up, right, forward,
	// 		             only_exists);
	// 	}
	// 	template <class ExecutionPolicy, class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true, std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	auto image(ExecutionPolicy&& policy, Predicate const& predicate, BinaryFun
	// binary_f, 	           Pose6f const& pose, std::size_t rows, std::size_t cols, float
	// vertical_fov, 	           float near_clip, float far_clip, Vec3f up = {0, 0, 1},
	// Vec3f right = {0, -1, 0}, Vec3f forward = {1, 0, 0}, 	           bool only_exists =
	// true) const
	// 	{
	// 		return image(std::forward<ExecutionPolicy>(policy), Base::node(), predicate,
	// binary_f, 		             pose, rows, cols, vertical_fov, near_clip, far_clip, up,
	// right, forward, 		             only_exists);
	// 	}

	// 	template <class ExecutionPolicy, class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	Image<Node> image(ExecutionPolicy&& policy, Node node, Predicate const&
	// predicate, 	                  Pose6f const& pose, std::size_t rows, std::size_t
	// cols, 	                  float vertical_fov, float near_clip, float far_clip, Vec3f
	// up = {0, 0, 1}, Vec3f right = {0, -1, 0}, 	                  Vec3f forward = {1, 0,
	// 0}, bool only_exists = true) const
	// 	{
	// 		return image(
	// 		    std::forward<ExecutionPolicy>(policy), node, predicate, [](Node, Ray) -> void
	// {}, 		    pose, rows, cols, vertical_fov, near_clip, far_clip, up, right, forward,
	// 		    only_exists);
	// 	}

	// 	template <class ExecutionPolicy, class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true, std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	auto image(ExecutionPolicy&& policy, Node node, Predicate const& predicate,
	// 	           BinaryFun binary_f, Pose6f const& pose, std::size_t rows, std::size_t
	// cols, 	           float vertical_fov, float near_clip, float far_clip, Vec3f up = {0,
	// 0, 1}, 	           Vec3f right = {0, -1, 0}, Vec3f forward = {1, 0, 0}, bool
	// only_exists = true) const
	// 	{
	// 		float aspect_ratio = cols / static_cast<float>(rows);

	// 		float e = far_clip * std::tan(vertical_fov / 2);

	// 		up *= e;
	// 		right *= e * aspect_ratio;
	// 		forward *= far_clip;

	// 		Vec3f tl = up - right + forward;
	// 		Vec3f tr = up + right + forward;
	// 		Vec3f bl = -up - right + forward;
	// 		pose.transformInPlace(tl);
	// 		pose.transformInPlace(tr);
	// 		pose.transformInPlace(bl);
	// 		Vec3f right_dir  = tr - tl;
	// 		Vec3f bottom_dir = bl - tl;

	// 		Image<Ray> rays(rows, cols, Ray(pose.translation, Coord()));

	// 		std::vector<std::size_t> indices(rows);
	// 		std::iota(std::begin(indices), std::end(indices), 0);

	// 		std::for_each(policy, std::begin(indices), std::end(indices),
	// 		              [tl, right_dir, bottom_dir, rows, cols, &rays](auto row) {
	// 			              auto r = (row + 0.5f) / rows;
	// 			              auto a = tl + bottom_dir * r;
	// 			              for (std::size_t col{}; col < cols; ++col) {
	// 				              auto c   = (col + 0.5f) / cols;
	// 				              auto end = a + right_dir * c;
	// 				              rays(row, col).direction =
	// 				                  (end - rays(row, col).origin).normalized();
	// 			              }
	// 		              });

	// 		if constexpr (std::is_void_v<std::invoke_result_t<BinaryFun, Node, Ray>> ||
	// 		              std::is_same_v<std::invoke_result_t<BinaryFun, Node, Ray>, bool>) {
	// 			Image<Node> image(rows, cols);

	// 			trace(std::forward<ExecutionPolicy>(policy), node, std::cbegin(rays),
	// 			      std::cend(rays), std::begin(image), predicate, binary_f, only_exists);

	// 			return image;
	// 		} else {
	// 			Image<std::pair<Node,
	// 			                typename std::invoke_result_t<BinaryFun, Node,
	// Ray>::second_type>> 			    image(rows, cols);

	// 			trace(std::forward<ExecutionPolicy>(policy), node, std::cbegin(rays),
	// 			      std::cend(rays), std::begin(image), predicate, binary_f, only_exists);

	// 			return image;
	// 		}
	// 	}

	// 	template <class ExecutionPolicy, class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	Image<Node> image(ExecutionPolicy&& policy, Code code, Predicate const&
	// predicate, 	                  Pose6f const& pose, std::size_t rows, std::size_t
	// cols, 	                  float vertical_fov, float near_clip, float far_clip, Vec3f
	// up = {0, 0, 1}, Vec3f right = {0, -1, 0}, 	                  Vec3f forward = {1, 0,
	// 0}, bool only_exists = true) const
	// 	{
	// 		return image(std::forward<ExecutionPolicy>(policy), Base::operator()(code),
	// predicate, 		             pose, rows, cols, vertical_fov, near_clip, far_clip, up,
	// right, forward, 		             only_exists);
	// 	}

	// 	template <class ExecutionPolicy, class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true, std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	auto image(ExecutionPolicy&& policy, Code code, Predicate const& predicate,
	// 	           BinaryFun binary_f, Pose6f const& pose, std::size_t rows, std::size_t
	// cols, 	           float vertical_fov, float near_clip, float far_clip, Vec3f up = {0,
	// 0, 1}, 	           Vec3f right = {0, -1, 0}, Vec3f forward = {1, 0, 0}, bool
	// only_exists = true) const
	// 	{
	// 		return image(std::forward<ExecutionPolicy>(policy), Base::operator()(code),
	// predicate, 		             binary_f, pose, rows, cols, vertical_fov, near_clip,
	// far_clip, up, right, 		             forward, only_exists);
	// 	}

	// 	template <class ExecutionPolicy, class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	Image<Node> image(ExecutionPolicy&& policy, Key key, Predicate const&
	// predicate, 	                  Pose6f const& pose, std::size_t rows, std::size_t
	// cols, 	                  float vertical_fov, float near_clip, float far_clip, Vec3f
	// up = {0, 0, 1}, Vec3f right = {0, -1, 0}, 	                  Vec3f forward = {1, 0,
	// 0}, bool only_exists = true) const
	// 	{
	// 		return image(std::forward<ExecutionPolicy>(policy), Base::operator()(key),
	// predicate, 		             pose, rows, cols, vertical_fov, near_clip, far_clip, up,
	// right, forward, 		             only_exists);
	// 	}

	// 	template <class ExecutionPolicy, class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true, std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	auto image(ExecutionPolicy&& policy, Key key, Predicate const& predicate,
	// 	           BinaryFun binary_f, Pose6f const& pose, std::size_t rows, std::size_t
	// cols, 	           float vertical_fov, float near_clip, float far_clip, Vec3f up = {0,
	// 0, 1}, 	           Vec3f right = {0, -1, 0}, Vec3f forward = {1, 0, 0}, bool
	// only_exists = true) const
	// 	{
	// 		return image(std::forward<ExecutionPolicy>(policy), Base::operator()(key),
	// predicate, 		             binary_f, pose, rows, cols, vertical_fov, near_clip,
	// far_clip, up, right, 		             forward, only_exists);
	// 	}

	// 	template <class ExecutionPolicy, class Predicate,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	Image<Node> image(ExecutionPolicy&& policy, Coord coord, Predicate const&
	// predicate, 	                  Pose6f const& pose, std::size_t rows, std::size_t
	// cols, 	                  float vertical_fov, float near_clip, float far_clip, Vec3f
	// up = {0, 0, 1}, Vec3f right = {0, -1, 0}, 	                  Vec3f forward = {1, 0,
	// 0}, bool only_exists = true, 	                  depth_t depth = 0) const
	// 	{
	// 		return image(std::forward<ExecutionPolicy>(policy), Base::operator()(coord,
	// depth), 		             predicate, pose, rows, cols, vertical_fov, near_clip,
	// far_clip, up, 		             right, forward, only_exists);
	// 	}

	// 	template <class ExecutionPolicy, class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true, std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	auto image(ExecutionPolicy&& policy, Coord coord, Predicate const& predicate,
	// 	           BinaryFun binary_f, Pose6f const& pose, std::size_t rows, std::size_t
	// cols, 	           float vertical_fov, float near_clip, float far_clip, Vec3f up = {0,
	// 0, 1}, 	           Vec3f right = {0, -1, 0}, Vec3f forward = {1, 0, 0}, bool
	// only_exists = true, depth_t depth = 0) const
	// 	{
	// 		return image(std::forward<ExecutionPolicy>(policy), Base::operator()(coord,
	// depth), 		             predicate, binary_f, pose, rows, cols, vertical_fov,
	// near_clip, far_clip, 		             up, right, forward, only_exists);
	// 	}
	// #endif

	// 	/**************************************************************************************
	// 	| | 	|                                     Image index | 	| |
	// 	**************************************************************************************/

	// 	template <class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true> 	auto imageIndex(InnerBinaryFun inner_f, RetBinaryFun ret_f, Pose6f
	// const& pose, 	                std::size_t rows, std::size_t cols, float
	// vertical_fov, float near_clip, float far_clip, Vec3f up = {0, 0, 1}, Vec3f right =
	// {0, -1, 0}, Vec3f forward = {1, 0, 0}) const
	// 	{
	// 		return imageIndex(Base::node(), inner_f, ret_f, pose, rows, cols, vertical_fov,
	// 		                  near_clip, far_clip, up, right, forward);
	// 	}

	// 	template <class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true> 	auto imageIndex(Node node, InnerBinaryFun inner_f, RetBinaryFun ret_f,
	// 	                Pose6f const& pose, std::size_t rows, std::size_t cols,
	// 	                float vertical_fov, float near_clip, float far_clip,
	// 	                Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0},
	// 	                Vec3f forward = {1, 0, 0}) const
	// 	{
	// 		float aspect_ratio = cols / static_cast<float>(rows);

	// 		float e = far_clip * std::tan(vertical_fov / 2);

	// 		up *= e;
	// 		right *= e * aspect_ratio;
	// 		forward *= far_clip;

	// 		Vec3f tl = up - right + forward;
	// 		Vec3f tr = up + right + forward;
	// 		Vec3f bl = -up - right + forward;
	// 		pose.transformInPlace(tl);
	// 		pose.transformInPlace(tr);
	// 		pose.transformInPlace(bl);
	// 		Vec3f right_dir  = tr - tl;
	// 		Vec3f bottom_dir = bl - tl;

	// 		Image<Ray> rays(rows, cols, Ray(pose.translation, Coord()));
	// 		for (std::size_t row{}; row < rows; ++row) {
	// 			auto r = (row + 0.5f) / rows;
	// 			for (std::size_t col{}; col < cols; ++col) {
	// 				auto c                   = (col + 0.5f) / cols;
	// 				auto end                 = tl + right_dir * c + bottom_dir * r;
	// 				rays(row, col).direction = (end - pose.translation).normalized();
	// 			}
	// 		};

	// 		if constexpr (std::is_same_v<std::invoke_result_t<RetBinaryFun, Index, Ray>,
	// bool>) { 			Image<Index> image(rows, cols);

	// 			traceIndex(node, std::cbegin(rays), std::cend(rays), std::begin(image), inner_f,
	// 			           ret_f);

	// 			return image;
	// 		} else {
	// 			Image<std::pair<
	// 			    Index, typename std::invoke_result_t<RetBinaryFun, Index,
	// Ray>::second_type>> 			    image(rows, cols);

	// 			traceIndex(node, std::cbegin(rays), std::cend(rays), std::begin(image), inner_f,
	// 			           ret_f);

	// 			return image;
	// 		}
	// 	}

	// 	template <class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true> 	auto imageIndex(Code code, InnerBinaryFun inner_f, RetBinaryFun ret_f,
	// 	                Pose6f const& pose, std::size_t rows, std::size_t cols,
	// 	                float vertical_fov, float near_clip, float far_clip,
	// 	                Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0},
	// 	                Vec3f forward = {1, 0, 0}) const
	// 	{
	// 		return imageIndex(Base::operator()(code), inner_f, ret_f, pose, rows, cols,
	// 		                  vertical_fov, near_clip, far_clip, up, right, forward);
	// 	}

	// 	template <class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true> 	auto imageIndex(Key key, InnerBinaryFun inner_f, RetBinaryFun ret_f,
	// Pose6f const& pose, 	                std::size_t rows, std::size_t cols, float
	// vertical_fov, float near_clip, 	                float far_clip, Vec3f up = {0, 0,
	// 1}, Vec3f right = {0, -1, 0}, 	                Vec3f forward = {1, 0, 0}) const
	// 	{
	// 		return imageIndex(Base::operator()(key), inner_f, ret_f, pose, rows, cols,
	// 		                  vertical_fov, near_clip, far_clip, up, right, forward);
	// 	}

	// 	template <class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true> 	auto imageIndex(Coord coord, InnerBinaryFun inner_f, RetBinaryFun
	// ret_f, 	                Pose6f const& pose, std::size_t rows, std::size_t cols,
	// float vertical_fov, float near_clip, float far_clip, 	                Vec3f up = {0,
	// 0, 1}, Vec3f right = {0, -1, 0}, 	                Vec3f forward = {1, 0, 0}, depth_t
	// depth = 0) const
	// 	{
	// 		return imageIndex(Base::operator()(coord, depth), inner_f, ret_f, pose, rows,
	// cols, 		                  vertical_fov, near_clip, far_clip, up, right, forward);
	// 	}

	// #ifdef UFO_TBB
	// 	template <class ExecutionPolicy, class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true,
	// 	          std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	auto imageIndex(ExecutionPolicy&& policy, InnerBinaryFun inner_f, RetBinaryFun
	// ret_f, 	                Pose6f const& pose, std::size_t rows, std::size_t cols,
	// float vertical_fov, float near_clip, float far_clip, 	                Vec3f up = {0,
	// 0, 1}, Vec3f right = {0, -1, 0}, 	                Vec3f forward = {1, 0, 0}) const
	// 	{
	// 		return imageIndex(std::forward<ExecutionPolicy>(policy), Base::node(), inner_f,
	// ret_f, 		                  pose, rows, cols, vertical_fov, near_clip, far_clip, up,
	// right, 		                  forward);
	// 	}

	// 	template <class ExecutionPolicy, class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true,
	// 	          std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	auto imageIndex(ExecutionPolicy&& policy, Node node, InnerBinaryFun inner_f,
	// 	                RetBinaryFun ret_f, Pose6f const& pose, std::size_t rows,
	// 	                std::size_t cols, float vertical_fov, float near_clip, float
	// far_clip, 	                Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0}, Vec3f
	// forward = {1, 0, 0}) const
	// 	{
	// 		float aspect_ratio = cols / static_cast<float>(rows);

	// 		float e = far_clip * std::tan(vertical_fov / 2);

	// 		up *= e;
	// 		right *= e * aspect_ratio;
	// 		forward *= far_clip;

	// 		Vec3f tl = up - right + forward;
	// 		Vec3f tr = up + right + forward;
	// 		Vec3f bl = -up - right + forward;
	// 		pose.transformInPlace(tl);
	// 		pose.transformInPlace(tr);
	// 		pose.transformInPlace(bl);
	// 		Vec3f right_dir  = tr - tl;
	// 		Vec3f bottom_dir = bl - tl;

	// 		Image<Ray> rays(rows, cols, Ray(pose.translation, Coord()));

	// 		std::vector<std::size_t> indices(rows);
	// 		std::iota(std::begin(indices), std::end(indices), 0);

	// 		std::for_each(policy, std::begin(indices), std::end(indices),
	// 		              [tl, right_dir, bottom_dir, rows, cols, &rays](auto row) {
	// 			              auto r = (row + 0.5f) / rows;
	// 			              auto a = tl + bottom_dir * r;
	// 			              for (std::size_t col{}; col < cols; ++col) {
	// 				              auto c   = (col + 0.5f) / cols;
	// 				              auto end = a + right_dir * c;
	// 				              rays(row, col).direction =
	// 				                  (end - rays(row, col).origin).normalized();
	// 			              }
	// 		              });

	// 		if constexpr (std::is_same_v<std::invoke_result_t<RetBinaryFun, Index, Ray>,
	// bool>) { 			Image<Index> image(rows, cols);

	// 			traceIndex(std::forward<ExecutionPolicy>(policy), node, std::cbegin(rays),
	// 			           std::cend(rays), std::begin(image), inner_f, ret_f);

	// 			return image;
	// 		} else {
	// 			Image<std::pair<
	// 			    Index, typename std::invoke_result_t<RetBinaryFun, Index,
	// Ray>::second_type>> 			    image(rows, cols);

	// 			trace(std::forward<ExecutionPolicy>(policy), node, std::cbegin(rays),
	// 			      std::cend(rays), std::begin(image), inner_f, ret_f);

	// 			return image;
	// 		}
	// 	}

	// 	template <class ExecutionPolicy, class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true,
	// 	          std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	auto imageIndex(ExecutionPolicy&& policy, Code code, InnerBinaryFun inner_f,
	// 	                RetBinaryFun ret_f, Pose6f const& pose, std::size_t rows,
	// 	                std::size_t cols, float vertical_fov, float near_clip, float
	// far_clip, 	                Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0}, Vec3f
	// forward = {1, 0, 0}) const
	// 	{
	// 		return imageIndex(std::forward<ExecutionPolicy>(policy), Base::operator()(code),
	// 		                  inner_f, ret_f, pose, rows, cols, vertical_fov, near_clip,
	// far_clip, 		                  up, right, forward);
	// 	}

	// 	template <class ExecutionPolicy, class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true,
	// 	          std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	auto imageIndex(ExecutionPolicy&& policy, Key key, InnerBinaryFun inner_f,
	// 	                RetBinaryFun ret_f, Pose6f const& pose, std::size_t rows,
	// 	                std::size_t cols, float vertical_fov, float near_clip, float
	// far_clip, 	                Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0}, Vec3f
	// forward = {1, 0, 0}) const
	// 	{
	// 		return imageIndex(std::forward<ExecutionPolicy>(policy), Base::operator()(key),
	// 		                  inner_f, ret_f, pose, rows, cols, vertical_fov, near_clip,
	// far_clip, 		                  up, right, forward);
	// 	}

	// 	template <class ExecutionPolicy, class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true,
	// 	          std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>,
	// 	                           bool>                                                =
	// true> 	auto imageIndex(ExecutionPolicy&& policy, Coord coord, InnerBinaryFun inner_f,
	// 	                RetBinaryFun ret_f, Pose6f const& pose, std::size_t rows,
	// 	                std::size_t cols, float vertical_fov, float near_clip, float
	// far_clip, 	                Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0}, Vec3f
	// forward = {1, 0, 0}, depth_t depth = 0) const
	// 	{
	// 		return imageIndex(std::forward<ExecutionPolicy>(policy),
	// 		                  Base::operator()(coord, depth), inner_f, ret_f, pose, rows,
	// cols, 		                  vertical_fov, near_clip, far_clip, up, right, forward);
	// 	}
	// #endif

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	Octree(length_t leaf_node_length, depth_t num_depth_levels)
	    : Base(leaf_node_length, num_depth_levels)
	{
	}

	Octree(Octree const& other) = default;

	Octree(Octree&& other) = default;

	template <class Derived2>
	Octree(Octree<Derived2, Block> const& other) : Base(other)
	{
	}

	template <class Derived2>
	Octree(Octree<Derived2, Block>&& other) : Base(std::move(other))
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~Octree() {}

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	Octree& operator=(Octree const& rhs) = default;

	Octree& operator=(Octree&& rhs) = default;

	template <class Derived2>
	Octree& operator=(Octree<Derived2, Block> const& rhs)
	{
		Base::operator=(rhs);
		return *this;
	}

	template <class Derived2>
	Octree& operator=(Octree<Derived2, Block>&& rhs)
	{
		Base::operator=(std::move(rhs));
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         Swap                                        |
	|                                                                                     |
	**************************************************************************************/

	friend void swap(Octree& lhs, Octree& rhs)
	{
		Base::swap(static_cast<Base&>(lhs), static_cast<Base&>(rhs));
	}

	// 	/**************************************************************************************
	// 	| | 	|                                        Image | 	| |
	// 	**************************************************************************************/

	// 	// [[nodiscard]] std::array<std::array<float, 4>, 4> perspective(float vertical_fov,
	// 	//                                                               float aspect,
	// 	//                                                               float near_clip,
	// 	//                                                               float far_clip)
	// const
	// 	// {
	// 	// 	assert(0.0f < std::abs(aspect - std::numeric_limits<float>::epsilon()));

	// 	// 	float const thf = std::tan(vertical_fov / 2.0f);

	// 	// 	std::array<std::array<float, 4>, 4> perspective{};
	// 	// 	perspective[0][0] = 1.0f / (aspect * thf);
	// 	// 	perspective[1][1] = 1.0f / thf;
	// 	// 	perspective[2][2] = far_clip / (far_clip - near_clip);
	// 	// 	perspective[2][3] = 1.0f;
	// 	// 	perspective[3][2] = -(far_clip * near_clip) / (far_clip - near_clip);
	// 	// 	return perspective;
	// 	// }

	// 	// [[nodiscard]] std::array<std::array<float, 4>, 4> inverse(
	// 	//     std::array<std::array<float, 4>, 4> mat) const
	// 	// {
	// 	// 	// TODO: Implement
	// 	// 	return mat;
	// 	// }

	// 	// template <std::size_t N>
	// 	// [[nodiscard]] std::array<float, N> multiply(
	// 	//     std::array<std::array<float, N>, N> const& mat, std::array<float, N> vec)
	// const
	// 	// {
	// 	// 	std::array<float, N> res{};
	// 	// 	for (std::size_t i{}; N > i; ++i) {
	// 	// 		for (std::size_t j{}; N > j; ++j) {
	// 	// 			res[i] += vec[j] * vec[i][j];
	// 	// 		}
	// 	// 	}
	// 	// 	return res;
	// 	// }

	// 	[[nodiscard]] Image<Ray> imageInit(Pose6f const& pose, std::size_t rows,
	// 	                                   std::size_t cols, float vertical_fov,
	// 	                                   float near_clip, float far_clip) const
	// 	{
	// 		// float aspect         = static_cast<float>(cols) / static_cast<float>(rows);
	// 		// auto  projection     = perspective(vertical_fov, aspect, near_clip, far_clip);
	// 		// auto  projection_inv = inverse(projection);

	// 		// float half_rows_factor = 2.0f / static_cast<float>(rows);
	// 		// float half_cols_factor = 2.0f / static_cast<float>(cols);

	// 		// Image<Ray> rays(rows, cols);
	// 		// for (std::size_t row{}; rows > row; ++row) {
	// 		// 	for (std::size_t col{}; cols > col; ++col) {
	// 		// 		std::array<float, 4> point_nds_h;
	// 		// 		point_nds_h[0] = static_cast<float>(row) * half_rows_factor - 1.0f;
	// 		// 		point_nds_h[1] = static_cast<float>(col) * half_cols_factor - 1.0f;
	// 		// 		point_nds_h[2] = -1.0f;
	// 		// 		point_nds_h[3] = 1.0f;

	// 		// 		auto dir_eye             = multiply(projection_inv, point_nds_h);
	// 		// 		dir_eye[3]               = 0.0f;
	// 		// 		Vec3f dir_world          = multiply(view_inv, dir_eye);
	// 		// 		Vec3f pos_world          = view_inv * Vec4f(vec3f(0.0f), 1.0f);
	// 		// 		rays(row, col).direction = dir_world.normalized();
	// 		// 	}
	// 		// }

	// 		// return rays;

	// 		float aspect_ratio = cols / static_cast<float>(rows);

	// 		float e = far_clip * std::tan(vertical_fov / 2);

	// 		up *= e;
	// 		right *= e * aspect_ratio;
	// 		forward *= far_clip;

	// 		Vec3f tl = up - right + forward;
	// 		Vec3f tr = up + right + forward;
	// 		Vec3f bl = -up - right + forward;
	// 		pose.transformInPlace(tl);
	// 		pose.transformInPlace(tr);
	// 		pose.transformInPlace(bl);
	// 		Vec3f right_dir  = tr - tl;
	// 		Vec3f bottom_dir = bl - tl;

	// 		Image<Ray> rays(rows, cols, Ray(pose.translation, Coord()));
	// 		for (std::size_t row{}; row < rows; ++row) {
	// 			auto r = (row + 0.5f) / rows;
	// 			for (std::size_t col{}; col < cols; ++col) {
	// 				auto c                   = (col + 0.5f) / cols;
	// 				auto end                 = tl + right_dir * c + bottom_dir * r;
	// 				rays(row, col).direction = (end - pose.translation).normalized();
	// 			}
	// 		};

	// 		return rays;
	// 	}

	// 	/**************************************************************************************
	// 	| | 	|                                        Trace | 	| |
	// 	**************************************************************************************/

	// 	[[nodiscard]] std::tuple<Vec3f, Vec3f, offset_t> traceInit(Index node, Ray ray)
	// const
	// 	{
	// 		return traceInit(ray, Base::center(node), Base::halfSize(node));
	// 	}

	// 	[[nodiscard]] std::tuple<Vec3f, Vec3f, offset_t> traceInit(Node node, Ray ray) const
	// 	{
	// 		return traceInit(ray, Base::center(node), Base::halfSize(node));
	// 	}

	// 	[[nodiscard]] std::tuple<Vec3f, Vec3f, offset_t> traceInit(Ray ray, Vec3f center,
	// 	                                                           float half_size) const
	// 	{
	// 		offset_t a = offset_t(0 > ray.direction[0]) | (offset_t(0 > ray.direction[1]) <<
	// 1) | 		             (offset_t(0 > ray.direction[2]) << 2);

	// 		Vec3f origin(0 > ray.direction[0] ? center[0] * 2 - ray.origin[0] : ray.origin[0],
	// 		             0 > ray.direction[1] ? center[1] * 2 - ray.origin[1] : ray.origin[1],
	// 		             0 > ray.direction[2] ? center[2] * 2 - ray.origin[2] :
	// ray.origin[2]);

	// 		Vec3f direction = Vec3f::abs(ray.direction);

	// 		Vec3f t0;
	// 		Vec3f t1;

	// 		for (std::size_t i{}; direction.size() > i; ++i) {
	// 			auto a = center[i] - half_size - origin[i];
	// 			auto b = center[i] + half_size - origin[i];
	// 			// TODO: Look at
	// 			t0[i] = 0 == direction[i] ? 1e+25f * a : a / direction[i];
	// 			t1[i] = 0 == direction[i] ? 1e+25f * b : b / direction[i];
	// 		}

	// 		return {t0, t1, a};
	// 	}

	// 	template <bool Exists, class NodeT, class Predicate, class BinaryFun,
	// 	          std::enable_if_t<pred::is_predicate_v<Predicate, Octree, Node>, bool> =
	// true, 	          std::enable_if_t<std::is_invocable_v<BinaryFun, Node, Ray>, bool> =
	// true> 	auto trace(NodeT node, Vec3f t0, Vec3f t1, offset_t const a, Ray ray,
	// Predicate const& predicate, BinaryFun binary_f) const
	// 	{
	// 		auto firstNode = [](Vec3f t0, Vec3f tm) -> offset_t {
	// 			// constexpr std::array<std::size_t, 3> lut_a{1, 0, 0};
	// 			// constexpr std::array<std::size_t, 3> lut_b{2, 2, 1};

	// 			auto max_comp = t0.maxElementIndex();

	// 			// auto a = lut_a[max_comp];
	// 			// auto b = lut_b[max_comp];

	// 			std::size_t a = 0 == max_comp;
	// 			std::size_t b = 2 - (2 == max_comp);

	// 			return (static_cast<offset_t>(tm[a] < t0[max_comp]) << a) |
	// 			       (static_cast<offset_t>(tm[b] < t0[max_comp]) << b);
	// 		};

	// 		constexpr std::array new_node_lut{
	// 		    std::array<offset_t, 3>{1, 2, 4}, std::array<offset_t, 3>{8, 3, 5},
	// 		    std::array<offset_t, 3>{3, 8, 6}, std::array<offset_t, 3>{8, 8, 7},
	// 		    std::array<offset_t, 3>{5, 6, 8}, std::array<offset_t, 3>{8, 7, 8},
	// 		    std::array<offset_t, 3>{7, 8, 8}, std::array<offset_t, 3>{8, 8, 8}};

	// 		if (0 > t1.min() || t0.max() >= t1.min()) {
	// 			if constexpr (std::is_void_v<std::invoke_result_t<BinaryFun, Node, Ray>> ||
	// 			              std::is_same_v<std::invoke_result_t<BinaryFun, Node, Ray>, bool>)
	// { 				return Node{}; 			} else { 				return std::pair< 				    Node,
	// typename std::invoke_result_t<BinaryFun, Node, Ray>::second_type>{};
	// 			}
	// 		}

	// 		if constexpr (std::is_void_v<std::invoke_result_t<BinaryFun, Node, Ray>>) {
	// 			if (pred::ValueCheck<Predicate>::apply(predicate, *this, node)) {
	// 				return node;
	// 			}
	// 		} else if constexpr (std::is_same_v<std::invoke_result_t<BinaryFun, Node, Ray>,
	// 		                                    bool>) {
	// 			if (pred::ValueCheck<Predicate>::apply(predicate, *this, node) &&
	// 			    binary_f(node, ray)) {
	// 				return node;
	// 			}
	// 		} else {
	// 			if (std::invoke_result_t<BinaryFun, Node, Ray> res;
	// 			    pred::ValueCheck<Predicate>::apply(predicate, *this, node) &&
	// 			    (res = binary_f(node, ray)).first) {
	// 				return std::pair<
	// 				    Node, typename std::invoke_result_t<BinaryFun, Node, Ray>::second_type>{
	// 				    node, res.second};
	// 			}
	// 		}

	// 		if constexpr (Exists) {
	// 			if (Base::isLeaf(node.index()) ||
	// 			    !pred::InnerCheck<Predicate>::apply(predicate, *this, node)) {
	// 				if constexpr (std::is_void_v<std::invoke_result_t<BinaryFun, Node, Ray>> ||
	// 				              std::is_same_v<std::invoke_result_t<BinaryFun, Node, Ray>,
	// bool>) { 					return Node{}; 				} else { 					return std::pair<
	// Node, typename std::invoke_result_t<BinaryFun, Node, Ray>::second_type>{};
	// 				}
	// 			}
	// 		} else {
	// 			if (!pred::InnerCheck<Predicate>::apply(predicate, *this, node)) {
	// 				if constexpr (std::is_void_v<std::invoke_result_t<BinaryFun, Node, Ray>> ||
	// 				              std::is_same_v<std::invoke_result_t<BinaryFun, Node, Ray>,
	// bool>) { 					return Node{}; 				} else { 					return std::pair<
	// Node, typename std::invoke_result_t<BinaryFun, Node, Ray>::second_type>{};
	// 				}
	// 			}
	// 		}

	// 		Vec3f tm = (t0 + t1) / 2;

	// 		offset_t cur_node = firstNode(t0, tm);

	// 		if (8 <= cur_node) {
	// 			if constexpr (std::is_void_v<std::invoke_result_t<BinaryFun, Node, Ray>> ||
	// 			              std::is_same_v<std::invoke_result_t<BinaryFun, Node, Ray>, bool>)
	// { 				return Node{}; 			} else { 				return std::pair< 				    Node,
	// typename std::invoke_result_t<BinaryFun, Node, Ray>::second_type>{};
	// 			}
	// 		}

	// 		std::array<std::tuple<NodeT, offset_t, Vec3f, Vec3f, Vec3f>,
	// Base::maxDepthLevels()> 		    stack; 		stack[0] = {node, cur_node, t0, t1, tm};

	// 		for (int index{}; 0 <= index;) {
	// 			std::tie(node, cur_node, t0, t1, tm) = stack[index];

	// 			if constexpr (Exists) {
	// 				node = Base::childUnsafe(node, cur_node ^ a);
	// 			} else {
	// 				node = Base::child(node, cur_node ^ a);
	// 			}

	// 			std::array mask{cur_node & offset_t(1), cur_node & offset_t(2),
	// 			                cur_node & offset_t(4)};

	// 			t0 = {mask[0] ? tm[0] : t0[0], mask[1] ? tm[1] : t0[1], mask[2] ? tm[2] :
	// t0[2]}; 			t1 = {mask[0] ? t1[0] : tm[0], mask[1] ? t1[1] : tm[1], mask[2] ? t1[2]
	// : tm[2]};

	// 			std::get<1>(stack[index]) = new_node_lut[cur_node][t1.minElementIndex()];
	// 			index -= 8 <= std::get<1>(stack[index]);

	// 			if (0 > t1.min()) {
	// 				continue;
	// 			}

	// 			if constexpr (std::is_void_v<std::invoke_result_t<BinaryFun, Node, Ray>>) {
	// 				if (pred::ValueCheck<Predicate>::apply(predicate, *this, node)) {
	// 					return node;
	// 				}
	// 			} else if constexpr (std::is_same_v<std::invoke_result_t<BinaryFun, Node, Ray>,
	// 			                                    bool>) {
	// 				if (pred::ValueCheck<Predicate>::apply(predicate, *this, node) &&
	// 				    binary_f(node, ray)) {
	// 					return node;
	// 				}
	// 			} else {
	// 				if (std::invoke_result_t<BinaryFun, Node, Ray> res;
	// 				    pred::ValueCheck<Predicate>::apply(predicate, *this, node) &&
	// 				    (res = binary_f(node, ray)).first) {
	// 					return std::pair<
	// 					    Node, typename std::invoke_result_t<BinaryFun, Node, Ray>::second_type>{
	// 					    node, res.second};
	// 				}
	// 			}

	// 			if constexpr (Exists) {
	// 				if (Base::isLeaf(node.index()) ||
	// 				    !pred::InnerCheck<Predicate>::apply(predicate, *this, node)) {
	// 					continue;
	// 				}
	// 			} else {
	// 				if (!pred::InnerCheck<Predicate>::apply(predicate, *this, node)) {
	// 					continue;
	// 				}
	// 			}

	// 			tm = (t0 + t1) / 2;

	// 			cur_node = firstNode(t0, tm);

	// 			stack[index + 1] = {node, cur_node, t0, t1, tm};
	// 			index += 8 > cur_node;
	// 		}

	// 		if constexpr (std::is_void_v<std::invoke_result_t<BinaryFun, Node, Ray>> ||
	// 		              std::is_same_v<std::invoke_result_t<BinaryFun, Node, Ray>, bool>) {
	// 			return Node{};
	// 		} else {
	// 			return std::pair<
	// 			    Node, typename std::invoke_result_t<BinaryFun, Node, Ray>::second_type>{};
	// 		}
	// 	}

	// 	/**************************************************************************************
	// 	| | 	|                                      Trace index | 	| |
	// 	**************************************************************************************/

	// 	template <class InnerBinaryFun, class RetBinaryFun,
	// 	          std::enable_if_t<std::is_invocable_r_v<bool, InnerBinaryFun, Index, Ray>,
	// 	                           bool>                                                =
	// true, 	          std::enable_if_t<std::is_invocable_v<RetBinaryFun, Index, Ray>,
	// bool> = true> 	auto traceIndex(Index node, Vec3f t0, Vec3f t1, offset_t const a, Ray
	// ray, 	                InnerBinaryFun inner_f, RetBinaryFun ret_f) const
	// 	{
	// 		auto firstNode = [](Vec3f t0, Vec3f tm) -> offset_t {
	// 			// constexpr std::array<std::size_t, 3> lut_a{1, 0, 0};
	// 			// constexpr std::array<std::size_t, 3> lut_b{2, 2, 1};

	// 			auto max_comp = t0.maxElementIndex();

	// 			// auto a = lut_a[max_comp];
	// 			// auto b = lut_b[max_comp];

	// 			std::size_t a = 0 == max_comp;
	// 			std::size_t b = 2 - (2 == max_comp);

	// 			return (static_cast<offset_t>(tm[a] < t0[max_comp]) << a) |
	// 			       (static_cast<offset_t>(tm[b] < t0[max_comp]) << b);
	// 		};

	// 		constexpr std::array new_node_lut{
	// 		    std::array<offset_t, 3>{1, 2, 4}, std::array<offset_t, 3>{8, 3, 5},
	// 		    std::array<offset_t, 3>{3, 8, 6}, std::array<offset_t, 3>{8, 8, 7},
	// 		    std::array<offset_t, 3>{5, 6, 8}, std::array<offset_t, 3>{8, 7, 8},
	// 		    std::array<offset_t, 3>{7, 8, 8}, std::array<offset_t, 3>{8, 8, 8}};

	// 		if (0 > t1.min() || t0.max() >= t1.min()) {
	// 			if constexpr (std::is_same_v<std::invoke_result_t<RetBinaryFun, Index, Ray>,
	// 			                             bool>) {
	// 				return Index{};
	// 			} else {
	// 				return std::pair<Index, typename std::invoke_result_t<RetBinaryFun, Index,
	// 				                                                      Ray>::second_type>{};
	// 			}
	// 		}

	// 		if constexpr (std::is_same_v<std::invoke_result_t<RetBinaryFun, Index, Ray>,
	// bool>) { 			if (ret_f(node, ray)) { 				return node;
	// 			}
	// 		} else {
	// 			if (std::invoke_result_t<RetBinaryFun, Index, Ray> res;
	// 			    (res = ret_f(node, ray)).first) {
	// 				return std::pair<
	// 				    Index, typename std::invoke_result_t<RetBinaryFun, Index,
	// Ray>::second_type>{ 				    node, res.second};
	// 			}
	// 		}

	// 		if (Base::isLeaf(node) || !inner_f(node, ray)) {
	// 			if constexpr (std::is_same_v<std::invoke_result_t<RetBinaryFun, Index, Ray>,
	// 			                             bool>) {
	// 				return Index{};
	// 			} else {
	// 				return std::pair<Index, typename std::invoke_result_t<RetBinaryFun, Index,
	// 				                                                      Ray>::second_type>{};
	// 			}
	// 		}

	// 		Vec3f tm = (t0 + t1) / 2;

	// 		offset_t cur_node = firstNode(t0, tm);

	// 		if (8 <= cur_node) {
	// 			if constexpr (std::is_same_v<std::invoke_result_t<RetBinaryFun, Index, Ray>,
	// 			                             bool>) {
	// 				return Index{};
	// 			} else {
	// 				return std::pair<Index, typename std::invoke_result_t<RetBinaryFun, Index,
	// 				                                                      Ray>::second_type>{};
	// 			}
	// 		}

	// 		std::array<std::tuple<Index, offset_t, Vec3f, Vec3f, Vec3f>,
	// Base::maxDepthLevels()> 		    stack; 		stack[0] = {node, cur_node, t0, t1, tm};

	// 		for (int index{}; 0 <= index;) {
	// 			std::tie(node, cur_node, t0, t1, tm) = stack[index];

	// 			node = Base::child(node, cur_node ^ a);

	// 			std::array mask{cur_node & offset_t(1), cur_node & offset_t(2),
	// 			                cur_node & offset_t(4)};

	// 			t0 = {mask[0] ? tm[0] : t0[0], mask[1] ? tm[1] : t0[1], mask[2] ? tm[2] :
	// t0[2]}; 			t1 = {mask[0] ? t1[0] : tm[0], mask[1] ? t1[1] : tm[1], mask[2] ? t1[2]
	// : tm[2]};

	// 			std::get<1>(stack[index]) = new_node_lut[cur_node][t1.minElementIndex()];
	// 			index -= 8 <= std::get<1>(stack[index]);

	// 			if (0 > t1.min()) {
	// 				continue;
	// 			}

	// 			if constexpr (std::is_same_v<std::invoke_result_t<RetBinaryFun, Index, Ray>,
	// 			                             bool>) {
	// 				if (ret_f(node, ray)) {
	// 					return node;
	// 				}
	// 			} else {
	// 				if (std::invoke_result_t<RetBinaryFun, Index, Ray> res;
	// 				    (res = ret_f(node, ray)).first) {
	// 					return std::pair<Index, typename std::invoke_result_t<RetBinaryFun, Index,
	// 					                                                      Ray>::second_type>{
	// 					    node, res.second};
	// 				}
	// 			}

	// 			if (Base::isLeaf(node) || !inner_f(node, ray)) {
	// 				continue;
	// 			}

	// 			tm = (t0 + t1) / 2;

	// 			cur_node = firstNode(t0, tm);

	// 			stack[index + 1] = {node, cur_node, t0, t1, tm};
	// 			index += 8 > cur_node;
	// 		}

	// 		if constexpr (std::is_same_v<std::invoke_result_t<RetBinaryFun, Index, Ray>,
	// bool>) { 			return Index{}; 		} else { 			return std::pair< 			    Index,
	// typename std::invoke_result_t<RetBinaryFun, Index, Ray>::second_type>{};
	// 		}
	// 	}
};
}  // namespace ufo

#endif  // UFO_CONTAINER_OCTREE_HPP