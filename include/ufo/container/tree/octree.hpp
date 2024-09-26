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
#include <ufo/container/tree/predicate/spatial.hpp>
#include <ufo/container/tree/tree.hpp>
#include <ufo/container/tree/type.hpp>
#include <ufo/core/image.hpp>
#include <ufo/geometry/shape/ray.hpp>
#include <ufo/math/pose3.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <iterator>
#include <numeric>
#include <type_traits>
#include <utility>

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

	/**************************************************************************************
	|                                                                                     |
	|                                        Trace                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class Pred,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool> = true>
	Node trace(Ray3 ray, Pred const& pred, bool only_exists = true) const
	{
		return trace(Base::node(), ray, pred, only_exists);
	}

	template <class Pred, class HitFun,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>    = true,
	          std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool> = true>
	auto trace(Ray3 ray, Pred const& pred, HitFun hit_f, bool only_exists = true) const
	{
		return trace(Base::node(), ray, pred, hit_f, only_exists);
	}

	template <class InputIt, class OutputIt, class Pred,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool> = true>
	OutputIt trace(InputIt first, InputIt last, OutputIt d_first, Pred const& pred,
	               bool only_exists = true) const
	{
		return trace(Base::node(), first, last, d_first, pred, only_exists);
	}

	template <class InputIt, class OutputIt, class Pred, class HitFun,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>    = true,
	          std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool> = true>
	OutputIt trace(InputIt first, InputIt last, OutputIt d_first, Pred const& pred,
	               HitFun hit_f, bool only_exists = true) const
	{
		return trace(Base::node(), first, last, d_first, pred, hit_f, only_exists);
	}

	template <class InputIt, class Pred,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool> = true>
	auto trace(InputIt first, InputIt last, Pred const& pred, bool only_exists = true) const
	{
		return trace(Base::node(), first, last, pred, only_exists);
	}

	template <class InputIt, class Pred, class HitFun,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>    = true,
	          std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool> = true>
	auto trace(InputIt first, InputIt last, Pred const& pred, HitFun hit_f,
	           bool only_exists = true) const
	{
		return trace(Base::node(), first, last, pred, hit_f, only_exists);
	}

	template <class RayRange, class Pred,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool> = true>
	auto trace(RayRange const& r, Pred const& pred, bool only_exists = true) const
	{
		using std::begin;
		using std::end;
		return trace(Base::node(), begin(r), end(r), pred, only_exists);
	}

	template <class RayRange, class Pred, class HitFun,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>    = true,
	          std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool> = true>
	auto trace(RayRange const& r, Pred const& pred, HitFun hit_f,
	           bool only_exists = true) const
	{
		using std::begin;
		using std::end;
		return trace(Base::node(), begin(r), end(r), pred, hit_f, only_exists);
	}

	template <class NodeType, class Pred,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>    = true>
	Node trace(NodeType node, Ray3 ray, Pred const& pred, bool only_exists = true) const
	{
		return trace(node, ray, pred, [](Node, Ray3) -> void {}, only_exists);
	}

	template <class NodeType, class Pred, class HitFun,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>    = true,
	          std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool> = true>
	auto trace(NodeType node, Ray3 ray, Pred const& pred, HitFun hit_f,
	           bool only_exists = true) const
	{
		pred::Init<Pred>::apply(pred, *this);

		// Ensure that the index-part points to the node of the code-part or the closest
		// ancestor. This is so the predicates can just use the index of the node.
		Node n = Base::node(node);

		auto [t0, t1, a] = traceInit(n, ray);

		if (only_exists) {
			return trace<true>(n, t0, t1, a, ray, pred, hit_f);
		} else {
			return trace<false>(n, t0, t1, a, ray, pred, hit_f);
		}
	}

	template <class NodeType, class InputIt, class OutputIt, class Pred,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>    = true>
	OutputIt trace(NodeType node, InputIt first, InputIt last, OutputIt d_first,
	               Pred const& pred, bool only_exists = true) const
	{
		return trace(
		    node, first, last, d_first, pred, [](Node, Ray3) -> void {}, only_exists);
	}

	template <class NodeType, class InputIt, class OutputIt, class Pred, class HitFun,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>    = true,
	          std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool> = true>
	OutputIt trace(NodeType node, InputIt first, InputIt last, OutputIt d_first,
	               Pred const& pred, HitFun hit_f, bool only_exists = true) const
	{
		pred::Init<Pred>::apply(pred, *this);

		// Ensure that the index-part points to the node of the code-part or the closest
		// ancestor. This is so the predicates can just use the index of the node.
		Node n = Base::node(node);

		auto center      = Base::center(node);
		auto half_length = Base::halfLength(node);

		auto fun_only_exists = [this, n, &pred, hit_f, center, half_length](Ray3 const& ray) {
			auto [t0, t1, a] = traceInit(ray, center, half_length);
			return trace<true>(n, t0, t1, a, ray, pred, hit_f);
		};

		auto fun_maybe_do_not_exists = [this, n, &pred, hit_f, center,
		                                half_length](Ray3 const& ray) {
			auto [t0, t1, a] = traceInit(ray, center, half_length);
			return trace<false>(n, t0, t1, a, ray, pred, hit_f);
		};

		if (only_exists) {
			return std::transform(first, last, d_first, fun_only_exists);
		} else {
			return std::transform(first, last, d_first, fun_maybe_do_not_exists);
		}
	}

	template <class NodeType, class InputIt, class Pred,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>    = true>
	auto trace(NodeType node, InputIt first, InputIt last, Pred const& pred,
	           bool only_exists = true) const
	{
		return trace(node, first, last, pred, [](Node, Ray3) -> void {}, only_exists);
	}

	template <class NodeType, class InputIt, class Pred, class HitFun,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>    = true,
	          std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool> = true>
	auto trace(NodeType node, InputIt first, InputIt last, Pred const& pred, HitFun hit_f,
	           bool only_exists = true) const
	{
		using ret_type = std::conditional_t<
		    is_pair_v<std::invoke_result_t<HitFun, Node, Ray3>>,
		    std::pair<Node, typename std::invoke_result_t<HitFun, Node, Ray3>::second_type>,
		    Node>;

		std::vector<ret_type> nodes;
		nodes.reserve(std::distance(first, last));
		trace(node, first, last, std::back_inserter(nodes), pred, hit_f, only_exists);
		return nodes;
	}

	template <class NodeType, class RayRange, class Pred,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>    = true>
	auto trace(NodeType node, RayRange const& r, Pred const& pred,
	           bool only_exists = true) const
	{
		using std::begin;
		using std::end;
		return trace(node, begin(r), end(r), pred, [](Node, Ray3) -> void {}, only_exists);
	}

	template <class NodeType, class RayRange, class Pred, class HitFun,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>    = true,
	          std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool> = true>
	auto trace(NodeType node, RayRange const& r, Pred const& pred, HitFun hit_f,
	           bool only_exists = true) const
	{
		using std::begin;
		using std::end;
		return trace(node, begin(r), end(r), pred, hit_f, only_exists);
	}

	template <
	    class ExecutionPolicy, class RandomIt1, class RandomIt2, class Pred,
	    std::enable_if_t<pred::is_pred_v<Pred, Octree, Node>, bool>                  = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	RandomIt2 trace(ExecutionPolicy&& policy, RandomIt1 first, RandomIt1 last,
	                RandomIt2 d_first, Pred const& pred, bool only_exists = true) const
	{
		return trace(std::forward<ExecutionPolicy>(policy), Base::node(), first, last,
		             d_first, pred, only_exists);
	}

	template <
	    class ExecutionPolicy, class RandomIt1, class RandomIt2, class Pred, class HitFun,
	    std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>                 = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool>              = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	RandomIt2 trace(ExecutionPolicy&& policy, RandomIt1 first, RandomIt1 last,
	                RandomIt2 d_first, Pred const& pred, HitFun hit_f,
	                bool only_exists = true) const
	{
		return trace(std::forward<ExecutionPolicy>(policy), Base::node(), first, last,
		             d_first, pred, hit_f, only_exists);
	}

	template <
	    class ExecutionPolicy, class RandomIt, class Pred,
	    std::enable_if_t<pred::is_pred_v<Pred, Octree, Node>, bool>                  = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	auto trace(ExecutionPolicy&& policy, RandomIt first, RandomIt last, Pred const& pred,
	           bool only_exists = true) const
	{
		return trace(std::forward<ExecutionPolicy>(policy), Base::node(), first, last, pred,
		             only_exists);
	}

	template <
	    class ExecutionPolicy, class RandomIt, class Pred, class HitFun,
	    std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>                 = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool>              = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	auto trace(ExecutionPolicy&& policy, RandomIt first, RandomIt last, Pred const& pred,
	           HitFun hit_f, bool only_exists = true) const
	{
		return trace(std::forward<ExecutionPolicy>(policy), Base::node(), first, last, pred,
		             hit_f, only_exists);
	}

	template <
	    class ExecutionPolicy, class RayRange, class Pred,
	    std::enable_if_t<pred::is_pred_v<Pred, Octree, Node>, bool>                  = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	auto trace(ExecutionPolicy&& policy, RayRange const& r, Pred const& pred,
	           bool only_exists = true) const
	{
		using std::begin;
		using std::end;
		return trace(std::forward<ExecutionPolicy>(policy), Base::node(), begin(r), end(r),
		             pred, only_exists);
	}

	template <
	    class ExecutionPolicy, class RayRange, class Pred, class HitFun,
	    std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>                 = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool>              = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	auto trace(ExecutionPolicy&& policy, RayRange const& r, Pred const& pred, HitFun hit_f,
	           bool only_exists = true) const
	{
		using std::begin;
		using std::end;
		return trace(std::forward<ExecutionPolicy>(policy), Base::node(), begin(r), end(r),
		             pred, hit_f, only_exists);
	}

	template <
	    class ExecutionPolicy, class NodeType, class RandomIt1, class RandomIt2, class Pred,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>              = true,
	    std::enable_if_t<pred::is_pred_v<Pred, Octree, Node>, bool>                  = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	RandomIt2 trace(ExecutionPolicy&& policy, NodeType node, RandomIt1 first,
	                RandomIt1 last, RandomIt2 d_first, Pred const& pred,
	                bool only_exists = true) const
	{
		return trace(
		    std::forward<ExecutionPolicy>(policy), node, first, last, d_first, pred,
		    [](Node, Ray3) -> void {}, only_exists);
	}

	template <
	    class ExecutionPolicy, class NodeType, class RandomIt1, class RandomIt2, class Pred,
	    class HitFun,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>              = true,
	    std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>                 = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool>              = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	RandomIt2 trace(ExecutionPolicy&& policy, NodeType node, RandomIt1 first,
	                RandomIt1 last, RandomIt2 d_first, Pred const& pred, HitFun hit_f,
	                bool only_exists = true) const
	{
		if constexpr (std::is_same_v<execution::sequenced_policy,
		                             std::decay_t<ExecutionPolicy>>) {
			return trace(node, first, last, d_first, pred, hit_f, only_exists);
		}

#if !defined(UFO_TBB) && !defined(UFO_OMP)
		return trace(node, first, last, d_first, pred, hit_f, only_exists);
#endif

		pred::Init<Pred>::apply(pred, *this);

		// Ensure that the index-part points to the node of the code-part or the closest
		// ancestor. This is so the predicates can just use the index of the node.
		Node n = Base::node(node);

		auto center      = Base::center(node);
		auto half_length = Base::halfLength(node);

		auto fun_only_exists = [this, n, &pred, hit_f, center, half_length](Ray3 const& ray) {
			auto [t0, t1, a] = traceInit(ray, center, half_length);
			return trace<true>(n, t0, t1, a, ray, pred, hit_f);
		};

		auto fun_maybe_do_not_exists = [this, n, &pred, hit_f, center,
		                                half_length](Ray3 const& ray) {
			auto [t0, t1, a] = traceInit(ray, center, half_length);
			return trace<false>(n, t0, t1, a, ray, pred, hit_f);
		};

#if defined(UFO_TBB)
		if (only_exists) {
			return std::transform(std::forward<ExecutionPolicy>(policy), first, last, d_first,
			                      fun_only_exists);
		} else {
			return std::transform(std::forward<ExecutionPolicy>(policy), first, last, d_first,
			                      fun_maybe_do_not_exists);
		}
#elif defined(UFO_OMP)
		if (only_exists) {
#pragma omp parallel for
			for (; first != last; ++d_first, ++first) {
				*d_first = fun_only_exists(*first);
			}
		} else {
#pragma omp parallel for
			for (; first != last; ++d_first, ++first) {
				*d_first = fun_maybe_do_not_exists(*first);
			}
		}

		return d_first;
#endif
	}

	template <
	    class ExecutionPolicy, class NodeType, class RandomIt, class Pred,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>              = true,
	    std::enable_if_t<pred::is_pred_v<Pred, Octree, Node>, bool>                  = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	auto trace(ExecutionPolicy&& policy, NodeType node, RandomIt first, RandomIt last,
	           Pred const& pred, bool only_exists = true) const
	{
		return trace(
		    std::forward<ExecutionPolicy>(policy), node, first, last, pred,
		    [](Node, Ray3) -> void {}, only_exists);
	}

	template <
	    class ExecutionPolicy, class NodeType, class RandomIt, class Pred, class HitFun,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>              = true,
	    std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>                 = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool>              = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	auto trace(ExecutionPolicy&& policy, NodeType node, RandomIt first, RandomIt last,
	           Pred const& pred, HitFun hit_f, bool only_exists = true) const
	{
		using ret_type = std::conditional_t<
		    is_pair_v<std::invoke_result_t<HitFun, Node, Ray3>>,
		    std::pair<Node, typename std::invoke_result_t<HitFun, Node, Ray3>::second_type>,
		    Node>;
		std::vector<ret_type> nodes(std::distance(first, last));
		trace(std::forward<ExecutionPolicy>(policy), node, first, last, nodes.begin(), pred,
		      hit_f, only_exists);
		return nodes;
	}

	template <
	    class ExecutionPolicy, class NodeType, class RayRange, class Pred,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>              = true,
	    std::enable_if_t<pred::is_pred_v<Pred, Octree, Node>, bool>                  = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	auto trace(ExecutionPolicy&& policy, NodeType node, RayRange const& r, Pred const& pred,
	           bool only_exists = true) const
	{
		using std::begin;
		using std::end;
		return trace(std::forward<ExecutionPolicy>(policy), node, begin(r), end(r), pred,
		             only_exists);
	}

	template <
	    class ExecutionPolicy, class NodeType, class RayRange, class Pred, class HitFun,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>              = true,
	    std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>                 = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool>              = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	auto trace(ExecutionPolicy&& policy, NodeType node, RayRange const& r, Pred const& pred,
	           HitFun hit_f, bool only_exists = true) const
	{
		using std::begin;
		using std::end;
		return trace(std::forward<ExecutionPolicy>(policy), node, begin(r), end(r), pred,
		             hit_f, only_exists);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Trace index                                     |
	|                                                                                     |
	**************************************************************************************/

	template <
	    class InnerFun, class HitFun,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool> = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>           = true>
	auto trace(Ray3 ray, InnerFun inner_f, HitFun hit_f) const
	{
		return trace(Base::index(), ray, inner_f, hit_f);
	}

	template <
	    class InputIt, class OutputIt, class InnerFun, class HitFun,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool> = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>           = true>
	OutputIt trace(InputIt first, InputIt last, OutputIt d_first, InnerFun inner_f,
	               HitFun hit_f) const
	{
		return trace(Base::index(), first, last, d_first, inner_f, hit_f);
	}

	template <
	    class InputIt, class InnerFun, class HitFun,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool> = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>           = true>
	auto trace(InputIt first, InputIt last, InnerFun inner_f, HitFun hit_f) const
	{
		return trace(Base::index(), first, last, inner_f, hit_f);
	}

	template <
	    class RayRange, class InnerFun, class HitFun,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool> = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>           = true>
	auto trace(RayRange const& r, InnerFun inner_f, HitFun hit_f) const
	{
		using std::begin;
		using std::end;
		return trace(Base::index(), begin(r), end(r), inner_f, hit_f);
	}

	template <
	    class NodeType, class InnerFun, class HitFun,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>            = true,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool> = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>           = true>
	auto trace(NodeType node, Ray3 ray, InnerFun inner_f, HitFun hit_f) const
	{
		using ret_type = std::conditional_t<
		    is_pair_v<std::invoke_result_t<HitFun, Index, Ray3>>,
		    std::pair<Index, typename std::invoke_result_t<HitFun, Index, Ray3>::second_type>,
		    Index>;

		if constexpr (!std::is_same_v<Index, std::decay_t<NodeType>>) {
			// Unless NodeType is Index, we need to check that the node actually exists
			if (!Base::exists(node)) {
				return ret_type{};
			}
		}

		Index n = Base::index(node);

		auto [t0, t1, a] = traceInit(n, ray);
		return trace(n, t0, t1, a, ray, inner_f, hit_f);
	}

	template <
	    class NodeType, class InputIt, class OutputIt, class InnerFun, class HitFun,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>            = true,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool> = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>           = true>
	OutputIt trace(NodeType node, InputIt first, InputIt last, OutputIt d_first,
	               InnerFun inner_f, HitFun hit_f) const
	{
		// TODO: using ret_type = std::conditional_t<
		//     is_pair_v<std::invoke_result_t<HitFun, Index, Ray3>>,
		//     std::pair<Index, typename std::invoke_result_t<HitFun, Index,
		//     Ray3>::second_type>, Index>;

		using ret_type = Index;

		if constexpr (!std::is_same_v<Index, std::decay_t<NodeType>>) {
			// Unless NodeType is Index, we need to check that the node actually exists
			if (!Base::exists(node)) {
				return ret_type{};
			}
		}

		Index n = Base::index(node);

		auto center      = Base::center(n);
		auto half_length = Base::halfLength(n);

		return std::transform(first, last, d_first, [&](Ray3 const& ray) {
			auto [t0, t1, a] = traceInit(ray, center, half_length);
			return trace(n, t0, t1, a, ray, inner_f, hit_f);
		});
	}

	template <
	    class NodeType, class InputIt, class InnerFun, class HitFun,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>            = true,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool> = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>           = true>
	auto trace(NodeType node, InputIt first, InputIt last, InnerFun inner_f,
	           HitFun hit_f) const
	{
		using ret_type = std::conditional_t<
		    is_pair_v<std::invoke_result_t<HitFun, Index, Ray3>>,
		    std::pair<Index, typename std::invoke_result_t<HitFun, Index, Ray3>::second_type>,
		    Index>;

		std::vector<ret_type> nodes(std::distance(first, last));
		trace(node, first, last, nodes.begin(), inner_f, hit_f);
		return nodes;
	}

	template <
	    class NodeType, class RayRange, class InnerFun, class HitFun,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>            = true,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool> = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>           = true>
	auto trace(NodeType node, RayRange const& r, InnerFun inner_f, HitFun hit_f) const
	{
		using std::begin;
		using std::end;
		return trace(node, begin(r), end(r), inner_f, hit_f);
	}

	template <
	    class ExecutionPolicy, class RandomIt1, class RandomIt2, class InnerFun,
	    class HitFun,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool>   = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>             = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	RandomIt2 trace(ExecutionPolicy&& policy, RandomIt1 first, RandomIt1 last,
	                RandomIt2 d_first, InnerFun inner_f, HitFun hit_f) const
	{
		return trace(std::forward<ExecutionPolicy>(policy), Base::node(), first, last,
		             d_first, inner_f, hit_f);
	}

	template <
	    class ExecutionPolicy, class RandomIt, class InnerFun, class HitFun,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool>   = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>             = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	auto trace(ExecutionPolicy&& policy, RandomIt first, RandomIt last, InnerFun inner_f,
	           HitFun hit_f) const
	{
		return trace(std::forward<ExecutionPolicy>(policy), Base::index(), first, last,
		             inner_f, hit_f);
	}

	template <
	    class ExecutionPolicy, class RayRange, class InnerFun, class HitFun,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool>   = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>             = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	auto trace(ExecutionPolicy&& policy, RayRange const& r, InnerFun inner_f,
	           HitFun hit_f) const
	{
		using std::begin;
		using std::end;
		return trace(std::forward<ExecutionPolicy>(policy), begin(r), end(r), inner_f, hit_f);
	}

	template <
	    class ExecutionPolicy, class NodeType, class RandomIt1, class RandomIt2,
	    class InnerFun, class HitFun,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>              = true,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool>   = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>             = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	RandomIt2 trace(ExecutionPolicy&& policy, NodeType node, RandomIt1 first,
	                RandomIt1 last, RandomIt2 d_first, InnerFun inner_f, HitFun hit_f) const
	{
		if constexpr (std::is_same_v<execution::sequenced_policy,
		                             std::decay_t<ExecutionPolicy>>) {
			return trace(node, first, last, d_first, inner_f, hit_f);
		}

#if !defined(UFO_TBB) && !defined(UFO_OMP)
		return trace(node, first, last, d_first, inner_f, hit_f);
#endif

		// TODO: using ret_type = std::conditional_t<
		//     is_pair_v<std::invoke_result_t<HitFun, Index, Ray3>>,
		//     std::pair<Index, typename std::invoke_result_t<HitFun, Index,
		//     Ray3>::second_type>, Index>;

		using ret_type = Index;

		if constexpr (!std::is_same_v<Index, std::decay_t<NodeType>>) {
			// Unless NodeType is Index, we need to check that the node actually exists
			if (!Base::exists(node)) {
				return ret_type{};
			}
		}

		Index n = Base::index(node);

		auto center      = Base::center(n);
		auto half_length = Base::halfLength(n);

#if defined(UFO_TBB)
		return std::transform(std::forward<ExecutionPolicy>(policy), first, last, d_first,
		                      [&](Ray3 const& ray) {
			                      auto [t0, t1, a] = traceInit(ray, center, half_length);
			                      return trace(n, t0, t1, a, ray, inner_f, hit_f);
		                      });
#elif defined(UFO_OMP)
#pragma omp parallel for
		for (; first != last; ++d_first, ++first) {
			Ray3 const& ray  = *first;
			auto [t0, t1, a] = traceInit(ray, center, half_length);
			*d_first         = trace(n, t0, t1, a, ray, inner_f, hit_f);
		}
#endif
	}

	template <
	    class ExecutionPolicy, class NodeType, class RandomIt, class InnerFun, class HitFun,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>              = true,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool>   = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>             = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	auto trace(ExecutionPolicy&& policy, NodeType node, RandomIt first, RandomIt last,
	           InnerFun inner_f, HitFun hit_f) const
	{
		using ret_type = std::conditional_t<
		    is_pair_v<std::invoke_result_t<HitFun, Index, Ray3>>,
		    std::pair<Index, typename std::invoke_result_t<HitFun, Index, Ray3>::second_type>,
		    Index>;

		std::vector<ret_type> nodes(std::distance(first, last));
		trace(std::forward<ExecutionPolicy>(policy), node, first, last, nodes.begin(),
		      inner_f, hit_f);
		return nodes;
	}

	template <
	    class ExecutionPolicy, class NodeType, class RayRange, class InnerFun, class HitFun,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>              = true,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool>   = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>             = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	auto trace(ExecutionPolicy&& policy, NodeType node, RayRange const& r, InnerFun inner_f,
	           HitFun hit_f) const
	{
		using std::begin;
		using std::end;
		return trace(std::forward<ExecutionPolicy>(policy), node, begin(r), end(r), inner_f,
		             hit_f);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Image                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Add a Camera class, put it in a repo called UFOVision (together with Image and
	// Color)

	template <class Pred,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool> = true>
	Image<Node> image(Pred const& pred, Pose3f const& pose, std::size_t rows,
	                  std::size_t cols, float vertical_fov, float near_clip, float far_clip,
	                  Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0},
	                  Vec3f forward = {1, 0, 0}, bool only_exists = true) const
	{
		return image(Base::node(), pred, pose, rows, cols, vertical_fov, near_clip, far_clip,
		             up, right, forward, only_exists);
	}

	template <class Pred, class HitFun,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>    = true,
	          std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool> = true>
	auto image(Pred const& pred, HitFun hit_f, Pose3f const& pose, std::size_t rows,
	           std::size_t cols, float vertical_fov, float near_clip, float far_clip,
	           Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0}, Vec3f forward = {1, 0, 0},
	           bool only_exists = true) const
	{
		return image(Base::node(), pred, hit_f, pose, rows, cols, vertical_fov, near_clip,
		             far_clip, up, right, forward, only_exists);
	}

	template <class NodeType, class Pred,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>    = true>
	Image<Node> image(NodeType node, Pred const& pred, Pose3f const& pose, std::size_t rows,
	                  std::size_t cols, float vertical_fov, float near_clip, float far_clip,
	                  Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0},
	                  Vec3f forward = {1, 0, 0}, bool only_exists = true) const
	{
		return image(
		    node, pred, [](Node, Ray3) -> void {}, pose, rows, cols, vertical_fov, near_clip,
		    far_clip, up, right, forward, only_exists);
	}

	template <class NodeType, class Pred, class HitFun,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>    = true,
	          std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool> = true>
	auto image(NodeType node, Pred const& pred, HitFun hit_f, Pose3f const& pose,
	           std::size_t rows, std::size_t cols, float vertical_fov, float near_clip,
	           float far_clip, Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0},
	           Vec3f forward = {1, 0, 0}, bool only_exists = true) const
	{
		float aspect_ratio = cols / static_cast<float>(rows);

		float e = far_clip * std::tan(vertical_fov / 2);

		up *= e;
		right *= e * aspect_ratio;
		forward *= far_clip;

		Vec3f tl         = up - right + forward;
		Vec3f tr         = up + right + forward;
		Vec3f bl         = -up - right + forward;
		tl               = transform(pose, tl);
		tr               = transform(pose, tr);
		bl               = transform(pose, bl);
		Vec3f right_dir  = tr - tl;
		Vec3f bottom_dir = bl - tl;

		Image<Ray3> rays(rows, cols, Ray3(pose.position, {}));
		for (std::size_t row{}; row < rows; ++row) {
			auto r = (row + 0.5f) / rows;
			for (std::size_t col{}; col < cols; ++col) {
				auto c                   = (col + 0.5f) / cols;
				auto end                 = tl + right_dir * c + bottom_dir * r;
				rays(row, col).direction = normalize(end - pose.position);
			}
		};

		using ret_type = std::conditional_t<
		    is_pair_v<std::invoke_result_t<HitFun, Node, Ray3>>,
		    std::pair<Node, typename std::invoke_result_t<HitFun, Node, Ray3>::second_type>,
		    Node>;

		Image<ret_type> img(rows, cols);

		trace(node, rays.begin(), rays.end(), img.begin(), pred, hit_f, only_exists);

		return img;
	}

	template <
	    class ExecutionPolicy, class Pred,
	    std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>                 = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	Image<Node> image(ExecutionPolicy&& policy, Pred const& pred, Pose3f const& pose,
	                  std::size_t rows, std::size_t cols, float vertical_fov,
	                  float near_clip, float far_clip, Vec3f up = {0, 0, 1},
	                  Vec3f right = {0, -1, 0}, Vec3f forward = {1, 0, 0},
	                  bool only_exists = true) const
	{
		return image(std::forward<ExecutionPolicy>(policy), Base::node(), pred, pose, rows,
		             cols, vertical_fov, near_clip, far_clip, up, right, forward,
		             only_exists);
	}
	template <
	    class ExecutionPolicy, class Pred, class HitFun,
	    std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>                 = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool>              = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	auto image(ExecutionPolicy&& policy, Pred const& pred, HitFun hit_f, Pose3f const& pose,
	           std::size_t rows, std::size_t cols, float vertical_fov, float near_clip,
	           float far_clip, Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0},
	           Vec3f forward = {1, 0, 0}, bool only_exists = true) const
	{
		return image(std::forward<ExecutionPolicy>(policy), Base::node(), pred, hit_f, pose,
		             rows, cols, vertical_fov, near_clip, far_clip, up, right, forward,
		             only_exists);
	}

	template <
	    class ExecutionPolicy, class NodeType, class Pred,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>              = true,
	    std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>                 = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	Image<Node> image(ExecutionPolicy&& policy, NodeType node, Pred const& pred,
	                  Pose3f const& pose, std::size_t rows, std::size_t cols,
	                  float vertical_fov, float near_clip, float far_clip,
	                  Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0},
	                  Vec3f forward = {1, 0, 0}, bool only_exists = true) const
	{
		return image(
		    std::forward<ExecutionPolicy>(policy), node, pred, [](Node, Ray3) -> void {},
		    pose, rows, cols, vertical_fov, near_clip, far_clip, up, right, forward,
		    only_exists);
	}

	template <
	    class ExecutionPolicy, class NodeType, class Pred, class HitFun,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>              = true,
	    std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>                 = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool>              = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	auto image(ExecutionPolicy&& policy, NodeType node, Pred const& pred, HitFun hit_f,
	           Pose3f const& pose, std::size_t rows, std::size_t cols, float vertical_fov,
	           float near_clip, float far_clip, Vec3f up = {0, 0, 1},
	           Vec3f right = {0, -1, 0}, Vec3f forward = {1, 0, 0},
	           bool only_exists = true) const
	{
		if constexpr (std::is_same_v<execution::sequenced_policy,
		                             std::decay_t<ExecutionPolicy>>) {
			return image(node, pred, hit_f, pose, rows, cols, vertical_fov, near_clip, far_clip,
			             up, right, forward, only_exists);
		}

#if !defined(UFO_TBB) && !defined(UFO_OMP)
		return image(node, pred, hit_f, pose, rows, cols, vertical_fov, near_clip, far_clip,
		             up, right, forward, only_exists);
#endif

		float aspect_ratio = cols / static_cast<float>(rows);

		float e = far_clip * std::tan(vertical_fov / 2);

		up *= e;
		right *= e * aspect_ratio;
		forward *= far_clip;

		Vec3f tl         = up - right + forward;
		Vec3f tr         = up + right + forward;
		Vec3f bl         = -up - right + forward;
		tl               = transform(pose, tl);
		tr               = transform(pose, tr);
		bl               = transform(pose, bl);
		Vec3f right_dir  = tr - tl;
		Vec3f bottom_dir = bl - tl;

		Image<Ray3> rays(rows, cols, Ray3(pose.position, {}));

#if defined(UFO_TBB)
		std::vector<std::size_t> indices(rows);
		std::iota(indices.begin(), indices.end(), 0);

		std::for_each(policy, indices.begin(), indices.end(),
		              [tl, right_dir, bottom_dir, rows, cols, &rays](auto row) {
			              auto r = (row + 0.5f) / rows;
			              auto a = tl + bottom_dir * r;
			              for (std::size_t col{}; col < cols; ++col) {
				              auto c                   = (col + 0.5f) / cols;
				              auto end                 = tl + right_dir * c + bottom_dir * r;
				              rays(row, col).direction = normalize(end - rays(row, col).origin);
			              }
		              });
#elif defined(UFO_OMP)
#pragma omp parallel for
		for (std::size_t row{}; row < rows; ++row) {
			auto r = (row + 0.5f) / rows;
			for (std::size_t col{}; col < cols; ++col) {
				auto c                   = (col + 0.5f) / cols;
				auto end                 = tl + right_dir * c + bottom_dir * r;
				rays(row, col).direction = normalize(end - pose.position);
			}
		};
#endif

		using ret_type = std::conditional_t<
		    is_pair_v<std::invoke_result_t<HitFun, Node, Ray3>>,
		    std::pair<Node, typename std::invoke_result_t<HitFun, Node, Ray3>::second_type>,
		    Node>;

		Image<ret_type> img(rows, cols);

		trace(std::forward<ExecutionPolicy>(policy), node, rays.begin(), rays.end(),
		      img.begin(), pred, hit_f, only_exists);

		return img;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Image index                                     |
	|                                                                                     |
	**************************************************************************************/

	template <
	    class InnerFun, class HitFun,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool> = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>           = true>
	auto image(InnerFun inner_f, HitFun hit_f, Pose3f const& pose, std::size_t rows,
	           std::size_t cols, float vertical_fov, float near_clip, float far_clip,
	           Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0},
	           Vec3f forward = {1, 0, 0}) const
	{
		return image(Base::index(), inner_f, hit_f, pose, rows, cols, vertical_fov, near_clip,
		             far_clip, up, right, forward);
	}

	template <
	    class NodeType, class InnerFun, class HitFun,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>            = true,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool> = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>           = true>
	auto image(NodeType node, InnerFun inner_f, HitFun hit_f, Pose3f const& pose,
	           std::size_t rows, std::size_t cols, float vertical_fov, float near_clip,
	           float far_clip, Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0},
	           Vec3f forward = {1, 0, 0}) const
	{
		float aspect_ratio = cols / static_cast<float>(rows);

		float e = far_clip * std::tan(vertical_fov / 2);

		up *= e;
		right *= e * aspect_ratio;
		forward *= far_clip;

		Vec3f tl         = up - right + forward;
		Vec3f tr         = up + right + forward;
		Vec3f bl         = -up - right + forward;
		tl               = transform(pose, tl);
		tr               = transform(pose, tr);
		bl               = transform(pose, bl);
		Vec3f right_dir  = tr - tl;
		Vec3f bottom_dir = bl - tl;

		Image<Ray3> rays(rows, cols, Ray3(pose.position, {}));
		for (std::size_t row{}; row < rows; ++row) {
			auto r = (row + 0.5f) / rows;
			for (std::size_t col{}; col < cols; ++col) {
				auto c                   = (col + 0.5f) / cols;
				auto end                 = tl + right_dir * c + bottom_dir * r;
				rays(row, col).direction = normalize(end - pose.position);
			}
		};

		// TODO: using ret_type = std::conditional_t<
		//     is_pair_v<std::invoke_result_t<HitFun, Index, Ray3>>,
		//     std::pair<Index, typename std::invoke_result_t<HitFun, Index,
		//     Ray3>::second_type>, Index>;

		using ret_type = Index;

		Image<ret_type> img(rows, cols);

		trace(node, rays.begin(), rays.end(), img.begin(), inner_f, hit_f);

		return img;
	}

	//
	//
	// PARALLEL UNDER
	//
	//

	template <
	    class ExecutionPolicy, class InnerFun, class HitFun,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool>   = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>             = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	auto image(ExecutionPolicy&& policy, InnerFun inner_f, HitFun hit_f, Pose3f const& pose,
	           std::size_t rows, std::size_t cols, float vertical_fov, float near_clip,
	           float far_clip, Vec3f up = {0, 0, 1}, Vec3f right = {0, -1, 0},
	           Vec3f forward = {1, 0, 0}) const
	{
		return image(std::forward<ExecutionPolicy>(policy), Base::index(), inner_f, hit_f,
		             pose, rows, cols, vertical_fov, near_clip, far_clip, up, right, forward);
	}

	template <
	    class ExecutionPolicy, class NodeType, class InnerFun, class HitFun,
	    std::enable_if_t<Base::template is_node_type_v<NodeType>, bool>              = true,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool>   = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>             = true,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	auto image(ExecutionPolicy&& policy, NodeType node, InnerFun inner_f, HitFun hit_f,
	           Pose3f const& pose, std::size_t rows, std::size_t cols, float vertical_fov,
	           float near_clip, float far_clip, Vec3f up = {0, 0, 1},
	           Vec3f right = {0, -1, 0}, Vec3f forward = {1, 0, 0}) const
	{
		if constexpr (std::is_same_v<execution::sequenced_policy,
		                             std::decay_t<ExecutionPolicy>>) {
			return image(node, inner_f, hit_f, pose, rows, cols, vertical_fov, near_clip,
			             far_clip, up, right, forward);
		}

#if !defined(UFO_TBB) && !defined(UFO_OMP)
		return image(node, inner_f, hit_f, pose, rows, cols, vertical_fov, near_clip,
		             far_clip, up, right, forward);
#endif

		float aspect_ratio = cols / static_cast<float>(rows);

		float e = far_clip * std::tan(vertical_fov / 2);

		up *= e;
		right *= e * aspect_ratio;
		forward *= far_clip;

		Vec3f tl         = up - right + forward;
		Vec3f tr         = up + right + forward;
		Vec3f bl         = -up - right + forward;
		tl               = transform(pose, tl);
		tr               = transform(pose, tr);
		bl               = transform(pose, bl);
		Vec3f right_dir  = tr - tl;
		Vec3f bottom_dir = bl - tl;

		Image<Ray3> rays(rows, cols, Ray3(pose.position, {}));

#if defined(UFO_TBB)
		std::vector<std::size_t> indices(rows);
		std::iota(indices.begin(), indices.end(), 0);

		std::for_each(policy, indices.begin(), indices.end(),
		              [tl, right_dir, bottom_dir, rows, cols, &rays](auto row) {
			              auto r = (row + 0.5f) / rows;
			              auto a = tl + bottom_dir * r;
			              for (std::size_t col{}; col < cols; ++col) {
				              auto c                   = (col + 0.5f) / cols;
				              auto end                 = tl + right_dir * c + bottom_dir * r;
				              rays(row, col).direction = normalize(end - rays(row, col).origin);
			              }
		              });
#elif defined(UFO_OMP)
#pragma omp parallel for
		for (std::size_t row{}; row < rows; ++row) {
			auto r = (row + 0.5f) / rows;
			for (std::size_t col{}; col < cols; ++col) {
				auto c                   = (col + 0.5f) / cols;
				auto end                 = tl + right_dir * c + bottom_dir * r;
				rays(row, col).direction = normalize(end - pose.position);
			}
		};
#endif

		// TODO: using ret_type = std::conditional_t<
		//     is_pair_v<std::invoke_result_t<HitFun, Index, Ray3>>,
		//     std::pair<Index, typename std::invoke_result_t<HitFun, Index,
		//     Ray3>::second_type>, Index>;

		using ret_type = Index;

		Image<ret_type> img(rows, cols);

		trace(std::forward<ExecutionPolicy>(policy), node, rays.begin(), rays.end(),
		      img.begin(), inner_f, hit_f);

		return img;
	}

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

	/**************************************************************************************
	|                                                                                     |
	|                                        Trace                                        |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] std::tuple<Vec3f, Vec3f, offset_t> traceInit(Index node, Ray3 ray) const
	{
		return traceInit(ray, Base::center(node), Base::halfLength(node));
	}

	[[nodiscard]] std::tuple<Vec3f, Vec3f, offset_t> traceInit(Node node, Ray3 ray) const
	{
		return traceInit(ray, Base::center(node), Base::halfLength(node));
	}

	[[nodiscard]] std::tuple<Vec3f, Vec3f, offset_t> traceInit(Ray3 ray, Vec3f center,
	                                                           float half_length) const
	{
		offset_t a = offset_t(0 > ray.direction[0]) | (offset_t(0 > ray.direction[1]) << 1) |
		             (offset_t(0 > ray.direction[2]) << 2);

		Vec3f origin(0 > ray.direction[0] ? center[0] * 2 - ray.origin[0] : ray.origin[0],
		             0 > ray.direction[1] ? center[1] * 2 - ray.origin[1] : ray.origin[1],
		             0 > ray.direction[2] ? center[2] * 2 - ray.origin[2] : ray.origin[2]);

		Vec3f direction = abs(ray.direction);

		Vec3f t0;
		Vec3f t1;

		for (std::size_t i{}; direction.size() > i; ++i) {
			auto a = center[i] - half_length - origin[i];
			auto b = center[i] + half_length - origin[i];
			// TODO: Look at
			t0[i] = 0 == direction[i] ? 1e+25f * a : a / direction[i];
			t1[i] = 0 == direction[i] ? 1e+25f * b : b / direction[i];
		}

		return {t0, t1, a};
	}

	template <bool Exists, class NodeType, class Pred, class HitFun,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true,
	          std::enable_if_t<pred::is_pred_v<Pred, Derived, Node>, bool>    = true,
	          std::enable_if_t<std::is_invocable_v<HitFun, Node, Ray3>, bool> = true>
	auto trace(NodeType node, Vec3f t0, Vec3f t1, offset_t const a, Ray3 ray,
	           Pred const& pred, HitFun hit_f) const
	{
		auto firstNode = [](Vec3f t0, Vec3f tm) -> offset_t {
			// constexpr std::array<std::size_t, 3> lut_a{1, 0, 0};
			// constexpr std::array<std::size_t, 3> lut_b{2, 2, 1};

			auto max_comp = maxIndex(t0);

			// auto a = lut_a[max_comp];
			// auto b = lut_b[max_comp];

			std::size_t a = 0 == max_comp;
			std::size_t b = 2 - (2 == max_comp);

			return (static_cast<offset_t>(tm[a] < t0[max_comp]) << a) |
			       (static_cast<offset_t>(tm[b] < t0[max_comp]) << b);
		};

		constexpr std::array new_node_lut{
		    std::array<offset_t, 3>{1, 2, 4}, std::array<offset_t, 3>{8, 3, 5},
		    std::array<offset_t, 3>{3, 8, 6}, std::array<offset_t, 3>{8, 8, 7},
		    std::array<offset_t, 3>{5, 6, 8}, std::array<offset_t, 3>{8, 7, 8},
		    std::array<offset_t, 3>{7, 8, 8}, std::array<offset_t, 3>{8, 8, 8}};

		if (0 > min(t1) || max(t0) >= min(t1)) {
			if constexpr (std::is_void_v<std::invoke_result_t<HitFun, Node, Ray3>> ||
			              std::is_same_v<std::invoke_result_t<HitFun, Node, Ray3>, bool>) {
				return Node{};
			} else {
				return std::pair<
				    Node, typename std::invoke_result_t<HitFun, Node, Ray3>::second_type>{};
			}
		}

		if constexpr (std::is_void_v<std::invoke_result_t<HitFun, Node, Ray3>>) {
			if (pred::ValueCheck<Pred>::apply(pred, *this, node)) {
				return node;
			}
		} else if constexpr (std::is_same_v<std::invoke_result_t<HitFun, Node, Ray3>, bool>) {
			if (pred::ValueCheck<Pred>::apply(pred, *this, node) && hit_f(node, ray)) {
				return node;
			}
		} else {
			if (std::invoke_result_t<HitFun, Node, Ray3> res;
			    pred::ValueCheck<Pred>::apply(pred, *this, node) &&
			    (res = hit_f(node, ray)).first) {
				return std::pair<Node,
				                 typename std::invoke_result_t<HitFun, Node, Ray3>::second_type>{
				    node, res.second};
			}
		}

		if constexpr (Exists) {
			if (Base::isLeaf(node.index()) ||
			    !pred::InnerCheck<Pred>::apply(pred, *this, node)) {
				if constexpr (std::is_void_v<std::invoke_result_t<HitFun, Node, Ray3>> ||
				              std::is_same_v<std::invoke_result_t<HitFun, Node, Ray3>, bool>) {
					return Node{};
				} else {
					return std::pair<
					    Node, typename std::invoke_result_t<HitFun, Node, Ray3>::second_type>{};
				}
			}
		} else {
			if (!pred::InnerCheck<Pred>::apply(pred, *this, node)) {
				if constexpr (std::is_void_v<std::invoke_result_t<HitFun, Node, Ray3>> ||
				              std::is_same_v<std::invoke_result_t<HitFun, Node, Ray3>, bool>) {
					return Node{};
				} else {
					return std::pair<
					    Node, typename std::invoke_result_t<HitFun, Node, Ray3>::second_type>{};
				}
			}
		}

		Vec3f tm = (t0 + t1) / 2.0f;

		offset_t cur_node = firstNode(t0, tm);

		if (8 <= cur_node) {
			if constexpr (std::is_void_v<std::invoke_result_t<HitFun, Node, Ray3>> ||
			              std::is_same_v<std::invoke_result_t<HitFun, Node, Ray3>, bool>) {
				return Node{};
			} else {
				return std::pair<
				    Node, typename std::invoke_result_t<HitFun, Node, Ray3>::second_type>{};
			}
		}

		std::array<std::tuple<NodeType, offset_t, Vec3f, Vec3f, Vec3f>,
		           Base::maxDepthLevels()>
		    stack;
		stack[0] = {node, cur_node, t0, t1, tm};

		for (int index{}; 0 <= index;) {
			std::tie(node, cur_node, t0, t1, tm) = stack[index];

			if constexpr (Exists) {
				node = Base::childUnsafe(node, cur_node ^ a);
			} else {
				node = Base::child(node, cur_node ^ a);
			}

			std::array mask{cur_node & offset_t(1), cur_node & offset_t(2),
			                cur_node & offset_t(4)};

			t0 = {mask[0] ? tm[0] : t0[0], mask[1] ? tm[1] : t0[1], mask[2] ? tm[2] : t0[2]};
			t1 = {mask[0] ? t1[0] : tm[0], mask[1] ? t1[1] : tm[1], mask[2] ? t1[2] : tm[2]};

			std::get<1>(stack[index]) = new_node_lut[cur_node][minIndex(t1)];
			index -= 8 <= std::get<1>(stack[index]);

			if (0 > min(t1)) {
				continue;
			}

			if constexpr (std::is_void_v<std::invoke_result_t<HitFun, Node, Ray3>>) {
				if (pred::ValueCheck<Pred>::apply(pred, *this, node)) {
					return node;
				}
			} else if constexpr (std::is_same_v<std::invoke_result_t<HitFun, Node, Ray3>,
			                                    bool>) {
				if (pred::ValueCheck<Pred>::apply(pred, *this, node) && hit_f(node, ray)) {
					return node;
				}
			} else {
				if (std::invoke_result_t<HitFun, Node, Ray3> res;
				    pred::ValueCheck<Pred>::apply(pred, *this, node) &&
				    (res = hit_f(node, ray)).first) {
					return std::pair<
					    Node, typename std::invoke_result_t<HitFun, Node, Ray3>::second_type>{
					    node, res.second};
				}
			}

			if constexpr (Exists) {
				if (Base::isLeaf(node.index()) ||
				    !pred::InnerCheck<Pred>::apply(pred, *this, node)) {
					continue;
				}
			} else {
				if (!pred::InnerCheck<Pred>::apply(pred, *this, node)) {
					continue;
				}
			}

			tm = (t0 + t1) / 2.0f;

			cur_node = firstNode(t0, tm);

			stack[index + 1] = {node, cur_node, t0, t1, tm};
			index += 8 > cur_node;
		}

		if constexpr (std::is_void_v<std::invoke_result_t<HitFun, Node, Ray3>> ||
		              std::is_same_v<std::invoke_result_t<HitFun, Node, Ray3>, bool>) {
			return Node{};
		} else {
			return std::pair<Node,
			                 typename std::invoke_result_t<HitFun, Node, Ray3>::second_type>{};
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Trace index                                     |
	|                                                                                     |
	**************************************************************************************/

	template <
	    class InnerFun, class HitFun,
	    std::enable_if_t<std::is_invocable_r_v<bool, InnerFun, Index, Ray3>, bool> = true,
	    std::enable_if_t<std::is_invocable_v<HitFun, Index, Ray3>, bool>           = true>
	auto trace(Index node, Vec3f t0, Vec3f t1, offset_t const a, Ray3 ray, InnerFun inner_f,
	           HitFun hit_f) const
	{
		auto firstNode = [](Vec3f t0, Vec3f tm) -> offset_t {
			// constexpr std::array<std::size_t, 3> lut_a{1, 0, 0};
			// constexpr std::array<std::size_t, 3> lut_b{2, 2, 1};

			auto max_comp = minIndex(t0);

			// auto a = lut_a[max_comp];
			// auto b = lut_b[max_comp];

			std::size_t a = 0 == max_comp;
			std::size_t b = 2 - (2 == max_comp);

			return (static_cast<offset_t>(tm[a] < t0[max_comp]) << a) |
			       (static_cast<offset_t>(tm[b] < t0[max_comp]) << b);
		};

		constexpr std::array new_node_lut{
		    std::array<offset_t, 3>{1, 2, 4}, std::array<offset_t, 3>{8, 3, 5},
		    std::array<offset_t, 3>{3, 8, 6}, std::array<offset_t, 3>{8, 8, 7},
		    std::array<offset_t, 3>{5, 6, 8}, std::array<offset_t, 3>{8, 7, 8},
		    std::array<offset_t, 3>{7, 8, 8}, std::array<offset_t, 3>{8, 8, 8}};

		if (0 > min(t1) || max(t0) >= min(t1)) {
			if constexpr (std::is_same_v<std::invoke_result_t<HitFun, Index, Ray3>, bool>) {
				return Index{};
			} else {
				return std::pair<
				    Index, typename std::invoke_result_t<HitFun, Index, Ray3>::second_type>{};
			}
		}

		if constexpr (std::is_same_v<std::invoke_result_t<HitFun, Index, Ray3>, bool>) {
			if (hit_f(node, ray)) {
				return node;
			}
		} else {
			if (std::invoke_result_t<HitFun, Index, Ray3> res; (res = hit_f(node, ray)).first) {
				return std::pair<Index,
				                 typename std::invoke_result_t<HitFun, Index, Ray3>::second_type>{
				    node, res.second};
			}
		}

		if (Base::isLeaf(node) || !inner_f(node, ray)) {
			if constexpr (std::is_same_v<std::invoke_result_t<HitFun, Index, Ray3>, bool>) {
				return Index{};
			} else {
				return std::pair<
				    Index, typename std::invoke_result_t<HitFun, Index, Ray3>::second_type>{};
			}
		}

		Vec3f tm = (t0 + t1) / 2.0f;

		offset_t cur_node = firstNode(t0, tm);

		if (8 <= cur_node) {
			if constexpr (std::is_same_v<std::invoke_result_t<HitFun, Index, Ray3>, bool>) {
				return Index{};
			} else {
				return std::pair<
				    Index, typename std::invoke_result_t<HitFun, Index, Ray3>::second_type>{};
			}
		}

		std::array<std::tuple<Index, offset_t, Vec3f, Vec3f, Vec3f>,
		           Base::maxNumDepthLevels()>
		    stack;
		stack[0] = {node, cur_node, t0, t1, tm};

		for (int index{}; 0 <= index;) {
			std::tie(node, cur_node, t0, t1, tm) = stack[index];

			node = Base::child(node, cur_node ^ a);

			std::array mask{cur_node & offset_t(1), cur_node & offset_t(2),
			                cur_node & offset_t(4)};

			t0 = {mask[0] ? tm[0] : t0[0], mask[1] ? tm[1] : t0[1], mask[2] ? tm[2] : t0[2]};
			t1 = {mask[0] ? t1[0] : tm[0], mask[1] ? t1[1] : tm[1], mask[2] ? t1[2] : tm[2]};

			std::get<1>(stack[index]) = new_node_lut[cur_node][minIndex(t1)];
			index -= 8 <= std::get<1>(stack[index]);

			if (0 > min(t1)) {
				continue;
			}

			if constexpr (std::is_same_v<std::invoke_result_t<HitFun, Index, Ray3>, bool>) {
				if (hit_f(node, ray)) {
					return node;
				}
			} else {
				if (std::invoke_result_t<HitFun, Index, Ray3> res;
				    (res = hit_f(node, ray)).first) {
					return std::pair<
					    Index, typename std::invoke_result_t<HitFun, Index, Ray3>::second_type>{
					    node, res.second};
				}
			}

			if (Base::isLeaf(node) || !inner_f(node, ray)) {
				continue;
			}

			tm = (t0 + t1) / 2.0f;

			cur_node = firstNode(t0, tm);

			stack[index + 1] = {node, cur_node, t0, t1, tm};
			index += 8 > cur_node;
		}

		if constexpr (std::is_same_v<std::invoke_result_t<HitFun, Index, Ray3>, bool>) {
			return Index{};
		} else {
			return std::pair<Index,
			                 typename std::invoke_result_t<HitFun, Index, Ray3>::second_type>{};
		}
	}
};
}  // namespace ufo

#endif  // UFO_CONTAINER_OCTREE_HPP