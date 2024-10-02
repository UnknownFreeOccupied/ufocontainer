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

#ifndef UFO_CONTAINER_QUADTREE_HPP
#define UFO_CONTAINER_QUADTREE_HPP

// UFO
#include <ufo/container/tree/tree.hpp>
#include <ufo/container/tree/type.hpp>
#include <ufo/vision/camera.hpp>
#include <ufo/vision/image.hpp>

namespace ufo
{
template <class Derived, template <TreeType> class Block>
class Quadtree : public Tree<Derived, Block<TreeType::QUAD>>
{
	using Base = Tree<Derived, Block<TreeType::QUAD>>;

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

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                       Render                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Implement the predicate once as well

	template <class InnerFun, class HitFun, class T>
	void render(Camera const& camera, Image<T>& image, InnerFun inner_f, HitFun hit_f,
	            T const& miss) const
	{
		render(Base::index(), camera, image, inner_f, hit_f, miss);
	}

	template <class InnerFun, class HitFun, class T>
	[[nodiscard]] Image<T> render(Camera const& camera, std::size_t rows, std::size_t cols,
	                              InnerFun inner_f, HitFun hit_f, T const& miss) const
	{
		return render(Base::index(), camera, rows, cols, inner_f, hit_f, miss);
	}

	template <class NodeType, class InnerFun, class HitFun, class T,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true>
	void render(NodeType node, Camera const& camera, Image<T>& image, InnerFun inner_f,
	            HitFun hit_f, T const& miss) const
	{
		Image<Ray3> rays = camera.rays(image.rows(), image.cols());
		trace3D(node, rays.begin(), rays.end(), image.begin(), inner_f, hit_f, miss);
	}

	template <class NodeType, class InnerFun, class HitFun, class T,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Image<T> render(NodeType node, Camera const& camera, std::size_t rows,
	                              std::size_t cols, InnerFun inner_f, HitFun hit_f,
	                              T const& miss) const
	{
		Image<T> image(rows, cols);
		render(node, camera, image, inner_f, hit_f, miss);
		return image;
	}

	template <
	    class ExecutionPolicy, class InnerFun, class HitFun, class T,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	void render(ExecutionPolicy&& policy, Camera const& camera, Image<T>& image,
	            InnerFun inner_f, HitFun hit_f, T const& miss) const
	{
		render(std::forward<ExecutionPolicy>(policy), Base::index(), camera, image, inner_f,
		       hit_f, miss);
	}

	template <
	    class ExecutionPolicy, class InnerFun, class HitFun, class T,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	[[nodiscard]] Image<T> render(ExecutionPolicy&& policy, Camera const& camera,
	                              std::size_t rows, std::size_t cols, InnerFun inner_f,
	                              HitFun hit_f, T const& miss) const
	{
		return render(std::forward<ExecutionPolicy>(policy), Base::index(), camera, rows,
		              cols, inner_f, hit_f, miss);
	}

	template <class ExecutionPolicy, class NodeType, class InnerFun, class HitFun, class T>
	void render(ExecutionPolicy&& policy, NodeType node, Camera const& camera,
	            Image<T>& image, InnerFun inner_f, HitFun hit_f, T const& miss) const
	{
		auto rows = image.rows();
		auto cols = image.cols();

		Image<Ray3> rays = camera.rays(policy, rows, cols);

		trace3D(std::forward<ExecutionPolicy>(policy), node, rays.begin(), rays.end(),
		        image.begin(), inner_f, hit_f, miss);
	}

	template <class ExecutionPolicy, class NodeType, class InnerFun, class HitFun, class T>
	[[nodiscard]] Image<T> render(ExecutionPolicy&& policy, NodeType node,
	                              Camera const& camera, std::size_t rows, std::size_t cols,
	                              InnerFun inner_f, HitFun hit_f, T const& miss) const
	{
		Image<T> image(rows, cols);
		render(policy, node, camera, image, inner_f, hit_f, miss);
		return image;
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	Quadtree(length_t leaf_node_length, depth_t num_depth_levels)
	    : Base(leaf_node_length, num_depth_levels)
	{
	}

	Quadtree(Quadtree const& other) = default;

	Quadtree(Quadtree&& other) = default;

	template <class Derived2>
	Quadtree(Quadtree<Derived2, Block> const& other) : Base(other)
	{
	}

	template <class Derived2>
	Quadtree(Quadtree<Derived2, Block>&& other) : Base(std::move(other))
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~Quadtree() {}

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	Quadtree& operator=(Quadtree const& rhs) = default;

	Quadtree& operator=(Quadtree&& rhs) = default;

	template <class Derived2>
	Quadtree& operator=(Quadtree<Derived2, Block> const& rhs)
	{
		Base::operator=(rhs);
		return *this;
	}

	template <class Derived2>
	Quadtree& operator=(Quadtree<Derived2, Block>&& rhs)
	{
		Base::operator=(std::move(rhs));
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         Swap                                        |
	|                                                                                     |
	**************************************************************************************/

	friend void swap(Quadtree& lhs, Quadtree& rhs)
	{
		Base::swap(static_cast<Base&>(lhs), static_cast<Base&>(rhs));
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Trace                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class NodeType, class InputIt, class OutputIt, class InnerFun, class HitFun,
	          class T,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true>
	OutputIt trace3D(NodeType node, InputIt first, InputIt last, OutputIt d_first,
	                 InnerFun inner_f, HitFun hit_f, T const& miss) const
	{
		if constexpr (!std::is_same_v<Index, std::decay_t<NodeType>>) {
			// Unless NodeType is Index, we need to check that the node actually exists
			if (!Base::exists(node)) {
				return miss;
			}
		}

		Index n = Base::index(node);

		auto center      = Base::center(n);
		auto half_length = Base::halfLength(n);

		return std::transform(first, last, d_first, [&](auto const& ray) {
			return trace3D(n, center, half_length, ray, inner_f, hit_f, miss);
		});
	}

	template <class ExecutionPolicy, class NodeType, class RandomIt1, class RandomIt2,
	          class InnerFun, class HitFun, class T>
	RandomIt2 trace3D(ExecutionPolicy&& policy, NodeType node, RandomIt1 first,
	                  RandomIt1 last, RandomIt2 d_first, InnerFun inner_f, HitFun hit_f,
	                  T const& miss) const
	{
		if constexpr (std::is_same_v<execution::sequenced_policy,
		                             std::decay_t<ExecutionPolicy>>) {
			return trace3D(node, first, last, d_first, inner_f, hit_f, miss);
		}

#if !defined(UFO_TBB) && !defined(UFO_OMP)
		return trace3D(node, first, last, d_first, inner_f, hit_f, miss);
#endif

		if constexpr (!std::is_same_v<Index, std::decay_t<NodeType>>) {
			// Unless NodeType is Index, we need to check that the node actually exists
			if (!Base::exists(node)) {
				return miss;
			}
		}

		Index n = Base::index(node);

		auto center      = Base::center(n);
		auto half_length = Base::halfLength(n);

#if defined(UFO_TBB)
		return std::transform(std::forward<ExecutionPolicy>(policy), first, last, d_first,
		                      [&](auto const& ray) {
			                      return trace3D(n, center, half_length, ray, inner_f, hit_f,
			                                     miss);
		                      });
#elif defined(UFO_OMP)
		std::size_t size = std::distance(first, last);

#pragma omp parallel for
		for (std::size_t i = 0; i != size; ++i) {
			auto const& ray = first[i];
			d_first[i]      = trace3D(n, center, half_length, ray, inner_f, hit_f, miss);
		}

		return std::next(d_first, size);
#endif
	}

	template <class InnerFun, class HitFun, class T>
	[[nodiscard]] T trace3D(Index node, Vec2f const& center, float half_length,
	                        Ray3 const& ray, InnerFun inner_f, HitFun hit_f,
	                        T const& miss) const
	{
		auto wrapped_inner_f = [&ray, inner_f](Index node, float distance) {
			return inner_f(node, ray, distance);
		};

		auto wrapped_hit_f = [&ray, hit_f](Index node, float distance) {
			return hit_f(node, ray, distance);
		};

		if (0.0f == ray.origin.z && 0.0f == ray.direction.z) {
			// 2D case
			Ray2 ray2(Vec2f(ray.origin), Vec2f(ray.direction));
			auto params = Base::traceInit(ray2, center, half_length);
			return Base::trace(node, params, wrapped_inner_f, wrapped_hit_f, miss);
		} else if (0.0f == ray.direction.z || sign(ray.origin.z) == sign(ray.direction.z)) {
			// Ray never intersects the 2D XY plane at Z=0
			return miss;
		}

		// Check where 3D ray intersects with 2D XY plane at z=0
		float t    = -ray.origin.z / ray.direction.z;
		Vec2f v    = Vec2f(ray.origin) + t * Vec2f(ray.direction);
		auto  code = Base::code(v);

		if (Base::code(node) != code.toDepth(Base::depth(node))) {
			// Different part of the tree, so return miss
			return miss;
		}

		for (int depth = Base::depth(node); 0 < depth; --depth) {
			if (auto [hit, value] = wrapped_hit_f(node, t); hit) {
				return value;
			} else if (Base::isLeaf(node) || !wrapped_inner_f(node, t)) {
				return miss;
			}

			node = Base::child(node, code.offset(depth - 1));
		}

		if (auto [hit, value] = wrapped_hit_f(node, t); hit) {
			return value;
		}

		return miss;
	}
};
}  // namespace ufo

#endif  // UFO_CONTAINER_QUADTREE_HPP