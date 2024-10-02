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

#ifndef UFO_CONTAINER_HEXTREE_HPP
#define UFO_CONTAINER_HEXTREE_HPP

// UFO
#include <ufo/container/tree/tree.hpp>
#include <ufo/container/tree/type.hpp>
#include <ufo/vision/camera.hpp>
#include <ufo/vision/image.hpp>

namespace ufo
{
template <class Derived, template <TreeType> class Block>
class Hextree : public Tree<Derived, Block<TreeType::HEX>>
{
	using Base = Tree<Derived, Block<TreeType::HEX>>;

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
	|                                        Trace                                        |
	|                                                                                     |
	**************************************************************************************/

	// TODO: Implement the predicate once as well

	template <class InnerFun, class HitFun, class T>
	[[nodiscard]] T trace(Ray4 const& ray, InnerFun inner_f, HitFun hit_f,
	                      T const& miss) const
	{
		return trace(Base::index(), ray, inner_f, hit_f, miss);
	}

	template <class InputIt, class OutputIt, class InnerFun, class HitFun, class T>
	OutputIt trace(InputIt first, InputIt last, OutputIt d_first, InnerFun inner_f,
	               HitFun hit_f, T const& miss) const
	{
		return trace(Base::index(), first, last, d_first, inner_f, hit_f, miss);
	}

	template <class InputIt, class InnerFun, class HitFun, class T>
	[[nodiscard]] std::vector<T> trace(InputIt first, InputIt last, InnerFun inner_f,
	                                   HitFun hit_f, T const& miss) const
	{
		return trace(Base::index(), first, last, inner_f, hit_f, miss);
	}

	template <class NodeType, class InnerFun, class HitFun, class T,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] T trace(NodeType node, Ray4 const& ray, InnerFun inner_f, HitFun hit_f,
	                      T const& miss) const
	{
		if constexpr (!std::is_same_v<Index, std::decay_t<NodeType>>) {
			// Unless NodeType is Index, we need to check that the node actually exists
			if (!Base::exists(node)) {
				return miss;
			}
		}

		Index n = Base::index(node);

		auto wrapped_inner_f = [&ray, inner_f](Index node, float distance) {
			return inner_f(node, ray, distance);
		};

		auto wrapped_hit_f = [&ray, hit_f](Index node, float distance) {
			return hit_f(node, ray, distance);
		};

		auto params = traceInit(n, ray);
		return trace(n, params, wrapped_inner_f, wrapped_hit_f, miss);
	}

	template <class NodeType, class InputIt, class OutputIt, class InnerFun, class HitFun,
	          class T,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true>
	OutputIt trace(NodeType node, InputIt first, InputIt last, OutputIt d_first,
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
			auto wrapped_inner_f = [&ray, inner_f](Index node, float distance) {
				return inner_f(node, ray, distance);
			};

			auto wrapped_hit_f = [&ray, hit_f](Index node, float distance) {
				return hit_f(node, ray, distance);
			};

			auto params = traceInit(ray, center, half_length);
			return trace(n, params, wrapped_inner_f, wrapped_hit_f, miss);
		});
	}

	template <class NodeType, class InputIt, class InnerFun, class HitFun, class T,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] std::vector<T> trace(NodeType node, InputIt first, InputIt last,
	                                   InnerFun inner_f, HitFun hit_f, T const& miss) const
	{
		std::vector<T> nodes(std::distance(first, last));
		trace(node, first, last, nodes.begin(), inner_f, hit_f, miss);
		return nodes;
	}

	template <
	    class ExecutionPolicy, class RandomIt1, class RandomIt2, class InnerFun,
	    class HitFun, class T,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	RandomIt2 trace(ExecutionPolicy&& policy, RandomIt1 first, RandomIt1 last,
	                RandomIt2 d_first, InnerFun inner_f, HitFun hit_f, T const& miss) const
	{
		return trace(std::forward<ExecutionPolicy>(policy), Base::node(), first, last,
		             d_first, inner_f, hit_f, miss);
	}

	template <
	    class ExecutionPolicy, class RandomIt, class InnerFun, class HitFun, class T,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	[[nodiscard]] std::vector<T> trace(ExecutionPolicy&& policy, RandomIt first,
	                                   RandomIt last, InnerFun inner_f, HitFun hit_f,
	                                   T const& miss) const
	{
		return trace(std::forward<ExecutionPolicy>(policy), Base::index(), first, last,
		             inner_f, hit_f, miss);
	}

	template <class ExecutionPolicy, class NodeType, class RandomIt1, class RandomIt2,
	          class InnerFun, class HitFun, class T>
	RandomIt2 trace(ExecutionPolicy&& policy, NodeType node, RandomIt1 first,
	                RandomIt1 last, RandomIt2 d_first, InnerFun inner_f, HitFun hit_f,
	                T const& miss) const
	{
		if constexpr (std::is_same_v<execution::sequenced_policy,
		                             std::decay_t<ExecutionPolicy>>) {
			return trace(node, first, last, d_first, inner_f, hit_f, miss);
		}

#if !defined(UFO_TBB) && !defined(UFO_OMP)
		return trace(node, first, last, d_first, inner_f, hit_f, miss);
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
		return std::transform(
		    std::forward<ExecutionPolicy>(policy), first, last, d_first,
		    [&](auto const& ray) {
			    auto wrapped_inner_f = [&ray, inner_f](Index node, float distance) {
				    return inner_f(node, ray, distance);
			    };

			    auto wrapped_hit_f = [&ray, hit_f](Index node, float distance) {
				    return hit_f(node, ray, distance);
			    };

			    auto params = traceInit(ray, center, half_length);
			    return trace(n, params, wrapped_inner_f, wrapped_hit_f, miss);
		    });
#elif defined(UFO_OMP)
		std::size_t size = std::distance(first, last);

#pragma omp parallel for
		for (std::size_t i = 0; i != size; ++i) {
			auto const& ray = first[i];

			auto wrapped_inner_f = [&ray, inner_f](Index node, float distance) {
				return inner_f(node, ray, distance);
			};

			auto wrapped_hit_f = [&ray, hit_f](Index node, float distance) {
				return hit_f(node, ray, distance);
			};
			auto params = traceInit(ray, center, half_length);
			d_first[i]  = trace(n, params, ray, wrapped_inner_f, wrapped_hit_f, miss);
		}

		return std::next(d_first, size);
#endif
	}

	template <class ExecutionPolicy, class NodeType, class RandomIt, class InnerFun,
	          class HitFun, class T>
	[[nodiscard]] std::vector<T> trace(ExecutionPolicy&& policy, NodeType node,
	                                   RandomIt first, RandomIt last, InnerFun inner_f,
	                                   HitFun hit_f, T const& miss) const
	{
		std::vector<T> nodes(std::distance(first, last));
		trace(std::forward<ExecutionPolicy>(policy), node, first, last, nodes.begin(),
		      inner_f, hit_f, miss);
		return nodes;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Render                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class InnerFun, class HitFun, class T>
	void render(float w, Camera const& camera, Image<T>& image, InnerFun inner_f,
	            HitFun hit_f, T const& miss) const
	{
		render(w, Base::index(), camera, image, inner_f, hit_f, miss);
	}

	template <class InnerFun, class HitFun, class T>
	[[nodiscard]] Image<T> render(float w, Camera const& camera, std::size_t rows,
	                              std::size_t cols, InnerFun inner_f, HitFun hit_f,
	                              T const& miss) const
	{
		return render(Base::index(), w, camera, rows, cols, inner_f, hit_f, miss);
	}

	template <class NodeType, class InnerFun, class HitFun, class T,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true>
	void render(NodeType node, float w, Camera const& camera, Image<T>& image,
	            InnerFun inner_f, HitFun hit_f, T const& miss) const
	{
		Image<Ray3> rays = camera.rays(image.rows(), image.cols());
		Image<Ray4> rays4d(image.rows(), image.cols());
		for (std::size_t row{}; image.rows() > row; ++row) {
			for (std::size_t col{}; image.cols() > col; ++col) {
				rays4d(row, col) =
				    Ray4(Vec4f(rays(row, col).origin, w), Vec4f(rays(row, col).direction, 0));
			}
		}
		trace(node, rays4d.begin(), rays4d.end(), image.begin(), inner_f, hit_f, miss);
	}

	template <class NodeType, class InnerFun, class HitFun, class T,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Image<T> render(NodeType node, float w, Camera const& camera,
	                              std::size_t rows, std::size_t cols, InnerFun inner_f,
	                              HitFun hit_f, T const& miss) const
	{
		Image<T> image(rows, cols);
		render(node, w, camera, image, inner_f, hit_f, miss);
		return image;
	}

	template <
	    class ExecutionPolicy, class InnerFun, class HitFun, class T,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	void render(ExecutionPolicy&& policy, float w, Camera const& camera, Image<T>& image,
	            InnerFun inner_f, HitFun hit_f, T const& miss) const
	{
		render(std::forward<ExecutionPolicy>(policy), Base::index(), w, camera, image,
		       inner_f, hit_f, miss);
	}

	template <
	    class ExecutionPolicy, class InnerFun, class HitFun, class T,
	    std::enable_if_t<is_execution_policy_v<std::decay_t<ExecutionPolicy>>, bool> = true>
	[[nodiscard]] Image<T> render(ExecutionPolicy&& policy, float w, Camera const& camera,
	                              std::size_t rows, std::size_t cols, InnerFun inner_f,
	                              HitFun hit_f, T const& miss) const
	{
		return render(std::forward<ExecutionPolicy>(policy), Base::index(), w, camera, rows,
		              cols, inner_f, hit_f, miss);
	}

	template <class ExecutionPolicy, class NodeType, class InnerFun, class HitFun, class T>
	void render(ExecutionPolicy&& policy, NodeType node, float w, Camera const& camera,
	            Image<T>& image, InnerFun inner_f, HitFun hit_f, T const& miss) const
	{
		auto rows = image.rows();
		auto cols = image.cols();

		Image<Ray3> rays = camera.rays(policy, rows, cols);
		Image<Ray4> rays4d(image.rows(), image.cols());
		for (std::size_t row{}; image.rows() > row; ++row) {
			for (std::size_t col{}; image.cols() > col; ++col) {
				rays4d(row, col) =
				    Ray4(Vec4f(rays(row, col).origin, w), Vec4f(rays(row, col).direction, 0));
			}
		}

		trace(std::forward<ExecutionPolicy>(policy), node, rays4d.begin(), rays4d.end(),
		      image.begin(), inner_f, hit_f, miss);
	}

	template <class ExecutionPolicy, class NodeType, class InnerFun, class HitFun, class T>
	[[nodiscard]] Image<T> render(ExecutionPolicy&& policy, NodeType node, float w,
	                              Camera const& camera, std::size_t rows, std::size_t cols,
	                              InnerFun inner_f, HitFun hit_f, T const& miss) const
	{
		Image<T> image(rows, cols);
		render(policy, node, w, camera, image, inner_f, hit_f, miss);
		return image;
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	Hextree(length_t leaf_node_length, depth_t num_depth_levels)
	    : Base(leaf_node_length, num_depth_levels)
	{
	}

	Hextree(Hextree const& other) = default;

	Hextree(Hextree&& other) = default;

	template <class Derived2>
	Hextree(Hextree<Derived2, Block> const& other) : Base(other)
	{
	}

	template <class Derived2>
	Hextree(Hextree<Derived2, Block>&& other) : Base(std::move(other))
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~Hextree() {}

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	Hextree& operator=(Hextree const& rhs) = default;

	Hextree& operator=(Hextree&& rhs) = default;

	template <class Derived2>
	Hextree& operator=(Hextree<Derived2, Block> const& rhs)
	{
		Base::operator=(rhs);
		return *this;
	}

	template <class Derived2>
	Hextree& operator=(Hextree<Derived2, Block>&& rhs)
	{
		Base::operator=(std::move(rhs));
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                         Swap                                        |
	|                                                                                     |
	**************************************************************************************/

	friend void swap(Hextree& lhs, Hextree& rhs)
	{
		Base::swap(static_cast<Base&>(lhs), static_cast<Base&>(rhs));
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Trace                                        |
	|                                                                                     |
	**************************************************************************************/

	struct TraceParams {
		Vec4f    t0;
		Vec4f    t1;
		unsigned a;
	};

	[[nodiscard]] TraceParams traceInit(Index node, Ray4 const& ray) const
	{
		return traceInit(ray, Base::center(node), Base::halfLength(node));
	}

	[[nodiscard]] TraceParams traceInit(Node node, Ray4 const& ray) const
	{
		return traceInit(ray, Base::center(node), Base::halfLength(node));
	}

	[[nodiscard]] static constexpr inline TraceParams traceInit(Ray4 const& ray,
	                                                            Vec4f       center,
	                                                            float half_length) noexcept
	{
		TraceParams params;

		Vec4f origin(0 > ray.direction[0] ? center[0] * 2 - ray.origin[0] : ray.origin[0],
		             0 > ray.direction[1] ? center[1] * 2 - ray.origin[1] : ray.origin[1],
		             0 > ray.direction[2] ? center[2] * 2 - ray.origin[2] : ray.origin[2],
		             0 > ray.direction[3] ? center[3] * 2 - ray.origin[3] : ray.origin[3]);

		auto direction = abs(ray.direction);

		for (std::size_t i{}; direction.size() > i; ++i) {
			auto a = center[i] - half_length - origin[i];
			auto b = center[i] + half_length - origin[i];
			// TODO: Look at
			params.t0[i] = 0 == direction[i] ? 1e+25 * a : a / direction[i];
			params.t1[i] = 0 == direction[i] ? 1e+25 * b : b / direction[i];
			// params.t0[i] = a * direction_reciprocal[i];
			// params.t1[i] = b * direction_reciprocal[i];
		}

		params.a =
		    (unsigned(0 > ray.direction[0]) << 0) | (unsigned(0 > ray.direction[1]) << 1) |
		    (unsigned(0 > ray.direction[2]) << 2) | (unsigned(0 > ray.direction[3]) << 3);

		return params;
	}

	[[nodiscard]] static constexpr inline unsigned firstNode(Vec4f t0, Vec4f tm) noexcept
	{
		auto        max_comp = maxIndex(t0);
		std::size_t a        = 1 - (1 == max_comp);
		std::size_t b        = 2 - (2 == max_comp);
		std::size_t c        = 3 - (3 == max_comp);
		return (static_cast<unsigned>(tm[a] < t0[max_comp]) << a) |
		       (static_cast<unsigned>(tm[b] < t0[max_comp]) << b) |
		       (static_cast<unsigned>(tm[c] < t0[max_comp]) << c);
	}

	template <class InnerFun, class HitFun, class T>
	[[nodiscard]] T trace(Index node, TraceParams const& params, InnerFun inner_f,
	                      HitFun hit_f, T const& miss) const
	{
		constexpr std::array new_node_lut{// -w, -z, -y, -x
		                                  // 0000
		                                  std::array<unsigned, 4>{1, 2, 4, 8},
		                                  // -w, -z, -y, +x
		                                  // 0001
		                                  std::array<unsigned, 4>{16, 3, 5, 9},
		                                  // -w, -z, +y, -x
		                                  // 0010
		                                  std::array<unsigned, 4>{3, 16, 6, 10},
		                                  // -w, -z, +y, +x
		                                  // 0011
		                                  std::array<unsigned, 4>{16, 16, 7, 11},
		                                  // -w, +z, -y, -x
		                                  // 0100
		                                  std::array<unsigned, 4>{5, 6, 16, 12},
		                                  // -w, +z, -y, +x
		                                  // 0101
		                                  std::array<unsigned, 4>{16, 7, 16, 13},
		                                  // -w, +z, +y, -x
		                                  // 0110
		                                  std::array<unsigned, 4>{7, 16, 16, 14},
		                                  // -w, +z, +y, +x
		                                  // 0111
		                                  std::array<unsigned, 4>{16, 16, 16, 15},
		                                  // +w, -z, -y, -x
		                                  // 1000
		                                  std::array<unsigned, 4>{9, 10, 12, 16},
		                                  // +w, -z, -y, +x
		                                  // 1001
		                                  std::array<unsigned, 4>{16, 11, 13, 16},
		                                  // +w, -z, +y, -x
		                                  // 1010
		                                  std::array<unsigned, 4>{11, 16, 14, 16},
		                                  // +w, -z, +y, +x
		                                  // 1011
		                                  std::array<unsigned, 4>{16, 16, 15, 16},
		                                  // +w, +z, -y, -x
		                                  // 1100
		                                  std::array<unsigned, 4>{13, 14, 16, 16},
		                                  // +w, +z, -y, +x
		                                  // 1101
		                                  std::array<unsigned, 4>{16, 15, 16, 16},
		                                  // +w, +z, +y, -x
		                                  // 1110
		                                  std::array<unsigned, 4>{15, 16, 16, 16},
		                                  // +w, +z, +y, +x
		                                  // 1111
		                                  std::array<unsigned, 4>{16, 16, 16, 16}};

		auto t0 = params.t0;
		auto t1 = params.t1;
		auto a  = params.a;

		if (max(t0) >= min(t1)) {
			return miss;
		}

		if (0.0f > min(t1)) {
			return miss;
		}

		float distance{};

		if (auto const& [hit, value] = hit_f(node, distance); hit) {
			return value;
		}

		if (Base::isLeaf(node) || !inner_f(node, distance)) {
			return miss;
		}

		auto tm = 0.5f * (t0 + t1);

		unsigned cur_node = firstNode(t0, tm);

		struct StackElement {
			Vec4f    t0;
			Vec4f    t1;
			Vec4f    tm;
			unsigned cur_node;
			Index    node;

			StackElement() = default;

			StackElement(Index node, unsigned cur_node, Vec4f t0, Vec4f t1, Vec4f tm)
			    : node(node), cur_node(cur_node), t0(t0), t1(t1), tm(tm)
			{
			}
		};

		std::array<StackElement, Base::maxNumDepthLevels()> stack;
		stack[0] = {node, cur_node, t0, t1, tm};

		for (int idx{}; 0 <= idx;) {
			node     = stack[idx].node;
			cur_node = stack[idx].cur_node;
			t0       = stack[idx].t0;
			t1       = stack[idx].t1;
			tm       = stack[idx].tm;

			node = Base::child(node, cur_node ^ a);

			std::bitset<4> mask(cur_node);
			t0 = {mask[0] ? tm[0] : t0[0], mask[1] ? tm[1] : t0[1], mask[2] ? tm[2] : t0[2],
			      mask[3] ? tm[3] : t0[3]};
			t1 = {mask[0] ? t1[0] : tm[0], mask[1] ? t1[1] : tm[1], mask[2] ? t1[2] : tm[2],
			      mask[3] ? t1[3] : tm[3]};

			distance = UFO_MAX(0.0f, max(t0));

			stack[idx].cur_node = new_node_lut[cur_node][minIndex(t1)];
			idx -= 16 <= stack[idx].cur_node;

			if (0.0f > min(t1)) {
				continue;
			}

			if (auto [hit, value] = hit_f(node, distance); hit) {
				return value;
			}

			if (Base::isLeaf(node) || !inner_f(node, distance)) {
				continue;
			}

			tm = 0.5f * (t0 + t1);

			cur_node = firstNode(t0, tm);

			stack[++idx] = {node, cur_node, t0, t1, tm};
		}

		return miss;
	}
};
}  // namespace ufo

#endif  // UFO_CONTAINER_HEXTREE_HPP