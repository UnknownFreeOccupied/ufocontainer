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
#include <ufo/container/tree/base.hpp>
#include <ufo/container/tree/predicate/spatial.hpp>
#include <ufo/container/tree/detail/tree.hpp>
#include <ufo/geometry/shape/ray.hpp>
#include <ufo/math/transform3.hpp>
#include <ufo/utility/type_traits.hpp>
#include <ufo/vision/camera.hpp>
#include <ufo/vision/image.hpp>

// STL
#include <numeric>
#include <type_traits>
#include <utility>

namespace ufo
{
template <class Derived, class... Ts>
class Tree<Derived, 3, Ts...> : public TreeBase<Derived, 3, Ts...>
{
 protected:
	using Base = TreeBase<Derived, 3, Ts...>;

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
		Base::trace(node, rays.begin(), rays.end(), image.begin(), inner_f, hit_f, miss);
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
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void render(ExecutionPolicy&& policy, Camera const& camera, Image<T>& image,
	            InnerFun inner_f, HitFun hit_f, T const& miss) const
	{
		render(std::forward<ExecutionPolicy>(policy), Base::index(), camera, image, inner_f,
		       hit_f, miss);
	}

	template <
	    class ExecutionPolicy, class InnerFun, class HitFun, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
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

		constexpr std::size_t const step_size = 1;

		if constexpr (1 == step_size) {
			Base::trace(std::forward<ExecutionPolicy>(policy), node, rays.begin(), rays.end(),
			            image.begin(), inner_f, hit_f, miss);
		} else {
			// Image<std::pair<TreeIndex, float>> iter_image(
			//     rows, cols, std::make_pair(TreeIndex{}, std::numeric_limits<float>::max()));

			Index n = Base::index(node);

			auto center      = Base::center(n);
			auto half_length = Base::halfLength(n);

			std::vector<std::size_t> row_idx(rows);
			std::iota(row_idx.begin(), row_idx.end(), 0);

			// std::vector<std::size_t> row_idx_samples()

			// std::generate(row_idx.begin(), row_idx.end(), [n = 0]() mutable {
			// 	auto v = n;
			// 	n += 2;
			// 	return v;
			// });

			// struct IterElement {
			// 	Index node;
			// 	float distance;
			// 	T     value;

			// 	IterElement() = default;

			// 	IterElement(Index node, float distance, T value)
			// 	    : node(node), distance(distance), value(value)
			// 	{
			// 	}
			// };

			// auto tmp_miss = IterElement(TreeIndex{}, std::numeric_limits<float>::max(),
			// miss);

			// std::sample(rays.begin(), rays.end(), std::back_inserter(out), 4,
			//             std::mt19937{std::random_device{}()});

			// std::transform(
			//     policy, rays.begin(), rays.end(), image.begin(), [&, this](auto const& ray) {
			// 	    auto wrapped_inner_f = [inner_f, &ray](auto node, auto distance) {
			// 		    return inner_f(node, ray, distance);
			// 	    };

			// 	    auto wrapped_hit_f = [hit_f, &ray](auto node, auto distance) {
			// 		    return hit_f(node, ray, distance);
			// 		    // auto [hit, value] = hit_f(node, ray, distance);
			// 		    // return std::make_pair(hit, IterElement(node, distance, value));
			// 	    };

			// 	    auto params = Base::traceInit(ray, center, half_length);
			// 	    return Base::trace(n, params, wrapped_inner_f, wrapped_hit_f, miss);
			//     });

			static std::size_t s{};
			++s;

			std::for_each(policy, row_idx.begin(), row_idx.end(), [&](std::size_t row) {
				std::size_t offset = (row + s) % step_size;

				for (std::size_t col = offset; cols > col; col += step_size) {
					auto const& ray = rays(row, col);

					auto wrapped_inner_f = [inner_f, &ray](auto node, auto distance) {
						return inner_f(node, ray, distance);
					};

					auto wrapped_hit_f = [hit_f, &ray](auto node, auto distance) {
						return hit_f(node, ray, distance);
						// auto [hit, value] = hit_f(node, ray, distance);
						// return std::make_pair(hit, IterElement(node, distance, value));
					};

					auto params     = Base::traceInit(ray, center, half_length);
					image(row, col) = Base::trace(n, params, wrapped_inner_f, wrapped_hit_f, miss);
					// auto ie              = Base::trace(n, params, wrapped_inner_f, wrapped_hit_f,
					// tmp_miss); image(row, col)      = ie.value; iter_image(row, col) = {ie.node,
					// ie.distance};
				}
			});

			if constexpr (2 == step_size) {
				// std::for_each(
				//     policy, row_idx.begin() + 1, row_idx.end() - 1, [&](std::size_t row) {
				// 	    std::size_t const offset = 1 - (row % step_size);

				// 	    for (std::size_t col = step_size - offset; cols - 1 > col;
				// 	         col += step_size) {
				// 		    auto idx = minIndex(Vec4f(
				// 		        iter_image(row - 1, col).second, iter_image(row + 1, col).second,
				// 		        iter_image(row, col - 1).second, iter_image(row, col + 1).second));

				// 		    std::size_t min_row = 0 == idx ? row - 1 : (1 == idx ? row + 1 : row);
				// 		    std::size_t min_col = 2 == idx ? col - 1 : (3 == idx ? col + 1 : col);

				// 		    auto const& [node, distance] = iter_image(min_row, min_col);
				// 		    if (Base::valid(node)) {
				// 			    auto ray = rays(row, col);
				// 			    // float dist = distance(ray.origin, min(bounds));
				// 			    if (auto [hit, value] = hit_f(node, ray, distance); hit) {
				// 				    image(row, col) = value;
				// 			    } else {
				// 				    image(row, col) = miss;
				// 			    }
				// 		    } else {
				// 			    image(row, col) = miss;
				// 		    }
				// 	    }
				//     });
			} else {
				// std::for_each(
				//     policy, row_idx.begin() + 1, row_idx.end() - 1, [&](std::size_t row) {
				// 	    std::size_t const offset = row % step_size;

				// 	    for (std::size_t col = 1; cols - 1 > col; ++col) {
				// 		    if (offset == (col % step_size)) {
				// 			    continue;
				// 		    }

				// 		    auto idx = minIndex(Vec4f(
				// 		        iter_image(row - 1, col).second, iter_image(row + 1, col).second,
				// 		        iter_image(row, col - 1).second, iter_image(row, col + 1).second));

				// 		    std::size_t min_row = 0 == idx ? row - 1 : (1 == idx ? row + 1 : row);
				// 		    std::size_t min_col = 2 == idx ? col - 1 : (3 == idx ? col + 1 : col);

				// 		    auto const& [node, distance] = iter_image(min_row, min_col);
				// 		    if (Base::valid(node)) {
				// 			    auto ray = rays(row, col);
				// 			    // float dist = distance(ray.origin, min(bounds));
				// 			    if (auto [hit, value] = hit_f(node, ray, distance); hit) {
				// 				    image(row, col) = value;
				// 			    } else {
				// 				    image(row, col) = miss;
				// 			    }
				// 		    } else {
				// 			    image(row, col) = miss;
				// 		    }
				// 	    }
				//     });
			}
		}
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

	Tree(length_t leaf_node_length, depth_t num_depth_levels)
	    : Base(leaf_node_length, num_depth_levels)
	{
	}

	Tree(Tree const& other) = default;

	Tree(Tree&& other) = default;

	template <class Derived2, class... Ts2>
	Tree(Tree<Derived2, 3, Ts2...> const& other) : Base(other)
	{
	}

	template <class Derived2, class... Ts2>
	Tree(Tree<Derived2, 3, Ts2...>&& other) : Base(std::move(other))
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~Tree() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	Tree& operator=(Tree const& rhs) = default;

	Tree& operator=(Tree&& rhs) = default;

	template <class Derived2, class... Ts2>
	Tree& operator=(Tree<Derived2, 3, Ts2...> const& rhs)
	{
		Base::operator=(rhs);
		return *this;
	}

	template <class Derived2, class... Ts2>
	Tree& operator=(Tree<Derived2, 3, Ts2...>&& rhs)
	{
		Base::operator=(std::move(rhs));
		return *this;
	}
};
}  // namespace ufo

#endif  // UFO_CONTAINER_OCTREE_HPP