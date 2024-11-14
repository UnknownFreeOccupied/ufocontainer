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
#include <ufo/container/tree/base.hpp>
#include <ufo/container/tree/detail/tree.hpp>
#include <ufo/vision/camera.hpp>
#include <ufo/vision/image.hpp>

namespace ufo
{
template <class Derived, class... Ts>
class Tree<Derived, 4, Ts...> : public TreeBase<Derived, 4, Ts...>
{
 protected:
	using Base = TreeBase<Derived, 4, Ts...>;

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
		Base::trace(node, rays4d.begin(), rays4d.end(), image.begin(), inner_f, hit_f, miss);
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
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void render(ExecutionPolicy&& policy, float w, Camera const& camera, Image<T>& image,
	            InnerFun inner_f, HitFun hit_f, T const& miss) const
	{
		render(std::forward<ExecutionPolicy>(policy), Base::index(), w, camera, image,
		       inner_f, hit_f, miss);
	}

	template <
	    class ExecutionPolicy, class InnerFun, class HitFun, class T,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
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

		Base::trace(std::forward<ExecutionPolicy>(policy), node, rays4d.begin(), rays4d.end(),
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

	Tree(length_t leaf_node_length, depth_t num_depth_levels)
	    : Base(leaf_node_length, num_depth_levels)
	{
	}

	Tree(Tree const& other) = default;

	Tree(Tree&& other) = default;

	template <class Derived2, class... Ts2>
	Tree(Tree<Derived2, 4, Ts2...> const& other) : Base(other)
	{
	}

	template <class Derived2, class... Ts2>
	Tree(Tree<Derived2, 4, Ts2...>&& other) : Base(std::move(other))
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
	Tree& operator=(Tree<Derived2, 4, Ts2...> const& rhs)
	{
		Base::operator=(rhs);
		return *this;
	}

	template <class Derived2, class... Ts2>
	Tree& operator=(Tree<Derived2, 4, Ts2...>&& rhs)
	{
		Base::operator=(std::move(rhs));
		return *this;
	}
};
}  // namespace ufo

#endif  // UFO_CONTAINER_HEXTREE_HPP