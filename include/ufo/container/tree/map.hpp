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

#ifndef UFO_CONTAINER_TREE_MAP_HPP
#define UFO_CONTAINER_TREE_MAP_HPP

// UFO
#include <ufo/container/tree/set_or_map.hpp>
#include <ufo/container/tree/type.hpp>

// STL
#include <algorithm>

namespace ufo
{
template <template <class, template <TreeType> class> class Tree, class T>
class TreeMap : public TreeSetOrMap<TreeMap<Tree, T>, Tree, T>
{
	using Base = TreeSetOrMap<TreeMap<Tree, T>, Tree, T>;

	//
	// Friends
	//

	friend Base;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	// UFO stuff
	using Index    = typename Base::Index;
	using Code     = typename Base::Code;
	using Point    = typename Base::Point;
	using depth_t  = typename Base::depth_t;
	using length_t = typename Base::length_t;

	// STL stuff
	using value_type      = typename Base::value_type;
	using reference       = typename Base::reference;
	using const_reference = typename Base::const_reference;
	using pointer         = typename Base::pointer;
	using const_pointer   = typename Base::const_pointer;
	using size_type       = typename Base::size_type;
	using difference_type = typename Base::difference_type;

	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	TreeMap(length_t leaf_node_length = static_cast<length_t>(0.1),
	        depth_t  num_depth_levels = std::max(static_cast<depth_t>(17),
	                                             Base::maxNumDepthLevels()))
	    : Base(leaf_node_length, num_depth_levels)
	{
	}

	template <class InputIt>
	TreeMap(InputIt first, InputIt last,
	        length_t leaf_node_length = static_cast<length_t>(0.1),
	        depth_t  num_depth_levels = std::max(static_cast<depth_t>(17),
	                                             Base::maxNumDepthLevels()))
	    : Base(first, last, leaf_node_length, num_depth_levels)
	{
	}

	TreeMap(std::initializer_list<value_type> init,
	        length_t                          leaf_node_length = static_cast<length_t>(0.1),
	        depth_t num_depth_levels = std::max(static_cast<depth_t>(17),
	                                            Base::maxNumDepthLevels()))
	    : Base(init, leaf_node_length, num_depth_levels)
	{
	}

	TreeMap(TreeMap const&) = default;

	TreeMap(TreeMap&&) = default;

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~TreeMap() {}

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	TreeMap& operator=(TreeMap const&) = default;

	TreeMap& operator=(TreeMap&&) = default;

	/**************************************************************************************
	|                                                                                     |
	|                                      Modifiers                                      |
	|                                                                                     |
	**************************************************************************************/

	template <class... Args>
	void emplace(Point point, Args&&... args)
	{
		Base::insert(value_type(point, T(std::forward<Args>(args)...)));
	}

	size_type erase(value_type const& value)
	{
		Code code = Base::code(value.first);

		std::array<Index, Base::maxNumDepthLevels()> trail;
		int const                                    root_depth = Base::depth();
		trail[root_depth]                                       = Base::index();
		for (auto depth = root_depth; 0 < depth; --depth) {
			if (Base::isLeaf(trail[depth])) {
				return 0;
			}
			trail[depth - 1] = Base::child(trail[depth], code.offset(depth - 1));
		}

		auto&     v           = values(trail[0]);
		size_type num_removed = v.size();
		v.remove_if([&value](auto const& x) {
			return equal(x.first, value.first) && x.second == value.second;
		});
		num_removed -= v.size();

		Base::size_ -= num_removed;

		Point min(std::numeric_limits<typename Point::scalar_t>::max());
		Point max(std::numeric_limits<typename Point::scalar_t>::lowest());
		for (auto const& [p, _] : v) {
			for (int i{}; Point::size() > i; ++i) {
				min[i] = UFO_MIN(min[i], p[i]);
				max[i] = UFO_MAX(max[i], p[i]);
			}
		}
		boundsMin(trail[0]) = min;
		boundsMax(trail[0]) = max;

		// Propagate
		for (int i = 1; root_depth >= i; ++i) {
			auto  block = trail[i - 1].pos;
			Point min   = boundsMin(Index(block, 0));
			Point max   = boundsMax(Index(block, 0));
			for (int o = 1; Base::branchingFactor() > o; ++o) {
				Point t_min = boundsMin(Index(block, o));
				Point t_max = boundsMax(Index(block, o));
				for (int j{}; Point::size() > j; ++j) {
					min[j] = UFO_MIN(min[j], t_min[j]);
					max[j] = UFO_MAX(max[j], t_max[j]);
				}
			}
			boundsMin(trail[i]) = min;
			boundsMax(trail[i]) = max;
		}

		return num_removed;
	}

	/*!
	 * @brief Exchanges the contents of the container with those of `other`.
	 *
	 * @param other	container to exchange the contents with
	 */
	void swap(TreeMap& other)
	{
		std::swap(static_cast<Base&>(*this), static_cast<Base&>(other));
	}
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_MAP_HPP