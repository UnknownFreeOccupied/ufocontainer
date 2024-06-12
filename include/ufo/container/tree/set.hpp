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

#ifndef UFO_CONTAINER_TREE_SET_HPP
#define UFO_CONTAINER_TREE_SET_HPP

// UFO
#include <ufo/container/tree/set_or_map.hpp>
#include <ufo/container/tree/type.hpp>

// STL
#include <algorithm>

namespace ufo
{
template <template <class, template <TreeType> class> class Tree>
class TreeSet : public TreeSetOrMap<TreeSet<Tree>, Tree>
{
	using Base = TreeSetOrMap<TreeSet<Tree>, Tree>;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	// UFO stuff
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

	TreeSet(length_t leaf_node_length = static_cast<length_t>(0.1),
	        depth_t  num_depth_levels = std::min(static_cast<depth_t>(17),
	                                             Base::maxNumDepthLevels()))
	    : Base(leaf_node_length, num_depth_levels)
	{
	}

	template <class InputIt>
	TreeSet(InputIt first, InputIt last,
	        length_t leaf_node_length = static_cast<length_t>(0.1),
	        depth_t  num_depth_levels = std::max(static_cast<depth_t>(17),
	                                             Base::maxNumDepthLevels()))
	    : Base(first, last, leaf_node_length, num_depth_levels)
	{
	}

	TreeSet(std::initializer_list<value_type> init,
	        length_t                          leaf_node_length = static_cast<length_t>(0.1),
	        depth_t num_depth_levels = std::max(static_cast<depth_t>(17),
	                                            Base::maxNumDepthLevels()))
	    : Base(init, leaf_node_length, num_depth_levels)
	{
	}

	template <class Range>
	TreeSet(Range const& range, length_t leaf_node_length = static_cast<length_t>(0.1),
	        depth_t num_depth_levels = std::max(static_cast<depth_t>(17),
	                                            Base::maxNumDepthLevels()))
	    : Base(range, leaf_node_length, num_depth_levels)
	{
	}

	TreeSet(TreeSet const&) = default;

	TreeSet(TreeSet&&) = default;

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~TreeSet() {}

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	TreeSet& operator=(TreeSet const&) = default;

	TreeSet& operator=(TreeSet&&) = default;

	/**************************************************************************************
	|                                                                                     |
	|                                      Modifiers                                      |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Exchanges the contents of the container with those of `other`.
	 *
	 * @param other	container to exchange the contents with
	 */
	void swap(TreeSet& other)
	{
		std::swap(static_cast<Base&>(*this), static_cast<Base&>(other));
	}
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_SET_HPP