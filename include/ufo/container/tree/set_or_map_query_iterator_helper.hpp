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

#ifndef UFO_CONTAINER_TREE_SET_OR_MAP_QUERY_ITERATOR_HELPER_HPP
#define UFO_CONTAINER_TREE_SET_OR_MAP_QUERY_ITERATOR_HELPER_HPP

// UFO
#include <ufo/container/tree/map_nearest.hpp>
#include <ufo/container/tree/predicate.hpp>
#include <ufo/container/tree/set_nearest.hpp>
#include <ufo/utility/macros.hpp>

// STL
#include <cmath>
#include <iterator>
#include <memory>
#include <queue>
#include <type_traits>
#include <vector>

namespace ufo::detail
{
template <class TreeSetOrMap>
class TreeSetOrMapQueryNearestIteratorHelper
{
	template <class TreeSetOrMap2>
	friend class TreeSetOrMapQueryNearestIteratorHelper;

 public:
	static constexpr bool const IsConst = std::is_const_v<TreeSetOrMap>;
	static constexpr bool const IsMap   = TreeSetOrMap::IsMap;

	using RawIterator =
	    std::conditional_t<IsConst, typename TreeSetOrMap::const_raw_iterator,
	                       typename TreeSetOrMap::raw_iterator>;
	using Index = typename TreeSetOrMap::Index;

 public:
	//
	// Tags
	//

	using iterator_category = std::forward_iterator_tag;
	using difference_type   = std::ptrdiff_t;
	using value_type =
	    std::conditional_t<IsMap, TreeMapNearest<RawIterator>, TreeSetNearest<RawIterator>>;
	using reference = value_type&;
	using pointer   = value_type*;

 public:
	TreeSetOrMapQueryNearestIteratorHelper() = default;
	TreeSetOrMapQueryNearestIteratorHelper(TreeSetOrMapQueryNearestIteratorHelper const&) =
	    default;
	TreeSetOrMapQueryNearestIteratorHelper(TreeSetOrMapQueryNearestIteratorHelper&&) =
	    default;

	template <
	    class TreeSetOrMap2,
	    typename std::enable_if_t<
	        IsConst && !TreeSetOrMapQueryNearestIteratorHelper<TreeSetOrMap2>::IsConst &&
	            std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	        bool> = true>
	TreeSetOrMapQueryNearestIteratorHelper(
	    TreeSetOrMapQueryNearestIteratorHelper<TreeSetOrMap2> const& other)
	    : t_(other.t_)
	{
	}

	TreeSetOrMapQueryNearestIteratorHelper(TreeSetOrMap* t = nullptr) : t_(t) {}

	virtual ~TreeSetOrMapQueryNearestIteratorHelper() {}

	[[nodiscard]] virtual value_type init(Index node) = 0;

	[[nodiscard]] virtual value_type next() = 0;

	[[nodiscard]] virtual TreeSetOrMapQueryNearestIteratorHelper* copy() const = 0;

	[[nodiscard]] virtual TreeSetOrMapQueryNearestIteratorHelper<TreeSetOrMap const>*
	copyToConst() const = 0;

 protected:
	[[nodiscard]] auto& values(Index node) const { return t_->values(node); }

	[[nodiscard]] auto boundsMin(Index node) const { return t_->boundsMin(node); }

	[[nodiscard]] auto boundsMax(Index node) const { return t_->boundsMax(node); }

	[[nodiscard]] auto children(Index node) const { return t_->children(node); }

	template <class Predicate>
	void initPredicate(Predicate& predicate) const
	{
		pred::init(predicate, *t_);
	}

	template <class Predicate>
	[[nodiscard]] bool validReturn(Predicate const&                         predicate,
	                               typename TreeSetOrMap::value_type const& value) const
	{
		return pred::valueCheck(predicate, value);
	}

	template <class Predicate>
	[[nodiscard]] bool validReturn(Predicate const& predicate, Index node) const
	{
		return t_->isPureLeaf(node) && !t_->empty(node) &&
		       pred::innerCheck(predicate, *t_, node);
	}

	template <class Predicate>
	[[nodiscard]] bool validInner(Predicate const& predicate, Index node) const
	{
		return t_->isParent(node) && pred::innerCheck(predicate, *t_, node);
	}

 protected:
	TreeSetOrMap* t_;
};
}  // namespace ufo::detail

#endif  // UFO_CONTAINER_TREE_SET_OR_MAP_QUERY_ITERATOR_HELPER_HPP