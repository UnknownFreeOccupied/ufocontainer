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

#ifndef UFO_CONTAINER_TREE_MAP_ITERATOR_HPP
#define UFO_CONTAINER_TREE_MAP_ITERATOR_HPP

// STL
#include <array>
#include <iterator>
#include <memory>
#include <queue>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace ufo
{
template <class TreeMap>
class TreeMapIterator
{
	template <class TreeMap2>
	friend class TreeMapIterator;

	static constexpr bool const IsConst = std::is_const_v<TreeMap>;

	using Index = typename TreeMap::Index;

 public:
	//
	// Tags
	//

	using iterator_category = std::forward_iterator_tag;
	using difference_type   = std::ptrdiff_t;
	using value_type        = typename TreeMap::value_type;
	using reference         = std::conditional_t<IsConst, value_type const&, value_type&>;
	using pointer           = std::conditional_t<IsConst, value_type const*, value_type*>;

	TreeMapIterator()                             = default;
	TreeMapIterator(TreeMapIterator const& other) = default;
	TreeMapIterator(TreeMapIterator&& other)      = default;

	template <class TreeMap2,
	          typename std::enable_if_t<
	              IsConst && !TreeMapIterator<TreeMap2>::IsConst &&
	              std::is_same_v<std::decay_t<TreeMap>, std::decay_t<TreeMap2>>> = true>
	TreeMapIterator(TreeMapIterator<TreeMap2> const& other)
	    : tm_(other.tm_)
	    , first_(other.first_)
	    , last_(other.last_)
	    , inner_nodes_(other.inner_nodes_)
	    , return_nodes_(other.return_nodes_)
	    , inner_index_(other.inner_index_)
	    , return_index_(other.return_index_)
	{
	}

	TreeMapIterator(TreeMap* tm, Index node) : tm_(tm)
	{
		if (tm_->isParent(node)) {
			inner_nodes_[inner_index_++] = node;
			next();
		} else if (tm_->isPureLeaf(node) && !tm_->empty(node)) {
			std::tie(first_, last_) = tm_->iters(node);
		}
	}

	TreeMapIterator& operator=(TreeMapIterator const& rhs) = default;
	TreeMapIterator& operator=(TreeMapIterator&& rhs)      = default;

	template <class TreeMap2,
	          typename std::enable_if_t<
	              IsConst && !TreeMapIterator<TreeMap2>::IsConst &&
	              std::is_same_v<std::decay_t<TreeMap>, std::decay_t<TreeMap2>>> = true>
	TreeMapIterator& operator=(TreeMapIterator<TreeMap2> const& rhs)
	{
		tm_           = rhs.tm_;
		first_        = rhs.first_;
		last_         = rhs.last_;
		inner_nodes_  = rhs.inner_nodes_;
		return_nodes_ = rhs.return_nodes_;
		inner_index_  = rhs.inner_index_;
		return_index_ = rhs.return_index_;
		return *this;
	}

	[[nodiscard]] reference operator*() const { return *first_; }

	[[nodiscard]] pointer operator->() const { return &*first_; }

	TreeMapIterator& operator++()
	{
		if (first_ == last_ || ++first_ == last_) {
			next();
		}
		return *this;
	}

	TreeMapIterator operator++(int)
	{
		auto tmp = *this;
		++*this;
		return tmp;
	}

	bool operator==(TreeMapIterator const& rhs) const { return first_ == rhs.first_; }

	template <class TreeMap2, typename std::enable_if_t<std::is_same_v<
	                              std::decay_t<TreeMap>, std::decay_t<TreeMap2>>> = true>
	bool operator==(TreeMapIterator<TreeMap2> const& rhs) const
	{
		return first_ == rhs.first_;
	}

	bool operator!=(TreeMapIterator const& rhs) const { return first_ != rhs.first_; }

	template <class TreeMap2, typename std::enable_if_t<std::is_same_v<
	                              std::decay_t<TreeMap>, std::decay_t<TreeMap2>>> = true>
	bool operator!=(TreeMapIterator<TreeMap2> const& rhs) const
	{
		return first_ != rhs.first_;
	}

 private:
	void next()
	{
		// Skip forward to next valid return node
		while (0 == return_index_ && inner_index_) {
			auto block = tm_->children(inner_nodes_[--inner_index_]);

			// Go down the tree
			for (int i = TreeMap::branchingFactor() - 1; 0 <= i; --i) {
				Index node(block, i);

				if (tm_->isParent(node)) {
					inner_nodes_[inner_index_++] = node;
				} else if (tm_->isPureLeaf(node) && !tm_->empty(node)) {
					return_nodes_[return_index_++] = node;
				}
			}
		}

		if (0 < return_index_) {
			auto node               = return_nodes_[--return_index_];
			std::tie(first_, last_) = tm_->iters(node);
		} else {
			first_ = {};
			last_  = {};
		}
	}

 private:
	using Iterator =
	    std::conditional_t<IsConst, typename std::vector<value_type>::const_iterator,
	                       typename std::vector<value_type>::iterator>;

	TreeMap* tm_ = nullptr;

	Iterator first_;
	Iterator last_;

	// To be processed inner nodes
	std::array<Index, TreeMap::branchingFactor() * (TreeMap::maxNumDepthLevels() - 1)>
	    inner_nodes_;
	// To be processed return nodes
	std::array<Index, TreeMap::branchingFactor()> return_nodes_;

	int inner_index_  = 0;
	int return_index_ = 0;
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_MAP_ITERATOR_HPP