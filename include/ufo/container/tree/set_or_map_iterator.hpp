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

#ifndef UFO_CONTAINER_TREE_SET_OR_MAP_ITERATOR_HPP
#define UFO_CONTAINER_TREE_SET_OR_MAP_ITERATOR_HPP

// STL
#include <array>
#include <iterator>
#include <type_traits>

namespace ufo
{
template <class TreeSetOrMap>
class TreeSetOrMapIterator
{
	template <class TreeSetOrMap2>
	friend class TreeSetOrMapIterator;

	friend TreeSetOrMap;

	static constexpr bool const IsConst = std::is_const_v<TreeSetOrMap>;
	static constexpr auto const BF      = TreeSetOrMap::branchingFactor();

	using Index = typename TreeSetOrMap::Index;
	using RawIterator =
	    std::conditional_t<IsConst, typename TreeSetOrMap::const_raw_iterator,
	                       typename TreeSetOrMap::raw_iterator>;

 public:
	//
	// Tags
	//

	using iterator_category = std::forward_iterator_tag;
	using difference_type   = std::ptrdiff_t;
	using value_type        = typename std::iterator_traits<RawIterator>::value_type;
	using reference         = typename std::iterator_traits<RawIterator>::reference;
	using pointer           = typename std::iterator_traits<RawIterator>::pointer;

	TreeSetOrMapIterator()                            = default;
	TreeSetOrMapIterator(TreeSetOrMapIterator const&) = default;
	TreeSetOrMapIterator(TreeSetOrMapIterator&&)      = default;

	template <
	    class TreeSetOrMap2,
	    typename std::enable_if_t<
	        IsConst && !TreeSetOrMapIterator<TreeSetOrMap2>::IsConst &&
	            std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	        bool> = true>
	TreeSetOrMapIterator(TreeSetOrMapIterator<TreeSetOrMap2> const& other)
	    : t_(other.t_)
	    , node_(other.node_)
	    , first_(other.first_)
	    , last_(other.last_)
	    , inner_nodes_(other.inner_nodes_)
	    , return_nodes_(other.return_nodes_)
	    , inner_index_(other.inner_index_)
	    , return_index_(other.return_index_)
	{
	}

	TreeSetOrMapIterator(TreeSetOrMap* t, Index node) : t_(t)
	{
		if (t_->isParent(node)) {
			inner_nodes_[inner_index_++] = node;
			next();
		} else if (t_->isPureLeaf(node) && !t_->empty(node)) {
			node_  = node;
			first_ = t_->values(node_).begin();
			last_  = t_->values(node_).end();
		}
	}

	TreeSetOrMapIterator(TreeSetOrMap& t, Index node) : TreeSetOrMapIterator(&t, node) {}

	TreeSetOrMapIterator& operator=(TreeSetOrMapIterator const&) = default;
	TreeSetOrMapIterator& operator=(TreeSetOrMapIterator&&)      = default;

	template <
	    class TreeSetOrMap2,
	    typename std::enable_if_t<
	        IsConst && !TreeSetOrMapIterator<TreeSetOrMap2>::IsConst &&
	            std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	        bool> = true>
	TreeSetOrMapIterator& operator=(TreeSetOrMapIterator<TreeSetOrMap2> const& rhs)
	{
		t_            = rhs.t_;
		node_         = rhs.node_;
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

	TreeSetOrMapIterator& operator++()
	{
		if (first_ == last_ || ++first_ == last_) {
			next();
		}
		return *this;
	}

	TreeSetOrMapIterator operator++(int)
	{
		auto tmp = *this;
		++*this;
		return tmp;
	}

	bool operator==(TreeSetOrMapIterator const& rhs) const { return first_ == rhs.first_; }

	template <class TreeSetOrMap2,
	          typename std::enable_if_t<
	              std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	              bool> = true>
	bool operator==(TreeSetOrMapIterator<TreeSetOrMap2> const& rhs) const
	{
		return first_ == rhs.first_;
	}

	bool operator!=(TreeSetOrMapIterator const& rhs) const { return first_ != rhs.first_; }

	template <class TreeSetOrMap2,
	          typename std::enable_if_t<
	              std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	              bool> = true>
	bool operator!=(TreeSetOrMapIterator<TreeSetOrMap2> const& rhs) const
	{
		return first_ != rhs.first_;
	}

 private:
	template <
	    class TreeSetOrMap2,
	    typename std::enable_if_t<
	        !IsConst && TreeSetOrMapIterator<TreeSetOrMap2>::IsConst &&
	            std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	        bool> = true>
	TreeSetOrMapIterator(TreeSetOrMap* t, TreeSetOrMapIterator<TreeSetOrMap2> const& other)
	    : t_(t)
	    , node_(other.node_)
	    , inner_nodes_(other.inner_nodes_)
	    , return_nodes_(other.return_nodes_)
	    , inner_index_(other.inner_index_)
	    , return_index_(other.return_index_)
	{
		if (t_->valid(node_)) {
			// To remove const from other's iterators
			first_ = t_->values(node_).erase(other.first_, other.first_);
			last_  = t_->values(node_).erase(other.last_, other.last_);
		}
	}

	template <
	    class TreeSetOrMap2,
	    typename std::enable_if_t<
	        !IsConst && TreeSetOrMapIterator<TreeSetOrMap2>::IsConst &&
	            std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	        bool> = true>
	TreeSetOrMapIterator(TreeSetOrMap& t, TreeSetOrMapIterator<TreeSetOrMap2> const& other)
	    : TreeSetOrMapIterator(&t, other)
	{
	}

	void next()
	{
		// Skip forward to next valid return node
		while (0 == return_index_ && inner_index_) {
			auto block = t_->children(inner_nodes_[--inner_index_]);

			// Go down the tree
			for (int i = BF - 1; 0 <= i; --i) {
				Index node(block, i);

				if (t_->isParent(node)) {
					inner_nodes_[inner_index_++] = node;
				} else if (t_->isPureLeaf(node) && !t_->empty(node)) {
					return_nodes_[return_index_++] = node;
				}
			}
		}

		if (0 < return_index_) {
			node_  = return_nodes_[--return_index_];
			first_ = t_->values(node_).begin();
			last_  = t_->values(node_).end();
		} else {
			node_  = Index();
			first_ = {};
			last_  = {};
		}
	}

	[[nodiscard]] RawIterator iterator() const { return first_; }

 private:
	TreeSetOrMap* t_ = nullptr;

	Index       node_;
	RawIterator first_;
	RawIterator last_;

	// To be processed inner nodes
	std::array<Index, BF*(TreeSetOrMap::maxNumDepthLevels() - 1)> inner_nodes_;
	// To be processed return nodes
	std::array<Index, BF> return_nodes_;

	int inner_index_  = 0;
	int return_index_ = 0;
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_SET_OR_MAP_ITERATOR_HPP