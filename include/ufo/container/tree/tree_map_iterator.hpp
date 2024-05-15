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
#include <type_traits>
#include <utility>

namespace ufo
{
template <class Iterator, class T, bool IsConst>
class TreeMapIterator
{
	template <class Iterator2, class T2, bool IsConst2>
	friend class TreeMapIterator;

 public:
	//
	// Tags
	//

	using iterator_category = typename Iterator::iterator_category;
	using difference_type   = typename Iterator::difference_type;
	using value_type        = std::conditional_t<IsConst, T const, T>;
	using reference         = value_type&;
	using pointer           = value_type*;

	TreeMapIterator()                             = default;
	TreeMapIterator(TreeMapIterator const& other) = default;
	TreeMapIterator(TreeMapIterator&& other)      = default;

	template <class Iterator2, bool WasConst,
	          typename std::enable_if_t<IsConst && !WasConst, bool> = true>
	TreeMapIterator(TreeMapIterator<Iterator2, T, WasConst> const& rhs) : it_(rhs.it_)
	{
	}

	TreeMapIterator(Iterator it) : it_(it) {}

	TreeMapIterator& operator=(TreeMapIterator const& rhs) = default;
	TreeMapIterator& operator=(TreeMapIterator&& rhs)      = default;

	template <class Iterator2, bool WasConst,
	          typename std::enable_if_t<IsConst && !WasConst, bool> = true>
	TreeMapIterator& operator=(TreeMapIterator<Iterator2, T, WasConst> const& rhs)
	{
		it_ = rhs.it_;
		return *this;
	}

	[[nodiscard]] reference operator*() const { return *it_; }

	[[nodiscard]] pointer operator->() const { return &*it_; }

	TreeMapIterator& operator++()
	{
		++it_;
		return *this;
	}

	TreeMapIterator operator++(int)
	{
		auto tmp = *this;
		++*this;
		return tmp;
	}

	TreeMapIterator& operator--()
	{
		--it_;
		return *this;
	}

	TreeMapIterator operator--(int)
	{
		auto tmp = *this;
		--*this;
		return tmp;
	}

	bool operator==(TreeMapIterator const& rhs) const { return it_ == rhs.it_; }

	template <class Iterator2, bool IsConst2>
	bool operator==(TreeMapIterator<Iterator2, T, IsConst2> const& rhs) const
	{
		return it_ == rhs.it_;
	}

	bool operator!=(TreeMapIterator const& rhs) const { return it_ != rhs.it_; }

	template <class Iterator2, bool IsConst2>
	bool operator!=(TreeMapIterator<Iterator2, T, IsConst2> const& rhs) const
	{
		return it_ != rhs.it_;
	}

 private:
	Iterator it_;
};

template <class Tree, class Predicate, class Iterator, class T, bool IsConst>
class TreeMapQueryIterator
{
	template <class Tree2, class Predicate2, class Iterator2, class T2, bool IsConst2>
	friend class TreeMapQueryIterator;

 public:
	//
	// Tags
	//

	using iterator_category = std::forward_iterator_tag;
	using difference_type   = std::ptrdiff_t;
	using value_type        = std::conditional_t<IsConst, T const, T>;
	using reference         = value_type&;
	using pointer           = value_type*;

	using Index     = typename std::decay_t<Tree>::Index;
	using Bounds    = typename std::decay_t<Tree>::Bounds;
	using node_type = std::pair<Index, Bounds>;

	TreeMapQueryIterator()                                  = default;
	TreeMapQueryIterator(TreeMapQueryIterator const& other) = default;
	TreeMapQueryIterator(TreeMapQueryIterator&& other)      = default;

	template <class Tree2, class Iterator2, bool WasConst,
	          typename std::enable_if_t<IsConst && !WasConst, bool> = true>
	TreeMapQueryIterator(
	    TreeMapQueryIterator<Tree2, Predicate, Iterator2, T, WasConst> const& rhs)
	    : it_(rhs.it_), tree_(rhs.tree_), predicate_(rhs.predicate_)
	{
	}

	TreeMapQueryIterator(Iterator it) : it_(it) {}

	TreeMapQueryIterator(Tree* tree, Index root, Predicate const& predicate)
	    : tree_(tree), predicate_(predicate)
	{
		assert(tree->valid(root));
		pred::Init<Predicate>::apply(predicate_, *tree_);

		if (tree_->isLeaf(root)) {
			if (pred::ValueCheck<Predicate>::apply(predicate, *tree_, node)) {
				
			}
		}
		//  pred::ValueCheck<Predicate>::apply(predicate, *tree_, node);
		// pred::InnerCheck<Predicate>::apply(predicate, *tree_, node);
		// TODO: Init
	}

	TreeMapQueryIterator& operator=(TreeMapQueryIterator const& rhs) = default;
	TreeMapQueryIterator& operator=(TreeMapQueryIterator&& rhs)      = default;

	template <class Tree2, class Iterator2, bool WasConst,
	          typename std::enable_if_t<IsConst && !WasConst, bool> = true>
	TreeMapQueryIterator& operator=(
	    TreeMapQueryIterator<Tree2, Predicate, Iterator2, T, WasConst> const& rhs)
	{
		it_        = rhs.it_;
		tree_      = rhs.tree_;
		predicate_ = rhs.predicate_;
		return *this;
	}

	[[nodiscard]] reference operator*() const { return *it_; }

	[[nodiscard]] pointer operator->() const { return &*it_; }

	TreeMapQueryIterator& operator++()
	{
		// TODO: Implement
		++it_;
		return *this;
	}

	TreeMapQueryIterator operator++(int)
	{
		auto tmp = *this;
		++*this;
		return tmp;
	}

	bool operator==(TreeMapQueryIterator const& rhs) const { return it_ == rhs.it_; }

	template <class Tree2, class Predicate2, class Iterator2, bool IsConst2>
	bool operator==(
	    TreeMapQueryIterator<Tree2, Predicate2, Iterator2, T, IsConst2> const& rhs) const
	{
		return it_ == rhs.it_;
	}

	bool operator!=(TreeMapQueryIterator const& rhs) const { return it_ != rhs.it_; }

	template <class Tree2, class Predicate2, class Iterator2, bool IsConst2>
	bool operator!=(
	    TreeMapQueryIterator<Tree2, Predicate2, Iterator2, T, IsConst2> const& rhs) const
	{
		return it_ != rhs.it_;
	}

 private:
	Iterator  it_;
	Tree*     tree_;
	Predicate predicate_;

	// To be processed inner nodes
	std::array<node_type, Tree::branchingFactor() * Tree::maxNumDepthLevels()> inner_nodes_;
	// To be processed return nodes
	std::array<node_type, Tree::branchingFactor()> return_nodes_;
};

template <class Tree, class Node, class Geometry, class Predicate, class Iterator,
          class T, bool IsConst>
class TreeMapNearestIterator
{
	template <class Tree2, class Node2, class Geometry2, class Predicate2, class Iterator2,
	          class T2, bool IsConst2>
	friend class TreeMapNearestIterator;

 public:
	//
	// Tags
	//

	using iterator_category = std::forward_iterator_tag;
	using difference_type   = std::ptrdiff_t;
	using value_type        = std::conditional_t<IsConst, T const, T>;
	using reference         = value_type&;
	using pointer           = value_type*;

	// TODO: Implement

 private:
	Iterator it_;
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_MAP_ITERATOR_HPP