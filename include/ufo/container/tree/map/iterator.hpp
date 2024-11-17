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

#ifndef UFO_CONTAINER_DETAIL_TREE_MAP_ITERATOR_HPP
#define UFO_CONTAINER_DETAIL_TREE_MAP_ITERATOR_HPP

// UFO
#include <ufo/container/tree/index.hpp>

// STL
#include <cassert>
#include <cstddef>
#include <iterator>
#include <type_traits>

namespace ufo
{
// Forward declare
template <std::size_t Dim, class T>
class TreeMap;

template <bool Const, std::size_t Dim, class T>
class TreeMapIterator
{
 private:
	//
	// Friends
	//

	friend class TreeMapIterator<!Const, Dim, T>;

	friend class TreeMap<Dim, T>;

 private:
	using Container = TreeMap<Dim, T>;

	static constexpr std::size_t const BF = Container::branchingFactor();

	using RawIterator = std::conditional_t<Const, typename Container::const_raw_iterator,
	                                       typename Container::raw_iterator>;

 public:
	//
	// Tags
	//

	using iterator_category = std::forward_iterator_tag;
	using difference_type   = std::ptrdiff_t;
	using value_type        = typename std::iterator_traits<RawIterator>::value_type;
	using reference         = typename std::iterator_traits<RawIterator>::reference;
	using pointer           = typename std::iterator_traits<RawIterator>::pointer;

	TreeMapIterator() = default;

	TreeMapIterator(TreeMapIterator const&) = default;

	template <bool Const2, class = std::enable_if_t<Const && !Const2>>
	TreeMapIterator(TreeMapIterator<Const2, Dim, T> const& other)
	    : tm_(other.tm_)
	    , root_(other.root_)
	    , cur_(other.cur_)
	    , it_(other.it_)
	    , last_(other.last_)
	{
	}

	TreeMapIterator& operator++()
	{
		if (++it_ == last_) {
			nextNode();
		}
		return *this;
	}

	TreeMapIterator operator++(int)
	{
		TreeMapIterator tmp(*this);
		++*this;
		return tmp;
	}

	reference operator*() const { return *it_; }

	pointer operator->() const { return &*it_; }

	template <bool Const1, bool Const2>
	friend bool operator==(TreeMapIterator<Const1, Dim, T> const& lhs,
	                       TreeMapIterator<Const2, Dim, T> const& rhs)
	{
		return lhs.it_ == rhs.it_;
	}

	template <bool Const1, bool Const2>
	friend bool operator!=(TreeMapIterator<Const1, Dim, T> const& lhs,
	                       TreeMapIterator<Const2, Dim, T> const& rhs)
	{
		return lhs.it_ != rhs.it_;
	}

 private:
	void nextNode()
	{
		if (root_ == cur_) {
			// We have visited all nodes
			it_   = {};
			last_ = {};
			return;
		}

		// Check if any sibling of cur is also a return node
		while (BF - 1 > cur_.offset) {
			++cur_.offset;
			if (!tm_->empty(cur_)) {
				// Found new return node
				it_   = tm_->values(cur_).begin();
				last_ = tm_->values(cur_).end();
				return;
			}
		}

		// No sibling was a return node, so check a different branch

		// Find next unexplored parent
		for (bool found_parent = false; !found_parent;) {
			while (BF - 1 == cur_.offset) {
				cur_ = tm_->parent(cur_);

				if (root_ == cur_) {
					// We have visited all nodes
					it_   = {};
					last_ = {};
					return;
				}
			}

			while (BF - 1 > cur_.offset) {
				++cur_.offset;
				if (tm_->isParent(cur_)) {
					found_parent = true;
					break;
				}
			}
		}

		// Go down the tree
		for (auto depth = tm_->depth(cur_); 0 < depth; --depth) {
			for (; BF > cur_.offset; ++cur_.offset) {
				if (tm_->isParent(cur_)) {
					cur_ = tm_->child(cur_, 0);
					break;
				}
			}

			assert(BF != cur_.offset);
		}

		// We are now at depth 0 (aka return depth)
		for (; BF > cur_.offset; ++cur_.offset) {
			if (!tm_->empty(cur_)) {
				// Found new return node
				it_   = tm_->values(cur_).begin();
				last_ = tm_->values(cur_).end();
				return;
			}
		}

		// Should never be able to get here
		assert(BF != cur_.offset);
	}

 private:
	TreeMapIterator(Container* tm, TreeIndex node) : tm_(tm), root_(node), cur_(node)
	{
		if (!tm_->isParent(root_)) {
			return;
		}

		cur_ = tm_->child(root_, 0);

		// Go down the tree
		for (auto depth = tm_->depth(cur_); 0 < depth; --depth) {
			for (; BF > cur_.offset; ++cur_.offset) {
				if (tm_->isParent(cur_)) {
					cur_ = tm_->child(cur_, 0);
					break;
				}
			}

			assert(BF != cur_.offset);
		}

		// We are now at depth 0 (aka return depth)
		for (; BF > cur_.offset; ++cur_.offset) {
			if (!tm_->empty(cur_)) {
				// Found new return node
				it_   = tm_->values(cur_).begin();
				last_ = tm_->values(cur_).end();
				return;
			}
		}

		// Should never be able to get here
		assert(BF != cur_.offset);
	}

	TreeMapIterator(Container& tm, TreeIndex node) : TreeMapIterator(&tm, node) {}

 private:
	Container* tm_ = nullptr;

	TreeIndex root_{};
	TreeIndex cur_{};

	RawIterator it_{};
	RawIterator last_{};
};
}  // namespace ufo

#endif  // UFO_CONTAINER_DETAIL_TREE_MAP_ITERATOR_HPP