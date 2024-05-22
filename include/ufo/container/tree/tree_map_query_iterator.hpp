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

#ifndef UFO_CONTAINER_TREE_MAP_QUERY_ITERATOR_HPP
#define UFO_CONTAINER_TREE_MAP_QUERY_ITERATOR_HPP

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
class TreeMapQueryIteratorHelper
{
	template <class TreeMap2>
	friend class TreeMapQueryIteratorHelper;

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

	using Iterator =
	    std::conditional_t<IsConst, typename std::vector<value_type>::const_iterator,
	                       typename std::vector<value_type>::iterator>;

 public:
	TreeMapQueryIteratorHelper(TreeMap* tm = nullptr) : tm_(tm) {}

	virtual ~TreeMapQueryIteratorHelper() {}

	[[nodiscard]] virtual Iterator init(Index node) = 0;

	[[nodiscard]] virtual Iterator next() = 0;

	[[nodiscard]] virtual TreeMapQueryIteratorHelper* copy() const = 0;

 protected:
	[[nodiscard]] auto iters(Index node) const { return tm_->iters(node); }

	[[nodiscard]] auto children(Index node) const { return tm_->children(node); }

	template <class Predicate>
	void initPredicate(Predicate& predicate) const
	{
		pred::Init<Predicate>::apply(predicate, *tm_);
	}

	template <class Predicate>
	[[nodiscard]] bool validReturn(Index node, Predicate const& predicate) const
	{
		return tm_->isPureLeaf(node) && !tm_->empty(node) &&
		       pred::ValueCheck<Predicate>::apply(predicate, *tm_, node);
	}

	template <class Predicate>
	[[nodiscard]] bool validInner(Index node, Predicate const& predicate) const
	{
		return tm_->isParent(node) &&
		       pred::InnerCheck<Predicate>::apply(predicate, *tm_, node);
	}

 protected:
	TreeMap* tm_;
};

template <class TreeMap>
class TreeMapQueryIteratorWrapper
{
	using TMQIH = TreeMapQueryIteratorHelper<TreeMap>;

	using Index = typename TreeMap::Index;

	static constexpr bool const IsConst = TMQIH::IsConst;

 public:
	//
	// Tags
	//

	using iterator_category = typename TMQIH::iterator_category;
	using difference_type   = typename TMQIH::difference_type;
	using value_type        = typename TMQIH::value_type;
	using reference         = typename TMQIH::reference;
	using pointer           = typename TMQIH::pointer;

	TreeMapQueryIteratorWrapper() = default;

	TreeMapQueryIteratorWrapper(TMQIH* it, Index node) : it_(it), cur_(it_->init(node)) {}

	TreeMapQueryIteratorWrapper(TreeMapQueryIteratorWrapper const& other)
	    : it_(other.it_->copy()), cur_(other.cur_)
	{
	}

	template <class TreeMap2,
	          typename std::enable_if_t<
	              IsConst && !TreeMapQueryIteratorWrapper<TreeMap2>::IsConst &&
	              std::is_same_v<std::decay_t<TreeMap>, std::decay_t<TreeMap2>>> = true>
	TreeMapQueryIteratorWrapper(TreeMapQueryIteratorWrapper<TreeMap2> const& other)
	    : it_(other.it_->copy()), cur_(other.cur_)
	{
	}

	TreeMapQueryIteratorWrapper& operator=(TreeMapQueryIteratorWrapper const& rhs)
	{
		it_  = rhs.it_->copy();
		cur_ = rhs.cur_;
		return *this;
	}

	template <class TreeMap2,
	          typename std::enable_if_t<
	              IsConst && !TreeMapQueryIteratorWrapper<TreeMap2>::IsConst &&
	              std::is_same_v<std::decay_t<TreeMap>, std::decay_t<TreeMap2>>> = true>
	TreeMapQueryIteratorWrapper& operator=(TreeMapQueryIteratorWrapper<TreeMap2> const& rhs)
	{
		it_  = rhs.it_->copy();
		cur_ = rhs.cur_;
		return *this;
	}

	[[nodiscard]] reference operator*() const { return *cur_; }

	[[nodiscard]] pointer operator->() const { return &*cur_; }

	TreeMapQueryIteratorWrapper& operator++()
	{
		cur_ = it_->next();
		return *this;
	}

	TreeMapQueryIteratorWrapper operator++(int)
	{
		auto tmp = *this;
		++*this;
		return tmp;
	}

	bool operator==(TreeMapQueryIteratorWrapper const& rhs) const
	{
		return cur_ == rhs.cur_;
	}

	template <class TreeMap2, typename std::enable_if_t<std::is_same_v<
	                              std::decay_t<TreeMap>, std::decay_t<TreeMap2>>> = true>
	bool operator==(TreeMapQueryIteratorWrapper<TreeMap2> const& rhs) const
	{
		return cur_ == rhs.cur_;
	}

	bool operator!=(TreeMapQueryIteratorWrapper const& rhs) const
	{
		return cur_ != rhs.cur_;
	}

	template <class TreeMap2, typename std::enable_if_t<std::is_same_v<
	                              std::decay_t<TreeMap>, std::decay_t<TreeMap2>>> = true>
	bool operator!=(TreeMapQueryIteratorWrapper<TreeMap2> const& rhs) const
	{
		return cur_ != rhs.cur_;
	}

 private:
	std::unique_ptr<TMQIH>   it_;
	typename TMQIH::Iterator cur_;
};

template <class TreeMap, class Predicate>
class TreeMapQueryIterator final : public TreeMapQueryIteratorHelper<TreeMap>
{
	using Base = TreeMapQueryIteratorHelper<TreeMap>;

	using Index = typename TreeMap::Index;

	using Iterator = typename Base::Iterator;

 public:
	//
	// Tags
	//

	using iterator_category = typename Base::iterator_category;
	using difference_type   = typename Base::difference_type;
	using value_type        = typename Base::value_type;
	using reference         = typename Base::reference;
	using pointer           = typename Base::pointer;

	TreeMapQueryIterator()                                  = default;
	TreeMapQueryIterator(TreeMapQueryIterator const& other) = default;
	TreeMapQueryIterator(TreeMapQueryIterator&& other)      = default;

	TreeMapQueryIterator(TreeMap* tm, Predicate const& predicate)
	    : Base(tm), predicate_(predicate)
	{
	}

	~TreeMapQueryIterator() override {}

	[[nodiscard]] Iterator init(Index node) override
	{
		Base::initPredicate(predicate_);

		if (Base::validReturn(node, predicate_)) {
			std::tie(first_, last_) = Base::iters(node);
			return first_;
		} else if (Base::validInner(node, predicate_)) {
			inner_nodes_[inner_index_++] = node;
			return next();
		}
	}

	[[nodiscard]] Iterator next() override
	{
		if (first_ != last_ && ++first_ != last_) {
			return first_;
		}

		// Skip forward to next valid return node
		while (0 == return_index_ && inner_index_) {
			auto block = Base::children(inner_nodes_[--inner_index_]);

			// Go down the tree
			for (auto i = TreeMap::branchingFactor(); 0 != i;) {
				Index node(block, i);

				if (Base::validReturn(node, predicate_)) {
					return_nodes_[return_index_++] = node;
				} else if (Base::validInner(node, predicate_)) {
					inner_nodes_[inner_index_++] = node;
				}
			}
		}

		if (0 < return_index_) {
			auto node               = return_nodes_[--return_index_];
			std::tie(first_, last_) = Base::iters(node);
		} else {
			first_ = {};
			last_  = {};
		}

		return first_;
	}

	[[nodiscard]] TreeMapQueryIterator* copy() const override
	{
		return new TreeMapQueryIterator(*this);
	}

 private:
	Iterator first_;
	Iterator last_;

	// Predicate that nodes has to fulfill
	Predicate predicate_{};

	// To be processed inner nodes
	std::array<Index, TreeMap::branchingFactor() * (TreeMap::maxNumDepthLevels() - 1)>
	    inner_nodes_;
	// To be processed return nodes
	std::array<Index, TreeMap::branchingFactor()> return_nodes_;

	int inner_index_  = 0;
	int return_index_ = 0;
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_MAP_QUERY_ITERATOR_HPP