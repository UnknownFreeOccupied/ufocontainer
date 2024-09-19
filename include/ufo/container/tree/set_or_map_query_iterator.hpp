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

#ifndef UFO_CONTAINER_TREE_SET_OR_MAP_QUERY_ITERATOR_HPP
#define UFO_CONTAINER_TREE_SET_OR_MAP_QUERY_ITERATOR_HPP

// UFO
#include <ufo/container/tree/predicate.hpp>

// STL
#include <iterator>
#include <memory>
#include <type_traits>
#include <vector>

namespace ufo
{
namespace detail
{
template <class TreeSetOrMap>
class TreeSetOrMapQueryIteratorHelper
{
	template <class TreeSetOrMap2>
	friend class TreeSetOrMapQueryIteratorHelper;

 public:
	static constexpr bool const IsConst = std::is_const_v<TreeSetOrMap>;

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
	using value_type        = typename std::iterator_traits<RawIterator>::value_type;
	using reference         = typename std::iterator_traits<RawIterator>::reference;
	using pointer           = typename std::iterator_traits<RawIterator>::pointer;

 public:
	TreeSetOrMapQueryIteratorHelper()                                       = default;
	TreeSetOrMapQueryIteratorHelper(TreeSetOrMapQueryIteratorHelper const&) = default;
	TreeSetOrMapQueryIteratorHelper(TreeSetOrMapQueryIteratorHelper&&)      = default;

	template <
	    class TreeSetOrMap2,
	    typename std::enable_if_t<
	        IsConst && !TreeSetOrMapQueryIteratorHelper<TreeSetOrMap2>::IsConst &&
	            std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	        bool> = true>
	TreeSetOrMapQueryIteratorHelper(
	    TreeSetOrMapQueryIteratorHelper<TreeSetOrMap2> const& other)
	    : t_(other.t_)
	{
	}

	TreeSetOrMapQueryIteratorHelper(TreeSetOrMap* t = nullptr) : t_(t) {}

	virtual ~TreeSetOrMapQueryIteratorHelper() {}

	[[nodiscard]] virtual RawIterator init(Index node) = 0;

	[[nodiscard]] virtual RawIterator next() = 0;

	[[nodiscard]] virtual TreeSetOrMapQueryIteratorHelper* copy() const = 0;

	[[nodiscard]] virtual TreeSetOrMapQueryIteratorHelper<TreeSetOrMap const>* copyToConst()
	    const = 0;

 protected:
	[[nodiscard]] auto& values(Index node) const { return t_->values(node); }

	[[nodiscard]] auto children(Index node) const { return t_->children(node); }

	template <class Predicate>
	void initPredicate(Predicate& predicate) const
	{
		pred::init(predicate, *t_);
	}

	template <class Predicate>
	[[nodiscard]] bool validReturn(Predicate const&  predicate,
	                               value_type const& value) const
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

template <class TreeSetOrMap, class Predicate>
class TreeSetOrMapQueryIterator final
    : public TreeSetOrMapQueryIteratorHelper<TreeSetOrMap>
{
	template <class TreeSetOrMap2, class Predicate2>
	friend class TreeSetOrMapQueryIterator;

	using Base = TreeSetOrMapQueryIteratorHelper<TreeSetOrMap>;

	using Index       = typename TreeSetOrMap::Index;
	using RawIterator = typename Base::RawIterator;

	static constexpr bool const IsConst = Base::IsConst;

 public:
	//
	// Tags
	//

	using iterator_category = typename Base::iterator_category;
	using difference_type   = typename Base::difference_type;
	using value_type        = typename Base::value_type;
	using reference         = typename Base::reference;
	using pointer           = typename Base::pointer;

	TreeSetOrMapQueryIterator()                                 = default;
	TreeSetOrMapQueryIterator(TreeSetOrMapQueryIterator const&) = default;
	TreeSetOrMapQueryIterator(TreeSetOrMapQueryIterator&&)      = default;

	template <
	    class TreeSetOrMap2,
	    typename std::enable_if_t<
	        IsConst && !TreeSetOrMapQueryIterator<TreeSetOrMap2, Predicate>::IsConst &&
	            std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	        bool> = true>
	TreeSetOrMapQueryIterator(
	    TreeSetOrMapQueryIterator<TreeSetOrMap2, Predicate> const& other)
	    : Base(other)
	    , first_(other.first_)
	    , last_(other.last_)
	    , predicate_(other.predicate_)
	    , inner_nodes_(other.inner_nodes_)
	    , return_nodes_(other.return_nodes_)
	    , inner_index_(other.inner_index_)
	    , return_index_(other.return_index_)
	{
	}

	TreeSetOrMapQueryIterator(TreeSetOrMap* t, Predicate const& predicate)
	    : Base(t), predicate_(predicate)
	{
	}

	~TreeSetOrMapQueryIterator() override {}

	[[nodiscard]] RawIterator init(Index node) override
	{
		Base::initPredicate(predicate_);

		if (Base::validReturn(predicate_, node)) {
			auto& v = Base::values(node);
			first_  = std::begin(v);
			last_   = std::end(v);
			return Base::validReturn(predicate_, *first_) ? first_ : next();
		} else if (Base::validInner(predicate_, node)) {
			inner_nodes_[inner_index_++] = node;
			return next();
		}
		return {};
	}

	[[nodiscard]] RawIterator next() override
	{
		if (first_ != last_) {
			++first_;
			for (; first_ != last_ && !Base::validReturn(predicate_, *first_); ++first_) {
			}
			if (first_ != last_) {
				return first_;
			}
		}

		// Skip forward to next valid return node
		while (0 == return_index_ && 0 != inner_index_) {
			auto block = Base::children(inner_nodes_[--inner_index_]);

			// Go down the tree
			for (int i = TreeSetOrMap::branchingFactor() - 1; 0 <= i; --i) {
				Index node(block, i);

				if (Base::validReturn(predicate_, node)) {
					return_nodes_[return_index_++] = node;
				} else if (Base::validInner(predicate_, node)) {
					inner_nodes_[inner_index_++] = node;
				}
			}
		}

		if (0 < return_index_) {
			auto  node = return_nodes_[--return_index_];
			auto& v    = Base::values(node);
			first_     = std::begin(v);
			last_      = std::end(v);
			return Base::validReturn(predicate_, *first_) ? first_ : next();
		} else {
			first_ = {};
			last_  = {};
			return first_;
		}
	}

	[[nodiscard]] TreeSetOrMapQueryIterator* copy() const override
	{
		return new TreeSetOrMapQueryIterator(*this);
	}

	[[nodiscard]] TreeSetOrMapQueryIterator<TreeSetOrMap const, Predicate>* copyToConst()
	    const override
	{
		return new TreeSetOrMapQueryIterator<TreeSetOrMap const, Predicate>(*this);
	}

 private:
	RawIterator first_;
	RawIterator last_;

	// Predicate that nodes has to fulfill
	Predicate predicate_{};

	// To be processed inner nodes
	std::array<Index,
	           TreeSetOrMap::branchingFactor() * (TreeSetOrMap::maxNumDepthLevels() - 1)>
	    inner_nodes_;
	// To be processed return nodes
	std::array<Index, TreeSetOrMap::branchingFactor()> return_nodes_;

	int inner_index_  = 0;
	int return_index_ = 0;
};
}  // namespace detail

template <class TreeSetOrMap>
class TreeSetOrMapQueryIterator
{
	using Iterator    = detail::TreeSetOrMapQueryIteratorHelper<TreeSetOrMap>;
	using RawIterator = typename Iterator::RawIterator;

	using Index = typename TreeSetOrMap::Index;

	static constexpr bool const IsConst = Iterator::IsConst;

 public:
	//
	// Tags
	//

	using iterator_category = typename Iterator::iterator_category;
	using difference_type   = typename Iterator::difference_type;
	using value_type        = typename Iterator::value_type;
	using reference         = typename Iterator::reference;
	using pointer           = typename Iterator::pointer;

	TreeSetOrMapQueryIterator() = default;

	TreeSetOrMapQueryIterator(Iterator* it, Index node) : it_(it), cur_(it_->init(node)) {}

	template <class Predicate>
	TreeSetOrMapQueryIterator(TreeSetOrMap* tree, Index node, Predicate const& pred)
	    : it_(std::make_unique<detail::TreeSetOrMapQueryIterator<TreeSetOrMap, Predicate>>(
	          tree, pred))
	    , cur_(it_->init(node))
	{
	}

	TreeSetOrMapQueryIterator(TreeSetOrMapQueryIterator const& other) : cur_(other.cur_)
	{
		if (other.it_) {
			it_.reset(other.it_->copy());
		}
	}

	TreeSetOrMapQueryIterator(TreeSetOrMapQueryIterator&&) = default;

	template <
	    class TreeSetOrMap2,
	    typename std::enable_if_t<
	        IsConst && !TreeSetOrMapQueryIterator<TreeSetOrMap2>::IsConst &&
	            std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	        bool> = true>
	TreeSetOrMapQueryIterator(TreeSetOrMapQueryIterator<TreeSetOrMap2> const& other)
	    : cur_(other.cur_)
	{
		if (other.it_) {
			it_.reset(other.it_->copyToConst());
		}
	}

	TreeSetOrMapQueryIterator& operator=(TreeSetOrMapQueryIterator const& rhs)
	{
		if (rhs.it_) {
			it_.reset(rhs.it_->copy());
		} else {
			it_.reset();
		}
		cur_ = rhs.cur_;
		return *this;
	}

	TreeSetOrMapQueryIterator& operator=(TreeSetOrMapQueryIterator&&) = default;

	template <
	    class TreeSetOrMap2,
	    typename std::enable_if_t<
	        IsConst && !TreeSetOrMapQueryIterator<TreeSetOrMap2>::IsConst &&
	            std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	        bool> = true>
	TreeSetOrMapQueryIterator& operator=(
	    TreeSetOrMapQueryIterator<TreeSetOrMap2> const& rhs)
	{
		if (rhs.it_) {
			it_.reset(rhs.it_->copyToConst());
		} else {
			it_.reset();
		}
		cur_ = rhs.cur_;
		return *this;
	}

	[[nodiscard]] reference operator*() const { return *cur_; }

	[[nodiscard]] pointer operator->() const { return &*cur_; }

	TreeSetOrMapQueryIterator& operator++()
	{
		cur_ = it_->next();
		return *this;
	}

	TreeSetOrMapQueryIterator operator++(int)
	{
		auto tmp = *this;
		++*this;
		return tmp;
	}

	bool operator==(TreeSetOrMapQueryIterator const& rhs) const { return cur_ == rhs.cur_; }

	template <class TreeSetOrMap2,
	          typename std::enable_if_t<
	              std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	              bool> = true>
	bool operator==(TreeSetOrMapQueryIterator<TreeSetOrMap2> const& rhs) const
	{
		return cur_ == rhs.cur_;
	}

	bool operator!=(TreeSetOrMapQueryIterator const& rhs) const { return !(*this == rhs); }

	template <class TreeSetOrMap2,
	          typename std::enable_if_t<
	              std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	              bool> = true>
	bool operator!=(TreeSetOrMapQueryIterator<TreeSetOrMap2> const& rhs) const
	{
		return !(*this == rhs);
	}

 private:
	[[nodiscard]] RawIterator iterator() const { return cur_; }

 private:
	std::unique_ptr<Iterator> it_;
	RawIterator               cur_;
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_SET_OR_MAP_QUERY_ITERATOR_HPP