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

#ifndef UFO_CONTAINER_TREE_MAP_NEAREST_ITERATOR_HPP
#define UFO_CONTAINER_TREE_MAP_NEAREST_ITERATOR_HPP

// UFO
#include <ufo/container/tree/tree_map_nearest.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <iterator>
#include <limits>
#include <memory>
#include <queue>
#include <type_traits>
#include <utility>
#include <vector>

namespace ufo
{
template <class TreeMap>
class TreeMapNearestIteratorHelper
{
	template <class TreeMap2>
	friend class TreeMapNearestIteratorHelper;

	static constexpr bool const IsConst = std::is_const_v<TreeMap>;

	using Index = typename TreeMap::Index;
	using Point = typename TreeMap::Point;

	using value_pointer = std::conditional_t<IsConst, typename TreeMap::const_pointer,
	                                         typename TreeMap::pointer>;

 public:
	//
	// Tags
	//

	using iterator_category = std::forward_iterator_tag;
	using difference_type   = std::ptrdiff_t;

	struct value_type {
		using value_type      = ...;
		using reference       = value_type&;
		using const_reference = value_type const&;
		using pointer         = value_type*;
		using const_pointer   = value_type const*;

		value_pointer value_ptr;
		float         distance_squared;

		value_type() = default;

		value_type(value_pointer value_ptr, float distance_squared)
		    : value_ptr(value_ptr), distance_squared(distance_squared)
		{
		}

		[[nodiscard]] reference operator*() const { return *value_ptr; }

		[[nodiscard]] pointer operator->() const { return value_ptr; }

		bool operator>(value_type rhs) const noexcept
		{
			return distance_squared > rhs.distance_squared;
		}
	};

	using value_type = std::pair<value_pointer, float>;
	using reference  = value_type&;
	using pointer    = value_type*;

 public:
	TreeMapNearestIteratorHelper(TreeMap* tm = nullptr) : tm_(tm) {}

	virtual ~TreeMapNearestIteratorHelper() {}

	[[nodiscard]] virtual value_type init(Index node) = 0;

	[[nodiscard]] virtual value_type next() = 0;

	[[nodiscard]] virtual TreeMapNearestIteratorHelper* copy() const = 0;

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
	[[nodiscard]] bool validReturn(Index node, Point point,
	                               Predicate const& predicate) const
	{
		return pred::ValueCheck<Predicate>::apply(predicate, *tm_, node, point);
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
class TreeMapNearestIteratorWrapper
{
	using TMNIH = TreeMapNearestIteratorHelper<TreeMap>;

	using Index = typename TreeMap::Index;

	static constexpr bool const IsConst = TMNIH::IsConst;

 public:
	//
	// Tags
	//

	using iterator_category = typename TMNIH::iterator_category;
	using difference_type   = typename TMNIH::difference_type;
	using value_type        = typename TMNIH::value_type;
	using reference         = typename TMNIH::reference;
	using pointer           = typename TMNIH::pointer;

	TreeMapNearestIteratorWrapper() = default;

	TreeMapNearestIteratorWrapper(TMNIH* it, Index node) : it_(it), cur_(it_->init(node)) {}

	TreeMapNearestIteratorWrapper(TreeMapNearestIteratorWrapper const& other)
	    : it_(other.it_->copy()), cur_(other.cur_)
	{
	}

	template <class TreeMap2,
	          typename std::enable_if_t<
	              IsConst && !TreeMapNearestIteratorWrapper<TreeMap2>::IsConst &&
	              std::is_same_v<std::decay_t<TreeMap>, std::decay_t<TreeMap2>>> = true>
	TreeMapNearestIteratorWrapper(TreeMapNearestIteratorWrapper<TreeMap2> const& other)
	    : it_(other.it_->copy()), cur_(other.cur_)
	{
	}

	TreeMapNearestIteratorWrapper& operator=(TreeMapNearestIteratorWrapper const& rhs)
	{
		it_  = rhs.it_->copy();
		cur_ = rhs.cur_;
		return *this;
	}

	template <class TreeMap2,
	          typename std::enable_if_t<
	              IsConst && !TreeMapNearestIteratorWrapper<TreeMap2>::IsConst &&
	              std::is_same_v<std::decay_t<TreeMap>, std::decay_t<TreeMap2>>> = true>
	TreeMapNearestIteratorWrapper& operator=(
	    TreeMapNearestIteratorWrapper<TreeMap2> const& rhs)
	{
		it_  = rhs.it_->copy();
		cur_ = rhs.cur_;
		return *this;
	}

	[[nodiscard]] reference operator*() const { return cur_; }

	[[nodiscard]] pointer operator->() const { return &cur_; }

	TreeMapNearestIteratorWrapper& operator++()
	{
		cur_ = it_->next();
		return *this;
	}

	TreeMapNearestIteratorWrapper operator++(int)
	{
		auto tmp = *this;
		++*this;
		return tmp;
	}

	bool operator==(TreeMapNearestIteratorWrapper const& rhs) const
	{
		return cur_ == rhs.cur_;
	}

	template <class TreeMap2, typename std::enable_if_t<std::is_same_v<
	                              std::decay_t<TreeMap>, std::decay_t<TreeMap2>>> = true>
	bool operator==(TreeMapNearestIteratorWrapper<TreeMap2> const& rhs) const
	{
		return cur_ == rhs.cur_;
	}

	bool operator!=(TreeMapNearestIteratorWrapper const& rhs) const
	{
		return cur_ != rhs.cur_;
	}

	template <class TreeMap2, typename std::enable_if_t<std::is_same_v<
	                              std::decay_t<TreeMap>, std::decay_t<TreeMap2>>> = true>
	bool operator!=(TreeMapNearestIteratorWrapper<TreeMap2> const& rhs) const
	{
		return cur_ != rhs.cur_;
	}

 private:
	std::unique_ptr<TMNIH> it_;
	value_type             cur_;
};

template <class TreeMap, class Predicate>
class TreeMapNearestIterator final : private TreeMapNearestIteratorHelper<TreeMap>
{
	using Base = TreeMapNearestIteratorHelper<TreeMap>;

	using pos_t = typename TreeMap::pos_t;
	using Index = typename TreeMap::Index;
	using Point = typename TreeMap::Point;

	static constexpr bool const IsPair = is_pair_v<typename TreeMap::value_type>;

 public:
	//
	// Tags
	//

	using iterator_category = typename Base::iterator_category;
	using difference_type   = typename Base::difference_type;
	using value_type        = typename Base::value_type;
	using reference         = typename Base::reference;
	using pointer           = typename Base::pointer;

	TreeMapNearestIterator()                                    = default;
	TreeMapNearestIterator(TreeMapNearestIterator const& other) = default;
	TreeMapNearestIterator(TreeMapNearestIterator&& other)      = default;

	TreeMapNearestIterator(TreeMap* tm, Predicate const& predicate, Point query,
	                       float epsilon)
	    : Base(tm), predicate_(predicate), query_(query), epsilon_sq_(epsilon * epsilon)
	{
	}

	~TreeMapNearestIterator() override {}

	[[nodiscard]] value_type init(Index node) override
	{
		Base::initPredicate(predicate_);

		if (Base::validReturn(node, predicate_)) {
			auto [first, last] = Base::iters(node);
			for (; last != first; ++first) {
				Point p;
				if constexpr (IsPair) {
					p = first->first;
				} else {
					p = *first;
				}
				if (Base::validReturn(node, p, predicate_)) {
					float dist_sq = squaredDistance(query_, p);
					value_queue_.emplace(*first, dist_sq);
				}
			}
			if (!value_queue_.empty()) {
				auto value     = value_queue_.top();
				value.distance = std::sqrt(value.distance);
				return value;
			} else {
				return {{}, std::numeric_limits<float>::quiet_NaN()};
			}

		} else if (Base::validInner(node, predicate_)) {
			// TODO: Implement
			float dist_sq = squaredDistance(query_, ...) + epsilon_sq_;
			inner_queue_.emplace(node, dist_sq);
			return next();
		}
	}

	[[nodiscard]] value_type next() override
	{
		if (!value_queue_.empty()) {
			value_queue_.pop();
		}

		// Skip forward to next valid return node
		while (!inner_queue_.empty() &&
		       (!value_queue_.empty() ||
		        value_queue_.top().distance > inner_queue_.top().distance)) {
			auto children = Base::children(inner_queue_.top().block);

			for (int i = 0; TreeMap::branchingFactor() > i; ++i) {
				Index node(children, i);

				if (Base::validReturn(node, predicate_)) {
					auto [first, last] = Base::iters(node);
					for (; last != first; ++first) {
						Point p;
						if constexpr (IsPair) {
							p = first->first;
						} else {
							p = *first;
						}
						if (Base::validReturn(node, p, predicate_)) {
							float dist_sq = squaredDistance(query_, p);
							value_queue_.emplace(*first, dist_sq);
						}
					}
				} else if (Base::validInner(node, predicate_)) {
					// TODO: Implement
					float dist_sq = squaredDistance(query_, ...) + epsilon_sq_;
					inner_queue_.emplace(node, dist_sq);
				}
			}
		}

		if (!value_queue_.empty()) {
			auto value     = value_queue_.top();
			value.distance = std::sqrt(value.distance);
			return value;
		} else {
			return {{}, std::numeric_limits<float>::quiet_NaN()};
		}
	}

	[[nodiscard]] TreeMapNearestIterator* copy() const override
	{
		return new TreeMapNearestIterator(*this);
	}

 private:
	struct Inner {
		Index node;
		float distance;

		Inner(Index node, float distance) : node(node), distance(distance) {}

		bool operator>(Inner rhs) const noexcept { return distance > rhs.distance; }
	};

	// Predicate that nodes has to fulfill
	Predicate predicate_{};

	Point query_;

	float epsilon_sq_;

	// Inner blocks queue
	std::priority_queue<Inner, std::vector<Inner>, std::greater<Inner>> inner_queue_;
	// Value queue
	std::priority_queue<TreeMapNearest, std::vector<TreeMapNearest>,
	                    std::greater<TreeMapNearest>>
	    value_queue_;
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_MAP_NEAREST_ITERATOR_HPP