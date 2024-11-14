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

#ifndef UFO_CONTAINER_TREE_SET_OR_MAP_QUERY_NEAREST_ITERATOR_HPP
#define UFO_CONTAINER_TREE_SET_OR_MAP_QUERY_NEAREST_ITERATOR_HPP

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

namespace ufo
{

namespace detail
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
		pred::Filter<Predicate>::init(predicate, *t_);
	}

	template <class Predicate>
	[[nodiscard]] bool returnable(Predicate const&                         predicate,
	                              typename TreeSetOrMap::value_type const& value) const
	{
		return pred::Filter<Predicate>::returnable(predicate, value);
	}

	template <class Predicate>
	[[nodiscard]] bool returnable(Predicate const& predicate, Index node) const
	{
		return t_->isPureLeaf(node) && !t_->empty(node) &&
		       pred::Filter<Predicate>::traversable(predicate, *t_, node);
	}

	template <class Predicate>
	[[nodiscard]] bool traversable(Predicate const& predicate, Index node) const
	{
		return t_->isParent(node) &&
		       pred::Filter<Predicate>::traversable(predicate, *t_, node);
	}

 protected:
	TreeSetOrMap* t_;
};

template <class TreeSetOrMap, class Predicate>
class TreeSetOrMapQueryNearestIterator final
    : public TreeSetOrMapQueryNearestIteratorHelper<TreeSetOrMap>
{
	template <class TreeSetOrMap2, class Predicate2>
	friend class TreeSetOrMapQueryNearestIterator;

	using Base = TreeSetOrMapQueryNearestIteratorHelper<TreeSetOrMap>;

	using Index       = typename TreeSetOrMap::Index;
	using Point       = typename TreeSetOrMap::Point;
	using RawIterator = typename Base::RawIterator;

	static constexpr bool const IsConst = Base::IsConst;
	static constexpr bool const IsMap   = Base::IsMap;

 public:
	//
	// Tags
	//

	using iterator_category = typename Base::iterator_category;
	using difference_type   = typename Base::difference_type;
	using value_type        = typename Base::value_type;
	using reference         = typename Base::reference;
	using pointer           = typename Base::pointer;

	TreeSetOrMapQueryNearestIterator()                                        = default;
	TreeSetOrMapQueryNearestIterator(TreeSetOrMapQueryNearestIterator const&) = default;
	TreeSetOrMapQueryNearestIterator(TreeSetOrMapQueryNearestIterator&&)      = default;

	template <
	    class TreeSetOrMap2,
	    typename std::enable_if_t<
	        IsConst &&
	            !TreeSetOrMapQueryNearestIterator<TreeSetOrMap2, Predicate>::IsConst &&
	            std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	        bool> = true>
	TreeSetOrMapQueryNearestIterator(
	    TreeSetOrMapQueryNearestIterator<TreeSetOrMap2, Predicate> const& other)
	    : Base(other), query_(other.query_), epsilon_sq_(other.epsilon_sq_)
	{
		auto inner_q = other.inner_queue_;
		while (!inner_q.empty()) {
			inner_queue_.emplace(inner_q.top().node, inner_q.top().distance);
			inner_q.pop();
		}
		auto value_q = other.value_queue_;
		while (!value_q.empty()) {
			value_queue_.emplace(value_q.top().node, value_q.top().it, value_q.top().distance);
			value_q.pop();
		}
	}

	TreeSetOrMapQueryNearestIterator(TreeSetOrMap* t, Point query,
	                                 Predicate const& predicate, float epsilon)
	    : Base(t), query_(query), epsilon_sq_(epsilon * epsilon)
	{
	}

	~TreeSetOrMapQueryNearestIterator() override {}

	[[nodiscard]] value_type init(Index node) override
	{
		Base::initPredicate(predicate_);

		if (Base::returnable(predicate_, node)) {
			auto& v = Base::values(node);
			for (auto it = std::begin(v), last = std::end(v); last != it; ++it) {
				if (!Base::returnable(predicate_, *it)) {
					continue;
				}

				Point p;
				if constexpr (IsMap) {
					p = it->first;
				} else {
					p = *it;
				}

				for (int i{}; Point::size() > i; ++i) {
					p[i] -= query_[i];
					p[i] *= p[i];
				}
				float dist_sq = p[0];
				for (int i = 1; Point::size() > i; ++i) {
					dist_sq += p[i];
				}
				value_queue_.emplace(node, it, dist_sq);
			}

			if (value_queue_.empty()) {
				return {};
			} else {
				auto v     = value_queue_.top();
				v.distance = std::sqrt(v.distance);
				return v;
			}
		} else if (Base::traversable(predicate_, node)) {
			// Distance does not matter here
			inner_queue_.emplace(node, 0.0f);
			return next();
		}
		return {};
	}

	[[nodiscard]] value_type next() override
	{
		if (!value_queue_.empty()) {
			value_queue_.pop();
		}

		// Skip forward to next valid return node
		while (!inner_queue_.empty() &&
		       (value_queue_.empty() || value_queue_.top() > inner_queue_.top())) {
			auto block = Base::children(inner_queue_.top().node);

			inner_queue_.pop();

			for (int i = 0; TreeSetOrMap::branchingFactor() > i; ++i) {
				Index node(block, i);

				if (Base::returnable(predicate_, node)) {
					auto& v = Base::values(node);
					for (auto it = std::begin(v), last = std::end(v); last != it; ++it) {
						if (!Base::returnable(predicate_, *it)) {
							continue;
						}

						Point p;
						if constexpr (IsMap) {
							p = it->first;
						} else {
							p = *it;
						}

						for (int i{}; Point::size() > i; ++i) {
							p[i] -= query_[i];
							p[i] *= p[i];
						}
						float dist_sq = p[0];
						for (int i = 1; Point::size() > i; ++i) {
							dist_sq += p[i];
						}
						value_queue_.emplace(node, it, dist_sq);
					}
				} else if (Base::traversable(predicate_, node)) {
					auto  min = Base::boundsMin(node);
					auto  max = Base::boundsMax(node);
					Point p;
					for (int i{}; Point::size() > i; ++i) {
						p[i] = UFO_CLAMP(query_[i], min[i], max[i]);
					}
					for (int i{}; Point::size() > i; ++i) {
						p[i] -= query_[i];
						p[i] *= p[i];
					}
					float dist_sq = p[0];
					for (int i = 1; Point::size() > i; ++i) {
						dist_sq += p[i];
					}
					inner_queue_.emplace(node, dist_sq + epsilon_sq_);
				}
			}
		}

		if (value_queue_.empty()) {
			return {};
		} else {
			auto v     = value_queue_.top();
			v.distance = std::sqrt(v.distance);
			return v;
		}
	}

	[[nodiscard]] TreeSetOrMapQueryNearestIterator* copy() const override
	{
		return new TreeSetOrMapQueryNearestIterator(*this);
	}

	[[nodiscard]] TreeSetOrMapQueryNearestIterator<TreeSetOrMap const, Predicate>*
	copyToConst() const override
	{
		return new TreeSetOrMapQueryNearestIterator<TreeSetOrMap const, Predicate>(*this);
	}

 private:
	struct Value;

	struct Inner {
		Index node;
		float distance;

		Inner(Index node, float distance) : node(node), distance(distance) {}

		bool operator>(Inner rhs) const noexcept { return distance > rhs.distance; }

		bool operator<(Value rhs) const noexcept { return distance < rhs.distance; }

		bool operator>(Value rhs) const noexcept { return distance > rhs.distance; }
	};

	struct Value {
		Index       node;
		RawIterator it;
		float       distance;

		Value(Index node, RawIterator it, float distance)
		    : node(node), it(it), distance(distance)
		{
		}

		operator value_type() const { return {*it, std::sqrt(distance)}; }

		bool operator>(Value rhs) const noexcept { return distance > rhs.distance; }

		bool operator<(Inner rhs) const noexcept { return distance < rhs.distance; }

		bool operator>(Inner rhs) const noexcept { return distance > rhs.distance; }
	};

	Point query_;
	float epsilon_sq_;

	// Predicate that nodes has to fulfill
	Predicate predicate_{};

	std::priority_queue<Inner, std::vector<Inner>, std::greater<Inner>> inner_queue_;
	std::priority_queue<Value, std::vector<Value>, std::greater<Value>> value_queue_;
};
}  // namespace detail

template <class TreeSetOrMap>
class TreeSetOrMapQueryNearestIterator
{
	using Iterator    = detail::TreeSetOrMapQueryNearestIteratorHelper<TreeSetOrMap>;
	using RawIterator = typename Iterator::RawIterator;

	using Index = typename TreeSetOrMap::Index;
	using Point = typename TreeSetOrMap::Point;

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

	TreeSetOrMapQueryNearestIterator() = default;

	TreeSetOrMapQueryNearestIterator(Iterator* it, Index node)
	    : it_(it), cur_(it_->init(node))
	{
	}

	template <class Predicate>
	TreeSetOrMapQueryNearestIterator(TreeSetOrMap* tree, Index node, Point query,
	                                 Predicate const& pred, float epsilon)
	    : it_(std::make_unique<
	          detail::TreeSetOrMapQueryNearestIterator<TreeSetOrMap, Predicate>>(
	          tree, query, pred, epsilon))
	    , cur_(it_->init(node))
	{
	}

	TreeSetOrMapQueryNearestIterator(TreeSetOrMapQueryNearestIterator const& other)
	    : cur_(other.cur_)
	{
		if (other.it_) {
			it_.reset(other.it_->copy());
		}
	}

	TreeSetOrMapQueryNearestIterator(TreeSetOrMapQueryNearestIterator&&) = default;

	template <
	    class TreeSetOrMap2,
	    typename std::enable_if_t<
	        IsConst && !TreeSetOrMapQueryNearestIterator<TreeSetOrMap2>::IsConst &&
	            std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	        bool> = true>
	TreeSetOrMapQueryNearestIterator(
	    TreeSetOrMapQueryNearestIterator<TreeSetOrMap2> const& other)
	    : cur_(other.cur_)
	{
		if (other.it_) {
			it_.reset(other.it_->copyToConst());
		}
	}

	TreeSetOrMapQueryNearestIterator& operator=(TreeSetOrMapQueryNearestIterator const& rhs)
	{
		if (rhs.it_) {
			it_.reset(rhs.it_->copy());
		} else {
			it_.reset();
		}
		cur_ = rhs.cur_;
		return *this;
	}

	TreeSetOrMapQueryNearestIterator& operator=(TreeSetOrMapQueryNearestIterator&&) =
	    default;

	template <
	    class TreeSetOrMap2,
	    typename std::enable_if_t<
	        IsConst && !TreeSetOrMapQueryNearestIterator<TreeSetOrMap2>::IsConst &&
	            std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	        bool> = true>
	TreeSetOrMapQueryNearestIterator& operator=(
	    TreeSetOrMapQueryNearestIterator<TreeSetOrMap2> const& rhs)
	{
		if (rhs.it_) {
			it_.reset(rhs.it_->copyToConst());
		} else {
			it_.reset();
		}
		cur_ = rhs.cur_;
		return *this;
	}

	[[nodiscard]] reference operator*() const { return cur_; }

	[[nodiscard]] pointer operator->() const { return &cur_; }

	TreeSetOrMapQueryNearestIterator& operator++()
	{
		cur_ = it_->next();
		return *this;
	}

	TreeSetOrMapQueryNearestIterator operator++(int)
	{
		auto tmp = *this;
		++*this;
		return tmp;
	}

	bool operator==(TreeSetOrMapQueryNearestIterator const& rhs) const
	{
		return cur_.it_ == rhs.cur_.it_;
	}

	template <class TreeSetOrMap2,
	          typename std::enable_if_t<
	              std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	              bool> = true>
	bool operator==(TreeSetOrMapQueryNearestIterator<TreeSetOrMap2> const& rhs) const
	{
		return cur_.it_ == rhs.cur_.it_;
	}

	bool operator!=(TreeSetOrMapQueryNearestIterator const& rhs) const
	{
		return !(*this == rhs);
	}

	template <class TreeSetOrMap2,
	          typename std::enable_if_t<
	              std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	              bool> = true>
	bool operator!=(TreeSetOrMapQueryNearestIterator<TreeSetOrMap2> const& rhs) const
	{
		return !(*this == rhs);
	}

 private:
	[[nodiscard]] RawIterator iterator() const { return cur_.it_; }

 private:
	std::unique_ptr<Iterator> it_;
	value_type                cur_;
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_SET_OR_MAP_QUERY_NEAREST_ITERATOR_HPP