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

#ifndef UFO_CONTAINER_TREE_SET_OR_MAP_NEAREST_ITERATOR_HPP
#define UFO_CONTAINER_TREE_SET_OR_MAP_NEAREST_ITERATOR_HPP

// UFO
#include <ufo/container/tree/map_nearest.hpp>
#include <ufo/container/tree/set_nearest.hpp>
#include <ufo/container/tree/set_or_map_nearest.hpp>

// STL
#include <iterator>
#include <queue>
#include <type_traits>
#include <vector>

namespace ufo
{
template <class TreeSetOrMap>
class TreeSetOrMapNearestIterator
{
	template <class TreeSetOrMap2>
	friend class TreeSetOrMapIterator;

	friend TreeSetOrMap;

	static constexpr bool const IsConst = std::is_const_v<TreeSetOrMap>;
	static constexpr bool const IsPair  = TreeSetOrMap::IsPair;

	using Index = typename TreeSetOrMap::Index;
	using Point = typename TreeSetOrMap::Point;

	static constexpr auto const BF = TreeSetOrMap::branchingFactor();

	using RawIterator = typename TreeSetOrMap::raw_iterator;

 public:
	//
	// Tags
	//

	using iterator_category = std::forward_iterator_tag;
	using difference_type   = std::ptrdiff_t;
	using value_type        = TreeSetOrMapNearest<typename TreeSetOrMap::value_type>;
	using reference         = std::conditional_t<IsConst, value_type const&, value_type&>;
	using pointer           = std::conditional_t<IsConst, value_type const*, value_type*>;

	TreeSetOrMapNearestIterator()                                   = default;
	TreeSetOrMapNearestIterator(TreeSetOrMapNearestIterator const&) = default;
	TreeSetOrMapNearestIterator(TreeSetOrMapNearestIterator&&)      = default;

	template <
	    class TreeSetOrMap2,
	    typename std::enable_if_t<
	        IsConst && !TreeSetOrMapNearestIterator<TreeSetOrMap2>::IsConst &&
	            std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	        bool> = true>
	TreeSetOrMapNearestIterator(TreeSetOrMapNearestIterator<TreeSetOrMap2> const& other)
	    : t_(other.t_)
	    , query_(other.query_)
	    , epsilon_sq_(other.epsilon_sq_)
	    , cur_(other.cur_)
	    , inner_queue_(other.inner_queue_)
	    , value_queue_(other.value_queue_)
	{
	}

	TreeSetOrMapNearestIterator(TreeSetOrMap* t, Index node, Point query,
	                            float epsilon = 0.0f)
	    : t_(t), query_(query), epsilon_sq_(epsilon * epsilon)
	{
		if (t_->isParent(node)) {
			// Distance does not matter here
			inner_queue_.emplace(node, 0.0f);
			next();
		} else if (t_->isPureLeaf(node) && !t_->empty(node)) {
			for (auto& v : t_->values(node)) {
				Point p;
				if constexpr (IsPair) {
					p = v.first;
				} else {
					p = v;
				}

				for (int i{}; Point::size() > i; ++i) {
					p[i] -= query_[i];
					p[i] *= p[i];
				}
				float dist_sq;
				for (int i{}; Point::size() > i; ++i) {
					dist_sq += p[i];
				}
				value_queue_.emplace(v, dist_sq);
			}
			cur_ = value_queue_.top();
		}
	}

	TreeSetOrMapNearestIterator(TreeSetOrMap& t, Index node, Point query,
	                            float epsilon = 0.0f)
	    : TreeSetOrMapNearestIterator(&t, node, query, epsilon)
	{
	}

	TreeSetOrMapNearestIterator& operator=(TreeSetOrMapNearestIterator const&) = default;
	TreeSetOrMapNearestIterator& operator=(TreeSetOrMapNearestIterator&&)      = default;

	template <
	    class TreeSetOrMap2,
	    typename std::enable_if_t<
	        IsConst && !TreeSetOrMapNearestIterator<TreeSetOrMap2>::IsConst &&
	            std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	        bool> = true>
	TreeSetOrMapNearestIterator& operator=(
	    TreeSetOrMapNearestIterator<TreeSetOrMap2> const& rhs)
	{
		t_           = rhs.t_;
		query_       = rhs.query_;
		epsilon_sq_  = rhs.epsilon_sq_;
		cur_         = rhs.cur_;
		inner_queue_ = rhs.inner_queue_;
		value_queue_ = rhs.value_queue_;
		return *this;
	}

	[[nodiscard]] reference operator*() const { return cur_; }

	[[nodiscard]] pointer operator->() const { return &cur_; }

	TreeSetOrMapNearestIterator& operator++()
	{
		next();
		return *this;
	}

	TreeSetOrMapNearestIterator operator++(int)
	{
		auto tmp = *this;
		++*this;
		return tmp;
	}

	bool operator==(TreeSetOrMapNearestIterator const& rhs) const
	{
		return (value_queue_.empty() == rhs.value_queue_.empty()) ||
		       (!value_queue_.empty() && value_queue_.top().it == rhs.value_queue_.top().it);
	}

	template <class TreeSetOrMap2,
	          typename std::enable_if_t<
	              std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	              bool> = true>
	bool operator==(TreeSetOrMapNearestIterator<TreeSetOrMap2> const& rhs) const
	{
		return (value_queue_.empty() == rhs.value_queue_.empty()) ||
		       (!value_queue_.empty() && value_queue_.top().it == rhs.value_queue_.top().it);
	}

	bool operator!=(TreeSetOrMapNearestIterator const& rhs) const
	{
		return !(*this == rhs);
	}

	template <class TreeSetOrMap2,
	          typename std::enable_if_t<
	              std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	              bool> = true>
	bool operator!=(TreeSetOrMapNearestIterator<TreeSetOrMap2> const& rhs) const
	{
		return !(*this == rhs);
	}

 private:
	template <
	    class TreeSetOrMap2,
	    typename std::enable_if_t<
	        !IsConst && TreeSetOrMapNearestIterator<TreeSetOrMap2>::IsConst &&
	            std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	        bool> = true>
	TreeSetOrMapNearestIterator(TreeSetOrMap*                                     t,
	                            TreeSetOrMapNearestIterator<TreeSetOrMap2> const& other)
	    : t_(t)
	    , query_(other.query_)
	    , epsilon_sq_(other.epsilon_sq_)
	    , cur_(other.cur_)
	    , inner_queue_(other.inner_queue_)
	    , value_queue_(other.value_queue_)
	{
	}

	template <
	    class TreeSetOrMap2,
	    typename std::enable_if_t<
	        !IsConst && TreeSetOrMapNearestIterator<TreeSetOrMap2>::IsConst &&
	            std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	        bool> = true>
	TreeSetOrMapNearestIterator(TreeSetOrMap&                                     t,
	                            TreeSetOrMapNearestIterator<TreeSetOrMap2> const& other)
	    : TreeSetOrMapNearestIterator(&t, other)
	{
	}

	void next()
	{
		if (!value_queue_.empty()) {
			value_queue_.pop();
		}

		// Skip forward to next valid return node
		while (!inner_queue_.empty() &&
		       (!value_queue_.empty() ||
		        value_queue_.top().distance > inner_queue_.top().distance)) {
			auto children = TreeSetOrMap::children(inner_queue_.top().block);

			for (int i = 0; TreeSetOrMap::branchingFactor() > i; ++i) {
				// Index node(children, i);

				// if (TreeSetOrMap::validReturn(node, predicate_)) {
				// 	auto [first, last] = TreeSetOrMap::iters(node);
				// 	for (; last != first; ++first) {
				// 		Point p;
				// 		if constexpr (IsPair) {
				// 			p = first->first;
				// 		} else {
				// 			p = *first;
				// 		}
				// 		if (TreeSetOrMap::validReturn(node, p, predicate_)) {
				// 			float dist_sq = squaredDistance(query_, p);
				// 			value_queue_.emplace(*first, dist_sq);
				// 		}
				// 	}
				// } else if (TreeSetOrMap::validInner(node, predicate_)) {
				// 	// TODO: Implement
				// 	float dist_sq = squaredDistance(query_, ...) + epsilon_sq_;
				// 	inner_queue_.emplace(node, dist_sq);
				// }
			}
		}

		if (!value_queue_.empty()) {
			// auto value     = value_queue_.top();
			// value.distance = std::sqrt(value.distance);
			// return value;
		} else {
			// return {{}, std::numeric_limits<float>::quiet_NaN()};
		}
	}

	[[nodiscard]] RawIterator iterator() const { return value_queue_.top().it; }

 private:
	struct Inner {
		Index node;
		float distance;

		Inner(Index node, float distance) : node(node), distance(distance) {}

		bool operator>(Inner rhs) const noexcept { return distance > rhs.distance; }
	};

	struct Value {
		RawIterator it;
		float       distance;

		Value(RawIterator it, float distance) : it(it), distance(distance) {}

		bool operator>(Value rhs) const noexcept { return distance > rhs.distance; }
	};

	TreeSetOrMap* t_;

	Point query_;
	float epsilon_sq_;

	value_type cur_;

	std::priority_queue<Inner, std::vector<Inner>, std::greater<Inner>> inner_queue_;
	std::priority_queue<Value, std::vector<Value>, std::greater<Value>> value_queue_;
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_SET_OR_MAP_NEAREST_ITERATOR_HPP