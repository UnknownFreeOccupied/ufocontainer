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
#include <ufo/utility/macros.hpp>

// STL
#include <cmath>
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
	friend class TreeSetOrMapNearestIterator;

	friend TreeSetOrMap;

	static constexpr bool const IsConst = std::is_const_v<TreeSetOrMap>;
	static constexpr bool const IsMap   = TreeSetOrMap::IsMap;
	static constexpr auto const BF      = TreeSetOrMap::branchingFactor();

	using Index = typename TreeSetOrMap::Index;
	using Point = typename TreeSetOrMap::Point;

	using RawIterator =
	    std::conditional_t<IsConst, typename TreeSetOrMap::const_raw_iterator,
	                       typename TreeSetOrMap::raw_iterator>;

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

	TreeSetOrMapNearestIterator(TreeSetOrMap* t, Index node, Point query,
	                            float epsilon = 0.0f)
	    : t_(t), query_(query), epsilon_sq_(epsilon * epsilon)
	{
		if (t_->isParent(node)) {
			// Distance does not matter here
			inner_queue_.emplace(node, 0.0f);
			next();
		} else if (t_->isPureLeaf(node) && !t_->empty(node)) {
			auto& v = t_->values(node);
			for (auto it = std::begin(v), last = std::end(v); last != it; ++it) {
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
			cur_          = value_queue_.top();
			cur_.distance = std::sqrt(cur_.distance);
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
		t_          = rhs.t_;
		query_      = rhs.query_;
		epsilon_sq_ = rhs.epsilon_sq_;
		cur_        = rhs.cur_;

		inner_queue_ = {};
		value_queue_ = {};

		auto inner_q = rhs.inner_queue_;
		while (!inner_q.empty()) {
			inner_queue_.emplace(inner_q.top().node, inner_q.top().distance);
			inner_q.pop();
		}
		auto value_q = rhs.value_queue_;
		while (!value_q.empty()) {
			value_queue_.emplace(value_q.top().node, value_q.top().it, value_q.top().distance);
			value_q.pop();
		}

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
		return cur_.it_ == rhs.cur_.it_;
	}

	template <class TreeSetOrMap2,
	          typename std::enable_if_t<
	              std::is_same_v<std::decay_t<TreeSetOrMap>, std::decay_t<TreeSetOrMap2>>,
	              bool> = true>
	bool operator==(TreeSetOrMapNearestIterator<TreeSetOrMap2> const& rhs) const
	{
		return cur_.it_ == rhs.cur_.it_;
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
	    : t_(t), query_(other.query_), epsilon_sq_(other.epsilon_sq_), cur_(other.cur_)
	{
		auto inner_q = other.inner_queue_;
		while (!inner_q.empty()) {
			inner_queue_.emplace(inner_q.top().node, inner_q.top().distance);
			inner_q.pop();
		}
		auto value_q = other.value_queue_;
		while (!value_q.empty()) {
			// To remove const from other's iterators
			auto        const_it = value_q.top().it;
			RawIterator it       = t_->values(value_q.top().node).erase(const_it, const_it);
			value_queue_.emplace(value_q.top().node, it, value_q.top().distance);
			value_q.pop();
		}
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
		       (value_queue_.empty() || value_queue_.top() > inner_queue_.top())) {
			auto children = t_->children(inner_queue_.top().node);

			inner_queue_.pop();

			for (int i = 0; TreeSetOrMap::branchingFactor() > i; ++i) {
				Index node(children, i);

				if (t_->isParent(node)) {
					auto  min = t_->boundsMin(node);
					auto  max = t_->boundsMax(node);
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
				} else if (t_->isPureLeaf(node) && !t_->empty(node)) {
					auto& v = t_->values(node);
					for (auto it = std::begin(v), last = std::end(v); last != it; ++it) {
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
				}
			}
		}

		if (!value_queue_.empty()) {
			cur_          = value_queue_.top();
			cur_.distance = std::sqrt(cur_.distance);
		} else {
			cur_ = {};
		}
	}

	[[nodiscard]] RawIterator iterator() const { return value_queue_.top().it_; }

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

	struct Value : value_type {
		Index node;

		Value(Index node, RawIterator it, float distance)
		    : node(node), value_type(it, distance)
		{
		}

		bool operator>(Value rhs) const noexcept { return this->distance > rhs.distance; }

		bool operator<(Inner rhs) const noexcept { return this->distance < rhs.distance; }

		bool operator>(Inner rhs) const noexcept { return this->distance > rhs.distance; }
	};

	TreeSetOrMap* t_;

	Point query_;
	float epsilon_sq_;

	mutable value_type cur_;

	std::priority_queue<Inner, std::vector<Inner>, std::greater<Inner>> inner_queue_;
	std::priority_queue<Value, std::vector<Value>, std::greater<Value>> value_queue_;
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_SET_OR_MAP_NEAREST_ITERATOR_HPP