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

#ifndef UFO_CONTAINER_TREE_SET_QUERY_NEAREST_ITERATOR_HPP
#define UFO_CONTAINER_TREE_SET_QUERY_NEAREST_ITERATOR_HPP

// UFO
#include <ufo/container/tree/index.hpp>
#include <ufo/container/tree/predicate/filter.hpp>
#include <ufo/container/tree/predicate/predicate.hpp>
#include <ufo/geometry/distance.hpp>
#include <ufo/geometry/dynamic_geometry.hpp>

// STL
#include <cassert>
#include <cstddef>
#include <iterator>
#include <queue>

namespace ufo
{
// Forward declare
template <std::size_t Dim>
class TreeSet;

template <bool Const, std::size_t Dim, class Predicate = pred::Predicate<TreeSet<Dim>>,
          class Geometry = DynamicGeometry>
class TreeSetQueryNearestIterator
{
 private:
	//
	// Friends
	//

	template <bool, std::size_t, class, class>
	friend class TreeSetQueryNearestIterator;

	friend class TreeSet<Dim>;

 private:
	static constexpr std::size_t const BF = TreeSet<Dim>::branchingFactor();

	using RawIterator =
	    std::conditional_t<Const, typename TreeSet<Dim>::container_type::const_iterator,
	                       typename TreeSet<Dim>::container_type::iterator>;

	using Point = typename TreeSet<Dim>::Point;

	struct S {
		float       dist_sq;
		TreeIndex   node;
		RawIterator it;

		S(float dist_sq, TreeIndex node, RawIterator it = {}) noexcept
		    : dist_sq(dist_sq), node(node), it(it)
		{
		}

		bool operator>(S rhs) const noexcept { return dist_sq > rhs.dist_sq; }
	};

	using Queue = std::priority_queue<S, std::vector<S>, std::greater<S>>;

 public:
	//
	// Tags
	//

	// TODO: Make it so this also returns the distance?

	using iterator_category = std::forward_iterator_tag;
	using difference_type   = std::ptrdiff_t;
	using value_type        = typename std::iterator_traits<RawIterator>::value_type;
	using reference         = typename std::iterator_traits<RawIterator>::reference;
	using pointer           = typename std::iterator_traits<RawIterator>::pointer;

	TreeSetQueryNearestIterator() = default;

	TreeSetQueryNearestIterator(TreeSetQueryNearestIterator const&) = default;

	// From non-const to const or change of predicate/geometry type
	template <
	    bool Const2, class Predicate2, class Geometry2,
	    std::enable_if_t<(Const && !Const2) ||
	                         (Const == Const2 && (!std::is_same_v<Predicate, Predicate2> ||
	                                              !std::is_same_v<Geometry, Geometry2>)),
	                     bool> = true>
	TreeSetQueryNearestIterator(
	    TreeSetQueryNearestIterator<Const2, Dim, Predicate2, Geometry2> const& other)
	    : tm_(other.tm_)
	    , pred_(other.pred_)
	    , query_(other.query_)
	    , epsilon_sq_(other.epsilon_sq_)
	{
		auto queue = other.queue_;
		while (!queue.empty()) {
			queue_.emplace(queue.top().dist_sq, queue.top().node, queue.top().it);
			queue.pop();
		}
	}

	TreeSetQueryNearestIterator& operator++()
	{
		queue_.pop();
		next();
		return *this;
	}

	TreeSetQueryNearestIterator operator++(int)
	{
		TreeSetQueryNearestIterator tmp(*this);
		++*this;
		return tmp;
	}

	reference operator*() const { return *queue_.top().it; }

	pointer operator->() const { return &*queue_.top().it; }

	template <bool Const2, class Predicate2, class Geometry2>
	friend bool operator==(
	    TreeSetQueryNearestIterator const&                                     lhs,
	    TreeSetQueryNearestIterator<Const2, Dim, Predicate2, Geometry2> const& rhs)
	{
		return lhs.queue_.empty() == rhs.queue_.empty() &&
		       (lhs.queue_.empty() || lhs.queue_.top().it == rhs.queue_.top().it);
	}

	template <bool Const2, class Predicate2, class Geometry2>
	friend bool operator!=(
	    TreeSetQueryNearestIterator const&                                     lhs,
	    TreeSetQueryNearestIterator<Const2, Dim, Predicate2, Geometry2> const& rhs)
	{
		return !(lhs == rhs);
	}

 private:
	[[nodiscard]] bool returnable(value_type const& value) const
	{
		return pred::Filter<Predicate>::returnable(pred_, value);
	}

	[[nodiscard]] bool returnable(TreeIndex node) const
	{
		return tm_->isPureLeaf(node) && !tm_->empty(node);
	}

	[[nodiscard]] bool returnable(S const& s) const { return RawIterator{} != s.it; }

	[[nodiscard]] bool traversable(TreeIndex node) const
	{
		return tm_->isParent(node) && pred::Filter<Predicate>::traversable(pred_, *tm_, node);
	}

	void next()
	{
		while (!queue_.empty()) {
			auto cur = queue_.top();
			if (returnable(cur)) {
				return;
			}

			queue_.pop();

			if (returnable(cur.node)) {
				auto& v = tm_->values(cur.node);
				using std::begin;
				using std::end;
				for (auto it = begin(v), last = end(v); it != last; ++it) {
					if (!returnable(*it)) {
						continue;
					}
					Point p       = *it;
					float dist_sq = distanceSquared(query_, p);
					queue_.emplace(dist_sq, cur.node, it);
				}
				continue;
			}

			TreeIndex node = tm_->child(cur.node, 0);
			for (; BF > node.offset; ++node.offset) {
				if (!traversable(node) && !returnable(node)) {
					continue;
				}

				float dist_sq = distanceSquared(query_, tm_->bounds(node)) + epsilon_sq_;
				queue_.emplace(dist_sq, node);
			}
		}
	}

	[[nodiscard]] RawIterator iterator() { return queue_.top().it; }

 private:
	TreeSetQueryNearestIterator(TreeSet<Dim>* tm, TreeIndex node, Predicate const& pred,
	                            Geometry const& query, float epsilon = 0.0f)
	    : tm_(tm), pred_(pred), query_(query), epsilon_sq_(epsilon * epsilon)
	{
		pred::Filter<Predicate>::init(pred_, *tm_);

		if (traversable(node) || returnable(node)) {
			float dist_sq = distanceSquared(query_, tm_->bounds(node));
			queue_.emplace(dist_sq, node);
			next();
		}
	}

	// From const to non-const
	template <bool Const2, class Predicate2, class Geometry2,
	          std::enable_if_t<!Const && Const2, bool> = true>
	TreeSetQueryNearestIterator(
	    TreeSetQueryNearestIterator<Const2, Dim, Predicate2, Geometry2> const& other)
	    : tm_(other.tm)
	    , pred_(other.pred_)
	    , query_(other.query_)
	    , epsilon_sq_(other.epsilon_sq_)
	{
		auto queue = other.queue_;
		while (!queue.empty()) {
			auto const& cur = queue.top();
			// Remove const from it
			queue_.emplace(cur.dist_sq, cur.node, tm_->values(cur.node).erase(cur.it, cur.it));
			queue.pop();
		}
	}

 private:
	TreeSet<Dim>* tm_ = nullptr;

	Predicate pred_{};

	Geometry query_;
	float    epsilon_sq_;

	Queue queue_;
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_SET_QUERY_ITERATOR_HPP