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

#ifndef UFO_CONTAINER_TREE_MAP_HPP
#define UFO_CONTAINER_TREE_MAP_HPP

// UFO
#include <ufo/container/tree/map/block.hpp>
#include <ufo/container/tree/map/iterator.hpp>
#include <ufo/container/tree/map/nearest_iterator.hpp>
#include <ufo/container/tree/map/query_iterator.hpp>
#include <ufo/container/tree/map/query_nearest_iterator.hpp>
#include <ufo/container/tree/predicate/predicate.hpp>
#include <ufo/container/tree/tree.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/geometry/dynamic_geometry.hpp>

// STL
#include <array>
#include <cstddef>
#include <limits>
#include <list>

namespace ufo
{
template <std::size_t Dim, class T>
class TreeMap
    : protected Tree<TreeMap<Dim, T>, Dim, false,
                     TreeMapBlock<Dim, std::size_t(1) << Dim, T>>
{
 protected:
	using Block = TreeMapBlock<Dim, std::size_t(1) << Dim, T>;
	using Base  = Tree<TreeMap, Dim, false, Block>;

	//
	// Friends
	//

	// First base friends :)
	friend Base;

	template <class Geometry, pred::SpatialTag Tag, bool Negated>
	friend class pred::Spatial;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                        Tags                                         |
	|                                                                                     |
	**************************************************************************************/

	// UFO stuff
	using Index    = typename Base::Index;
	using Node     = typename Base::Node;
	using Code     = typename Base::Code;
	using Key      = typename Base::Key;
	using Point    = typename Base::Point;
	using Coord    = typename Base::Coord;
	using Bounds   = typename Base::Bounds;
	using Length   = typename Base::Length;
	using coord_t  = typename Base::coord_t;
	using depth_t  = typename Base::depth_t;
	using offset_t = typename Base::offset_t;
	using length_t = typename Base::length_t;
	using pos_t    = typename Base::pos_t;

	// STL stuff
	using value_type      = typename Block::value_type;
	using mapped_type     = T;
	using reference       = value_type&;
	using const_reference = value_type const&;
	using pointer         = value_type*;
	using const_pointer   = value_type const*;
	using size_type       = std::size_t;
	using difference_type = std::ptrdiff_t;

	// TODO: Add `iterator` postfix

	// Iterators
	using iterator       = TreeMapIterator<false, Dim, T>;
	using const_iterator = TreeMapIterator<true, Dim, T>;

	template <class Predicate>
	using query_iterator_pred = TreeMapQueryIterator<false, Dim, T, Predicate>;
	template <class Predicate>
	using const_query_iterator_pred = TreeMapQueryIterator<true, Dim, T, Predicate>;

	using query_iterator       = query_iterator_pred<pred::Predicate<TreeMap>>;
	using const_query_iterator = const_query_iterator_pred<pred::Predicate<TreeMap>>;

	template <class Geometry>
	using nearest_iterator_geom = TreeMapNearestIterator<false, Dim, T, Geometry>;
	template <class Geometry>
	using const_nearest_iterator_geom = TreeMapNearestIterator<true, Dim, T, Geometry>;

	using nearest_iterator       = nearest_iterator_geom<DynamicGeometry>;
	using const_nearest_iterator = const_nearest_iterator_geom<DynamicGeometry>;

	template <class Predicate, class Geometry>
	using query_nearest_iterator_pred_geom =
	    TreeMapQueryNearestIterator<false, Dim, T, Predicate, Geometry>;
	template <class Predicate, class Geometry>
	using const_query_nearest_iterator_pred_geom =
	    TreeMapQueryNearestIterator<true, Dim, T, Predicate, Geometry>;

	using query_nearest_iterator =
	    query_nearest_iterator_pred_geom<pred::Predicate<TreeMap>, DynamicGeometry>;
	using const_query_nearest_iterator =
	    const_query_nearest_iterator_pred_geom<pred::Predicate<TreeMap>, DynamicGeometry>;

	template <class Predicate>
	using Query = IteratorWrapper<query_iterator_pred<Predicate>, query_iterator>;
	template <class Predicate>
	using ConstQuery =
	    IteratorWrapper<const_query_iterator_pred<Predicate>, const_query_iterator>;

	template <class Geometry>
	using Nearest = IteratorWrapper<nearest_iterator_geom<Geometry>, nearest_iterator>;
	template <class Geometry>
	using ConstNearest =
	    IteratorWrapper<const_nearest_iterator_geom<Geometry>, const_nearest_iterator>;

	template <class Predicate, class Geometry>
	using QueryNearest =
	    IteratorWrapper<query_nearest_iterator_pred_geom<Predicate, Geometry>,
	                    query_nearest_iterator>;
	template <class Predicate, class Geometry>
	using ConstQueryNearest =
	    IteratorWrapper<const_query_nearest_iterator_pred_geom<Predicate, Geometry>,
	                    const_query_nearest_iterator>;

	//
	// Friend iterators
	//

	template <bool, std::size_t, class>
	friend class TreeMapIterator;

	template <bool, std::size_t, class, class>
	friend class TreeMapQueryIterator;

	template <bool, std::size_t, class, class>
	friend class TreeMapNearestIterator;

	template <bool, std::size_t, class, class, class>
	friend class TreeMapQueryNearestIterator;

 private:
	using container_type = typename Block::container_type;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	TreeMap(Length  leaf_node_length = Length(0.1),
	        depth_t num_depth_levels = std::min(depth_t(17), Base::maxNumDepthLevels()))
	    : Base(leaf_node_length, num_depth_levels)
	{
	}

	TreeMap(length_t leaf_node_length,
	        depth_t  num_depth_levels = std::min(depth_t(17), Base::maxNumDepthLevels()))
	    : TreeMap(Length(leaf_node_length), num_depth_levels)
	{
	}

	template <class InputIt>
	TreeMap(InputIt first, InputIt last, length_t leaf_node_length = length_t(0.1),
	        depth_t num_depth_levels = std::min(depth_t(17), Base::maxNumDepthLevels()))
	    : TreeMap(leaf_node_length, num_depth_levels)
	{
		insert(first, last);
	}

	TreeMap(std::initializer_list<value_type> init,
	        length_t                          leaf_node_length = length_t(0.1),
	        depth_t num_depth_levels = std::min(depth_t(17), Base::maxNumDepthLevels()))
	    : TreeMap(init.begin(), init.end(), leaf_node_length, num_depth_levels)
	{
	}

	template <class Range>
	TreeMap(Range const& range, length_t leaf_node_length = length_t(0.1),
	        depth_t num_depth_levels = std::min(depth_t(17), Base::maxNumDepthLevels()))
	    : TreeMap(std::begin(range), std::end(range), leaf_node_length, num_depth_levels)
	{
	}

	TreeMap(TreeMap const&) = default;

	TreeMap(TreeMap&&) = default;

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~TreeMap() = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	TreeMap& operator=(TreeMap const&) = default;

	TreeMap& operator=(TreeMap&&) = default;

	/**************************************************************************************
	|                                                                                     |
	|                                      Iterators                                      |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] iterator begin() { return iterator(this, Base::index()); }

	[[nodiscard]] const_iterator begin() const
	{
		return const_iterator(const_cast<TreeMap*>(this), Base::index());
	}

	[[nodiscard]] const_iterator cbegin() const { return begin(); }

	[[nodiscard]] iterator end() { return iterator(); }

	[[nodiscard]] const_iterator end() const { return const_iterator(); }

	[[nodiscard]] const_iterator cend() const { return end(); }

	/**************************************************************************************
	|                                                                                     |
	|                                   Query iterators                                   |
	|                                                                                     |
	**************************************************************************************/

	template <class Predicate>
	[[nodiscard]] query_iterator_pred<Predicate> beginQuery(Predicate const& pred)
	{
		return query_iterator_pred<Predicate>(this, Base::index(), pred);
	}

	template <class Predicate>
	[[nodiscard]] const_query_iterator_pred<Predicate> beginQuery(
	    Predicate const& pred) const
	{
		return const_query_iterator_pred<Predicate>(const_cast<TreeMap*>(this), Base::index(),
		                                            pred);
	}

	template <class Predicate>
	[[nodiscard]] const_query_iterator_pred<Predicate> cbeginQuery(
	    Predicate const& pred) const
	{
		return beginQuery(pred);
	}

	[[nodiscard]] query_iterator endQuery() { return query_iterator(); }

	[[nodiscard]] const_query_iterator endQuery() const { return const_query_iterator(); }

	[[nodiscard]] const_query_iterator cendQuery() const { return endQuery(); }

	/**************************************************************************************
	|                                                                                     |
	|                                  Nearest iterators                                  |
	|                                                                                     |
	**************************************************************************************/

	template <class Geometry>
	[[nodiscard]] nearest_iterator_geom<Geometry> beginNearest(Geometry const& query,
	                                                           float epsilon = 0.0f)
	{
		return nearest_iterator_geom<Geometry>(this, Base::index(), query, epsilon);
	}

	template <class Geometry>
	[[nodiscard]] const_nearest_iterator_geom<Geometry> beginNearest(
	    Geometry const& query, float epsilon = 0.0f) const
	{
		return const_nearest_iterator_geom<Geometry>(const_cast<TreeMap*>(this),
		                                             Base::index(), query, epsilon);
	}

	template <class Geometry>
	[[nodiscard]] const_nearest_iterator_geom<Geometry> cbeginNearest(
	    Geometry const& query, float epsilon = 0.0f) const
	{
		return beginNearest(query, epsilon);
	}

	[[nodiscard]] nearest_iterator endNearest() { return nearest_iterator(); }

	[[nodiscard]] const_nearest_iterator endNearest() const
	{
		return const_nearest_iterator();
	}

	[[nodiscard]] const_nearest_iterator cendNearest() const { return endNearest(); }

	/**************************************************************************************
	|                                                                                     |
	|                               Query nearest iterators                               |
	|                                                                                     |
	**************************************************************************************/

	template <class Predicate, class Geometry>
	[[nodiscard]] query_nearest_iterator_pred_geom<Predicate, Geometry> beginQueryNearest(
	    Predicate const& pred, Geometry const& query, float epsilon = 0.0f)
	{
		return query_nearest_iterator_pred_geom<Predicate, Geometry>(this, Base::index(),
		                                                             pred, query, epsilon);
	}

	template <class Predicate, class Geometry>
	[[nodiscard]] const_query_nearest_iterator_pred_geom<Predicate, Geometry>
	beginQueryNearest(Predicate const& pred, Geometry const& query,
	                  float epsilon = 0.0f) const
	{
		return const_query_nearest_iterator_pred_geom<Predicate, Geometry>(
		    const_cast<TreeMap*>(this), Base::index(), pred, query, epsilon);
	}

	template <class Predicate, class Geometry>
	[[nodiscard]] const_query_nearest_iterator_pred_geom<Predicate, Geometry>
	cbeginQueryNearest(Predicate const& pred, Geometry const& query,
	                   float epsilon = 0.0f) const
	{
		return beginQueryNearest(pred, query, epsilon);
	}

	[[nodiscard]] query_nearest_iterator endQueryNearest()
	{
		return query_nearest_iterator();
	}

	[[nodiscard]] const_query_nearest_iterator endQueryNearest() const
	{
		return const_query_nearest_iterator();
	}

	[[nodiscard]] const_query_nearest_iterator cendQueryNearest() const
	{
		return endQueryNearest();
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Capacity                                       |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Checks if the container has no elements.
	 *
	 * @return `true` if the container is empty, `false` otherwise.
	 */
	[[nodiscard]] bool empty() const noexcept { return 0 == size(); }

	/*!
	 * @brief Returns the number of elements in the container.
	 *
	 * @return The number of elements in the container.
	 */
	[[nodiscard]] size_type size() const noexcept { return size_; }

	/*!
	 * @brief Returns the minimum `Bounds` able to contain all values stored
	 * in the container.
	 *
	 * @return The minimum `Bounds` able to contain all values stored in the container.
	 */
	[[nodiscard]] Bounds bounds() const { return bounds(Base::index()); }

	/**************************************************************************************
	|                                                                                     |
	|                                      Modifiers                                      |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Erases all elements from the container. After this call, `size()` returns
	 * zero.
	 */
	void clear()
	{
		Base::clear();
		size_ = 0;
	}

	template <class... Args>
	void emplace(Point point, Args&&... args)
	{
		insert(value_type(point, T(std::forward<Args>(args)...)));
	}

	void insert(value_type const& value)
	{
		Index node = Base::create(value.first);
		insert(node, value);
	}

	void insert(value_type&& value)
	{
		Index node = Base::create(value.first);
		insert(node, std::move(value));
	}

	template <class InputIt>
	void insert(InputIt first, InputIt last)
	{
		std::vector<Point> points;

		std::transform(first, last, std::back_inserter(points),
		               [](auto const& v) { return v.first; });

		auto nodes = Base::create(points.begin(), points.end());

		for (std::size_t i{}; first != last; ++i, ++first) {
			insert(nodes[i], *first);
		}
	}

	template <
	    class ExecutionPolicy, class RandomIt,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insert(ExecutionPolicy&& policy, RandomIt first, RandomIt last)
	{
		std::vector<Point> points(std::distance(first, last));

		if constexpr (execution::is_stl_v<ExecutionPolicy>) {
			std::transform(execution::toSTL(policy), first, last, points.begin(),
			               [](auto const& v) { return v.first; });
		}
#if defined(UFO_PAR_GCD)
		else if constexpr (execution::is_gcd_v<ExecutionPolicy>) {
			// TODO: Implement
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "insert not implemented for that execution policy");
		}
#endif
#if defined(UFO_PAR_TBB)
		else if constexpr (execution::is_tbb_v<ExecutionPolicy>) {
			// TODO: Implement
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "insert not implemented for that execution policy");
		}
#endif
		else if constexpr (execution::is_omp_v<ExecutionPolicy>) {
			if constexpr (execution::is_seq_v<ExecutionPolicy>) {
				for (std::size_t i = 0; points.size() > i; ++i) {
					points[i] = first[i].first;
				}
			} else if constexpr (execution::is_unseq_v<ExecutionPolicy>) {
#pragma omp simd
				for (std::size_t i = 0; points.size() > i; ++i) {
					points[i] = first[i].first;
				}
			} else if constexpr (execution::is_par_v<ExecutionPolicy>) {
#pragma omp parallel for
				for (std::size_t i = 0; points.size() > i; ++i) {
					points[i] = first[i].first;
				}
			} else if constexpr (execution::is_par_unseq_v<ExecutionPolicy>) {
#pragma omp parallel for simd
				for (std::size_t i = 0; points.size() > i; ++i) {
					points[i] = first[i].first;
				}
			}
		} else {
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		}

		auto nodes =
		    Base::create(std::forward<ExecutionPolicy>(policy), points.begin(), points.end());

		for (std::size_t i{}; first != last; ++i, ++first) {
			insert(nodes[i], *first);
		}
	}

	void insert(std::initializer_list<value_type> ilist)
	{
		insert(ilist.begin(), ilist.end());
	}

	template <
	    class ExecutionPolicy,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insert(ExecutionPolicy&& policy, std::initializer_list<value_type> ilist)
	{
		insert(std::forward<ExecutionPolicy>(policy), ilist.begin(), ilist.end());
	}

	template <class Range>
	void insert(Range const& r)
	{
		using std::begin;
		using std::end;
		insert(begin(r), end(r));
	}

	template <
	    class ExecutionPolicy, class Range,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	void insert(ExecutionPolicy&& policy, Range const& r)
	{
		using std::begin;
		using std::end;
		insert(std::forward<ExecutionPolicy>(policy), begin(r), end(r));
	}

	size_type erase(value_type const& value)
	{
		auto node = Base::index(value.first);

		if (!Base::isPureLeaf(node)) {
			return 0;
		}

		auto&     v           = values(node);
		size_type num_removed = v.size();
		v.remove_if([&value](auto const& x) {
			return x.first == value.first && x.second == value.second;
		});
		num_removed -= v.size();

		size_ -= num_removed;

		erasePropagate(node);

		return num_removed;
	}

	iterator erase(iterator pos)
	{
		auto it = pos.iterator();
		++pos;
		erase(it);
		return pos;
	}

	iterator erase(const_iterator pos)
	{
		auto it = pos.iterator();
		++pos;
		erase(it);
		return iterator(pos);
	}

	template <class Predicate>
	query_iterator_pred<Predicate> erase(query_iterator_pred<Predicate> pos)
	{
		auto it = pos.iterator();
		++pos;
		erase(it);
		return pos;
	}

	template <class Predicate>
	query_iterator_pred<Predicate> erase(const_query_iterator_pred<Predicate> pos)
	{
		auto it = pos.iterator();
		++pos;
		erase(it);
		return query_iterator_pred<Predicate>(pos);
	}

	template <class Geometry>
	nearest_iterator_geom<Geometry> erase(nearest_iterator_geom<Geometry> pos)
	{
		auto it = pos.iterator();
		++pos;
		erase(it);
		return pos;
	}

	template <class Geometry>
	nearest_iterator_geom<Geometry> erase(const_nearest_iterator_geom<Geometry> pos)
	{
		auto it = pos.iterator();
		++pos;
		erase(it);
		return nearest_iterator_geom<Geometry>(pos);
	}

	template <class Predicate, class Geometry>
	query_nearest_iterator_pred_geom<Predicate, Geometry> erase(
	    query_nearest_iterator_pred_geom<Predicate, Geometry> pos)
	{
		auto it = pos.iterator();
		++pos;
		erase(it);
		return pos;
	}

	template <class Predicate, class Geometry>
	query_nearest_iterator_pred_geom<Predicate, Geometry> erase(
	    const_query_nearest_iterator_pred_geom<Predicate, Geometry> pos)
	{
		auto it = pos.iterator();
		++pos;
		erase(it);
		return query_nearest_iterator_pred_geom<Predicate, Geometry>(pos);
	}

	iterator erase(iterator first, iterator last)
	{
		while (last != first) {
			auto it = first.iterator();
			++first;
			erase(it);
		}
		return first;
	}

	iterator erase(const_iterator first, const_iterator last)
	{
		while (last != first) {
			auto it = first.iterator();
			++first;
			erase(it);
		}
		return iterator(first);
	}

	template <class Predicate1, class Predicate2>
	query_iterator_pred<Predicate1> erase(query_iterator_pred<Predicate1> first,
	                                      query_iterator_pred<Predicate2> last)
	{
		while (last != first) {
			auto it = first.iterator();
			++first;
			erase(it);
		}
		return first;
	}

	template <class Predicate1, class Predicate2>
	query_iterator_pred<Predicate1> erase(const_query_iterator_pred<Predicate1> first,
	                                      const_query_iterator_pred<Predicate2> last)
	{
		while (last != first) {
			auto it = first.iterator();
			++first;
			erase(it);
		}
		return query_iterator_pred<Predicate1>(first);
	}

	template <class Predicate>
	query_iterator_pred<Predicate> erase(Query<Predicate> query)
	{
		return erase(query.begin(), query.end());
	}

	template <class Predicate>
	query_iterator_pred<Predicate> erase(ConstQuery<Predicate> query)
	{
		return erase(query.begin(), query.end());
	}

	template <class Geometry1, class Geometry2>
	nearest_iterator_geom<Geometry1> erase(nearest_iterator_geom<Geometry1> first,
	                                       nearest_iterator_geom<Geometry2> last)
	{
		while (last != first) {
			auto it = first.iterator();
			++first;
			erase(it);
		}
		return first;
	}

	template <class Geometry1, class Geometry2>
	nearest_iterator_geom<Geometry1> erase(const_nearest_iterator_geom<Geometry1> first,
	                                       const_nearest_iterator_geom<Geometry2> last)
	{
		while (last != first) {
			auto it = first.iterator();
			++first;
			erase(it);
		}
		return nearest_iterator_geom<Geometry1>(first);
	}

	template <class Predicate1, class Geometry1, class Predicate2, class Geometry2>
	query_nearest_iterator_pred_geom<Predicate1, Geometry1> erase(
	    query_nearest_iterator_pred_geom<Predicate1, Geometry1> first,
	    query_nearest_iterator_pred_geom<Predicate2, Geometry2> last)
	{
		while (last != first) {
			auto it = first.iterator();
			++first;
			erase(it);
		}
		return first;
	}

	template <class Predicate1, class Geometry1, class Predicate2, class Geometry2>
	query_nearest_iterator_pred_geom<Predicate1, Geometry1> erase(
	    const_query_nearest_iterator_pred_geom<Predicate1, Geometry1> first,
	    const_query_nearest_iterator_pred_geom<Predicate2, Geometry2> last)
	{
		while (last != first) {
			auto it = first.iterator();
			++first;
			erase(it);
		}
		return query_nearest_iterator_pred_geom<Predicate1, Geometry1>(first);
	}

	template <class Predicate, class Geometry>
	query_nearest_iterator_pred_geom<Predicate, Geometry> erase(
	    QueryNearest<Predicate, Geometry> query)
	{
		return erase(query.begin(), query.end());
	}

	template <class Predicate, class Geometry>
	query_nearest_iterator_pred_geom<Predicate, Geometry> erase(
	    ConstQueryNearest<Predicate, Geometry> query)
	{
		return erase(query.begin(), query.end());
	}

	size_type erase(Point point)
	{
		auto node = Base::index(point);

		if (!Base::isPureLeaf(node)) {
			return 0;
		}

		auto&     v           = values(node);
		size_type num_removed = v.size();
		v.remove_if([point](auto const& x) { return x.first == point; });
		num_removed -= v.size();

		size_ -= num_removed;

		erasePropagate(node);

		return num_removed;
	}

	/*!
	 * @brief Exchanges the contents of the container with those of `other`.
	 *
	 * @param other	container to exchange the contents with
	 */
	void swap(TreeMap& other)
	{
		std::swap(size_, other.size_);
		std::swap(static_cast<Base&>(*this), static_cast<Base&>(other));
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Lookup                                        |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Returns the number of elements with Point that compares equivalent to the
	 * specified argument.
	 *
	 * @param point point of the elements to count
	 * @return Number of elements with Point that compares equivalent to `point`.
	 */
	[[nodiscard]] size_type count(Point point) const
	{
		auto const& v = values(Base::index(point));
		return std::count_if(v.begin(), v.end(),
		                     [point](auto const& x) { return x.first == point; });
	}

	/*!
	 * @brief Checks if there is an element with Point equivalent to `point` in the
	 * container.
	 *
	 * @param point point of the element to search for
	 * @return `true` if there is such an element, otherwise `false`.
	 */
	[[nodiscard]] bool contains(Point point) const
	{
		auto const& v = values(Base::index(point));
		return std::end(v) != std::find_if(v.begin(), v.end(), [point](auto const& x) {
			       return x.first == point;
		       });
	}

	template <class Predicate>
	[[nodiscard]] Query<Predicate> query(Predicate const& pred)
	{
		return Query<Predicate>(beginQuery(pred), endQuery());
	}

	template <class Predicate>
	[[nodiscard]] ConstQuery<Predicate> query(Predicate const& pred) const
	{
		return ConstQuery<Predicate>(beginQuery(pred), endQuery());
	}

	template <class Geometry>
	[[nodiscard]] Nearest<Geometry> nearest(Geometry const& query, float epsilon = 0.0f)
	{
		return Nearest<Geometry>(beginNearest(query, epsilon), endNearest());
	}

	template <class Geometry>
	[[nodiscard]] ConstNearest<Geometry> nearest(Geometry const& query,
	                                             float           epsilon = 0.0f) const
	{
		return ConstNearest<Geometry>(beginNearest(query, epsilon), endNearest());
	}

	template <class Predicate, class Geometry>
	[[nodiscard]] QueryNearest<Predicate, Geometry> queryNearest(Predicate const& pred,
	                                                             Geometry const&  query,
	                                                             float epsilon = 0.0f)
	{
		return QueryNearest<Predicate, Geometry>(beginQueryNearest(pred, query, epsilon),
		                                         endQueryNearest());
	}

	template <class Predicate, class Geometry>
	[[nodiscard]] ConstQueryNearest<Predicate, Geometry> queryNearest(
	    Predicate const& pred, Geometry const& query, float epsilon = 0.0f) const
	{
		return ConstQueryNearest<Predicate, Geometry>(beginQueryNearest(pred, query, epsilon),
		                                              endQueryNearest());
	}

	template <class Geometry>
	[[nodiscard]] float nearestDistance(
	    Geometry const& query, float max_distance = std::numeric_limits<float>::infinity(),
	    float                  epsilon    = 0.0f,
	    NearestSearchAlgorithm search_alg = NearestSearchAlgorithm::DEPTH_FIRST) const
	{
		float max_distance_sq = max_distance * max_distance;
		float dist_sq =
		    Base::nearest(
		        Base::index(), search_alg,
		        [this, &query](Index node) {
			        float dist_sq = std::numeric_limits<float>::infinity();
			        for (auto e : values(node)) {
				        dist_sq = UFO_MIN(dist_sq, distanceSquared(query, e.first));
			        }
			        return dist_sq;
		        },
		        [this, &query](Index node) { return distanceSquared(query, bounds(node)); },
		        max_distance_sq, epsilon * epsilon)
		        .first;

		return max_distance_sq <= dist_sq ? max_distance : std::sqrt(dist_sq);
	}

	template <class Geometry>
	[[nodiscard]] std::pair<value_type*, float> nearestPoint(
	    Geometry const& query, float max_distance = std::numeric_limits<float>::infinity(),
	    float                  epsilon    = 0.0f,
	    NearestSearchAlgorithm search_alg = NearestSearchAlgorithm::DEPTH_FIRST)
	{
		float max_distance_sq = max_distance * max_distance;
		auto [dist_sq, node]  = Base::nearest(
        Base::index(), search_alg,
        [this, &query](Index node) {
          float dist_sq = std::numeric_limits<float>::infinity();
          for (auto e : values(node)) {
            dist_sq = UFO_MIN(dist_sq, distanceSquared(query, e.first));
          }
          return dist_sq;
        },
        [this, &query](Index node) { return distanceSquared(query, bounds(node)); },
        max_distance_sq, epsilon * epsilon);

		value_type* value = nullptr;

		for (auto& e : values(node)) {
			value = distanceSquared(query, e.first) == dist_sq ? &e : nullptr;
		}

		return {value, max_distance_sq <= dist_sq ? max_distance : std::sqrt(dist_sq)};
	}

	template <class Geometry>
	[[nodiscard]] std::pair<value_type const*, float> nearestPoint(
	    Geometry const& query, float max_distance = std::numeric_limits<float>::infinity(),
	    float                  epsilon    = 0.0f,
	    NearestSearchAlgorithm search_alg = NearestSearchAlgorithm::DEPTH_FIRST) const
	{
		float max_distance_sq = max_distance * max_distance;
		auto [dist_sq, node]  = Base::nearest(
        Base::index(), search_alg,
        [this, &query](Index node) {
          float dist_sq = std::numeric_limits<float>::infinity();
          for (auto e : values(node)) {
            dist_sq = UFO_MIN(dist_sq, distanceSquared(query, e.first));
          }
          return dist_sq;
        },
        [this, &query](Index node) { return distanceSquared(query, bounds(node)); },
        max_distance_sq, epsilon * epsilon);

		value_type const* value = nullptr;

		for (auto const& e : values(node)) {
			value = distanceSquared(query, e.first) == dist_sq ? &e : nullptr;
		}

		return {value, max_distance_sq <= dist_sq ? max_distance : std::sqrt(dist_sq)};
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Bounds                                        |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Returns the minimum `Bounds` able to contain all values stored
	 * in the node and all of its children.
	 *
	 * @param node the node to check
	 * @return The minimum `Bounds` able to contain all values stored in the node.
	 */
	template <class NodeType,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Bounds& bounds(NodeType node)
	{
		TreeIndex n = Base::index(node);
		return Base::treeBlock(n).bounds[n.offset];
	}

	/*!
	 * @brief Returns the minimum `Bounds` able to contain all values stored
	 * in the node and all of its children.
	 *
	 * @param node the node to check
	 * @return The minimum `Bounds` able to contain all values stored in the node.
	 */
	template <class NodeType,
	          std::enable_if_t<Base::template is_node_type_v<NodeType>, bool> = true>
	[[nodiscard]] Bounds const& bounds(NodeType node) const
	{
		TreeIndex n = Base::index(node);
		return Base::treeBlock(n).bounds[n.offset];
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Compare                                        |
	|                                                                                     |
	**************************************************************************************/

	template <std::size_t D, class U>
	friend bool operator==(TreeMap<D, U> const& lhs, TreeMap<D, U> const& rhs);

	template <std::size_t D, class U>
	friend bool operator!=(TreeMap<D, U> const& lhs, TreeMap<D, U> const& rhs);

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                               Functions 'Tree" expect                               |
	|                                                                                     |
	**************************************************************************************/

	void onInitRoot() {}

	void onInitChildren(Index /* node */, pos_t /* children */) {}

	void onPruneChildren(Index /* node */, pos_t /* children */) {}

	/**************************************************************************************
	|                                                                                     |
	|                                      Capacity                                       |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Checks if the node and none of its children have any elements.
	 *
	 * @param node the node to check
	 * @return `true` if the node is empty, `false` otherwise.
	 */
	[[nodiscard]] bool empty(Index node) const noexcept
	{
		return boundsMin(node)[0] > boundsMax(node)[0];
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modifiers                                      |
	|                                                                                     |
	**************************************************************************************/

	void insert(Index node, value_type const& value)
	{
		Point p = value.first;
		values(node).push_back(value);
		++size_;

		insertPropagate(node, p);
	}

	void insert(Index node, value_type&& value)
	{
		Point p = value.first;
		values(node).push_back(std::move(value));
		++size_;

		insertPropagate(node, p);
	}

	void insertPropagate(Index node, Point p)
	{
		// Propagate
		auto root_depth = Base::depth();
		auto depth      = Base::depth(node);
		for (; root_depth > depth; ++depth) {
			Point& min = boundsMin(node);
			Point& max = boundsMax(node);
			min        = ufo::min(min, p);
			max        = ufo::max(max, p);
			node       = Base::parent(node);
		}

		// Root
		Point& min = boundsMin(node);
		Point& max = boundsMax(node);
		min        = ufo::min(min, p);
		max        = ufo::max(max, p);
	}

	void erase(typename container_type::const_iterator it)
	{
		auto node = Base::index(it->first);

		if (!Base::isPureLeaf(node)) {
			return;
		}

		auto& v = values(node);
		v.erase(it);
		--size_;

		erasePropagate(node);
	}

	void erasePropagate(Index node)
	{
		Point min(std::numeric_limits<typename Point::value_type>::max());
		Point max(std::numeric_limits<typename Point::value_type>::lowest());
		for (auto const& [p, _] : values(node)) {
			min = ufo::min(min, p);
			max = ufo::max(max, p);
		}
		boundsMin(node) = min;
		boundsMax(node) = max;

		if (Base::isRoot(node)) {
			return;
		}

		auto root_depth  = Base::depth();
		auto child_block = node.pos;
		node             = Base::parent(node);
		auto depth       = Base::depth(node);
		for (; root_depth > depth; ++depth) {
			Point min = boundsMin(Index(child_block, 0));
			Point max = boundsMax(Index(child_block, 0));
			for (std::size_t i = 1; Base::BF > i; ++i) {
				min = ufo::min(min, boundsMin(Index(child_block, i)));
				max = ufo::max(max, boundsMax(Index(child_block, i)));
			}
			boundsMin(node) = min;
			boundsMax(node) = max;
			child_block     = node.pos;
			node            = Base::parent(node);
		}

		// Root
		min = boundsMin(Index(child_block, 0));
		max = boundsMax(Index(child_block, 0));
		for (std::size_t i = 1; Base::BF > i; ++i) {
			min = ufo::min(min, boundsMin(Index(child_block, i)));
			max = ufo::max(max, boundsMax(Index(child_block, i)));
		}
		boundsMin(node) = min;
		boundsMax(node) = max;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                    Internal data                                    |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] auto& values(pos_t block) { return Base::treeBlock(block).values; }

	[[nodiscard]] auto const& values(pos_t block) const
	{
		return Base::treeBlock(block).values;
	}

	[[nodiscard]] auto& values(Index node)
	{
		return Base::treeBlock(node).values[node.offset];
	}

	[[nodiscard]] auto const& values(Index node) const
	{
		return Base::treeBlock(node).values[node.offset];
	}

	/*!
	 * @brief Returns the minimum point of the `Bounds` able to contain all values stored
	 * in the node and all of its children.
	 *
	 * @param node the node to return the minimum bounds point
	 * @return The minimum point of the `Bounds` able to contain all values stored in the
	 * node.
	 */
	[[nodiscard]] Point& boundsMin(Index node)
	{
		return Base::treeBlock(node).bounds[node.offset].min;
	}

	/*!
	 * @brief Returns the minimum point of the `Bounds` able to contain all values stored
	 * in the node and all of its children.
	 *
	 * @param node the node to return the minimum bounds point
	 * @return The minimum point of the `Bounds` able to contain all values stored in the
	 * node.
	 */
	[[nodiscard]] Point boundsMin(Index node) const
	{
		return Base::treeBlock(node).bounds[node.offset].min;
	}

	/*!
	 * @brief Returns the maximum point of the `Bounds` able to contain all values stored
	 * in the node and all of its children.
	 *
	 * @param node the node to return the maximum bounds point
	 * @return The maximum point of the `Bounds` able to contain all values stored in the
	 * node.
	 */
	[[nodiscard]] Point& boundsMax(Index node)
	{
		return Base::treeBlock(node).bounds[node.offset].max;
	}

	/*!
	 * @brief Returns the maximum point of the `Bounds` able to contain all values stored
	 * in the node and all of its children.
	 *
	 * @param node the node to return the maximum bounds point
	 * @return The maximum point of the `Bounds` able to contain all values stored in the
	 * node.
	 */
	[[nodiscard]] Point boundsMax(Index node) const
	{
		return Base::treeBlock(node).bounds[node.offset].max;
	}

 protected:
	size_type size_{};
};

//
// Compare
//

template <std::size_t Dim, class T>
bool operator==(TreeMap<Dim, T> const& lhs, TreeMap<Dim, T> const& rhs)
{
	using Base = typename TreeMap<Dim, T>::Base;

	Base const& lhs_b = static_cast<Base const&>(lhs);
	Base const& rhs_b = static_cast<Base const&>(rhs);

	return std::equal(lhs_b.begin(), lhs_b.end(), rhs_b.begin(), rhs_b.end(),
	                  [&lhs, &rhs](auto const& l, auto const& r) {
		                  auto        lc = l.code;
		                  auto        rc = r.code;
		                  auto const& lv = lhs.values(l.index);
		                  auto const& rv = rhs.values(r.index);

		                  return lc == rc && std::is_permutation(lv.begin(), lv.end(),
		                                                         rv.begin(), rv.end());
	                  });
}

template <std::size_t Dim, class T>
bool operator!=(TreeMap<Dim, T> const& lhs, TreeMap<Dim, T> const& rhs)
{
	return !(lhs == rhs);
}

template <class T>
using BinaryTreeMap = TreeMap<1, T>;

template <class T>
using QuadtreeMap = TreeMap<2, T>;

template <class T>
using OctreeMap = TreeMap<3, T>;

template <class T>
using HextreeMap = TreeMap<4, T>;
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_MAP_HPP