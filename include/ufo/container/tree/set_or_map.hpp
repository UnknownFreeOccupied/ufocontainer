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

#ifndef UFO_CONTAINER_TREE_SET_OR_MAP_HPP
#define UFO_CONTAINER_TREE_SET_OR_MAP_HPP

// UFO
#include <ufo/container/tree/predicate/spatial.hpp>
#include <ufo/container/tree/set_or_map_block.hpp>
#include <ufo/container/tree/set_or_map_iterator.hpp>
#include <ufo/container/tree/set_or_map_nearest_iterator.hpp>
#include <ufo/container/tree/set_or_map_query_iterator.hpp>
#include <ufo/container/tree/set_or_map_query_nearest_iterator.hpp>
#include <ufo/container/tree/tree.hpp>
#include <ufo/container/tree/type.hpp>
#include <ufo/utility/macros.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <algorithm>
#include <list>
#include <type_traits>
#include <vector>

namespace ufo
{
namespace detail
{
template <class T>
struct TreeSetOrMapBlockHelper {
	template <TreeType TT>
	using Block = TreeSetOrMapBlock<TT, T>;
};
}  // namespace detail

template <class Derived, template <class, template <TreeType> class> class TTree,
          class T = void>
class TreeSetOrMap
    : protected TTree<Derived, detail::TreeSetOrMapBlockHelper<T>::template Block>
{
	using Base = TTree<Derived, detail::TreeSetOrMapBlockHelper<T>::template Block>;

	static constexpr bool const IsMap = !std::is_void_v<T>;

	//
	// Friends
	//

	template <class Derived2, template <TreeType> class TreeBlock, TreeType TT>
	friend class Tree;

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
	using coord_t  = typename Base::coord_t;
	using depth_t  = typename Base::depth_t;
	using offset_t = typename Base::offset_t;
	using length_t = typename Base::length_t;
	using pos_t    = typename Base::pos_t;

	// STL stuff
	using value_type      = std::conditional_t<IsMap, std::pair<Point const, T>, Point>;
	using mapped_type     = T;
	using reference       = value_type&;
	using const_reference = value_type const&;
	using pointer         = value_type*;
	using const_pointer   = value_type const*;
	using size_type       = std::size_t;
	using difference_type = std::ptrdiff_t;

 public:
	// Iterators
	using iterator       = TreeSetOrMapIterator<TreeSetOrMap>;
	using const_iterator = TreeSetOrMapIterator<TreeSetOrMap const>;

	using query_iterator       = TreeSetOrMapQueryIterator<TreeSetOrMap>;
	using const_query_iterator = TreeSetOrMapQueryIterator<TreeSetOrMap const>;

	using nearest_iterator       = TreeSetOrMapNearestIterator<TreeSetOrMap>;
	using const_nearest_iterator = TreeSetOrMapNearestIterator<TreeSetOrMap const>;

	using query_nearest_iterator = TreeSetOrMapQueryNearestIterator<TreeSetOrMap>;
	using const_query_nearest_iterator =
	    TreeSetOrMapQueryNearestIterator<TreeSetOrMap const>;

	using Query      = IteratorWrapper<query_iterator>;
	using ConstQuery = IteratorWrapper<const_query_iterator>;

	using Nearest      = IteratorWrapper<nearest_iterator>;
	using ConstNearest = IteratorWrapper<const_nearest_iterator>;

	using QueryNearest      = IteratorWrapper<query_nearest_iterator>;
	using ConstQueryNearest = IteratorWrapper<const_query_nearest_iterator>;

	//
	// Friend iterators
	//

	friend iterator;
	friend const_iterator;

	friend query_iterator;
	friend const_query_iterator;
	template <class>
	friend class detail::TreeSetOrMapQueryIteratorHelper;
	template <class, class>
	friend class detail::TreeSetOrMapQueryIterator;

	friend nearest_iterator;
	friend const_nearest_iterator;

	friend query_nearest_iterator;
	friend const_query_nearest_iterator;
	template <class>
	friend class detail::TreeSetOrMapQueryNearestIteratorHelper;
	template <class, class>
	friend class detail::TreeSetOrMapQueryNearestIterator;

 private:
	using raw_iterator       = typename std::list<value_type>::iterator;
	using const_raw_iterator = typename std::list<value_type>::const_iterator;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	TreeSetOrMap(length_t leaf_node_length, depth_t num_depth_levels)
	    : Base(leaf_node_length, num_depth_levels)
	{
	}

	template <class InputIt>
	TreeSetOrMap(InputIt first, InputIt last, length_t leaf_node_length,
	             depth_t num_depth_levels)
	    : TreeSetOrMap(leaf_node_length, num_depth_levels)
	{
		insert(first, last);
	}

	TreeSetOrMap(std::initializer_list<value_type> init, length_t leaf_node_length,
	             depth_t num_depth_levels)
	    : TreeSetOrMap(std::begin(init), std::end(init), leaf_node_length, num_depth_levels)
	{
	}

	template <class Range>
	TreeSetOrMap(Range const& range, length_t leaf_node_length, depth_t num_depth_levels)
	    : TreeSetOrMap(std::begin(range), std::end(range), leaf_node_length,
	                   num_depth_levels)
	{
	}

	TreeSetOrMap(TreeSetOrMap const&) = default;

	TreeSetOrMap(TreeSetOrMap&&) = default;

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~TreeSetOrMap() {}

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	TreeSetOrMap& operator=(TreeSetOrMap const&) = default;

	TreeSetOrMap& operator=(TreeSetOrMap&&) = default;

	/**************************************************************************************
	|                                                                                     |
	|                                      Iterators                                      |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] iterator       begin() { return {this, Base::index()}; }
	[[nodiscard]] const_iterator begin() const { return {this, Base::index()}; }
	[[nodiscard]] const_iterator cbegin() const { return begin(); }

	[[nodiscard]] iterator       end() { return {}; }
	[[nodiscard]] const_iterator end() const { return {}; }
	[[nodiscard]] const_iterator cend() const { return end(); }

	/**************************************************************************************
	|                                                                                     |
	|                                   Query iterators                                   |
	|                                                                                     |
	**************************************************************************************/

	template <class Predicate>
	[[nodiscard]] query_iterator beginQuery(Predicate const& pred)
	{
		return {this, Base::index(), pred};
	}

	template <class Predicate>
	[[nodiscard]] const_query_iterator beginQuery(Predicate const& pred) const
	{
		return {this, Base::index(), pred};
	}

	template <class Predicate>
	[[nodiscard]] const_query_iterator cbeginQuery(Predicate const& pred) const
	{
		return beginQuery(pred);
	}

	[[nodiscard]] query_iterator       endQuery() { return {}; }
	[[nodiscard]] const_query_iterator endQuery() const { return {}; }
	[[nodiscard]] const_query_iterator cendQuery() const { return endQuery(); }

	/**************************************************************************************
	|                                                                                     |
	|                                  Nearest iterators                                  |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] nearest_iterator beginNearest(Point query, float epsilon = 0.0f)
	{
		return {this, Base::index(), query, epsilon};
	}

	[[nodiscard]] const_nearest_iterator beginNearest(Point query,
	                                                  float epsilon = 0.0f) const
	{
		return {this, Base::index(), query, epsilon};
	}

	[[nodiscard]] const_nearest_iterator cbeginNearest(Point query,
	                                                   float epsilon = 0.0f) const
	{
		return beginNearest(query, epsilon);
	}

	[[nodiscard]] nearest_iterator endNearest() { return {}; }

	[[nodiscard]] const_nearest_iterator endNearest() const { return {}; }

	[[nodiscard]] const_nearest_iterator cendNearest() const { return endNearest(); }

	/**************************************************************************************
	|                                                                                     |
	|                               Query nearest iterators                               |
	|                                                                                     |
	**************************************************************************************/

	template <class Predicate>
	[[nodiscard]] query_nearest_iterator beginQueryNearest(Point            query,
	                                                       Predicate const& pred,
	                                                       float            epsilon = 0.0f)
	{
		return {this, Base::index(), query, pred, epsilon};
	}

	template <class Predicate>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(Point            query,
	                                                             Predicate const& pred,
	                                                             float epsilon = 0.0f) const
	{
		return {this, Base::index(), query, pred, epsilon};
	}

	template <class Predicate>
	[[nodiscard]] const_query_nearest_iterator cbeginQueryNearest(
	    Point query, Predicate const& pred, float epsilon = 0.0f) const
	{
		return beginQueryNearest(query, pred, epsilon);
	}

	[[nodiscard]] query_nearest_iterator endQueryNearest() { return {}; }

	[[nodiscard]] const_query_nearest_iterator endQueryNearest() const { return {}; }

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
	[[nodiscard]] Bounds bounds() const
	{
		return {boundsMin(Base::index()), boundsMax(Base::index())};
	}

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

	void insert(value_type const& value)
	{
		Point point;
		if constexpr (IsMap) {
			point = value.first;
		} else {
			point = value;
		}
		Code code = Base::code(point);

		std::array<Index, Base::maxNumDepthLevels()> trail;
		int const                                    root_depth = Base::depth();
		trail[root_depth]                                       = Base::index();
		for (auto depth = root_depth; 0 < depth; --depth) {
			trail[depth - 1] = Base::createChild(trail[depth], code.offset(depth - 1));
		}

		values(trail[0]).push_back(value);
		++size_;

		// Propagate
		for (int i{}; root_depth >= i; ++i) {
			Point& min = boundsMin(trail[i]);
			Point& max = boundsMax(trail[i]);
			for (int j{}; Point::size() > j; ++j) {
				min[j] = UFO_MIN(min[j], point[j]);
				max[j] = UFO_MAX(max[j], point[j]);
			}
		}
	}

	void insert(value_type&& value)
	{
		Point point;
		if constexpr (IsMap) {
			point = value.first;
		} else {
			point = value;
		}
		Code code = Base::code(point);

		std::array<Index, Base::maxNumDepthLevels()> trail;
		int const                                    root_depth = Base::depth();
		trail[root_depth]                                       = Base::index();
		for (auto depth = root_depth; 0 < depth; --depth) {
			trail[depth - 1] = Base::createChild(trail[depth], code.offset(depth - 1));
		}

		values(trail[0]).push_back(std::move(value));
		++size_;

		// Propagate
		for (int i{}; root_depth >= i; ++i) {
			Point& min = boundsMin(trail[i]);
			Point& max = boundsMax(trail[i]);
			for (int j{}; Point::size() > j; ++j) {
				min[j] = UFO_MIN(min[j], point[j]);
				max[j] = UFO_MAX(max[j], point[j]);
			}
		}
	}

	template <class InputIt>
	void insert(InputIt first, InputIt last)
	{
		using non_const_value_type = std::conditional_t<IsMap, std::pair<Point, T>, Point>;

		std::vector<std::pair<Code, non_const_value_type>> code_and_value;
		code_and_value.reserve(std::distance(first, last));

		for (; last != first; ++first) {
			if constexpr (IsMap) {
				code_and_value.emplace_back(Base::code(first->first), *first);
			} else {
				code_and_value.emplace_back(Base::code(*first), *first);
			}
		}

		std::sort(std::begin(code_and_value), std::end(code_and_value),
		          [](auto const& a, auto const& b) { return a.first < b.first; });

		std::array<Index, Base::maxNumDepthLevels()> trail;
		int const                                    root_depth = Base::depth();
		trail[root_depth]                                       = Base::index();
		Code prev_code                                          = Base::code();
		int  depth                                              = root_depth;
		for (auto const& [code, value] : code_and_value) {
			// Propagate
			for (int const same_depth = Code::depthWhereEqual(prev_code, code);
			     same_depth > depth; ++depth) {
				Point  prev_min = boundsMin(trail[depth]);
				Point  prev_max = boundsMax(trail[depth]);
				Point& min      = boundsMin(trail[depth + 1]);
				Point& max      = boundsMax(trail[depth + 1]);
				for (int i{}; Point::size() > i; ++i) {
					min[i] = UFO_MIN(min[i], prev_min[i]);
					max[i] = UFO_MAX(max[i], prev_max[i]);
				}
			}

			prev_code = code;

			// Go down
			for (; 0 < depth; --depth) {
				trail[depth - 1] = Base::createChild(trail[depth], code.offset(depth - 1));
			}
			values(trail[0]).push_back(value);

			Point point;
			if constexpr (IsMap) {
				point = value.first;
			} else {
				point = value;
			}

			Point& min = boundsMin(trail[0]);
			Point& max = boundsMax(trail[0]);
			for (int i{}; Point::size() > i; ++i) {
				min[i] = UFO_MIN(min[i], point[i]);
				max[i] = UFO_MAX(max[i], point[i]);
			}
		}

		size_ += std::size(code_and_value);

		// Propagate
		for (; root_depth > depth; ++depth) {
			Point  prev_min = boundsMin(trail[depth]);
			Point  prev_max = boundsMax(trail[depth]);
			Point& min      = boundsMin(trail[depth + 1]);
			Point& max      = boundsMax(trail[depth + 1]);
			for (int i{}; Point::size() > i; ++i) {
				min[i] = UFO_MIN(min[i], prev_min[i]);
				max[i] = UFO_MAX(max[i], prev_max[i]);
			}
		}
	}

	void insert(std::initializer_list<value_type> ilist)
	{
		insert(std::begin(ilist), std::end(ilist));
	}

	template <class Range>
	void insert(Range const& range)
	{
		insert(std::begin(range), std::end(range));
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
		return {this, pos};
	}

	query_iterator erase(query_iterator pos)
	{
		auto it = pos.iterator();
		++pos;
		erase(it);
		return pos;
	}

	query_iterator erase(const_query_iterator pos)
	{
		auto it = pos.iterator();
		++pos;
		erase(it);
		return {this, pos};
	}

	nearest_iterator erase(nearest_iterator pos)
	{
		auto it = pos.iterator();
		++pos;
		erase(it);
		return pos;
	}

	nearest_iterator erase(const_nearest_iterator pos)
	{
		auto it = pos.iterator();
		++pos;
		erase(it);
		return {this, pos};
	}

	query_nearest_iterator erase(query_nearest_iterator pos)
	{
		auto it = pos.iterator();
		++pos;
		erase(it);
		return pos;
	}

	query_nearest_iterator erase(const_query_nearest_iterator pos)
	{
		auto it = pos.iterator();
		++pos;
		erase(it);
		return {this, pos};
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
		return {this, first};
	}

	query_iterator erase(query_iterator first, query_iterator last)
	{
		while (last != first) {
			auto it = first.iterator();
			++first;
			erase(it);
		}
		return first;
	}

	query_iterator erase(const_query_iterator first, const_query_iterator last)
	{
		while (last != first) {
			auto it = first.iterator();
			++first;
			erase(it);
		}
		return {this, first};
	}

	query_iterator erase(Query query) { return erase(std::begin(query), std::end(query)); }

	query_iterator erase(ConstQuery query)
	{
		return erase(std::begin(query), std::end(query));
	}

	nearest_iterator erase(nearest_iterator first, nearest_iterator last)
	{
		while (last != first) {
			auto it = first.iterator();
			++first;
			erase(it);
		}
		return first;
	}

	nearest_iterator erase(const_nearest_iterator first, const_nearest_iterator last)
	{
		while (last != first) {
			auto it = first.iterator();
			++first;
			erase(it);
		}
		return {this, first};
	}

	nearest_iterator erase(Nearest nearest)
	{
		return erase(std::begin(nearest), std::end(nearest));
	}

	nearest_iterator erase(ConstNearest nearest)
	{
		return erase(std::begin(nearest), std::end(nearest));
	}

	query_nearest_iterator erase(query_nearest_iterator first, query_nearest_iterator last)
	{
		while (last != first) {
			auto it = first.iterator();
			++first;
			erase(it);
		}
		return first;
	}

	query_nearest_iterator erase(const_query_nearest_iterator first,
	                             const_query_nearest_iterator last)
	{
		while (last != first) {
			auto it = first.iterator();
			++first;
			erase(it);
		}
		return {this, first};
	}

	query_nearest_iterator erase(QueryNearest query)
	{
		return erase(std::begin(query), std::end(query));
	}

	query_nearest_iterator erase(ConstQueryNearest query)
	{
		return erase(std::begin(query), std::end(query));
	}

	size_type erase(Point point)
	{
		Code code = Base::code(point);

		std::array<Index, Base::maxNumDepthLevels()> trail;
		int const                                    root_depth = Base::depth();
		trail[root_depth]                                       = Base::index();
		for (auto depth = root_depth; 0 < depth; --depth) {
			if (Base::isLeaf(trail[depth])) {
				return 0;
			}
			trail[depth - 1] = Base::child(trail[depth], code.offset(depth - 1));
		}

		auto&     v           = values(trail[0]);
		size_type num_removed = v.size();
		v.remove_if([point](auto const& x) {
			if constexpr (IsMap) {
				return equal(x.first, point);
			} else {
				return equal(x, point);
			}
		});
		num_removed -= v.size();

		size_ -= num_removed;

		Point min(std::numeric_limits<typename Point::value_type>::max());
		Point max(std::numeric_limits<typename Point::value_type>::lowest());
		if constexpr (IsMap) {
			for (auto const& [p, _] : v) {
				for (int i{}; Point::size() > i; ++i) {
					min[i] = UFO_MIN(min[i], p[i]);
					max[i] = UFO_MAX(max[i], p[i]);
				}
			}
		} else {
			for (Point p : v) {
				for (int i{}; Point::size() > i; ++i) {
					min[i] = UFO_MIN(min[i], p[i]);
					max[i] = UFO_MAX(max[i], p[i]);
				}
			}
		}
		boundsMin(trail[0]) = min;
		boundsMax(trail[0]) = max;

		// Propagate
		for (int i = 1; root_depth >= i; ++i) {
			auto  block = trail[i - 1].pos;
			Point min   = boundsMin(Index(block, 0));
			Point max   = boundsMax(Index(block, 0));
			for (int o = 1; Base::branchingFactor() > o; ++o) {
				Point t_min = boundsMin(Index(block, o));
				Point t_max = boundsMax(Index(block, o));
				for (int j{}; Point::size() > j; ++j) {
					min[j] = UFO_MIN(min[j], t_min[j]);
					max[j] = UFO_MAX(max[j], t_max[j]);
				}
			}
			boundsMin(trail[i]) = min;
			boundsMax(trail[i]) = max;
		}

		return num_removed;
	}

	/*!
	 * @brief Exchanges the contents of the container with those of `other`.
	 *
	 * @param other	container to exchange the contents with
	 */
	void swap(TreeSetOrMap& other)
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
		return std::count_if(std::begin(v), std::end(v), [point](auto const& x) {
			if constexpr (IsMap) {
				return equal(x.first, point);
			} else {
				return equal(x, point);
			}
		});
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
		return std::end(v) !=
		       std::find_if(std::begin(v), std::end(v), [point](auto const& x) {
			       if constexpr (IsMap) {
				       return equal(x.first, point);
			       } else {
				       return equal(x, point);
			       }
		       });
	}

	template <class Predicate>
	[[nodiscard]] Query query(Predicate const& pred)
	{
		return {beginQuery(pred), endQuery()};
	}

	template <class Predicate>
	[[nodiscard]] ConstQuery query(Predicate const& pred) const
	{
		return {beginQuery(pred), endQuery()};
	}

	[[nodiscard]] Nearest nearest(Point query, float epsilon = 0.0f)
	{
		return {beginNearest(query, epsilon), endNearest()};
	}

	[[nodiscard]] ConstNearest nearest(Point query, float epsilon = 0.0f) const
	{
		return {beginNearest(query, epsilon), endNearest()};
	}

	template <class Predicate>
	[[nodiscard]] QueryNearest queryNearest(Point query, Predicate const& pred,
	                                        float epsilon = 0.0f)
	{
		return {beginQueryNearest(query, pred, epsilon), endQueryNearest()};
	}

	template <class Predicate>
	[[nodiscard]] ConstQueryNearest queryNearest(Point query, Predicate const& pred,
	                                             float epsilon = 0.0f) const
	{
		return {beginQueryNearest(query, pred, epsilon), endQueryNearest()};
	}

 protected:
	/**************************************************************************************
	|                                                                                     |
	|                                       Derived                                       |
	|                                                                                     |
	**************************************************************************************/

	/*!
	 * @brief Returns a reference of the derived class.
	 *
	 * @return A reference of the derived class.
	 */
	[[nodiscard]] constexpr Derived& derived() { return *static_cast<Derived*>(this); }

	/*!
	 * @brief Returns a reference of the derived class.
	 *
	 * @return A reference of the derived class.
	 */
	[[nodiscard]] constexpr Derived const& derived() const
	{
		return *static_cast<Derived const*>(this);
	}

	/**************************************************************************************
	|                                                                                     |
	|                               Functions 'Tree" expect                               |
	|                                                                                     |
	**************************************************************************************/

	void derivedClear() {}

	void derivedCreateBlock(Index parent) {}

	void derivedFillBlock(Index parent, pos_t block) {}

	void derivedPruneBlock(Index parent, pos_t block) {}

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

	/*!
	 * @brief Returns the minimum `Bounds` able to contain all values stored
	 * in the node and all of its children.
	 *
	 * @param node the node to check
	 * @return The minimum `Bounds` able to contain all values stored in the node.
	 */
	[[nodiscard]] Bounds bounds(Index node) const
	{
		return {boundsMin(node), boundsMax(node)};
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Modifiers                                      |
	|                                                                                     |
	**************************************************************************************/

	void erase(const_raw_iterator it)
	{
		Code code;
		if constexpr (IsMap) {
			code = Base::code(it->first);
		} else {
			code = Base::code(*it);
		}

		std::array<Index, Base::maxNumDepthLevels()> trail;
		int const                                    root_depth = Base::depth();
		trail[root_depth]                                       = Base::index();
		for (auto depth = root_depth; 0 < depth; --depth) {
			trail[depth - 1] = Base::child(trail[depth], code.offset(depth - 1));
		}

		auto& v = values(trail[0]);
		v.erase(it);
		--size_;

		Point min(std::numeric_limits<typename Point::value_type>::max());
		Point max(std::numeric_limits<typename Point::value_type>::lowest());
		if constexpr (IsMap) {
			for (auto const& [p, _] : v) {
				for (int i{}; Point::size() > i; ++i) {
					min[i] = UFO_MIN(min[i], p[i]);
					max[i] = UFO_MAX(max[i], p[i]);
				}
			}
		} else {
			for (Point p : v) {
				for (int i{}; Point::size() > i; ++i) {
					min[i] = UFO_MIN(min[i], p[i]);
					max[i] = UFO_MAX(max[i], p[i]);
				}
			}
		}
		boundsMin(trail[0]) = min;
		boundsMax(trail[0]) = max;

		// Propagate
		for (int i = 1; root_depth >= i; ++i) {
			auto  block = trail[i - 1].pos;
			Point min   = boundsMin(Index(block, 0));
			Point max   = boundsMax(Index(block, 0));
			for (int o = 1; Base::branchingFactor() > o; ++o) {
				Point t_min = boundsMin(Index(block, o));
				Point t_max = boundsMax(Index(block, o));
				for (int j{}; Point::size() > j; ++j) {
					min[j] = UFO_MIN(min[j], t_min[j]);
					max[j] = UFO_MAX(max[j], t_max[j]);
				}
			}
			boundsMin(trail[i]) = min;
			boundsMax(trail[i]) = max;
		}
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

	[[nodiscard]] auto& boundsMin(pos_t block) { return Base::treeBlock(block).min; }

	[[nodiscard]] auto const& boundsMin(pos_t block) const
	{
		return Base::treeBlock(block).min;
	}

	[[nodiscard]] Point& boundsMin(Index node)
	{
		return Base::treeBlock(node).min[node.offset];
	}

	[[nodiscard]] Point boundsMin(Index node) const
	{
		return Base::treeBlock(node).min[node.offset];
	}

	[[nodiscard]] auto& boundsMax(pos_t block) { return Base::treeBlock(block).max; }

	[[nodiscard]] auto const& boundsMax(pos_t block) const
	{
		return Base::treeBlock(block).max;
	}

	[[nodiscard]] Point& boundsMax(Index node)
	{
		return Base::treeBlock(node).max[node.offset];
	}

	[[nodiscard]] Point boundsMax(Index node) const
	{
		return Base::treeBlock(node).max[node.offset];
	}

 protected:
	size_type size_{};
};

/**************************************************************************************
|                                                                                     |
|                                       Compare                                       |
|                                                                                     |
**************************************************************************************/

// template <class Derived, template <class, template <TreeType> class> class TTree,
//           class T = void>
// [[nodiscard]] constexpr Vec<1, bool> operator==(
//     TreeSetOrMap<Derived, TTree, T> const& lhs,
//     TreeSetOrMap<Derived, TTree, T> const& rhs) noexcept
// {
// 	// TODO: Implement
// }

// template <class Derived, template <class, template <TreeType> class> class TTree,
//           class T = void>
// [[nodiscard]] constexpr Vec<1, bool> operator!=(
//     TreeSetOrMap<Derived, TTree, T> const& lhs,
//     TreeSetOrMap<Derived, TTree, T> const& rhs) noexcept
// {
// 	return {lhs.x != rhs.x};
// }
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_SET_OR_MAP_HPP

// [[nodiscard]] reference nearestSingle(Index node, Point p, float epsilon = 0.0f)
// 	{
// 		auto root = Base::index();
// 		if (Base::isLeaf(root)) {
// 			return max_distance;
// 		}

// 		auto children       = Base::children(root);
// 		auto children_depth = Base::depth() - 1;

// 		return std::sqrt(0.0f < epsilon
// 		                     ? nearestSingle(p, children, children_depth, epsilon * epsilon)
// 		                     : nearestSingle(p, children, children_depth));
// 	}

// 	[[nodiscard]] const_reference nearestSingle(Index node, Point p,
// 	                                            float epsilon = 0.0f) const
// 	{
// 		// TODO: Implement
// 	}

// 	[[nodiscard]] reference nearestSingle(Point query_point, pos_t block,
// 	                                      depth_t block_depth, float epsilon)
// 	{
// 		static constexpr auto const BF = Base::branchingFactor();

// 		float closest_distance_sq = std::numeric_limits<float>::max();

// 		std::array<std::pair<int, std::array<std::pair<float, pos_t>, BF>>,
// 		           Base::maxNumDepthLevels() - 1>
// 		    stack;
// 		stack[block_depth].first                 = BF - 1;
// 		stack[block_depth].second[BF - 1].first  = 0.0f;
// 		stack[block_depth].second[BF - 1].second = block;

// 		for (depth_t max_depth = block_depth + static_cast<depth_t>(1);
// 		     max_depth > block_depth;) {
// 			auto& [idx, c] = stack[block_depth];

// 			if (BF - 1 < idx || c[idx].first + epsilon >= closest_distance_sq) {
// 				++block_depth;
// 				continue;
// 			}

// 			block = c[idx].second;
// 			++idx;

// 			stack[block_depth - static_cast<depth_t>(1)].first = 0;
// 			auto& candidates = stack[block_depth - static_cast<depth_t>(1)].second;

// 			auto child_block = Base::children(block);

// 			parents_checked2 += BF;
// 			for (int i{}; BF > i; ++i) {
// 				Point coord;
// 				for (int j{}; Point::size() > j; ++j) {
// 					coord[j] =
// 					    UFO_CLAMP(query_point[j], extra_[block].min[i][j],
// extra_[block].max[i][j]);
// 				}

// 				for (int j{}; Point::size() > j; ++j) {
// 					coord[j] -= query_point[j];
// 					coord[j] *= coord[j];
// 				}

// 				candidates[i].first = coord[0];
// 				for (int j = 1; Point::size() > j; ++j) {
// 					candidates[i].first += coord[j];
// 				}
// 				candidates[i].second = child_block[i];
// 			}

// 			if (static_cast<depth_t>(1) == block_depth) {
// 				std::array<float, BF> d;
// 				for (auto [dist_sq, child_block] : candidates) {
// 					if (dist_sq + epsilon >= closest_distance_sq) {
// 						continue;
// 					}

// 					leaves_checked2 += BF;
// 					for (int i{}; BF > i; ++i) {
// 						auto p = extra_[child_block].point[i];
// 						for (int j{}; Point::size() > j; ++j) {
// 							p[j] -= query_point[j];
// 							p[j] *= p[j];
// 						}
// 						d[i] = p[0];
// 						for (int j = 1; Point::size() > j; ++j) {
// 							d[i] += p[j];
// 						}
// 					}
// 					if constexpr (2 == BF) {
// 						UFO_MIN_2(d);
// 					} else if constexpr (4 == BF) {
// 						UFO_MIN_4(d);
// 					} else if constexpr (8 == BF) {
// 						UFO_MIN_8(d);
// 					} else if constexpr (16 == BF) {
// 						UFO_MIN_16(d);
// 					} else {
// 						for (int i = 1; BF > i; ++i) {
// 							d[0] = UFO_MIN(d[0], d[i]);
// 						}
// 					}
// 					closest_distance_sq = UFO_MIN(closest_distance_sq, d[0]);
// 				}
// 			} else {
// 				if constexpr (2 == BF) {
// 					UFO_SORT_PAIR_FIRST_2(candidates);
// 				} else if constexpr (4 == BF) {
// 					UFO_SORT_PAIR_FIRST_4(candidates);
// 				} else if constexpr (8 == BF) {
// 					UFO_SORT_PAIR_FIRST_8(candidates);
// 				} else if constexpr (16 == BF) {
// 					UFO_SORT_PAIR_FIRST_16(candidates);
// 				} else {
// 					std::sort(std::begin(candidates), std::end(candidates),
// 					          [](auto a, auto b) { return a.first < b.first; });
// 				}
// 				--block_depth;
// 			}
// 		}

// 		return closest_distance_sq;
// 	}