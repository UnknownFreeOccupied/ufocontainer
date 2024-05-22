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

#ifndef UFO_CONTAINER_TREE_TREE_MAP_HPP
#define UFO_CONTAINER_TREE_TREE_MAP_HPP

// UFO
#include <ufo/container/tree/tree.hpp>
#include <ufo/container/tree/tree_container.hpp>
#include <ufo/container/tree/tree_map_block.hpp>
#include <ufo/container/tree/tree_map_iterator.hpp>
#include <ufo/container/tree/tree_types.hpp>
#include <ufo/utility/iterator_wrapper.hpp>

// STL
#include <algorithm>
#include <array>
#include <iterator>
#include <utility>

namespace ufo
{
namespace detail
{
template <class T>
struct TreeMapBlockHelper {
	template <TreeType TT>
	using TMB = TreeMapBlock<T, TT>;
};
}  // namespace detail

template <template <class, template <TreeType> class> class TBase, class T>
class TreeMap
    : public TBase<TreeMap<TBase, T>, detail::TreeMapBlockHelper<T>::template TMB>
{
	using Base = TBase<TreeMap<TBase, T>, detail::TreeMapBlockHelper<T>::template TMB>;

	//
	// Friends
	//

	friend Base;

	friend class TreeMapIterator<TreeMap>;
	friend class TreeMapIterator<TreeMap const>;

	template <class TM, class Predicate>
	friend class TreeMapQueryIterator;

	template <class TM, class Predicate>
	friend class TreeMapNearestIterator;

 public:
	//
	// Tags
	//

	using Index    = typename Base::Index;
	using Node     = typename Base::Node;
	using Code     = typename Base::Code;
	using Key      = typename Base::Key;
	using Point    = typename Base::Point;
	using Coord    = typename Base::Coord;
	using coord_t  = typename Base::coord_t;
	using depth_t  = typename Base::depth_t;
	using offset_t = typename Base::offset_t;
	using length_t = typename Base::length_t;

	using key_type    = Point;
	using mapped_type = T;
	using value_type  = std::pair<Point const, T>;

	using reference       = value_type&;
	using const_reference = value_type const&;
	using pointer         = value_type*;
	using const_pointer   = value_type const*;
	using size_type       = std::size_t;
	using difference_type = std::ptrdiff_t;
	using iterator        = TreeMapIterator<TreeMap>;
	using const_iterator  = TreeMapIterator<TreeMap const>;

	using query_iterator       = TreeMapIteratorWrapper<TreeMap>;
	using const_query_iterator = TreeMapIteratorWrapper<TreeMap const>;

	using nearest_iterator       = TreeMapNearestIterator<TreeMap, pred::True>;
	using const_nearest_iterator = TreeMapNearestIterator<TreeMap const, pred::True>;

	using query_nearest_iterator       = TreeMapIteratorWrapper<TreeMap>;
	using const_query_nearest_iterator = TreeMapIteratorWrapper<TreeMap const>;

	using iterator_wrapper       = IteratorWrapper<iterator>;
	using const_iterator_wrapper = IteratorWrapper<const_iterator>;

	using Query             = IteratorWrapper<query_iterator>;
	using ConstQuery        = IteratorWrapper<const_query_iterator>;
	using Nearest           = IteratorWrapper<nearest_iterator>;
	using ConstNearest      = IteratorWrapper<const_nearest_iterator>;
	using QueryNearest      = IteratorWrapper<query_nearest_iterator>;
	using ConstQueryNearest = IteratorWrapper<const_query_nearest_iterator>;

	// using query_iterator = ...;
	// using const_query_iterator = ...;

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	TreeMap(length_t leaf_node_length = static_cast<length_t>(0.1),
	        depth_t  num_depth_levels = std::max(static_cast<depth_t>(17),
	                                             Base::maxNumDepthLevels()))
	    : Base(leaf_node_length, num_depth_levels)
	{
	}

	template <class InputIt>
	TreeMap(length_t leaf_node_length, depth_t num_depth_levels, InputIt first,
	        InputIt last)
	    : TreeMap(leaf_node_length, num_depth_levels)
	{
		insert(first, last);
	}

	TreeMap(length_t leaf_node_length, depth_t num_depth_levels,
	        std::initializer_list<value_type> init)
	    : TreeMap(leaf_node_length, num_depth_levels, std::begin(init), std::end(init))
	{
	}

	TreeMap(TreeMap const& other) = default;

	TreeMap(TreeMap&& other) = default;

	/**************************************************************************************
	|                                                                                     |
	|                                     Destructor                                      |
	|                                                                                     |
	**************************************************************************************/

	~TreeMap() {}

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	TreeMap& operator=(TreeMap const& other) = default;

	TreeMap& operator=(TreeMap&& other) = default;

	/**************************************************************************************
	|                                                                                     |
	|                                        Swap                                         |
	|                                                                                     |
	**************************************************************************************/

	friend void swap(TreeMap& lhs, TreeMap& rhs)
	{
		std::swap(static_cast<Base&>(lhs), static_cast<Base&>(rhs));
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Iterators                                      |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] iterator       begin() noexcept { return begin(Base::index()); }
	[[nodiscard]] const_iterator begin() const noexcept { return begin(Base::index()); }
	[[nodiscard]] const_iterator cbegin() const noexcept { return begin(); }

	[[nodiscard]] iterator       begin(Index node) { return {this, node}; }
	[[nodiscard]] const_iterator begin(Index node) const { return {this, node}; }
	[[nodiscard]] const_iterator cbegin(Index node) const { return begin(node); }

	[[nodiscard]] iterator       begin(Node node) { return begin(Base::index(node)); }
	[[nodiscard]] const_iterator begin(Node node) const { return begin(Base::index(node)); }
	[[nodiscard]] const_iterator cbegin(Node node) const { return begin(node); }

	[[nodiscard]] iterator       begin(Code node) { return begin(Base::index(node)); }
	[[nodiscard]] const_iterator begin(Code node) const { return begin(Base::index(node)); }
	[[nodiscard]] const_iterator cbegin(Code node) const { return begin(node); }

	[[nodiscard]] iterator       begin(Key node) { return begin(Base::index(node)); }
	[[nodiscard]] const_iterator begin(Key node) const { return begin(Base::index(node)); }
	[[nodiscard]] const_iterator cbegin(Key node) const { return begin(node); }

	[[nodiscard]] iterator       begin(Coord node) { return begin(Base::index(node)); }
	[[nodiscard]] const_iterator begin(Coord node) const
	{
		return begin(Base::index(node));
	}
	[[nodiscard]] const_iterator cbegin(Coord node) const { return begin(node); }

	[[nodiscard]] iterator       end() noexcept { return {}; }
	[[nodiscard]] const_iterator end() const noexcept { return {}; }
	[[nodiscard]] const_iterator cend() const noexcept { return end(); }

	/**************************************************************************************
	|                                                                                     |
	|                                   Query iterators                                   |
	|                                                                                     |
	**************************************************************************************/

	template <class Predicate>
	[[nodiscard]] query_iterator beginQuery(Predicate const& pred)
	{
		return beginQuery(Base::index(), pred);
	}

	template <class Predicate>
	[[nodiscard]] const_query_iterator beginQuery(Predicate const& pred) const
	{
		return beginQuery(Base::index(), pred);
	}

	template <class Predicate>
	[[nodiscard]] const_query_iterator cbeginQuery(Predicate const& pred) const
	{
		return beginQuery(pred);
	}

	template <class Predicate>
	[[nodiscard]] query_iterator beginQuery(Index node, Predicate const& pred)
	{
		return {this, node, pred};
	}

	template <class Predicate>
	[[nodiscard]] const_query_iterator beginQuery(Index node, Predicate const& pred) const
	{
		return {this, node, pred};
	}

	template <class Predicate>
	[[nodiscard]] const_query_iterator cbeginQuery(Index node, Predicate const& pred) const
	{
		return beginQuery(node, pred);
	}

	template <class Predicate>
	[[nodiscard]] query_iterator beginQuery(Node node, Predicate const& pred)
	{
		return beginQuery(Base::index(node), pred);
	}

	template <class Predicate>
	[[nodiscard]] const_query_iterator beginQuery(Node node, Predicate const& pred) const
	{
		return beginQuery(Base::index(node), pred);
	}

	template <class Predicate>
	[[nodiscard]] const_query_iterator cbeginQuery(Node node, Predicate const& pred) const
	{
		return beginQuery(node, pred);
	}

	template <class Predicate>
	[[nodiscard]] query_iterator beginQuery(Code node, Predicate const& pred)
	{
		return beginQuery(Base::index(node), pred);
	}

	template <class Predicate>
	[[nodiscard]] const_query_iterator beginQuery(Code node, Predicate const& pred) const
	{
		return beginQuery(Base::index(node), pred);
	}

	template <class Predicate>
	[[nodiscard]] const_query_iterator cbeginQuery(Code node, Predicate const& pred) const
	{
		return beginQuery(node, pred);
	}

	template <class Predicate>
	[[nodiscard]] query_iterator beginQuery(Key node, Predicate const& pred)
	{
		return beginQuery(Base::index(node), pred);
	}

	template <class Predicate>
	[[nodiscard]] const_query_iterator beginQuery(Key node, Predicate const& pred) const
	{
		return beginQuery(Base::index(node), pred);
	}

	template <class Predicate>
	[[nodiscard]] const_query_iterator cbeginQuery(Key node, Predicate const& pred) const
	{
		return beginQuery(node, pred);
	}

	template <class Predicate>
	[[nodiscard]] query_iterator beginQuery(Coord node, Predicate const& pred)
	{
		return beginQuery(Base::index(node), pred);
	}

	template <class Predicate>
	[[nodiscard]] const_query_iterator beginQuery(Coord node, Predicate const& pred) const
	{
		return beginQuery(Base::index(node), pred);
	}

	template <class Predicate>
	[[nodiscard]] const_query_iterator cbeginQuery(Coord node, Predicate const& pred) const
	{
		return beginQuery(node, pred);
	}

	[[nodiscard]] query_iterator       endQuery() { return {}; }
	[[nodiscard]] const_query_iterator endQuery() const { return {}; }
	[[nodiscard]] const_query_iterator cendQuery() const { return endQuery(); }

	/**************************************************************************************
	|                                                                                     |
	|                                        Query                                        |
	|                                                                                     |
	**************************************************************************************/

	template <class Predicate>
	[[nodiscard]] Query query(Predicate const& predicate)
	{
		return query(Base::index(), predicate);
	}

	template <class Predicate>
	[[nodiscard]] ConstQuery query(Predicate const& predicate) const
	{
		return query(Base::index(), predicate);
	}

	template <class Predicate>
	[[nodiscard]] Query query(Index node, Predicate const& predicate)
	{
		return {beginQuery(node, predicate), endQuery()};
	}

	template <class Predicate>
	[[nodiscard]] ConstQuery query(Index node, Predicate const& predicate) const
	{
		return {beginQuery(node, predicate), endQuery()};
	}

	template <class Predicate>
	[[nodiscard]] Query query(Node node, Predicate const& predicate)
	{
		return query(Base::index(node), predicate);
	}

	template <class Predicate>
	[[nodiscard]] ConstQuery query(Node node, Predicate const& predicate) const
	{
		return query(Base::index(node), predicate);
	}

	template <class Predicate>
	[[nodiscard]] Query query(Code node, Predicate const& predicate)
	{
		return query(Base::index(node), predicate);
	}

	template <class Predicate>
	[[nodiscard]] ConstQuery query(Code node, Predicate const& predicate) const
	{
		return query(Base::index(node), predicate);
	}

	template <class Predicate>
	[[nodiscard]] Query query(Key node, Predicate const& predicate)
	{
		return query(Base::index(node), predicate);
	}

	template <class Predicate>
	[[nodiscard]] ConstQuery query(Key node, Predicate const& predicate) const
	{
		return query(Base::index(node), predicate);
	}

	template <class Predicate>
	[[nodiscard]] Query query(Coord node, Predicate const& predicate)
	{
		return query(Base::index(node), predicate);
	}

	template <class Predicate>
	[[nodiscard]] ConstQuery query(Coord node, Predicate const& predicate) const
	{
		return query(Base::index(node), predicate);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                  Nearest iterators                                  |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] nearest_iterator beginNearest(Point query, float epsilon = 0.0f)
	{
		return beginNearest(Base::index(), query, epsilon);
	}

	[[nodiscard]] const_nearest_iterator beginNearest(Point query,
	                                                  float epsilon = 0.0f) const
	{
		return beginNearest(Base::index(), query, epsilon);
	}

	[[nodiscard]] const_nearest_iterator cbeginNearest(Point query,
	                                                   float epsilon = 0.0f) const
	{
		return beginNearest(query, epsilon);
	}

	[[nodiscard]] nearest_iterator beginNearest(Index node, Point query,
	                                            float epsilon = 0.0f)
	{
		// TODO: Implement
	}

	[[nodiscard]] const_nearest_iterator beginNearest(Index node, Point query,
	                                                  float epsilon = 0.0f) const
	{
		// TODO: Implement
	}

	[[nodiscard]] const_nearest_iterator cbeginNearest(Index node, Point query,
	                                                   float epsilon = 0.0f) const
	{
		return beginNearest(node, query, epsilon);
	}

	[[nodiscard]] nearest_iterator beginNearest(Node node, Point query,
	                                            float epsilon = 0.0f)
	{
		return beginNearest(Base::index(node), query, epsilon);
	}

	[[nodiscard]] const_nearest_iterator beginNearest(Node node, Point query,
	                                                  float epsilon = 0.0f) const
	{
		return beginNearest(Base::index(node), query, epsilon);
	}

	[[nodiscard]] const_nearest_iterator cbeginNearest(Node node, Point query,
	                                                   float epsilon = 0.0f) const
	{
		return beginNearest(node, query, epsilon);
	}

	[[nodiscard]] nearest_iterator beginNearest(Code node, Point query,
	                                            float epsilon = 0.0f)
	{
		return beginNearest(Base::index(node), query, epsilon);
	}

	[[nodiscard]] const_nearest_iterator beginNearest(Code node, Point query,
	                                                  float epsilon = 0.0f) const
	{
		return beginNearest(Base::index(node), query, epsilon);
	}

	[[nodiscard]] const_nearest_iterator cbeginNearest(Code node, Point query,
	                                                   float epsilon = 0.0f) const
	{
		return beginNearest(node, query, epsilon);
	}

	[[nodiscard]] nearest_iterator beginNearest(Key node, Point query, float epsilon = 0.0f)
	{
		return beginNearest(Base::index(node), query, epsilon);
	}

	[[nodiscard]] const_nearest_iterator beginNearest(Key node, Point query,
	                                                  float epsilon = 0.0f) const
	{
		return beginNearest(Base::index(node), query, epsilon);
	}

	[[nodiscard]] const_nearest_iterator cbeginNearest(Key node, Point query,
	                                                   float epsilon = 0.0f) const
	{
		return beginNearest(node, query, epsilon);
	}

	[[nodiscard]] nearest_iterator beginNearest(Coord node, Point query,
	                                            float epsilon = 0.0f)
	{
		return beginNearest(Base::index(node), query, epsilon);
	}

	[[nodiscard]] const_nearest_iterator beginNearest(Coord node, Point query,
	                                                  float epsilon = 0.0f) const
	{
		return beginNearest(Base::index(node), query, epsilon);
	}

	[[nodiscard]] const_nearest_iterator cbeginNearest(Coord node, Point query,
	                                                   float epsilon = 0.0f) const
	{
		return beginNearest(node, query, epsilon);
	}

	[[nodiscard]] nearest_iterator endNearest()
	{
		// TODO: Implement
	}

	[[nodiscard]] const_nearest_iterator endNearest() const
	{
		// TODO: Implement
	}

	[[nodiscard]] const_nearest_iterator cendNearest() const { return endNearest(); }

	/**************************************************************************************
	|                                                                                     |
	|                                       Nearest                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] Nearest nearest(Point query, float epsilon = 0.0f)
	{
		return nearest(Base::index(), query, epsilon);
	}

	[[nodiscard]] ConstNearest nearest(Point query, float epsilon = 0.0f) const
	{
		return nearest(Base::index(), query, epsilon);
	}

	[[nodiscard]] Nearest nearest(Index node, Point query, float epsilon = 0.0f)
	{
		return {beginNearest(node, query, epsilon), endNearest()};
	}

	[[nodiscard]] ConstNearest nearest(Index node, Point query, float epsilon = 0.0f) const
	{
		return {beginNearest(node, query, epsilon), endNearest()};
	}

	[[nodiscard]] Nearest nearest(Node node, Point query, float epsilon = 0.0f)
	{
		return nearest(Base::index(node), query, epsilon);
	}

	[[nodiscard]] ConstNearest nearest(Node node, Point query, float epsilon = 0.0f) const
	{
		return nearest(Base::index(node), query, epsilon);
	}

	[[nodiscard]] Nearest nearest(Code node, Point query, float epsilon = 0.0f)
	{
		return nearest(Base::index(node), query, epsilon);
	}

	[[nodiscard]] ConstNearest nearest(Code node, Point query, float epsilon = 0.0f) const
	{
		return nearest(Base::index(node), query, epsilon);
	}

	[[nodiscard]] Nearest nearest(Key node, Point query, float epsilon = 0.0f)
	{
		return nearest(Base::index(node), query, epsilon);
	}

	/**************************************************************************************
	|                                                                                     |
	|                               Query nearest iterators                               |
	|                                                                                     |
	**************************************************************************************/

	template <class Predicate>
	[[nodiscard]] query_nearest_iterator beginQueryNearest(Point            query,
	                                                       Predicate const& predicate,
	                                                       float            epsilon = 0.0f)
	{
		return beginQueryNearest(Base::index(), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(Point            query,
	                                                             Predicate const& predicate,
	                                                             float epsilon = 0.0f) const
	{
		return beginQueryNearest(Base::index(), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] const_query_nearest_iterator cbeginQueryNearest(
	    Point query, Predicate const& predicate, float epsilon = 0.0f) const
	{
		return beginQueryNearest(query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] query_nearest_iterator beginQueryNearest(Index node, Point query,
	                                                       Predicate const& predicate,
	                                                       float            epsilon = 0.0f)
	{
		// TODO: Implement
	}

	template <class Predicate>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(Index node, Point query,
	                                                             Predicate const& predicate,
	                                                             float epsilon = 0.0f) const
	{
		// TODO: Implement
	}

	template <class Predicate>
	[[nodiscard]] const_query_nearest_iterator cbeginQueryNearest(
	    Index node, Point query, Predicate const& predicate, float epsilon = 0.0f) const
	{
		return beginQueryNearest(node, query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] query_nearest_iterator beginQueryNearest(Node node, Point query,
	                                                       Predicate const& predicate,
	                                                       float            epsilon = 0.0f)
	{
		return beginQueryNearest(Base::index(node), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(Node node, Point query,
	                                                             Predicate const& predicate,
	                                                             float epsilon = 0.0f) const
	{
		return beginQueryNearest(Base::index(node), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] const_query_nearest_iterator cbeginQueryNearest(
	    Node node, Point query, Predicate const& predicate, float epsilon = 0.0f) const
	{
		return beginQueryNearest(node, query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] query_nearest_iterator beginQueryNearest(Code node, Point query,
	                                                       Predicate const& predicate,
	                                                       float            epsilon = 0.0f)
	{
		return beginQueryNearest(Base::index(node), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(Code node, Point query,
	                                                             Predicate const& predicate,
	                                                             float epsilon = 0.0f) const
	{
		return beginQueryNearest(Base::index(node), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] const_query_nearest_iterator cbeginQueryNearest(
	    Code node, Point query, Predicate const& predicate, float epsilon = 0.0f) const
	{
		return beginQueryNearest(node, query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] query_nearest_iterator beginQueryNearest(Key node, Point query,
	                                                       Predicate const& predicate,
	                                                       float            epsilon = 0.0f)
	{
		return beginQueryNearest(Base::index(node), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(Key node, Point query,
	                                                             Predicate const& predicate,
	                                                             float epsilon = 0.0f) const
	{
		return beginQueryNearest(Base::index(node), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] const_query_nearest_iterator cbeginQueryNearest(
	    Key node, Point query, Predicate const& predicate, float epsilon = 0.0f) const
	{
		return beginQueryNearest(node, query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] query_nearest_iterator beginQueryNearest(Coord node, Point query,
	                                                       Predicate const& predicate,
	                                                       float            epsilon = 0.0f)
	{
		return beginQueryNearest(Base::index(node), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] const_query_nearest_iterator beginQueryNearest(Coord node, Point query,
	                                                             Predicate const& predicate,
	                                                             float epsilon = 0.0f) const
	{
		return beginQueryNearest(Base::index(node), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] const_query_nearest_iterator cbeginQueryNearest(
	    Coord node, Point query, Predicate const& predicate, float epsilon = 0.0f) const
	{
		return beginQueryNearest(node, query, predicate, epsilon);
	}

	[[nodiscard]] query_nearest_iterator endQueryNearest()
	{
		// TODO: Implement
	}

	[[nodiscard]] const_query_nearest_iterator endQueryNearest() const
	{
		// TODO: Implement
	}

	[[nodiscard]] const_query_nearest_iterator cendQueryNearest() const
	{
		return endQueryNearest();
	}

	/**************************************************************************************
	|                                                                                     |
	|                                    Query nearest                                    |
	|                                                                                     |
	**************************************************************************************/

	template <class Predicate>
	[[nodiscard]] ConstQueryNearest queryNearest(Key node, Point query,
	                                             float epsilon = 0.0f) const
	{
		return queryNearest(Base::index(node), query, epsilon);
	}

	[[nodiscard]] QueryNearest queryNearest(Coord node, Point query, float epsilon = 0.0f)
	{
		return queryNearest(Base::index(node), query, epsilon);
	}

	[[nodiscard]] ConstQueryNearest queryNearest(Coord node, Point query,
	                                             float epsilon = 0.0f) const
	{
		return queryNearest(Base::index(node), query, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] QueryNearest queryNearest(Point query, Predicate const& predicate,
	                                        float epsilon = 0.0f)
	{
		return queryNearest(Base::index(), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] ConstQueryNearest queryNearest(Point query, Predicate const& predicate,
	                                             float epsilon = 0.0f) const
	{
		return queryNearest(Base::index(), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] QueryNearest queryNearest(Index node, Point query,
	                                        Predicate const& predicate,
	                                        float            epsilon = 0.0f)
	{
		return {beginQueryNearest(node, query, predicate, epsilon), endQueryNearest()};
	}

	template <class Predicate>
	[[nodiscard]] ConstQueryNearest queryNearest(Index node, Point query,
	                                             Predicate const& predicate,
	                                             float            epsilon = 0.0f) const
	{
		return {beginQueryNearest(node, query, predicate, epsilon), endQueryNearest()};
	}

	template <class Predicate>
	[[nodiscard]] QueryNearest queryNearest(Node node, Point query,
	                                        Predicate const& predicate,
	                                        float            epsilon = 0.0f)
	{
		return queryNearest(Base::index(node), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] ConstQueryNearest queryNearest(Node node, Point query,
	                                             Predicate const& predicate,
	                                             float            epsilon = 0.0f) const
	{
		return queryNearest(Base::index(node), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] QueryNearest queryNearest(Code node, Point query,
	                                        Predicate const& predicate,
	                                        float            epsilon = 0.0f)
	{
		return queryNearest(Base::index(node), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] ConstQueryNearest queryNearest(Code node, Point query,
	                                             Predicate const& predicate,
	                                             float            epsilon = 0.0f) const
	{
		return queryNearest(Base::index(node), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] QueryNearest queryNearest(Key node, Point query,
	                                        Predicate const& predicate,
	                                        float            epsilon = 0.0f)
	{
		return queryNearest(Base::index(node), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] ConstQueryNearest queryNearest(Key node, Point query,
	                                             Predicate const& predicate,
	                                             float            epsilon = 0.0f) const
	{
		return queryNearest(Base::index(node), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] QueryNearest queryNearest(Coord node, Point query,
	                                        Predicate const& predicate,
	                                        float            epsilon = 0.0f)
	{
		return queryNearest(Base::index(node), query, predicate, epsilon);
	}

	template <class Predicate>
	[[nodiscard]] ConstQueryNearest queryNearest(Coord node, Point query,
	                                             Predicate const& predicate,
	                                             float            epsilon = 0.0f) const
	{
		return queryNearest(Base::index(node), query, predicate, epsilon);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                       Nearest                                       |
	|                                                                                     |
	**************************************************************************************/

	// [[nodiscard]] reference nearest(Point p, float epsilon = 0.0f)
	// {
	// 	return nearest(Base::index(), p, epsilon);
	// }

	// [[nodiscard]] const_reference nearest(Point p, float epsilon = 0.0f) const
	// {
	// 	return nearest(Base::index(), p, epsilon);
	// }

	// [[nodiscard]] reference nearest(Index node, Point p, float epsilon = 0.0f)
	// {
	// 	// TODO: Implement
	// }

	// [[nodiscard]] const_reference nearest(Index node, Point p, float epsilon = 0.0f)
	// const
	// {
	// 	// TODO: Implement
	// }

	/**************************************************************************************
	|                                                                                     |
	|                                      Capacity                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] bool      empty() const noexcept { return 0 == size(); }
	[[nodiscard]] size_type size() const noexcept { return size_; }

	/**************************************************************************************
	|                                                                                     |
	|                                      Modifiers                                      |
	|                                                                                     |
	**************************************************************************************/

	template <class... Args>
	iterator emplace(Point p, Args&&... args)
	{
		assert(Base::isInside(p));

		auto node  = Base::create(p);
		auto block = Base::treeBlock(node);
		block.value[node.offset].push_back({p, std::forward<Args>(args)...});

		// TODO: Implement
	}

	iterator insert(Point p, T const& value)
	{
		assert(Base::isInside(p));

		auto node  = Base::create(p);
		auto block = Base::treeBlock(node);
		block.value[node.offset].emplace_back(p, value);

		// TODO: Implement
	}

	iterator insert(value_type const& value) { return insert(value.first, value.second); }

	iterator insert(value_type&& value)
	{
		assert(Base::isInside(value.first));

		auto node  = Base::create(value.first);
		auto block = Base::treeBlock(node);
		block.value[node.offset].push_back(std::move(value));

		// TODO: Implement
	}

	template <class InputIt>
	void insert(InputIt first, InputIt last)
	{
		// TODO: Implement
	}

	void insert(std::initializer_list<value_type> ilist)
	{
		insert(std::begin(ilist), std::end(ilist));
	}

	/*!
	 * @brief
	 *
	 * @note The iterator pos must be valid and dereferenceable. Thus the end() iterator
	 * (which is valid, but is not dereferenceable) cannot be used as a value for pos.
	 *
	 * @param pos
	 * @return iterator
	 */
	iterator erase(iterator pos)
	{
		if (begin() != pos) {
			// Need to check end iterator of nodes before
			auto prev  = std::prev(pos);
			auto trail = Base::trail(prev->code);
			for (auto node : trail) {
				if (iters(node).second == pos) {
					++iters(node).second;
					if (iters(node).first == iters(node).second) {
						++iters(node).first;
					}
				}
			}
		}

		auto trail = Base::trail(pos->code);
		for (auto node : trail) {
			if (iters(node).first == pos) {
				++iters(node).first;
				if (iters(node).first == iters(node).second) {
					++iters(node).second;
				}
			}
		}
		// TODO: Implement
		return val_.erase(pos);
	}

	/*!
	 * @brief
	 *
	 * @note The iterator pos must be valid and dereferenceable. Thus the end() iterator
	 * (which is valid, but is not dereferenceable) cannot be used as a value for pos.
	 *
	 * @param pos
	 * @return iterator
	 */
	iterator erase(const_iterator pos)
	{
		if (cbegin() != pos) {
			// Need to check end iterator of nodes before
			auto prev  = std::prev(pos);
			auto trail = Base::trail(prev->code);
			for (auto node : trail) {
				if (iters(node).second == pos) {
					++iters(node).second;
					if (iters(node).first == iters(node).second) {
						++iters(node).first;
					}
				}
			}
		}

		auto trail = Base::trail(pos->code);
		for (auto node : trail) {
			if (iters(node).first == pos) {
				++iters(node).first;
				if (iters(node).first == iters(node).second) {
					++iters(node).second;
				}
			}
		}
		// TODO: Implement
		return val_.erase(pos);
	}

	/*!
	 * @brief
	 *
	 * @note The iterator first does not need to be dereferenceable if first == last:
	 * erasing an empty range is a no-op.
	 *
	 * @param pos
	 * @return iterator
	 */
	iterator erase(const_iterator first, const_iterator last)
	{
		if (first == last) {
			return last;
		}

		if (cbegin() != first) {
			// Need to check end iterator of nodes before
			auto prev  = std::prev(first);
			auto trail = Base::trail(prev->code);
			for (auto node : trail) {
				if (iters(node).second == first) {
					++iters(node).second;
					if (iters(node).first == iters(node).second) {
						++iters(node).first;
					}
				}
			}
		}

		auto last_prev = std::prev(last);

		// TODO: Handle the ones in the middle
		reset(first->code, last_prev->code, last);

		auto first_trail     = Base::trail(first->code);
		auto last_prev_trail = Base::trail(last_prev->code);

		// TODO: Correct below
		for (auto node : first_trail) {
			if (iters(node).first == first) {
				++iters(node).first;
				if (iters(node).first == iters(node).second) {
					++iters(node).second;
				}
			}
		}
		for (auto node : last_prev_trail) {
			if (iters(node).second == last_prev) {
				++iters(node).second;
				if (iters(node).first == iters(node).second) {
					++iters(node).first;
				}
			}
		}

		// TODO: Implement
		return val_.erase(first, last);
	}

	void clear()
	{
		// TODO: Implement
		val_.clear();
		Base::clear();
	}

	void clear(Index node)
	{
		auto range = at(node);
		val_.erase(range.begin(), range.end());
		// TODO: Implement
	}

	void clear(Node node)
	{
		auto range = at(node);
		val_.erase(range.begin(), range.end());
		// TODO: Implement
	}

	void clear(Code node)
	{
		auto range = at(node);
		val_.erase(range.begin(), range.end());
		// TODO: Implement
	}

	void clear(Key node)
	{
		auto range = at(node);
		val_.erase(range.begin(), range.end());
		// TODO: Implement
	}

	void clear(Coord node)
	{
		auto range = at(node);
		val_.erase(range.begin(), range.end());
		// TODO: Implement
	}

 protected:
	[[nodiscard]] std::pair<iterator, iterator>& iters(Index node)
	{
		return this->treeBlock(node).value[node.offset];
	}

	[[nodiscard]] std::pair<const_iterator, const_iterator> iters(Index node) const
	{
		auto [first, last] = this->treeBlock(node).value[node.offset];
		return {first, last};
	}

	void reset(Code first, Code last, const_iterator pos) const
	{
		auto first_node = Base::index(first);
		auto last_node  = Base::index(last);

		if (first_node == last_node) {
			return;
		}

		auto first_trail = Base::trail(first);
		// So we do not try to visit root's non-existing siblings
		first_trail[0].offset = Base::branchingFactor();

		for (auto it = std::rbegin(first_trail); std::rend(first_trail) != it; ++it) {
			auto node = *it;
			for (++node.offset; Base::branchingFactor() > node.offset; ++node.offset) {
				if (resetRecurs(node, last_node, pos)) {
					return;
				}
			}
		}
	}

	bool resetRecurs(Index node, Index last, const_iterator pos) const
	{
		if (node == last) {
			return true;
		}

		if (isLeaf(node)) {
			iters(node).first  = pos;
			iters(node).second = pos;
			return false;
		}

		auto c = Base::children(node);
		for (offset_t i{}; Base::branchingFactor() > i; ++i) {
			if (resetRecurs(Index(c, i), last, pos)) {
				return true;
			}
		}

		iters(node).first  = pos;
		iters(node).second = pos;
	}

 private:
	size_type size_ = 0;
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_TREE_MAP_HPP