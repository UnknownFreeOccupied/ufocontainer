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

#ifndef UFO_CONTAINER_TREE_TREE_ITERATOR_HPP
#define UFO_CONTAINER_TREE_TREE_ITERATOR_HPP

// UFO
#include <ufo/container/tree/tree_index.hpp>
#include <ufo/container/tree/tree_node_nearest.hpp>
#include <ufo/container/tree/tree_predicate.hpp>
#include <ufo/geometry/bounding_volume.hpp>
#include <ufo/geometry/minimum_distance.hpp>

// STL
#include <cstddef>   // For std::ptrdiff_t
#include <iterator>  // For std::forward_iterator_tag
#include <memory>
#include <queue>
#include <set>
#include <stack>
#include <type_traits>
// #include <utility>

namespace ufo
{
template <class Tree, class Node>
class TreeIterator
{
 public:
	using offset_t = typename Tree::offset_t;

	using iterator_category = std::forward_iterator_tag;
	using difference_type   = std::ptrdiff_t;
	using value_type        = Node;
	using pointer           = value_type const*;
	using reference         = value_type const&;

	constexpr TreeIterator(Tree const* tree) : tree_(tree) {}

	virtual ~TreeIterator() {}

	virtual void next() = 0;

	virtual TreeIterator* copy() = 0;

	virtual reference data() const = 0;

	constexpr Tree const* tree() const { return tree_; }

	virtual bool equal(TreeIterator const& other) const = 0;

	virtual std::size_t status() const = 0;

 protected:
	template <class TNode>
	[[nodiscard]] bool isPureLeaf(TNode const& node) const
	{
		return tree_->isPureLeaf(node);
	}

	[[nodiscard]] bool isPureLeaf(TreeIndex node) const { return tree_->isPureLeaf(node); }

	template <class TNode>
	[[nodiscard]] bool isParent(TNode const& node) const
	{
		return tree_->isParent(node.index());
	}

	[[nodiscard]] bool isParent(TreeIndex node) const { return tree_->isParent(node); }

	template <bool OnlyExists, class TNode>
	[[nodiscard]] TNode child(TNode const& node, offset_t child_index) const
	{
		if constexpr (OnlyExists) {
			return tree_->childUnsafe(node, child_index);
		} else {
			return tree_->child(node, child_index);
		}
	}

	[[nodiscard]] TreeIndex child(TreeIndex node, offset_t child_index) const
	{
		return tree_->child(node, child_index);
	}

	template <bool OnlyExists, class TNode>
	[[nodiscard]] TNode sibling(TNode const& node, offset_t sibling_index) const
	{
		if constexpr (OnlyExists) {
			return tree_->siblingUnsafe(node, sibling_index);
		} else {
			return tree_->sibling(node, sibling_index);
		}
	}

	[[nodiscard]] TreeIndex sibling(TreeIndex node, offset_t sibling_index) const
	{
		return tree_->sibling(node, sibling_index);
	}

	template <class TNode>
	[[nodiscard]] auto boundingVolume(TNode const& node) const
	{
		return tree_->boundingVolume(node);
	}

	[[nodiscard]] auto boundingVolume(TreeIndex node) const
	{
		return tree_->boundingVolume(node);
	}

	template <class TNode>
	[[nodiscard]] bool exists(TNode const& node) const
	{
		return tree_->exists(node.index());
	}

	[[nodiscard]] bool exists(TreeIndex node) const { return tree_->exists(node); }

	template <class Predicate>
	constexpr void initPredicate(Predicate const& predicate) const
	{
		pred::Init<Predicate>::apply(predicate, *tree_);
	}

	template <bool OnlyExists, class TNode, class Predicate>
	[[nodiscard]] bool validInner(TNode const& node, Predicate const& predicate) const
	{
		if constexpr (OnlyExists) {
			return isParent(node) &&
			       pred::InnerCheck<Predicate>::apply(predicate, *tree_, node);
		} else {
			return !isPureLeaf(node) &&
			       pred::InnerCheck<Predicate>::apply(predicate, *tree_, node);
		}
	}

	template <class Predicate>
	[[nodiscard]] bool validInner(TreeIndex node, Predicate const& predicate) const
	{
		return isParent(node) && pred::InnerCheck<Predicate>::apply(predicate, *tree_, node);
	}

	template <class TNode, class Predicate>
	[[nodiscard]] bool validReturn(TNode const& node, Predicate const& predicate) const
	{
		return pred::ValueCheck<Predicate>::apply(predicate, *tree_, node);
	}

	template <class Predicate>
	[[nodiscard]] bool validReturn(TreeIndex node, Predicate const& predicate) const
	{
		return pred::ValueCheck<Predicate>::apply(predicate, *tree_, node);
	}

 protected:
	Tree const* tree_;
};

template <class Tree, class Node>
class TreeIteratorWrapper
{
 private:
	using TI = TreeIterator<Tree, Node>;

 public:
	// Tags
	using typename TI::difference_type;
	using typename TI::iterator_category;
	using typename TI::pointer;
	using typename TI::reference;
	using typename TI::value_type;

	TreeIteratorWrapper(TI* tree_iterator) : tree_iterator_(tree_iterator) {}

	TreeIteratorWrapper(TreeIteratorWrapper const& other)
	    : tree_iterator_(other.tree_iterator_->copy())
	{
	}

	TreeIteratorWrapper& operator++()
	{
		tree_iterator_->next();
		return *this;
	}

	TreeIteratorWrapper operator++(int)
	{
		TreeIteratorWrapper result(tree_iterator_->copy());
		++(*this);
		return result;
	}

	pointer operator->() const { return &(tree_iterator_->data()); }

	reference operator*() const { return tree_iterator_->data(); }

	friend bool operator==(TreeIteratorWrapper const& lhs, TreeIteratorWrapper const& rhs)
	{
		return lhs.tree_iterator_->equal(*(rhs.tree_iterator_));
	}

	friend bool operator!=(TreeIteratorWrapper const& lhs, TreeIteratorWrapper const& rhs)
	{
		return !(lhs == rhs);
	}

 private:
	std::unique_ptr<TI> tree_iterator_;
};

template <class Tree, class Node, class Predicate, bool OnlyExists, bool EarlyStopping>
class TreeForwardIterator final : public TreeIterator<Tree, Node>
{
 private:
	using TI = TreeIterator<Tree, TreeNodeNearest<Node>>;

	static constexpr bool const OnlyLeavesOrFixedDepth =
	    pred::contains_always_predicate_v<pred::PureLeaf, Predicate> ||
	    pred::contains_always_predicate_v<pred::DepthE, Predicate> || EarlyStopping;

 public:
	using offset_t = typename Tree::offset_t;

	// Tags
	using typename TI::difference_type;
	using typename TI::iterator_category;
	using typename TI::pointer;
	using typename TI::reference;
	using typename TI::value_type;

	TreeForwardIterator(Tree const* tree) : TI(tree) {}

	TreeForwardIterator(Tree const* tree, Node const& root, Predicate const& predicate)
	    : TI(tree), predicate_(predicate)
	{
		init(root);
	}

	void next() override
	{
		return_index_ -= return_index_ ? 1 : 0;

		// Skip forward to next valid return node
		while (0 == return_index_ && inner_index_) {
			auto current = this->template child<OnlyExists>(inner_nodes_[--inner_index_], 7);

			// Go down the tree
			for (offset_t i{Tree::branchingFactor()}; 0 != i;) {
				current = this->template sibling<OnlyExists>(current, --i);

				if constexpr (OnlyLeavesOrFixedDepth) {
					if (this->validReturn(current, predicate_)) {
						return_nodes_[return_index_++] = current;
					} else if (this->template validInner<OnlyExists>(current, predicate_)) {
						inner_nodes_[inner_index_++] = current;
					}
				} else {
					if (this->validReturn(current, predicate_)) {
						return_nodes_[return_index_++] = current;
					}
					if (this->template validInner<OnlyExists>(current, predicate_)) {
						inner_nodes_[inner_index_++] = current;
					}
				}
			}
		}
	}

	TreeForwardIterator* copy() override { return new TreeForwardIterator(*this); }

	reference data() const override { return return_nodes_[return_index_ - 1]; }

	bool equal(TI const& other) const override
	{
		return status() == other.status();
		// return other.tree() == this->tree() && other.status() == status() &&
		//        (!return_nodes_.empty() && other.data() == data());
	}

	std::size_t status() const override { return inner_index_ + return_index_; }

 private:
	void init(Node const& node)
	{
		this->initPredicate(predicate_);

		if constexpr (OnlyExists) {
			if (!this->exists(node)) {
				return;
			}
		}

		if constexpr (OnlyLeavesOrFixedDepth) {
			if (this->validReturn(node, predicate_)) {
				return_nodes_[return_index_++] = node;
			} else if (this->template validInner<OnlyExists>(node, predicate_)) {
				inner_nodes_[inner_index_++] = node;
				next();
			}
		} else {
			if (this->validReturn(node, predicate_)) {
				return_nodes_[return_index_++] = node;
				if (this->template validInner<OnlyExists>(node, predicate_)) {
					inner_nodes_[inner_index_++] = node;
				}
			} else if (this->template validInner<OnlyExists>(node, predicate_)) {
				inner_nodes_[inner_index_++] = node;
				next();
			}
		}
	}

 private:
	// Predicate that nodes has to fulfill
	Predicate const predicate_{};

	// To be processed inner nodes
	std::array<Node, Tree::childrenPerParent() * Tree::maxDepthLevels()> inner_nodes_;
	// To be processed return nodes
	std::array<Node, Tree::childrenPerParent()> return_nodes_;

	int inner_index_{};
	int return_index_{};
};

template <class Tree, class Node, class Geometry, class Predicate, bool OnlyExists,
          bool EarlyStopping>
class TreeNearestIterator final : public TreeIterator<Tree, Node>
{
 private:
	using TI = TreeIterator<Tree, Node>;

	static constexpr bool const OnlyLeavesOrFixedDepth =
	    pred::contains_always_predicate_v<pred::PureLeaf, Predicate> ||
	    pred::contains_always_predicate_v<pred::DepthE, Predicate> || EarlyStopping;

 public:
	using offset_t = typename Tree::offset_t;

	// Tags
	using typename TI::difference_type;
	using typename TI::iterator_category;
	using typename TI::pointer;
	using typename TI::reference;
	using typename TI::value_type;

	TreeNearestIterator(Tree const* tree, Node const& root, Geometry const& geometry,
	                    Predicate const& predicate, double epsilon = 0.0)
	    : TI(tree), predicate_(predicate), geometry_(geometry), epsilon_(epsilon)
	{
		init(root);
	}

	void next() override
	{
		if (!return_nodes_.empty()) {
			return_nodes_.pop();
		}

		// Skip forward to next valid return node
		while (!inner_nodes_.empty()) {
			if (!return_nodes_.empty() && return_nodes_.top() <= inner_nodes_.top()) {
				return;
			}

			auto current =
			    this->template child<OnlyExists>(static_cast<Node>(inner_nodes_.top()), 0);
			inner_nodes_.pop();

			for (offset_t idx{}; Tree::branchingFactor() != idx; ++idx) {
				current = this->template sibling<OnlyExists>(current, idx);

				if constexpr (OnlyLeavesOrFixedDepth) {
					if (this->validReturn(current, predicate_)) {
						return_nodes_.emplace(current, sqDistance(current));
					} else if (this->template validInner<OnlyExists>(current, predicate_)) {
						inner_nodes_.emplace(current, sqDistance(current) + epsilon_);
					}
				} else {
					if (this->validReturn(current, predicate_)) {
						auto dist_sq = sqDistance(current);
						return_nodes_.emplace(current, dist_sq);
						if (this->template validInner<OnlyExists>(current, predicate_)) {
							inner_nodes_.emplace(current, dist_sq + epsilon_);
						}
					} else if (this->template validInner<OnlyExists>(current, predicate_)) {
						inner_nodes_.emplace(current, sqDistance(current) + epsilon_);
					}
				}
			}
		}
	}

	TreeNearestIterator* copy() override { return new TreeNearestIterator(*this); }

	reference data() const override { return return_nodes_.top(); }

	bool equal(TI const& other) const override
	{
		return other.tree() == this->tree() && other.status() == status() &&
		       (!return_nodes_.empty() && other.data() == data());
	}

	std::size_t status() const override
	{
		return inner_nodes_.size() + return_nodes_.size();
	}

 private:
	double sqDistance(Node const& node) const
	{
		return squaredDistance(this->boundingVolume(node), geometry_);
	}

	void init(Node const& node)
	{
		this->initPredicate(predicate_);

		if (OnlyExists) {
			if (!this->exists(node)) {
				return;
			}
		}

		if constexpr (OnlyLeavesOrFixedDepth) {
			if (this->validReturn(node, predicate_)) {
				return_nodes_.emplace(node, sqDistance(node));
			} else if (this->template validInner<OnlyExists>(node, predicate_)) {
				std::vector<value_type> container;
				container.reserve(256);
				return_nodes_ = std::priority_queue<value_type, std::vector<value_type>,
				                                    std::greater<value_type>>(
				    std::greater<value_type>(), std::move(container));
				inner_nodes_ = return_nodes_;

				inner_nodes_.emplace(node, sqDistance(node) + epsilon_);
				next();
			}
		} else {
			if (this->validReturn(node, predicate_)) {
				std::vector<value_type> container;
				container.reserve(256);
				return_nodes_ = std::priority_queue<value_type, std::vector<value_type>,
				                                    std::greater<value_type>>(
				    std::greater<value_type>(), std::move(container));
				inner_nodes_ = return_nodes_;

				auto const dist_sq = sqDistance(node);

				return_nodes_.emplace(node, dist_sq);
				if (this->template validInner<OnlyExists>(node, predicate_)) {
					inner_nodes_.emplace(node, dist_sq + epsilon_);
				}
			} else if (this->template validInner<OnlyExists>(node, predicate_)) {
				std::vector<value_type> container;
				container.reserve(256);
				return_nodes_ = std::priority_queue<value_type, std::vector<value_type>,
				                                    std::greater<value_type>>(
				    std::greater<value_type>(), std::move(container));
				inner_nodes_ = return_nodes_;

				inner_nodes_.emplace(node, sqDistance(node) + epsilon_);
				next();
			}
		}
	}

 private:
	using Queue =
	    std::priority_queue<value_type, std::vector<value_type>, std::greater<value_type>>;

	Predicate const predicate_{};   // Predicate that nodes has to fulfill
	Geometry const  geometry_;      // Geometry to find nearest to
	double const    epsilon_{};     // Epsilon for approximate search
	Queue           inner_nodes_;   // To be processed inner nodes
	Queue           return_nodes_;  // To be processed return nodes
};

// template <class Tree, typename T>
// class TreeIteratorBase
// {
//  public:
// 	// Tags
// 	using iterator_category = std::forward_iterator_tag;
// 	using difference_type   = std::ptrdiff_t;
// 	using value_type        = T;
// 	using pointer           = value_type const*;
// 	using reference         = value_type const&;

// 	constexpr TreeIteratorBase(Tree const* tree) : tree_(tree) {}

// 	virtual ~TreeIteratorBase() {}

// 	virtual void next() = 0;

// 	virtual TreeIteratorBase* copy() = 0;

// 	virtual reference data() const = 0;

// 	constexpr Tree const* tree() const { return tree_; }

// 	virtual bool equal(TreeIteratorBase const& other) const = 0;

// 	virtual std::size_t status() const = 0;

//  protected:
// 	template <class Node>
// 	[[nodiscard]] constexpr bool isPureLeaf(Node const& node) const
// 	{
// 		return tree_->isPureLeaf(node);
// 	}

// 	template <class Node>
// 	[[nodiscard]] constexpr bool isParent(Node const& node) const
// 	{
// 		return tree_->isParent(node.index());
// 	}

// 	template <bool OnlyExists, class Node>
// 	[[nodiscard]] constexpr Node child(Node const& node, offset_t child) const
// 	{
// 		if constexpr (OnlyExists) {
// 			return tree_->childUnsafe(node, child);
// 		} else {
// 			return tree_->child(node, child);
// 		}
// 	}

// 	template <bool OnlyExists, class Node>
// 	[[nodiscard]] constexpr Node sibling(Node const& node, offset_t sibling) const
// 	{
// 		if constexpr (OnlyExists) {
// 			return tree_->siblingUnsafe(node, sibling);
// 		} else {
// 			return tree_->sibling(node, sibling);
// 		}
// 	}

// 	template <class Node>
// 	[[nodiscard]] auto boundingVolume(Node const& node) const
// 	{
// 		return tree_->boundingVolume(node);
// 	}

// 	template <class Node>
// 	[[nodiscard]] constexpr bool exists(Node const& node) const
// 	{
// 		return tree_->exists(node.index());
// 	}

// 	template <class Predicate>
// 	constexpr void initPredicate(Predicate const& predicate) const
// 	{
// 		pred::Init<Predicate>::apply(predicate, *tree_);
// 	}

// 	template <bool OnlyExists, class Node, class Predicate>
// 	[[nodiscard]] constexpr bool validInner(Node const&      node,
// 	                                        Predicate const& predicate) const
// 	{
// 		if constexpr (OnlyExists) {
// 			return isParent(node) &&
// 			       pred::InnerCheck<Predicate>::apply(predicate, *tree_, node);
// 		} else {
// 			return !isPureLeaf(node) &&
// 			       pred::InnerCheck<Predicate>::apply(predicate, *tree_, node);
// 		}
// 	}

// 	template <class Node, class Predicate>
// 	[[nodiscard]] constexpr bool validReturn(Node const&      node,
// 	                                         Predicate const& predicate) const
// 	{
// 		return pred::ValueCheck<Predicate>::apply(predicate, *tree_, node);
// 	}

//  protected:
// 	// The UFOMap
// 	Tree const* tree_;
// };

// template <class Tree, typename T>
// class TreeIteratorWrapper
// {
//  private:
// 	using Base = TreeIteratorBase<Tree, T>;

//  public:
// 	// Tags
// 	using difference_type   = typename Base::difference_type;
// 	using iterator_category = typename Base::iterator_category;
// 	using pointer           = typename Base::pointer;
// 	using reference         = typename Base::reference;
// 	using value_type        = typename Base::value_type;

// 	TreeIteratorWrapper(Base* it_base) : it_base_(it_base) {}

// 	TreeIteratorWrapper(TreeIteratorWrapper const& other) :
// it_base_(other.it_base_->copy())
// 	{
// 	}

// 	TreeIteratorWrapper& operator++()
// 	{
// 		it_base_->next();
// 		return *this;
// 	}

// 	TreeIteratorWrapper operator++(int)
// 	{
// 		TreeIteratorWrapper result(it_base_->copy());
// 		++(*this);
// 		return result;
// 	}

// 	pointer operator->() const { return &(it_base_->data()); }

// 	reference operator*() const { return it_base_->data(); }

// 	friend bool operator==(TreeIteratorWrapper const& lhs, TreeIteratorWrapper const& rhs)
// 	{
// 		return lhs.it_base_->equal(*(rhs.it_base_));
// 	}

// 	friend bool operator!=(TreeIteratorWrapper const& lhs, TreeIteratorWrapper const& rhs)
// 	{
// 		return !(lhs == rhs);
// 	}

//  private:
// 	std::unique_ptr<Base> it_base_;
// };

// template <bool OnlyExists, bool EarlyStopping, class Tree, class Node,
//           class Predicate = pred::TRUE>
// class TreeIterator final : public TreeIteratorBase<Tree, Node>
// {
//  private:
// 	static constexpr bool const OnlyLeavesOrFixedDepth =
// 	    pred::contains_always_predicate_v<pred::PureLeaf, Predicate> ||
// 	    pred::contains_always_predicate_v<pred::DepthE, Predicate> || EarlyStopping;
// 	static constexpr offset_t const N = Tree::childrenPerParent();

// 	using Base = TreeIteratorBase<Tree, Node>;

//  public:
// 	// Tags
// 	using typename Base::difference_type;
// 	using typename Base::iterator_category;
// 	using typename Base::pointer;
// 	using typename Base::reference;
// 	using typename Base::value_type;

// 	TreeIterator(Tree const* tree) : Base(tree) {}

// 	TreeIterator(Tree const* tree, Node const& root, Predicate const& predicate)
// 	    : Base(tree), predicate_(predicate)
// 	{
// 		init(root);
// 	}

// 	void next() override
// 	{
// 		return_index_ -= return_index_ ? 1 : 0;

// 		// Skip forward to next valid return node
// 		while (0 == return_index_ && inner_index_) {
// 			auto current = this->template child<OnlyExists>(inner_nodes_[--inner_index_], 7);

// 			// Go down the tree
// 			for (offset_t i{N}; 0 != i;) {
// 				current = this->template sibling<OnlyExists>(current, --i);

// 				if constexpr (OnlyLeavesOrFixedDepth) {
// 					if (this->validReturn(current, predicate_)) {
// 						return_nodes_[return_index_++] = current;
// 					} else if (this->template validInner<OnlyExists>(current, predicate_)) {
// 						inner_nodes_[inner_index_++] = current;
// 					}
// 				} else {
// 					if (this->validReturn(current, predicate_)) {
// 						return_nodes_[return_index_++] = current;
// 					}
// 					if (this->template validInner<OnlyExists>(current, predicate_)) {
// 						inner_nodes_[inner_index_++] = current;
// 					}
// 				}
// 			}
// 		}
// 	}

// 	TreeIterator* copy() override { return new TreeIterator(*this); }

// 	reference data() const override { return return_nodes_[return_index_ - 1]; }

// 	bool equal(Base const& other) const override
// 	{
// 		return status() == other.status();
// 		// return other.tree() == this->tree() && other.status() == status() &&
// 		//        (!return_nodes_.empty() && other.data() == data());
// 	}

// 	std::size_t status() const override { return inner_index_ + return_index_; }

//  private:
// 	void init(Node const& node)
// 	{
// 		this->initPredicate(predicate_);

// 		if constexpr (OnlyExists) {
// 			if (!this->exists(node)) {
// 				return;
// 			}
// 		}

// 		if constexpr (OnlyLeavesOrFixedDepth) {
// 			if (this->validReturn(node, predicate_)) {
// 				return_nodes_[return_index_++] = node;
// 			} else if (this->template validInner<OnlyExists>(node, predicate_)) {
// 				inner_nodes_[inner_index_++] = node;
// 				next();
// 			}
// 		} else {
// 			if (this->validReturn(node, predicate_)) {
// 				return_nodes_[return_index_++] = node;
// 				if (this->template validInner<OnlyExists>(node, predicate_)) {
// 					inner_nodes_[inner_index_++] = node;
// 				}
// 			} else if (this->template validInner<OnlyExists>(node, predicate_)) {
// 				inner_nodes_[inner_index_++] = node;
// 				next();
// 			}
// 		}
// 	}

//  private:
// 	Predicate const predicate_{};  // Predicate that nodes has to fulfill

// 	std::array<Node, N * Tree::maxDepthLevels()>
// 	                    inner_nodes_;   // To be processed inner nodes
// 	std::array<Node, N> return_nodes_;  // To be processed return nodes
// 	int                 inner_index_{};
// 	int                 return_index_{};
// };

// template <bool OnlyExists, bool EarlyStopping, class Tree, class Node, class Geometry,
//           class Predicate = pred::TRUE>
// class TreeIteratorNearest final : public TreeIteratorBase<Tree, TreeNodeNearest<Node>>
// {
//  private:
// 	static constexpr bool const OnlyLeavesOrFixedDepth =
// 	    pred::contains_always_predicate_v<pred::PureLeaf, Predicate> ||
// 	    pred::contains_always_predicate_v<pred::DepthE, Predicate> || EarlyStopping;
// 	static constexpr offset_t const N = numChildren(Tree::treeType());

// 	using Base = TreeIteratorBase<Tree, TreeNodeNearest<Node>>;

//  public:
// 	// Tags
// 	using typename Base::difference_type;
// 	using typename Base::iterator_category;
// 	using typename Base::pointer;
// 	using typename Base::reference;
// 	using typename Base::value_type;

//  private:
// 	using Queue =
// 	    std::priority_queue<value_type, std::vector<value_type>,
// std::greater<value_type>>;

//  public:
// 	TreeIteratorNearest(Tree const* tree, Node const& root, Geometry const& geometry,
// 	                    Predicate const& predicate, double epsilon = 0.0)
// 	    : Base(tree), predicate_(predicate), geometry_(geometry), epsilon_(epsilon)
// 	{
// 		init(root);
// 	}

// 	void next() override
// 	{
// 		if (!return_nodes_.empty()) {
// 			return_nodes_.pop();
// 		}

// 		// Skip forward to next valid return node
// 		while (!inner_nodes_.empty()) {
// 			if (!return_nodes_.empty() && return_nodes_.top() <= inner_nodes_.top()) {
// 				return;
// 			}

// 			auto current =
// 			    this->template child<OnlyExists>(static_cast<Node>(inner_nodes_.top()), 0);
// 			inner_nodes_.pop();

// 			for (offset_t idx{}; N != idx; ++idx) {
// 				current = this->template sibling<OnlyExists>(current, idx);

// 				if constexpr (OnlyLeavesOrFixedDepth) {
// 					if (this->validReturn(current, predicate_)) {
// 						return_nodes_.emplace(current, sqDistance(current));
// 					} else if (this->template validInner<OnlyExists>(current, predicate_)) {
// 						inner_nodes_.emplace(current, sqDistance(current) + epsilon_);
// 					}
// 				} else {
// 					if (this->validReturn(current, predicate_)) {
// 						auto dist_sq = sqDistance(current);
// 						return_nodes_.emplace(current, dist_sq);
// 						if (this->template validInner<OnlyExists>(current, predicate_)) {
// 							inner_nodes_.emplace(current, dist_sq + epsilon_);
// 						}
// 					} else if (this->template validInner<OnlyExists>(current, predicate_)) {
// 						inner_nodes_.emplace(current, sqDistance(current) + epsilon_);
// 					}
// 				}
// 			}
// 		}
// 	}

// 	TreeIteratorNearest* copy() override { return new TreeIteratorNearest(*this); }

// 	reference data() const override { return return_nodes_.top(); }

// 	bool equal(Base const& other) const override
// 	{
// 		return other.tree() == this->tree() && other.status() == status() &&
// 		       (!return_nodes_.empty() && other.data() == data());
// 	}

// 	std::size_t status() const override
// 	{
// 		return inner_nodes_.size() + return_nodes_.size();
// 	}

//  private:
// 	double sqDistance(Node const& node) const
// 	{
// 		return squaredDistance(this->boundingVolume(node), geometry_);
// 	}

// 	void init(Node const& node)
// 	{
// 		this->initPredicate(predicate_);

// 		if (OnlyExists) {
// 			if (!this->exists(node)) {
// 				return;
// 			}
// 		}

// 		if constexpr (OnlyLeavesOrFixedDepth) {
// 			if (this->validReturn(node, predicate_)) {
// 				return_nodes_.emplace(node, sqDistance(node));
// 			} else if (this->template validInner<OnlyExists>(node, predicate_)) {
// 				std::vector<value_type> container;
// 				container.reserve(256);
// 				return_nodes_ = std::priority_queue<value_type, std::vector<value_type>,
// 				                                    std::greater<value_type>>(
// 				    std::greater<value_type>(), std::move(container));
// 				inner_nodes_ = return_nodes_;

// 				inner_nodes_.emplace(node, sqDistance(node) + epsilon_);
// 				next();
// 			}
// 		} else {
// 			if (this->validReturn(node, predicate_)) {
// 				std::vector<value_type> container;
// 				container.reserve(256);
// 				return_nodes_ = std::priority_queue<value_type, std::vector<value_type>,
// 				                                    std::greater<value_type>>(
// 				    std::greater<value_type>(), std::move(container));
// 				inner_nodes_ = return_nodes_;

// 				auto const dist_sq = sqDistance(node);

// 				return_nodes_.emplace(node, dist_sq);
// 				if (this->template validInner<OnlyExists>(node, predicate_)) {
// 					inner_nodes_.emplace(node, dist_sq + epsilon_);
// 				}
// 			} else if (this->template validInner<OnlyExists>(node, predicate_)) {
// 				std::vector<value_type> container;
// 				container.reserve(256);
// 				return_nodes_ = std::priority_queue<value_type, std::vector<value_type>,
// 				                                    std::greater<value_type>>(
// 				    std::greater<value_type>(), std::move(container));
// 				inner_nodes_ = return_nodes_;

// 				inner_nodes_.emplace(node, sqDistance(node) + epsilon_);
// 				next();
// 			}
// 		}
// 	}

//  private:
// 	Predicate const predicate_{};   // Predicate that nodes has to fulfill
// 	Geometry const  geometry_;      // Geometry to find nearest to
// 	double const    epsilon_{};     // Epsilon for approximate search
// 	Queue           inner_nodes_;   // To be processed inner nodes
// 	Queue           return_nodes_;  // To be processed return nodes
// };
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_TREE_ITERATOR_HPP
