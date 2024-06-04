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

#ifndef UFO_CONTAINER_TREE_PREDICATE_DYNAMIC_HPP
#define UFO_CONTAINER_TREE_PREDICATE_DYNAMIC_HPP

// UFO
#include <ufo/container/tree/predicate/predicate.hpp>

// STL
#include <memory>

namespace ufo::pred
{
namespace detail
{
template <class Tree>
struct Dynamic {
	virtual ~Dynamic() {}

	[[nodiscard]] virtual bool valueCheck(Tree const&, typename Tree::Node) const = 0;

	[[nodiscard]] virtual bool innerCheck(Tree const&, typename Tree::Node) const = 0;

	[[nodiscard]] virtual Dynamic* clone() const = 0;
};

template <class Tree, class Predicate>
struct DynamicPredicate
    : Dynamic<Tree>
    , Predicate {
	template <class... Args>
	DynamicPredicate(Args&&... args) : Predicate(std::forward<Args>(args)...)
	{
	}

	template <class... Args>
	DynamicPredicate(Tree const&, Args&&... args) : Predicate(std::forward<Args>(args)...)
	{
	}

	DynamicPredicate(Predicate const& pred) : Predicate(pred) {}

	DynamicPredicate(Predicate&& pred) : Predicate(std::move(pred)) {}

	DynamicPredicate(Tree const&, Predicate const& pred) : Predicate(pred) {}

	DynamicPredicate(Tree const&, Predicate&& pred) : Predicate(std::move(pred)) {}

	virtual ~DynamicPredicate() {}

	[[nodiscard]] bool valueCheck(Tree const& tree, typename Tree::Node node) const override
	{
		return valueCheck(static_cast<Predicate const&>(*this), tree, node);
	}

	[[nodiscard]] bool innerCheck(Tree const& tree, typename Tree::Node node) const override
	{
		return innerCheck(static_cast<Predicate const&>(*this), tree, node);
	}

 protected:
	DynamicPredicate* clone() const override { return new DynamicPredicate(*this); }
};

template <class Predicate, class Tree, class Node>
[[nodiscard]] bool valueCheck(DynamicPredicate<Tree, Predicate> const& p, Tree const& t,
                              Node n)
{
	return p.valueCheck(t, n);
}

template <class Predicate, class Tree, class Node>
[[nodiscard]] bool innerCheck(DynamicPredicate<Tree, Predicate> const& p, Tree const& t,
                              Node n)
{
	return p.innerCheck(t, n);
}
}  // namespace detail

template <class Tree>
struct Predicate {
 public:
	Predicate() = default;

	Predicate(Predicate const& other)
	{
		if (other.hasPredicate()) {
			predicate_.reset(other.predicate_->clone());
		}
	}

	Predicate(Predicate&& other) = default;

	Predicate(detail::Dynamic<Tree> const& pred) : predicate_(pred.clone()) {}

	template <class Pred>
	Predicate(Pred const& pred)
	    : predicate_(std::make_unique<detail::DynamicPredicate<Tree, Pred>>(pred))
	{
	}

	template <class Pred>
	Predicate(Pred&& pred)
	    : predicate_(std::make_unique<detail::DynamicPredicate<Tree, Pred>>(
	          std::forward<Pred>(pred)))
	{
	}

	Predicate& operator=(Predicate const& rhs)
	{
		if (rhs.hasPredicate()) {
			predicate_.reset(rhs.predicate_->clone());
		} else {
			predicate_.reset();
		}
		return *this;
	}

	Predicate& operator=(Predicate&&) = default;

	Predicate& operator=(detail::Dynamic<Tree> const& rhs)
	{
		predicate_.reset(rhs.clone());
		return *this;
	}

	template <class Pred>
	Predicate& operator=(Pred const& pred)
	{
		predicate_ = std::make_unique<detail::DynamicPredicate<Tree, Pred>>(pred);
		return *this;
	}

	template <class Pred>
	Predicate& operator=(Pred&& pred)
	{
		predicate_ =
		    std::make_unique<detail::DynamicPredicate<Tree, Pred>>(std::forward<Pred>(pred));
		return *this;
	}

	Predicate& operator&=(Predicate const& rhs)
	{
		if (rhs.hasPredicate()) {
			if (hasPredicate()) {
				*this = *this && rhs;
			} else {
				predicate_.reset(rhs.predicate_->clone());
			}
		}
		return *this;
	}

	Predicate& operator|=(Predicate const& rhs)
	{
		if (rhs.hasPredicate()) {
			if (hasPredicate()) {
				*this = *this || rhs;
			} else {
				predicate_.reset(rhs.predicate_->clone());
			}
		}
		return *this;
	}

	[[nodiscard]] bool hasPredicate() const { return !!predicate_; }

	[[nodiscard]] bool valueCheck(Tree const& tree, typename Tree::Node node) const
	{
		return !hasPredicate() || predicate_->valueCheck(tree, node);
	}

	[[nodiscard]] bool innerCheck(Tree const& tree, typename Tree::Node node) const
	{
		return !hasPredicate() || predicate_->innerCheck(tree, node);
	}

 private:
	std::unique_ptr<detail::Dynamic<Tree>> predicate_;
};

template <class Tree, class Node>
[[nodiscard]] bool valueCheck(Predicate<Tree> const& p, Tree const& t, Node n)
{
	return p.valueCheck(t, n);
}

template <class Tree, class Node>
[[nodiscard]] bool innerCheck(Predicate<Tree> const& p, Tree const& t, Node n)
{
	return p.innerCheck(t, n);
}
}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_DYNAMIC_HPP