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

#ifndef UFO_CONTAINER_TREE_BLOCK_HPP
#define UFO_CONTAINER_TREE_BLOCK_HPP

// UFO
#include <ufo/container/tree/code.hpp>
#include <ufo/container/tree/index.hpp>
#include <ufo/geometry/shape/aabb.hpp>
#include <ufo/math/vec.hpp>
#include <ufo/utility/create_array.hpp>

// STL
#include <array>
#include <atomic>
#include <cstddef>

namespace ufo
{
template <std::size_t Dim, std::size_t BF, bool WithCenter = false>
struct TreeBlock {
	static constexpr bool const HasCenter = WithCenter;

	using Code     = TreeCode<Dim>;
	using length_t = double;
	using Point    = Vec<Dim, float>;

	std::array<std::atomic<TreeIndex::pos_t>, BF> children;

	constexpr TreeBlock()
	{
		for (std::size_t i{}; BF > i; ++i) {
			children[i].store(TreeIndex::NULL_POS, std::memory_order_relaxed);
		}
	}

	constexpr TreeBlock(TreeBlock const& other)
	    : parent_block_(other.parent_block_), code_(other.code_)
	{
		for (std::size_t i{}; BF > i; ++i) {
			children[i].store(other.children[i].load(std::memory_order_relaxed),
			                  std::memory_order_relaxed);
		}
	}

	constexpr TreeBlock(TreeIndex::pos_t parent_block, Code code, Point /* center */,
	                    length_t /* half_length */)
	    : parent_block_(parent_block), code_(code)
	{
		for (std::size_t i{}; BF > i; ++i) {
			children[i].store(TreeIndex::NULL_POS, std::memory_order_relaxed);
		}
	}

	constexpr TreeBlock& operator=(TreeBlock const& rhs)
	{
		for (std::size_t i{}; BF > i; ++i) {
			children[i].store(rhs.children[i].load(std::memory_order_relaxed),
			                  std::memory_order_relaxed);
		}
		parent_block_ = rhs.parent_block_;
		code_         = rhs.code_;
		return *this;
	}

	constexpr void fill(TreeIndex::pos_t parent_block, TreeBlock const& parent,
	                    std::size_t offset, length_t /* half_length */)
	{
		parent_block_ = parent_block;
		code_         = parent.code(offset).firstborn();
	}

	[[nodiscard]] constexpr TreeIndex::pos_t parentBlock() const { return parent_block_; }

	[[nodiscard]] constexpr TreeIndex parent() const
	{
		return TreeIndex(parent_block_, code_.offset(code_.depth() + 1));
	}

	[[nodiscard]] constexpr Code code(std::size_t idx) const
	{
		return code_.firstbornSibling(idx);
	}

	[[nodiscard]] constexpr Code parentCode() const { return code_.parent(); }

	/*!
	 * @return The depth of the block.
	 */
	[[nodiscard]] constexpr auto depth() const noexcept(noexcept(code_.depth()))
	{
		return code_.depth();
	}

	[[nodiscard]] constexpr bool valid() const { return code_.valid(); }

	template <std::size_t Dim2, std::size_t BF2, bool WithCenter2>
	friend bool operator==(TreeBlock<Dim2, BF2, WithCenter2> const& lhs,
	                       TreeBlock<Dim2, BF2, WithCenter2> const& rhs);

	template <std::size_t Dim2, std::size_t BF2, bool WithCenter2>
	friend bool operator!=(TreeBlock<Dim2, BF2, WithCenter2> const& lhs,
	                       TreeBlock<Dim2, BF2, WithCenter2> const& rhs);

 private:
	// Position of the parent block
	TreeIndex::pos_t parent_block_ = TreeIndex::NULL_POS;
	// FIXME: Adding padding for GPU
	float _pad0;
	// Code to the first node of the block
	Code code_ = Code::invalid();
};

template <std::size_t Dim, std::size_t BF>
struct TreeBlock<Dim, BF, true> : TreeBlock<Dim, BF, false> {
	using Base = TreeBlock<Dim, BF, false>;

	static constexpr bool const HasCenter = true;

	using Code     = TreeCode<Dim>;
	using Point    = Vec<Dim, float>;
	using length_t = double;

	constexpr TreeBlock() = default;

	constexpr TreeBlock(TreeIndex::pos_t parent_block, Code code, Point center,
	                    length_t half_length)
	    : Base(parent_block, code, center, half_length)
	{
	}

	constexpr void fill(TreeIndex::pos_t parent_block, TreeBlock const& parent,
	                    std::size_t offset, length_t half_length)
	{
		Base::fill(parent_block, static_cast<Base const&>(parent), offset, half_length);

		for (std::size_t i{}; Point::size() > i; ++i) {
			center_[i] = (offset & std::size_t(1u << i)) ? parent.center_[i] + half_length
			                                             : parent.center_[i] - half_length;
		}
	}

	[[nodiscard]] Point center(std::size_t offset, length_t half_length) const
	{
		Point ret;

		for (std::size_t i{}; Point::size() > i; ++i) {
			ret[i] = (offset & std::size_t(1u << i)) ? center_[i] + half_length
			                                         : center_[i] - half_length;
		}

		return ret;
	}

	[[nodiscard]] float centerAxis(std::size_t offset, length_t half_length,
	                               std::size_t axis) const
	{
		assert(Dim > axis);

		return (offset & std::size_t(1u << axis)) ? center_[axis] + half_length
		                                          : center_[axis] - half_length;
	}

	template <std::size_t Dim2, std::size_t BF2>
	friend bool operator==(TreeBlock<Dim2, BF2, true> const& lhs,
	                       TreeBlock<Dim2, BF2, true> const& rhs);

	template <std::size_t Dim2, std::size_t BF2>
	friend bool operator!=(TreeBlock<Dim2, BF2, true> const& lhs,
	                       TreeBlock<Dim2, BF2, true> const& rhs);

 private:
	Point center_;
};

template <std::size_t Dim, std::size_t BF, bool WithCenter>
bool operator==(TreeBlock<Dim, BF, WithCenter> const& lhs,
                TreeBlock<Dim, BF, WithCenter> const& rhs)
{
	bool center = true;
	if constexpr (WithCenter) {
		center = lhs.center_ == rhs.center_;
	}

	return center && lhs.children == rhs.children && lhs.code_ == rhs.code_;
}

template <std::size_t Dim, std::size_t BF, bool WithCenter>
bool operator!=(TreeBlock<Dim, BF, WithCenter> const& lhs,
                TreeBlock<Dim, BF, WithCenter> const& rhs)
{
	return !(lhs == rhs);
}
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_BLOCK_HPP