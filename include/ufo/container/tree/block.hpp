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
#include <ufo/container/tree/type.hpp>
#include <ufo/geometry/shape/aabb.hpp>
#include <ufo/math/vec.hpp>
#include <ufo/utility/create_array.hpp>

// STL
#include <array>
#include <cstddef>

namespace ufo
{
template <TreeType TT, bool WithCenter = false>
struct TreeBlock {
	static constexpr TreeType const tree_type = TT;

	static constexpr std::size_t const BF  = branchingFactor<TT>();
	static constexpr std::size_t const Dim = dimensions<TT>();

	static constexpr bool const HasCenter = WithCenter;

	using Code     = TreeCode<Dim>;
	using length_t = double;
	using Point    = Vec<Dim, float>;

	std::array<TreeIndex::pos_t, BF> children = createArray<BF>(TreeIndex::NULL_POS);

	constexpr TreeBlock()                 = default;
	constexpr TreeBlock(TreeBlock const&) = default;

	constexpr TreeBlock(Code code, Point /* center */, length_t /* half_length */)
	    : code_(code)
	{
	}

	constexpr TreeBlock(TreeBlock const& parent, std::size_t offset,
	                    length_t /* half_length */)
	    : code_(parent.code(offset).firstborn())
	{
	}

	constexpr void fill(TreeBlock const& parent, std::size_t offset,
	                    length_t /* half_length */)
	{
		code_ = parent.code(offset).firstborn();
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

 private:
	// Code to the first node of the block
	Code code_ = Code::invalid();
};

template <TreeType TT>
struct TreeBlock<TT, true> {
	static constexpr TreeType const tree_type = TT;

	static constexpr std::size_t const BF  = branchingFactor<TT>();
	static constexpr std::size_t const Dim = dimensions<TT>();

	static constexpr bool const HasCenter = true;

	using Code     = TreeCode<Dim>;
	using Point    = Vec<Dim, float>;
	using length_t = double;

	// Code to the first node of the block
	Code                             code;
	Point                            center;
	std::array<TreeIndex::pos_t, BF> children = createArray<BF>(TreeIndex::NULL_POS);

	constexpr TreeBlock()                 = default;
	constexpr TreeBlock(TreeBlock const&) = default;

	constexpr TreeBlock(Code code, Point center, length_t /* half_length */)
	    : code(code), center(center)
	{
	}

	constexpr TreeBlock(TreeBlock const& parent, std::size_t offset, length_t half_length)
	    : code(parent.code.child(offset))
	{
		for (std::size_t i{}; Point::size() > i; ++i) {
			center[i] = offset & std::size_t(1u << i) ? parent.center[i] + half_length
			                                          : parent.center[i] - half_length;
		}
	}

	constexpr void fill(TreeBlock const& parent, std::size_t offset, length_t half_length)
	{
		code = parent.code.child(offset);

		for (std::size_t i{}; Point::size() > i; ++i) {
			center[i] = offset & std::size_t(1u << i) ? parent.center[i] + half_length
			                                          : parent.center[i] - half_length;
		}
	}

	/*!
	 * @return The depth of the block.
	 */
	[[nodiscard]] constexpr auto depth() const noexcept(noexcept(code.depth()))
	{
		// One less than the parent
		return code.depth() - 1;
	}
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_BLOCK_HPP