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
#include <ufo/math/vec.hpp>
#include <ufo/utility/create_array.hpp>

// STL
#include <array>
#include <cstddef>

namespace ufo
{
template <TreeType TT>
struct TreeBlock {
	static constexpr std::size_t const BF  = branchingFactor<TT>();
	static constexpr std::size_t const Dim = dimensions<TT>();

	using Code = TreeCode<Dim>;

	Code                             parent_code;
	std::array<TreeIndex::pos_t, BF> children = createArray<BF>(TreeIndex::NULL_POS);

	constexpr TreeBlock()                 = default;
	constexpr TreeBlock(TreeBlock const&) = default;

	constexpr TreeBlock(Code parent_code) : parent_code(parent_code) {}

	constexpr TreeBlock(TreeBlock const& parent, std::size_t offset)
	    : parent_code(parent.parent_code.child(offset))
	{
	}

	constexpr void fill(TreeBlock const& parent, std::size_t offset)
	{
		this->parent_code = parent.parent_code.child(offset);
	}

	/*!
	 * @return The depth of the block.
	 */
	[[nodiscard]] constexpr auto depth() const noexcept(noexcept(parent_code.depth()))
	{
		// One less than the parent
		return parent_code.depth() - 1;
	}
};

// TODO: Implement
template <TreeType TT>
struct TreeBlockCenter {
	static constexpr std::size_t const BF  = branchingFactor<TT>();
	static constexpr std::size_t const Dim = dimensions<TT>();

	using Code  = TreeCode<Dim>;
	using Point = Vec<Dim, float>;

	Code                             parent_code;
	Point                            center;
	std::array<TreeIndex::pos_t, BF> children = createArray<BF>(TreeIndex::NULL_POS);

	constexpr TreeBlockCenter()                       = default;
	constexpr TreeBlockCenter(TreeBlockCenter const&) = default;

	constexpr TreeBlockCenter(Code parent_code) : parent_code(parent_code) {}

	constexpr TreeBlockCenter(TreeBlockCenter const& parent, std::size_t offset)
	    : parent_code(parent.parent_code.child(offset))
	{
	}

	constexpr void fill(TreeBlockCenter const& parent, std::size_t offset)
	{
		this->parent_code = parent.parent_code.child(offset);
	}

	/*!
	 * @return The depth of the block.
	 */
	[[nodiscard]] constexpr auto depth() const noexcept(noexcept(parent_code.depth()))
	{
		// One less than the parent
		return parent_code.depth() - 1;
	}
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_BLOCK_HPP