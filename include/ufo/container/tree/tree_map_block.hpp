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

#ifndef UFO_CONTAINER_TREE_MAP_BLOCK_HPP
#define UFO_CONTAINER_TREE_MAP_BLOCK_HPP

// UFO
#include <ufo/container/tree/tree_index.hpp>
#include <ufo/container/tree/tree_map_value.hpp>
#include <ufo/container/tree/tree_type.hpp>
#include <ufo/container/tree/tree_types.hpp>
#include <ufo/utility/create_array.hpp>

// STL
#include <array>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

namespace ufo
{
template <class T, TreeType TT>
struct TreeMapBlock {
	using Code                     = typename TreeTypes<TT>::Code;
	using Point                    = typename TreeTypes<TT>::Point;
	static constexpr auto const BF = branchingFactor(TT);

	Code                             parent_code;
	std::array<TreeIndex::pos_t, BF> children = createArray<BF>(TreeIndex::NULL_POS);
	std::unique_ptr<std::array<std::vector<std::pair<Point, T>>, BF>> value = nullptr;

	constexpr TreeMapBlock() = default;

	constexpr TreeMapBlock(TreeMapBlock const& parent, std::size_t offset)
	    : parent_code(parent.parent_code.child(offset))
	{
		fillValue(parent, offset);
	}

	constexpr void fill(TreeMapBlock const& parent, std::size_t offset)
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

#endif  // UFO_CONTAINER_TREE_MAP_BLOCK_HPP