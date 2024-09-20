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

#ifndef UFO_CONTAINER_TREE_CODE_HPP
#define UFO_CONTAINER_TREE_CODE_HPP

// UFO
#include <ufo/container/tree/key.hpp>
#include <ufo/math/utility.hpp>
#include <ufo/morton/morton.hpp>

// STL
#include <cstddef>
#include <cstdint>
#include <functional>

namespace ufo
{
template <std::size_t Dim>
class TreeCode
{
 public:
	using code_t    = std::uint64_t;
	using key_t     = typename TreeKey<Dim>::key_t;
	using depth_t   = code_t;
	using size_type = std::size_t;

 private:
	static constexpr code_t const OFFSET_MASK = ~(static_cast<code_t>(-1) << Dim);

 public:
	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	constexpr TreeCode() noexcept                = default;
	constexpr TreeCode(TreeCode const&) noexcept = default;

	constexpr TreeCode(code_t code, depth_t depth) : code_(code), depth_(depth)
	{
		assert(valid());
	}

	constexpr explicit TreeCode(code_t code) : TreeCode(code, 0) {}

	constexpr explicit TreeCode(TreeKey<Dim> key)
	    : TreeCode(Morton<Dim>::encode(key) << (Dim * key.depth()), key.depth())
	{
	}

	[[nodiscard]] static constexpr TreeCode invalid() noexcept
	{
		TreeCode code;
		code.invalidate();
		return code;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	constexpr TreeCode& operator=(TreeCode const&) noexcept = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Conversion operator                                 |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr explicit operator TreeKey<Dim>() const noexcept
	{
		return TreeKey<Dim>(Morton<Dim>::decode(code_ >> (Dim * depth_)), depth_);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                   Element access                                    |
	|                                                                                     |
	**************************************************************************************/

	// [[nodiscard]] constexpr key_t& operator[](size_type pos) noexcept
	// {
	// 	assert(size() > pos);
	// 	return key[pos];
	// }

	// [[nodiscard]] constexpr key_t const& operator[](size_type pos) const noexcept
	// {
	// 	assert(size() > pos);
	// 	return key[pos];
	// }

	/**************************************************************************************
	|                                                                                     |
	|                                      Capacity                                       |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] static constexpr size_type size() noexcept { return Dim; }

	/**************************************************************************************
	|                                                                                     |
	|                                     Operations                                      |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] constexpr bool valid() const noexcept
	{
		bool a = maxDepth() >= depth();
		// Check if unused upper bits are set to zero
		bool b = 0 == (code() >> (Dim * maxDepth()));
		// Check if unused lower bits are set to zero
		bool c = code() == ((code() >> (Dim * depth())) << (Dim * depth()));

		return a && b && c;
	}

	constexpr void invalidate() noexcept { depth_ = maxDepth() + 1; }

	[[nodiscard]] static constexpr std::size_t branchingFactor() noexcept
	{
		return ipow(std::size_t(2), Dim);
	}

	[[nodiscard]] constexpr code_t code() const noexcept { return code_; }

	[[nodiscard]] static constexpr depth_t maxDepth() noexcept
	{
		// Root does not need any bits (since it only has one child).
		// Shifting with `Dim * maxDepth()` causes problems when
		// `std::numeric_limits<code_t>::digits <= Dim * maxDepth()` because you are trying to
		// shift more bits than are allowed and has a well-defined behaviour in the C++.
		// Therefore, we have this check otherwise would cause "shift count overflow".
		if constexpr (0 == std::numeric_limits<code_t>::digits % size()) {
			// All the time you have to leave the space.
			// Meaning, we ensure some bits can be "used" for the root.
			return (std::numeric_limits<code_t>::digits / size()) - 1;
		} else {
			// Thanks, you have left the space.
			// Meaning, we have extra bits that can be "used" for the root.
			return std::numeric_limits<code_t>::digits / size();
		}
	}

	[[nodiscard]] constexpr depth_t depth() const noexcept { return depth_; }

	[[nodiscard]] static constexpr bool equalAtDepth(TreeCode const& lhs,
	                                                 TreeCode const& rhs,
	                                                 depth_t         depth) noexcept
	{
		assert(maxDepth() >= depth);
		assert(lhs.valid() && rhs.valid());
		return (lhs.code_ >> (Dim * depth)) == (rhs.code_ >> (Dim * depth));
	}

	[[nodiscard]] static constexpr depth_t depthWhereEqual(TreeCode const& lhs,
	                                                       TreeCode const& rhs) noexcept
	{
		assert(lhs.valid() && rhs.valid());
		auto   depth = std::max(lhs.depth(), rhs.depth());
		code_t code  = (lhs.code_ ^ rhs.code_) >> (Dim * depth);
		for (; code; code >>= Dim, ++depth) {
		}
		return depth;
	}

	[[nodiscard]] constexpr TreeCode toDepth(depth_t depth) const
	{
		assert(maxDepth() >= depth);
		return TreeCode((code_ >> (Dim * depth)) << (Dim * depth), depth);
	}

	constexpr void setDepth(depth_t depth)
	{
		assert(maxDepth() >= depth);
		code_  = (code_ >> (Dim * depth)) << (Dim * depth);
		depth_ = depth;
	}

	/*!
	 * @brief Get the offset at a specific depth for this code.
	 *
	 * @param depth The depth the index is requested for.
	 * @return The offset at the specified depth.
	 */
	[[nodiscard]] constexpr code_t offset(depth_t depth) const
	{
		assert(maxDepth() >= depth);
		return (code_ >> (Dim * depth)) & OFFSET_MASK;
	}

	[[nodiscard]] constexpr TreeCode parent() const { return toDepth(depth_ + 1); }

	/*!
	 * @brief Get the code of a specific child of this code
	 *
	 * @param idx The index of the child
	 * @return TreeCode The child code
	 */
	[[nodiscard]] constexpr TreeCode child(std::size_t idx) const
	{
		assert(0 < depth_);
		assert(branchingFactor() > idx);
		depth_t depth = depth_ - 1;
		return TreeCode(code_ | (static_cast<code_t>(idx) << (Dim * depth)), depth);
	}
	/*!
	 * @brief Get the code of the firstborn child of this code (same as child(0)).
	 *
	 * @return TreeCode The firstborn child code
	 */
	[[nodiscard]] constexpr TreeCode firstborn() const
	{
		assert(0 < depth_);
		return TreeCode(code_, depth_ - 1);
	}

	/*!
	 * @brief Get the code of a specific sibling of this code
	 *
	 * @param idx The index of the sibling
	 * @return TreeCode The sibling code
	 */
	[[nodiscard]] constexpr TreeCode sibling(std::size_t idx) const
	{
		assert(branchingFactor() > idx);
		auto s = Dim * depth_;
		return TreeCode(((code_ >> s) << s) | (static_cast<code_t>(idx) << s), depth_);
	}

	/*!
	 * @brief Get the code of a specific sibling of this firstborn code
	 *
	 * @note Should only be called on firstborn codes (i.e., `offset(depth())` should be
	 * `0`)
	 *
	 * @param idx The index of the sibling
	 * @return TreeCode The sibling code
	 */
	[[nodiscard]] constexpr TreeCode firstbornSibling(std::size_t idx) const
	{
		assert(0 == offset(depth_));
		assert(branchingFactor() > idx);
		return TreeCode(code_ | (static_cast<code_t>(idx) << (Dim * depth_)), depth_);
	}

	void swap(TreeCode& other) noexcept
	{
		std::swap(code_, other.code_);
		std::swap(depth_, other.depth_);
	}

 private:
	code_t  code_{};
	depth_t depth_{};
};

using BinaryCode = TreeCode<1>;
using QuadCode   = TreeCode<2>;
using OctCode    = TreeCode<3>;
using HexCode    = TreeCode<4>;

template <std::size_t Dim>
std::ostream& operator<<(std::ostream& os, TreeCode<Dim> const& code)
{
	return os << "code: " << code.code() << " depth: " << code.depth();
}

template <std::size_t Dim>
void swap(TreeCode<Dim>& lhs, TreeCode<Dim>& rhs) noexcept
{
	lhs.swap(rhs);
}

/**************************************************************************************
|                                                                                     |
|                                       Compare                                       |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim>
[[nodiscard]] constexpr bool operator==(TreeCode<Dim> const& lhs,
                                        TreeCode<Dim> const& rhs) noexcept
{
	return lhs.code() == rhs.code() && lhs.depth() == rhs.depth();
}

template <std::size_t Dim>
[[nodiscard]] constexpr bool operator!=(TreeCode<Dim> const& lhs,
                                        TreeCode<Dim> const& rhs) noexcept
{
	return !(lhs == rhs);
}

template <std::size_t Dim>
[[nodiscard]] constexpr bool operator<(TreeCode<Dim> const& lhs,
                                       TreeCode<Dim> const& rhs) noexcept
{
	return lhs.code() < rhs.code();
}

template <std::size_t Dim>
[[nodiscard]] constexpr bool operator<=(TreeCode<Dim> const& lhs,
                                        TreeCode<Dim> const& rhs) noexcept
{
	return lhs.code() <= rhs.code();
}

template <std::size_t Dim>
[[nodiscard]] constexpr bool operator>(TreeCode<Dim> const& lhs,
                                       TreeCode<Dim> const& rhs) noexcept
{
	return lhs.code() > rhs.code();
}

template <std::size_t Dim>
[[nodiscard]] constexpr bool operator>=(TreeCode<Dim> const& lhs,
                                        TreeCode<Dim> const& rhs) noexcept
{
	return lhs.code() >= rhs.code();
}
}  // namespace ufo

namespace std
{
template <std::size_t Dim>
struct hash<ufo::TreeCode<Dim>> {
	std::size_t operator()(ufo::TreeCode<Dim> code) const
	{
		return static_cast<std::size_t>(code.code());
	}
};
}  // namespace std

#endif  // UFO_CONTAINER_TREE_CODE_HPP