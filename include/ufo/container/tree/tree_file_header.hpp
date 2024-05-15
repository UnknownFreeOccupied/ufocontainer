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

#ifndef UFO_CONTAINER_TREE_TREE_FILE_HPP
#define UFO_CONTAINER_TREE_TREE_FILE_HPP

// UFO
#include <ufo/container/tree/tree_type.hpp>
#include <ufo/utility/io/buffer.hpp>

// STL
#if __cplusplus >= 202002L
#include <bit>
#endif
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace ufo
{
struct TreeFileHeader {
	using node_size_t = double;
	using depth_t     = std::uint32_t;

	static constexpr std::string_view const FILE_HEADER   = "# UFO tree file";
	static constexpr std::uint8_t const     CURRENT_MAJOR = 1;
	static constexpr std::uint8_t const     CURRENT_MINOR = 0;
	static constexpr std::uint8_t const     CURRENT_PATCH = 0;
	// static constexpr std::uint8_t     IS_LITTLE_ENDIAN =
	//     std::endian::native == std::endian::little;

	std::uint8_t major;
	std::uint8_t minor;
	std::uint8_t patch;
	// std::uint8_t is_little_endian;

	TreeType tree_type;
	bool     compressed;
	double   leaf_size;
	depth_t  depth_levels;

	TreeFileHeader() = default;

	TreeFileHeader(TreeType tree_type, bool compressed, node_size_t leaf_size,
	               depth_t depth_levels)
	    : major(CURRENT_MAJOR)
	    , minor(CURRENT_MINOR)
	    , patch(CURRENT_PATCH)
	    , tree_type(tree_type)
	    , compressed(compressed)
	    , leaf_size(leaf_size)
	    , depth_levels(depth_levels)
	{
	}

	TreeFileHeader(std::filesystem::path const& filename) { read(filename); }

	TreeFileHeader(std::istream& in) { read(in); }

	TreeFileHeader(ReadBuffer& in) { read(in); }

	[[nodiscard]] bool correct(std::filesystem::path const& filename,
	                           bool                         consume = false) const
	{
		std::ifstream file;
		file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		file.imbue(std::locale());
		file.open(filename, std::ios::in | std::ios::binary);
		return correct(file, consume);
	}

	[[nodiscard]] bool correct(std::istream& in, bool consume = false) const
	{
		std::string line;
		auto        pos = in.tellg();
		std::getline(in, line);
		if (!consume) {
			in.seekg(pos);
		}
		return FILE_HEADER == line;
	}

	[[nodiscard]] bool correct(ReadBuffer& in, bool consume = false)
	{
		auto pos    = readIndex();
		auto length = FILE_HEADER.length();
		if (in.readLeft() < length) {
			return false
		}

		std::string line(length, ' ');
		in.read(line.data(), length);
		if (!consume) {
			in.setReadIndex(pos);
		}
		return FILE_HEADER == line;
	}

	void read(std::filesystem::path const& filename)
	{
		std::ifstream file;
		file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		file.imbue(std::locale());
		file.open(filename, std::ios::in | std::ios::binary);
		read(file);
	}

	std::istream& read(std::istream& in)
	{
		if (!correct(in, true)) {
			throw std::runtime_error("Trying to read a none UFO tree type file");
		}

		in.read(reinterpret_cast<char*>(&major), sizeof(major));
		in.read(reinterpret_cast<char*>(&minor), sizeof(minor));
		in.read(reinterpret_cast<char*>(&patch), sizeof(patch));
		// in.read(reinterpret_cast<char*>(&is_little_endian),
		// sizeof(is_little_endian));

		in.read(reinterpret_cast<char*>(&tree_type), sizeof(tree_type));

		std::uint8_t c;
		in.read(reinterpret_cast<char*>(&c), sizeof(c));
		compressed = c & 1U;

		in.read(reinterpret_cast<char*>(&leaf_size), sizeof(leaf_size));
		in.read(reinterpret_cast<char*>(&depth_levels), sizeof(depth_levels));

		return in;
	}

	ReadBuffer& read(ReadBuffer& in)
	{
		if (!correct(in, true)) {
			throw std::runtime_error("Trying to read a none UFO tree type file");
		}

		// Skipping new line
		in.skipRead(1);

		in.read(&major, sizeof(major));
		in.read(&minor, sizeof(minor));
		in.read(&patch, sizeof(patch));
		// in.read(&is_little_endian, sizeof(is_little_endian));

		in.read(&tree_type, sizeof(tree_type));

		std::uint8_t c;
		in.read(&c, sizeof(c));
		compressed = c & 1U;

		in.read(&leaf_size, sizeof(leaf_size));
		in.read(&depth_levels, sizeof(depth_levels));

		return in;
	}

	void write(std::filesystem::path const& filename) const
	{
		std::ofstream file;
		file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		file.imbue(std::locale());
		file.open(filename, std::ios::out | std::ios::binary);
		write(file);
	}

	std::ostream& write(std::ostream& out) const
	{
		out << FILE_HEADER << '\n';
		out.write(reinterpret_cast<char const*>(&major), sizeof(major));
		out.write(reinterpret_cast<char const*>(&minor), sizeof(minor));
		out.write(reinterpret_cast<char const*>(&patch), sizeof(patch));
		// out.write(reinterpret_cast<char const*>(&is_little_endian),
		//           sizeof(is_little_endian));

		out.write(reinterpret_cast<char const*>(&tree_type), sizeof(tree_type));

		std::uint8_t c = compressed ? std::uint8_t(1) : std::uint8_t(0);

		out.write(reinterpret_cast<char const*>(&c), sizeof(c));
		out.write(reinterpret_cast<char const*>(&leaf_size), sizeof(leaf_size));
		out.write(reinterpret_cast<char const*>(&depth_levels), sizeof(depth_levels));

		return out;
	}

	WriteBuffer& write(WriteBuffer& out) const
	{
		out.write(FILE_HEADER.data(), FILE_HEADER.length());
		char nl = '\n';
		out.write(&nl, sizeof(nl));
		out.write(&major, sizeof(major));
		out.write(&minor, sizeof(minor));
		out.write(&patch, sizeof(patch));
		// out.write(&is_little_endian, sizeof(is_little_endian));

		out.write(&tree_type, sizeof(tree_type));

		std::uint8_t c = compressed ? std::uint8_t(1) : std::uint8_t(0);

		out.write(&c, sizeof(c));
		out.write(&leaf_size, sizeof(leaf_size));
		out.write(&depth_levels, sizeof(depth_levels));

		return out;
	}
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_TREE_FILE_HPP