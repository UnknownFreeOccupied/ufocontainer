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

#ifndef UFO_CONTAINER_TREE_CONTAINER_HPP
#define UFO_CONTAINER_TREE_CONTAINER_HPP

// UFO
#include <ufo/container/tree/container_iterator.hpp>
#include <ufo/utility/execution.hpp>

// STL
#include <atomic>
#include <cstddef>
#include <cstring>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <type_traits>

namespace ufo
{
// NOTE: Can use placement new to seperate allocation of memory and construction of
// objects, making it possible to release the requirement of T being default constructable

template <class T, std::size_t NumBuckets = std::size_t(1) << (32 - 20),
          std::size_t NumBlocksPerBucket = std::size_t(1) << 20>
class TreeContainer
{
 private:
	using Bucket = std::unique_ptr<T[]>;

 public:
	// types
	using value_type      = T;
	using pointer         = value_type*;
	using const_pointer   = value_type const*;
	using reference       = value_type&;
	using const_reference = value_type const&;
	using size_type       = std::size_t;
	using difference_type = std::ptrdiff_t;

	class const_iterator
	{
		friend TreeContainer;

	 public:
		// Tags
		using iterator_category = std::random_access_iterator_tag;
		using difference_type   = std::ptrdiff_t;
		using value_type        = typename TreeContainer::value_type;
		using pointer           = typename TreeContainer::pointer;
		using const_pointer     = typename TreeContainer::const_pointer;
		using reference         = typename TreeContainer::reference;
		using const_reference   = typename TreeContainer::const_reference;

		const_iterator() = default;

		const_iterator& operator++()
		{
			++idx_;
			return *this;
		}

		const_iterator& operator--()
		{
			--idx_;
			return *this;
		}

		const_iterator operator++(int)
		{
			const_iterator tmp(*this);
			++idx_;
			return tmp;
		}

		const_iterator operator--(int)
		{
			const_iterator tmp(*this);
			--idx_;
			return tmp;
		}

		const_iterator operator+(difference_type n)
		{
			const_iterator tmp(*this);
			tmp.idx_ += n;
			return tmp;
		}

		const_iterator operator-(difference_type n)
		{
			const_iterator tmp(*this);
			tmp.idx_ -= n;
			return tmp;
		}

		const_iterator& operator+=(difference_type n)
		{
			idx_ += n;
			return *this;
		}

		const_iterator& operator-=(difference_type n)
		{
			idx_ -= n;
			return *this;
		}

		[[nodiscard]] const_reference operator[](difference_type pos) const
		{
			return (*data_)[idx_ + pos];
		}

		[[nodiscard]] const_reference operator*() const { return operator[](0); }

		[[nodiscard]] const_pointer operator->() const { return &(operator[](0)); }

		difference_type operator-(const_iterator const& rhs) const { return idx_ - rhs.idx_; }

		[[nodiscard]] bool operator==(const_iterator other) const
		{
			return idx_ == other.idx_ && data_ == other.data_;
		}

		[[nodiscard]] bool operator!=(const_iterator other) const
		{
			return !(*this == other);
		}

		[[nodiscard]] bool operator<(const_iterator other) const { return idx_ < other.idx_; }

		[[nodiscard]] bool operator<=(const_iterator other) const
		{
			return idx_ <= other.idx_;
		}

		[[nodiscard]] bool operator>(const_iterator other) const { return idx_ > other.idx_; }

		[[nodiscard]] bool operator>=(const_iterator other) const
		{
			return idx_ >= other.idx_;
		}

	 protected:
		const_iterator(TreeContainer* data, size_type idx) : data_(data), idx_(idx) {}

	 protected:
		TreeContainer* data_{};
		size_type      idx_{};
	};

	class iterator : public const_iterator
	{
		friend TreeContainer;

	 public:
		iterator() = default;

		iterator& operator++()
		{
			++this->idx_;
			return *this;
		}

		iterator& operator--()
		{
			--this->idx_;
			return *this;
		}

		iterator operator++(int)
		{
			iterator tmp(*this);
			++this->idx_;
			return tmp;
		}

		iterator operator--(int)
		{
			iterator tmp(*this);
			--this->idx_;
			return tmp;
		}

		iterator operator+(difference_type n)
		{
			iterator tmp(*this);
			tmp.idx_ += n;
			return tmp;
		}

		iterator operator-(difference_type n)
		{
			iterator tmp(*this);
			tmp.idx_ -= n;
			return tmp;
		}

		iterator& operator+=(difference_type n)
		{
			this->idx_ += n;
			return *this;
		}

		iterator& operator-=(difference_type n)
		{
			this->idx_ -= n;
			return *this;
		}

		[[nodiscard]] reference operator[](difference_type pos)
		{
			return (*this->data_)[this->idx_ + pos];
		}

		[[nodiscard]] reference operator*() { return operator[](0); }

		[[nodiscard]] pointer operator->() { return &(operator[](0)); }

		difference_type operator-(iterator const& rhs) const { return this->idx_ - rhs.idx_; }

		[[nodiscard]] bool operator==(iterator other) const
		{
			return this->idx_ == other.idx_ && this->data_ == other.data_;
		}

		[[nodiscard]] bool operator!=(iterator other) const { return !(*this == other); }

		[[nodiscard]] bool operator<(iterator other) const { return this->idx_ < other.idx_; }

		[[nodiscard]] bool operator<=(iterator other) const
		{
			return this->idx_ <= other.idx_;
		}

		[[nodiscard]] bool operator>(iterator other) const { return this->idx_ > other.idx_; }

		[[nodiscard]] bool operator>=(iterator other) const
		{
			return this->idx_ >= other.idx_;
		}

	 protected:
		iterator(TreeContainer* data, size_type idx) : const_iterator(data, idx) {}
	};

	using reverse_iterator       = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	// construct/copy/destroy
	// constexpr vector() noexcept(noexcept(Allocator())) : vector(Allocator()) {}
	// constexpr explicit vector(const Allocator&) noexcept;
	// constexpr explicit vector(size_type n, const Allocator& = Allocator());
	// constexpr vector(size_type n, const T& value, const Allocator& = Allocator());
	// template <class InputIt>
	// constexpr vector(InputIt first, InputIt last, const Allocator& = Allocator());
	// template <__container_compatible_range<T> R>
	// constexpr vector(from_range_t, R&& rg, const Allocator& = Allocator());
	// constexpr vector(const vector& x);
	// constexpr vector(vector&&) noexcept;
	// constexpr vector(const vector&, const type_identity_t<Allocator>&);
	// constexpr vector(vector&&, const type_identity_t<Allocator>&);
	// constexpr vector& operator=(const vector& x);
	// constexpr vector& operator=(vector&& x) noexcept(
	//     allocator_traits<Allocator>::propagate_on_container_move_assignment::value ||
	//     allocator_traits<Allocator>::is_always_equal::value);

	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	TreeContainer() = default;

	TreeContainer(TreeContainer const& other) : size_(other.size_)
	{
		for (std::size_t i{}; other.num_buckets() > i; ++i) {
			createBucket(buckets_[i]);
			std::copy(other.buckets_[i].get(), other.buckets_[i].get() + NumBlocksPerBucket,
			          buckets_[i].get());
		}
	}

	TreeContainer(TreeContainer&&) = default;

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	constexpr TreeContainer& operator=(TreeContainer const& rhs)
	{
		size_ = rhs.size_;
		for (std::size_t i{}; rhs.num_buckets() > i; ++i) {
			createBucket(buckets_[i]);
			std::copy(rhs.buckets_[i].get(), rhs.buckets_[i].get() + NumBlocksPerBucket,
			          buckets_[i].get());
		}
		return *this;
	}

	constexpr TreeContainer& operator=(TreeContainer&&) = default;

	// iterators
	iterator begin() noexcept { return iterator(this, 0); }

	const_iterator begin() const noexcept
	{
		return const_iterator(const_cast<TreeContainer*>(this), 0);
	}

	iterator end() noexcept { return iterator(this, size_); }

	const_iterator end() const noexcept
	{
		return const_iterator(const_cast<TreeContainer*>(this), size_);
	}

	reverse_iterator rbegin() noexcept { return std::make_reverse_iterator(end()); }

	const_reverse_iterator rbegin() const noexcept
	{
		return std::make_reverse_iterator(end());
	}

	reverse_iterator rend() noexcept { return std::make_reverse_iterator(begin()); }

	const_reverse_iterator rend() const noexcept
	{
		return std::make_reverse_iterator(begin());
	}

	const_iterator cbegin() const noexcept { return begin(); }

	const_iterator cend() const noexcept { return end(); }

	const_reverse_iterator crbegin() const noexcept { return rbegin(); }

	const_reverse_iterator crend() const noexcept { return rend(); }

	// capacity
	[[nodiscard]] constexpr bool empty() const noexcept { return 0 == size_; }

	[[nodiscard]] constexpr size_type size() const noexcept { return size_; }

	[[nodiscard]] size_type cap() const noexcept
	{
		return cap_.load(std::memory_order_acquire);
	}

	[[nodiscard]] constexpr size_type max_size() const noexcept
	{
		return NumBuckets * NumBlocksPerBucket;
	}

	// TODO: Make private
	void setSize(size_type size) { size_ = size; }

	void reserve(size_type cap)
	{
		size_type last = bucket_pos(cap - 1);
		for (size_type i{}; last >= i; ++i) {
			auto& bucket = buckets_[i];
			if (nullptr == bucket) {
				bucket = std::make_unique<T[]>(NumBlocksPerBucket);
			}
		}
		cap_ = (last + 1) * NumBlocksPerBucket;
	}

	constexpr void resize(size_type sz)
	{
		if (0 == sz) {
			clear();
			return;
		}

		// TODO: Implement correctly
		// if (size_ > sz) {
		// 	size_type cur_last = bucket_pos(size_ - 1);
		// 	size_type new_last = bucket_pos(sz - 1);
		// 	for (size_type i = new_last + 1; cur_last >= i; ++i) {
		// 		buckets_.reset();
		// 	}
		// 	auto b    = buckets_[new_last];
		// 	auto last = cur_last == new_last ? block_pos(size_) : NumBlocksPerBucket;
		// 	for (size_type i = block_pos(sz); last != i; ++i) {
		// 		b[i] = T();
		// 	}
		// } else if (size_ < sz) {
		// 	// TODO: Optimize
		// 	while (size_ != sz) {
		// 		emplace_back();
		// 	}
		// }
		// size_ = sz;
	}

	constexpr void resize(size_type sz, T const& c)
	{
		if (0 == sz) {
			clear();
			return;
		}

		// TODO: Implement correctly
		if (size_ > sz) {
			size_type cur_last = bucket_pos(size_ - 1);
			size_type new_last = bucket_pos(sz - 1);
			for (size_type i = new_last + 1; cur_last >= i; ++i) {
				buckets_.reset();
			}
			auto b    = buckets_[new_last];
			auto last = cur_last == new_last ? block_pos(size_) : NumBlocksPerBucket;
			for (size_type i = block_pos(sz); last != i; ++i) {
				b[i] = T();
			}
		} else if (size_ < sz) {
			// TODO: Optimize
			while (size_ != sz) {
				push_back(c);
			}
		}
		size_ = sz;
	}

	// element access
	reference operator[](size_type n) { return block(n); }

	const_reference operator[](size_type n) const { return block(n); }

	const_reference at(size_type n) const
	{
		if (size() <= n) {
			// TODO: Add error message
			throw std::out_of_range("Out of range");
		}
		return block(n);
	}

	reference at(size_type n)
	{
		if (size() <= n) {
			// TODO: Add error message
			throw std::out_of_range("Out of range");
		}
		return block(n);
	}

	reference front() { return operator[](0); }

	const_reference front() const { return operator[](0); }

	reference back() { return operator[](size() - 1); }

	const_reference back() const { return operator[](size() - 1); }

	// modifiers
	template <class... Args>
	reference emplace_back(Args&&... args)
	{
		size_type s = size_;
		Bucket&   b = bucket(s);
		if (nullptr == b) {
			createBucket(b);
		}
		block(b, s) = T(std::forward<Args>(args)...);
		++size_;
		// size_ = NumBlocksPerBucket;
		return block(b, s);
	}

	void push_back(T const& x)
	{
		size_type s = size_;
		Bucket&   b = bucket(s);
		if (nullptr == b) {
			createBucket(b);
		}
		block(b, s) = x;
		++size_;
	}

	void push_back(T&& x)
	{
		size_type s = size_;
		Bucket&   b = bucket(s);
		if (nullptr == b) {
			createBucket(b);
		}
		block(b, s) = std::move(x);
		++size_;
	}

	void pop_back()
	{
		--size_;
		if (bucket_pos(size_) != bucket_pos(size_ - 1)) {
			// TODO: Delete bucket
		}
	}

	friend void swap(TreeContainer& lhs, TreeContainer& rhs)
	{
		std::swap(lhs.buckets_, rhs.buckets_);
		std::swap(lhs.size_, rhs.size_);
	}

	void clear()
	{
		// TODO: Look at
		// size_type last = bucket_pos(size_);
		// for (size_type i{}; last >= i; ++i) {
		// 	buckets_[i].reset();
		// }
		size_ = 0;
	}

	[[nodiscard]] constexpr size_type serialized_size() const
	{
		return num_buckets() * NumBlocksPerBucket * sizeof(T);
	}

	template <class OutputIt>
	OutputIt copy(OutputIt d_first) const
	{
		return copy(execution::seq, d_first);
	}

	template <
	    class ExecutionPolicy, class ForwardIt,
	    std::enable_if_t<execution::is_execution_policy_v<ExecutionPolicy>, bool> = true>
	ForwardIt copy(ExecutionPolicy&& policy, ForwardIt d_first) const
	{
		if (empty()) {
			return d_first;
		}

		if constexpr (execution::is_seq_v<ExecutionPolicy>) {
			auto n = num_buckets() - 1;
			for (std::size_t i{}; n != i; ++i) {
				d_first = std::copy(std::begin(buckets_[i]), std::end(buckets_[i]), d_first);
			}

			// Handle last bucket
			return std::copy(std::begin(buckets_[n]),
			                 std::next(std::begin(buckets_[n]), block_pos(size_)), d_first);
		} else if constexpr (execution::is_tbb_v<ExecutionPolicy>) {
			auto n = num_buckets() - 1;
			for (std::size_t i{}; n != i; ++i) {
				d_first =
				    std::copy(policy, std::begin(buckets_[i]), std::end(buckets_[i]), d_first);
			}

			// Handle last bucket
			return std::copy(policy, std::begin(buckets_[n]),
			                 std::next(std::begin(buckets_[n]), block_pos(size_)), d_first);
		} else if constexpr (execution::is_omp_v<ExecutionPolicy>) {
			// TODO: Implement
		} else {
			// TODO: Error
		}
	}

 private:
	[[nodiscard]] constexpr size_type num_buckets() const noexcept
	{
		return empty() ? 0 : bucket_pos(size_ - 1) + 1;
	}

	[[nodiscard]] constexpr size_type bucket_pos(size_type n) const noexcept
	{
		// FIXME: Is this correct?
		return n / NumBlocksPerBucket;
		// return n >> Exp;
	}

	[[nodiscard]] constexpr size_type block_pos(size_type n) const noexcept
	{
		// FIXME: Is this correct?
		return n % NumBlocksPerBucket;
		// return n & BlockMask;
	}

	[[nodiscard]] Bucket& bucket(size_type n) { return buckets_[bucket_pos(n)]; }

	[[nodiscard]] Bucket const& bucket(size_type n) const
	{
		return buckets_[bucket_pos(n)];
	}

	[[nodiscard]] reference block(Bucket& bucket, size_type n)
	{
		return bucket[block_pos(n)];
	}

	[[nodiscard]] const_reference block(Bucket const& bucket, size_type n) const
	{
		return bucket[block_pos(n)];
	}

	[[nodiscard]] reference block(size_type n) { return block(bucket(n), n); }

	[[nodiscard]] const_reference block(size_type n) const { return block(bucket(n), n); }

	void createBucket(Bucket& bucket)
	{
		bucket = std::make_unique<T[]>(NumBlocksPerBucket);
		cap_ += NumBlocksPerBucket;
	}

 private:
	// TODO: Add indicator for if bucket has change
	// TODO: Add functions to upload (changed) buckets to GPU
	std::unique_ptr<Bucket[]> buckets_ = std::make_unique<Bucket[]>(NumBuckets);
	size_type                 size_{};
	std::atomic<size_type>    cap_{};
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_CONTAINER_HPP