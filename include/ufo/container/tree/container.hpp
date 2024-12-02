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
#include <ufo/container/tree/code.hpp>
#include <ufo/container/tree/container_bucket_iterator.hpp>
#include <ufo/container/tree/container_iterator.hpp>
#include <ufo/container/tree/index.hpp>
#include <ufo/utility/execution.hpp>
#include <ufo/utility/iterator_wrapper.hpp>
#include <ufo/utility/spinlock.hpp>

// STL
#include <atomic>
#include <cstddef>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>

namespace ufo
{
template <class... Ts>
class TreeContainer
{
 private:
	static constexpr std::size_t const NUM                   = 14;
	static constexpr std::size_t const NUM_BUCKETS           = std::size_t(1) << (32 - NUM);
	static constexpr std::size_t const NUM_BLOCKS_PER_BUCKET = std::size_t(1) << NUM;

 public:
	//
	// Tags
	//

	template <class T>
	using Data = std::array<T, NUM_BLOCKS_PER_BUCKET>;

	template <class T>
	struct alignas(8) S {
		Data<T> data;
		alignas(8) bool modified = false;
	};

	using value_type = std::tuple<S<Ts>...>;
	using Bucket     = std::atomic<value_type*>;

	template <class T>
	using bucket_type = S<T>;
	using size_type   = std::size_t;
	using pos_t       = TreeIndex::pos_t;

	template <class T>
	using iterator = TreeContainterIterator<T, false, Ts...>;
	template <class T>
	using const_iterator = TreeContainterIterator<T, true, Ts...>;
	template <class T>
	using reverse_iterator = std::reverse_iterator<iterator<T>>;
	template <class T>
	using const_reverse_iterator = std::reverse_iterator<const_iterator<T>>;

	template <class T>
	using bucket_iterator = TreeContainterBucketIterator<T, false, Ts...>;
	template <class T>
	using const_bucket_iterator = TreeContainterBucketIterator<T, true, Ts...>;
	template <class T>
	using reverse_bucket_iterator = std::reverse_iterator<bucket_iterator<T>>;
	template <class T>
	using const_reverse_bucket_iterator = std::reverse_iterator<const_bucket_iterator<T>>;

 public:
	TreeContainer() = default;

	TreeContainer(TreeContainer const& other)
	    : free_blocks_(other.free_blocks_), size_(other.size_.load())
	{
		auto num_buckets = other.numBuckets();
		for (std::size_t i{}; num_buckets > i; ++i) {
			buckets_[i] = new value_type(*other.buckets_[i]);
		}
	}

	~TreeContainer()
	{
		for (std::size_t i{}; NUM_BUCKETS > i; ++i) {
			if (nullptr == buckets_[i]) {
				break;
			}

			delete buckets_[i];
		}
	}

	TreeContainer& operator=(TreeContainer const& rhs)
	{
		auto num_buckets = rhs.numBuckets();
		for (std::size_t i{}; num_buckets > i; ++i) {
			buckets_[i] = new value_type(*rhs.buckets_[i]);
		}

		free_blocks_ = rhs.free_blocks_;
		size_        = rhs.size_.load();

		return *this;
	}

	template <class T>
	[[nodiscard]] iterator<T> begin()
	{
		return iterator<T>(this, 0);
	}

	template <class T>
	[[nodiscard]] const_iterator<T> begin() const
	{
		return const_iterator<T>(const_cast<TreeContainer*>(this), 0);
	}

	template <class T>
	[[nodiscard]] const_iterator<T> cbegin() const
	{
		return begin<T>();
	}

	template <class T>
	[[nodiscard]] iterator<T> end()
	{
		return iterator<T>(this, size_);
	}

	template <class T>
	[[nodiscard]] const_iterator<T> end() const
	{
		return const_iterator<T>(const_cast<TreeContainer*>(this), size_);
	}

	template <class T>
	[[nodiscard]] const_iterator<T> cend() const
	{
		return end<T>();
	}

	template <class T>
	[[nodiscard]] reverse_iterator<T> rbegin()
	{
		return std::make_reverse_iterator(end<T>());
	}

	template <class T>
	[[nodiscard]] const_reverse_iterator<T> rbegin() const
	{
		return std::make_reverse_iterator(end<T>());
	}

	template <class T>
	[[nodiscard]] const_reverse_iterator<T> crbegin() const
	{
		return rbegin<T>();
	}

	template <class T>
	[[nodiscard]] reverse_iterator<T> rend()
	{
		return std::make_reverse_iterator(begin<T>());
	}

	template <class T>
	[[nodiscard]] const_reverse_iterator<T> rend() const
	{
		return std::make_reverse_iterator(begin<T>());
	}

	template <class T>
	[[nodiscard]] const_reverse_iterator<T> crend() const
	{
		return rend<T>();
	}

	template <class T>
	[[nodiscard]] IteratorWrapper<iterator<T>> iter()
	{
		return IteratorWrapper<iterator<T>>(begin<T>(), end<T>());
	}

	template <class T>
	[[nodiscard]] IteratorWrapper<const_iterator<T>> iter() const
	{
		return IteratorWrapper<const_iterator<T>>(begin<T>(), end<T>());
	}

	template <class T>
	[[nodiscard]] IteratorWrapper<reverse_iterator<T>> riter()
	{
		return IteratorWrapper<reverse_iterator<T>>(rbegin<T>(), rend<T>());
	}

	template <class T>
	[[nodiscard]] IteratorWrapper<const_reverse_iterator<T>> riter() const
	{
		return IteratorWrapper<const_reverse_iterator<T>>(rbegin<T>(), rend<T>());
	}

	template <class T>
	[[nodiscard]] bucket_iterator<T> beginBucket()
	{
		return bucket_iterator<T>(this, 0);
	}

	template <class T>
	[[nodiscard]] const_bucket_iterator<T> beginBucket() const
	{
		return const_bucket_iterator<T>(const_cast<TreeContainer*>(this), 0);
	}

	template <class T>
	[[nodiscard]] const_bucket_iterator<T> cbeginBucket() const
	{
		return beginBucket<T>();
	}

	template <class T>
	[[nodiscard]] bucket_iterator<T> endBucket()
	{
		return bucket_iterator<T>(this, numBuckets());
	}

	template <class T>
	[[nodiscard]] const_bucket_iterator<T> endBucket() const
	{
		return const_bucket_iterator<T>(const_cast<TreeContainer*>(this), numBuckets());
	}

	template <class T>
	[[nodiscard]] const_bucket_iterator<T> cendBucket() const
	{
		return endBucket<T>();
	}

	template <class T>
	[[nodiscard]] reverse_bucket_iterator<T> rbeginBucket()
	{
		return std::make_reverse_iterator(endBucket<T>());
	}

	template <class T>
	[[nodiscard]] const_reverse_bucket_iterator<T> rbeginBucket() const
	{
		return std::make_reverse_iterator(endBucket<T>());
	}

	template <class T>
	[[nodiscard]] const_reverse_bucket_iterator<T> crbeginBucket() const
	{
		return rbeginBucket<T>();
	}

	template <class T>
	[[nodiscard]] reverse_bucket_iterator<T> rendBucket()
	{
		return std::make_reverse_iterator(beginBucket<T>());
	}

	template <class T>
	[[nodiscard]] const_reverse_bucket_iterator<T> rendBucket() const
	{
		return std::make_reverse_iterator(beginBucket<T>());
	}

	template <class T>
	[[nodiscard]] const_reverse_bucket_iterator<T> crendBucket() const
	{
		return rendBucket<T>();
	}

	template <class T>
	[[nodiscard]] IteratorWrapper<bucket_iterator<T>> iterBucket()
	{
		return IteratorWrapper<bucket_iterator<T>>(beginBucket<T>(), endBucket<T>());
	}

	template <class T>
	[[nodiscard]] IteratorWrapper<const_bucket_iterator<T>> iterBucket() const
	{
		return IteratorWrapper<const_bucket_iterator<T>>(beginBucket<T>(), endBucket<T>());
	}

	template <class T>
	[[nodiscard]] IteratorWrapper<reverse_bucket_iterator<T>> riterBucket()
	{
		return IteratorWrapper<reverse_bucket_iterator<T>>(rbeginBucket<T>(),
		                                                   rendBucket<T>());
	}

	template <class T>
	[[nodiscard]] IteratorWrapper<const_reverse_bucket_iterator<T>> riterBucket() const
	{
		return IteratorWrapper<const_reverse_bucket_iterator<T>>(rbeginBucket<T>(),
		                                                         rendBucket<T>());
	}

	template <class T>
	[[nodiscard]] S<T>& bucket(std::size_t idx)
	{
		return std::get<S<T>>(bucket(idx));
	}

	template <class T>
	[[nodiscard]] S<T> const& bucket(std::size_t idx) const
	{
		return std::get<S<T>>(bucket(idx));
	}

	template <std::size_t I>
	[[nodiscard]] auto& bucket(std::size_t idx)
	{
		return std::get<I>(bucket(idx));
	}

	template <std::size_t I>
	[[nodiscard]] auto const& bucket(std::size_t idx) const
	{
		return std::get<I>(bucket(idx));
	}

	template <class T>
	[[nodiscard]] bool& bucketModified(std::size_t idx)
	{
		return bucket<T>(idx).modified;
	}

	template <class T>
	[[nodiscard]] bool bucketModified(std::size_t idx) const
	{
		return bucket<T>(idx).modified;
	}

	template <std::size_t I>
	[[nodiscard]] bool& bucketModified(std::size_t idx)
	{
		return bucket<I>(idx).modified;
	}

	template <std::size_t I>
	[[nodiscard]] bool bucketModified(std::size_t idx) const
	{
		return bucket<I>(idx).modified;
	}

	template <class T>
	[[nodiscard]] Data<T>& bucketData(std::size_t idx)
	{
		auto& x    = bucket<T>(idx);
		x.modified = true;
		return x.data;
	}

	template <class T>
	[[nodiscard]] Data<T> const& bucketData(std::size_t idx) const
	{
		return bucket<T>(idx).data;
	}

	template <std::size_t I>
	[[nodiscard]] auto& bucketData(std::size_t idx)
	{
		auto& x    = bucket<I>(idx);
		x.modified = true;
		return x.data;
	}

	template <std::size_t I>
	[[nodiscard]] auto const& bucketData(std::size_t idx) const
	{
		return bucket<I>(idx).data;
	}

	template <class T>
	[[nodiscard]] T& get(pos_t pos)
	{
		return bucketData<T>(pos)[blockPos(pos)];
	}

	template <class T>
	[[nodiscard]] T const& get(pos_t pos) const
	{
		return bucketData<T>(pos)[blockPos(pos)];
	}

	template <std::size_t I>
	[[nodiscard]] auto& get(pos_t pos)
	{
		return bucketData<I>(pos)[blockPos(pos)];
	}

	template <std::size_t I>
	[[nodiscard]] auto const& get(pos_t pos) const
	{
		return bucketData<I>(pos)[blockPos(pos)];
	}

	[[nodiscard]] constexpr pos_t numBuckets() const noexcept
	{
		return empty() ? 0 : bucketPos(size_ - 1) + 1;
	}

	[[nodiscard]] constexpr pos_t numBlocksPerBucket() const noexcept
	{
		return NUM_BLOCKS_PER_BUCKET;
	}

	[[nodiscard]] constexpr pos_t bucketPos(pos_t pos) const noexcept
	{
		return pos / NUM_BLOCKS_PER_BUCKET;
	}

	[[nodiscard]] constexpr pos_t blockPos(pos_t pos) const noexcept
	{
		return pos % NUM_BLOCKS_PER_BUCKET;
	}

	[[nodiscard]] pos_t create()
	{
		if (!free_blocks_.empty()) {
			pos_t idx = free_blocks_.front();
			free_blocks_.pop_front();
			return idx;
		}

		pos_t idx = size_++;

		pos_t bucket = bucketPos(idx);

		if (nullptr == buckets_[bucket]) {
			// Create bucket
			buckets_[bucket] = new value_type();
		}

		return idx;
	}

	[[nodiscard]] pos_t createThreadSafe()
	{
		if (!free_blocks_.empty()) {
			if (std::unique_lock lock{free_blocks_mutex_, std::try_to_lock};
			    lock && !free_blocks_.empty()) {
				pos_t idx = free_blocks_.front();
				free_blocks_.pop_front();
				return idx;
			}
		}

		pos_t idx = size_++;

		pos_t bucket = bucketPos(idx);
		pos_t block  = blockPos(idx);

		if (0 == block) {
			// This will create bucket if it does not exist
			if (nullptr == buckets_[bucket]) {
				// Create bucket
				buckets_[bucket] = new value_type();
			}
		} else {
			// This will wait for someone else to create the bucket if it does not exist
			while (nullptr == buckets_[bucket]) {
				// Wait
			}
		}

		return idx;
	}

	void eraseBlock(pos_t block)
	{
		std::scoped_lock lock(free_blocks_mutex_);
		free_blocks_.push_front(block);
	}

	void clear()
	{
		// Reset all existing buckets to default value
		for (std::size_t i{}; NUM_BUCKETS > i; ++i) {
			if (nullptr == buckets_[i]) {
				break;
			}

			*buckets_[i] = value_type();
		}

		size_ = 0;
		std::scoped_lock lock(free_blocks_mutex_);
		free_blocks_.clear();
	}

	void reserve(pos_t cap)
	{
		// FIXME: Can be improved
		pos_t first = bucketPos(size_);
		pos_t last  = bucketPos(cap);
		for (; last >= first; ++first) {
			if (nullptr == buckets_[first]) {
				// Create bucket
				buckets_[first] = new value_type();
				break;
			}
		}

		// We know that the rest of the buckets do not exist yet

		for (; last >= first; ++first) {
			// Create bucket
			buckets_[first] = new value_type();
		}
	}

	void shrinkToFit()
	{
		// TODO: Implement
	}

	[[nodiscard]] bool empty() const { return 0 == size_; }

	[[nodiscard]] pos_t size() const { return size_ - free_blocks_.size(); }

	template <class T>
	[[nodiscard]] constexpr size_type serializedBucketSize() const
	{
		// return sizeof(Data<T>);
		return NUM_BLOCKS_PER_BUCKET * sizeof(T);
	}

	[[nodiscard]] constexpr size_type serializedSize() const
	{
		return numBuckets() * serializedBucketSize();
	}

 private:
	[[nodiscard]] value_type& bucket(std::size_t idx)
	{
		return *buckets_[bucketPos(idx)].load(std::memory_order_relaxed);
	}

	[[nodiscard]] value_type const& bucket(std::size_t idx) const
	{
		return *buckets_[bucketPos(idx)].load(std::memory_order_relaxed);
	}

 private:
	std::unique_ptr<Bucket[]> buckets_ = std::make_unique<Bucket[]>(NUM_BUCKETS);

	Spinlock          free_blocks_mutex_;
	std::deque<pos_t> free_blocks_;

	std::atomic<pos_t> size_{};
};
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_CONTAINER_HPP