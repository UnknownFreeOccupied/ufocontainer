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

#ifndef UFO_CONTAINER_TREE_DATA_HPP
#define UFO_CONTAINER_TREE_DATA_HPP

// UFO
#include <ufo/compute/compute.hpp>
#include <ufo/container/tree/container.hpp>
#include <ufo/container/tree/index.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <cstddef>
#include <cstring>
#include <utility>

namespace ufo
{
template <class Derived, bool GPU, class... Ts>
class TreeData
{
 public:
	using Data = TreeContainer<Ts...>;

	using Index    = TreeIndex;
	using pos_t    = Index::pos_t;
	using offset_t = Index::offset_t;

 public:
	[[nodiscard]] Data& data() { return data_; }

	[[nodiscard]] Data const& data() const { return data_; }

	/*!
	 * @brief Checks if a block is valid.
	 *
	 * @param block the block to check
	 * @return `true` if the block is valid, `false` otherwise.
	 */
	[[nodiscard]] bool valid(pos_t block) const { return data_.size() > block; }

	void swap(TreeData& other)
	{
		using std::swap;
		swap(data_, other.data_);
	}

 protected:
	[[nodiscard]] std::size_t size() const { return data_.numUsedBlocks(); }

	void reserve(std::size_t cap) { data_.reserve(cap); }

	void clear() { data_.clear(); }

	[[nodiscard]] pos_t create() { return data_.create(); }

	[[nodiscard]] pos_t createThreadSafe() { return data_.createThreadSafe(); }

	void eraseBlock(pos_t block) { data_.eraseBlock(block); }

	template <class T>
	[[nodiscard]] T& data(pos_t block)
	{
		return data_.template get<T>(block);
	}

	template <class T>
	[[nodiscard]] T const& data(pos_t block) const
	{
		return data_.template get<T>(block);
	}

	template <class T>
	[[nodiscard]] T const& dataConst(pos_t block) const
	{
		return data(block);
	}

	template <class T>
	[[nodiscard]] T& data(TreeIndex node)
	{
		return data(node.pos);
	}

	template <class T>
	[[nodiscard]] T const& data(TreeIndex node) const
	{
		return data(node.pos);
	}

	template <class T>
	[[nodiscard]] T const& dataConst(TreeIndex node) const
	{
		return data(node);
	}

 protected:
	Data data_;
};

#if defined(UFO_WEBGPU)

template <class Derived, class Block, class... Blocks>
class TreeData<Derived, true, Block, Blocks...>
    : public TreeData<Derived, false, Block, Blocks...>
{
 private:
	using Base = TreeData<Derived, false, Block, Blocks...>;

	static constexpr std::size_t const NumBuffers = 1 + sizeof...(Blocks);

 public:
	~TreeData() { gpuRelease(); }

	bool gpuInit(WGPUPowerPreference power_preference = WGPUPowerPreference_HighPerformance,
	             WGPUBackendType     backend_type     = WGPUBackendType_Undefined)
	{
		if (nullptr != device_) {
			return false;
		}

		instance_ = compute::createInstance();
		adapter_ = compute::createAdapter(instance_, nullptr, power_preference, backend_type);
		device_  = compute::createDevice(adapter_, requiredLimits(adapter_));
		queue_   = compute::queue(device_);

		return true;
	}

	bool gpuInit(WGPUDevice device)
	{
		if (nullptr != device_) {
			return false;
		}

		assert(nullptr != device);

		// Increase reference count
		wgpuDeviceReference(device);

		device_ = device;
		queue_  = compute::queue(device);

		return true;
	}

	void gpuRelease()
	{
		for (WGPUBuffer& buf : buffers_) {
			if (nullptr != buf) {
				wgpuBufferRelease(buf);
				buf = nullptr;
			}
		}

		if (nullptr != queue_) {
			wgpuQueueRelease(queue_);
			queue_ = nullptr;
		}

		if (nullptr != device_) {
			wgpuDeviceRelease(device_);
			device_ = nullptr;
		}

		if (nullptr != adapter_) {
			wgpuAdapterRelease(adapter_);
			adapter_ = nullptr;
		}

		if (nullptr != instance_) {
			wgpuInstanceRelease(instance_);
			instance_ = nullptr;
		}
	}

	[[nodiscard]] WGPUInstance gpuInstance() const { return instance_; }

	[[nodiscard]] WGPUAdapter gpuAdapter() const { return adapter_; }

	[[nodiscard]] WGPUDevice gpuDevice() const { return device_; }

	[[nodiscard]] std::array<WGPUBuffer, NumBuffers> const& gpuBuffers() const
	{
		return buffers_;
	}

	template <class T>
	[[nodiscard]] WGPUBuffer gpuBuffer() const
	{
		return buffers_[index_v<T, Block, Blocks...>];
	}

	template <class T>
	[[nodiscard]] std::size_t gpuBufferSize() const
	{
		double      size_factor  = sizeof(T) / static_cast<double>(sizeof(Block));
		std::size_t content_size = static_cast<std::size_t>(size_factor * tree_buffer_size_);
		return compute::bufferPaddedSize(content_size);
	}

	// template <class Predicate>
	// void gpuUpdateBuffers(Predicate const& pred)
	// {
	// 	// TODO: Implement
	// 	derived().onGpuUpdateBuffers(pred);
	// }

	void gpuUpdateBuffers()
	{
		gpuUpdateBuffer<Block>();
		(gpuUpdateBuffer<Blocks>(), ...);
	}

	template <class T>
	void gpuUpdateBuffer()
	{
		WGPUBuffer& buffer = buffers_[index_v<T, Block, Blocks...>];

			std::size_t size   = this->data_.template serializedBucketSize<T>();

		if (nullptr == buffer) {
			double size_factor = sizeof(T) / static_cast<double>(sizeof(Block));

			std::size_t content_size =
			    static_cast<std::size_t>(size_factor * tree_buffer_size_);

			buffer = compute::createBuffer(
			    device_, content_size, WGPUBufferUsage_Storage | WGPUBufferUsage_CopyDst, true);

			assert(nullptr != buffer);

			void* buf = wgpuBufferGetMappedRange(buffer, 0, content_size);

			for (auto& bucket : this->data_.template iterBucket<T>()) {
				std::memcpy(buf, bucket.data.data(), size);
				buf             = static_cast<unsigned char*>(buf) + size;
				bucket.modified = false;
			}

			wgpuBufferUnmap(buffer);
		} else {
			std::size_t offset = 0;
			for (auto it   = this->data_.template beginBucket<T>(),
			          last = this->data_.template endBucket<T>();
			     last != it; ++it, offset += size) {
				if (it->modified) {
					wgpuQueueWriteBuffer(queue_, buffer, offset, it->data.data(), size);
					it->modified = false;
				}
			}
		}
	}

	//
	// Tree
	//

	[[nodiscard]] WGPUBuffer gpuTreeBuffer() const { return gpuBuffer<Block>(); }

	[[nodiscard]] std::size_t gpuTreeBufferSize() const { return gpuBufferSize<Block>(); }

	void swap(TreeData& other)
	{
		using std::swap;
		swap(static_cast<Base&>(*this), static_cast<Base&>(other));
		swap(instance_, other.instance_);
		swap(adapter_, other.adapter_);
		swap(device_, other.device_);
		swap(queue_, other.queue_);
		swap(buffers_, other.buffers_);
	}

 private:
	WGPURequiredLimits requiredLimits(WGPUAdapter adapter)
	{
		WGPUSupportedLimits supported{};
		supported.nextInChain = nullptr;
		wgpuAdapterGetLimits(adapter, &supported);

		WGPURequiredLimits required{};
		compute::setDefault(required.limits);

		// These two limits are different because they are "minimum" limits,
		// they are the only ones we may forward from the adapter's supported limits.
		required.limits.minUniformBufferOffsetAlignment =
		    supported.limits.minUniformBufferOffsetAlignment;
		required.limits.minStorageBufferOffsetAlignment =
		    supported.limits.minStorageBufferOffsetAlignment;

		tree_buffer_size_ = std::min(
		    tree_buffer_size_, static_cast<std::size_t>(supported.limits.maxBufferSize));
		tree_buffer_size_ =
		    std::min(tree_buffer_size_,
		             static_cast<std::size_t>(supported.limits.maxStorageBufferBindingSize));

		required.limits.maxBufferSize               = tree_buffer_size_;
		required.limits.maxStorageBufferBindingSize = tree_buffer_size_;

		required.limits.maxComputeWorkgroupStorageSize    = 16352;
		required.limits.maxComputeInvocationsPerWorkgroup = 256;
		required.limits.maxComputeWorkgroupSizeX          = 256;
		required.limits.maxComputeWorkgroupSizeY          = 256;
		required.limits.maxComputeWorkgroupSizeZ          = 64;
		required.limits.maxComputeWorkgroupsPerDimension  = 65535;

		required.limits.maxUniformBuffersPerShaderStage = 12;
		required.limits.maxUniformBufferBindingSize     = 16 << 10;  // (16 KiB)

		return required;
	}

 protected:
	WGPUInstance                       instance_ = nullptr;
	WGPUAdapter                        adapter_  = nullptr;
	WGPUDevice                         device_   = nullptr;
	WGPUQueue                          queue_    = nullptr;
	std::array<WGPUBuffer, NumBuffers> buffers_{};

	std::size_t tree_buffer_size_ = 2'147'483'648;
};

template <class Derived, bool GPU, class... Ts>
void swap(TreeData<Derived, GPU, Ts...>& lhs, TreeData<Derived, GPU, Ts...>& rhs)
{
	lhs.swap(rhs);
}

#endif
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_DATA_HPP