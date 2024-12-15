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
#include <ufo/container/tree/container.hpp>
#include <ufo/container/tree/index.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <utility>

// WebGPU
#if defined(UFO_WEBGPU)
#include <webgpu/webgpu.h>
#endif

namespace ufo
{
template <bool GPU, class... Ts>
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

template <class... Ts>
class TreeData<true, Ts...> : public TreeData<false, Ts...>
{
 private:
	using Base = TreeData<false, Ts...>;

	static constexpr std::size_t const NumBuffers = 1 + sizeof...(Blocks);

 public:
	~TreeData() { gpuRelease(); }

	// TODO: Add extra methods

	void gpuInit(WGPUPowerPreference power_preference = WGPUPowerPreference_HighPerformance,
	             WGPUBackendType     backend_type     = WGPUBackendType_Undefined)
	{
		instance_ = compute::createInstance();
		adapter_ = compute::createAdapter(instance_, nullptr, power_preference, backend_type);
		device_  = compute::createDevice(adapter_, requiredLimits(adapter_));
		queue_   = compute::queue(device_);
		// TODO: Implement
	}

	void gpuRelease()
	{
		for (WGPUBuffer buf : buffers_) {
			if (nullptr != buf) {
				wgpuBufferRelease(buf);
			}
		}

		if (!borrowed_device_) {
			if (nullptr != queue_) {
				wgpuQueueRelease(queue_);
			}
			if (nullptr != device_) {
				wgpuDeviceRelease(device_);
			}
		}

		if (nullptr != adapter_) {
			wgpuAdapterRelease(adapter_);
		}

		if (nullptr != instance_) {
			wgpuInstanceRelease(instance_);
		}
	}

	[[nodiscard]] WGPUDevice gpuDevice() { device_; }

	void gpuDevice(WGPUDevice device)
	{
		release();
		device_ = device;
		queue_  = wgpuDeviceGetQueue(device);

		// TODO: Create buffers

		// TODO: Mark every block as modified in data_
	}

	[[nodiscard]] std::array<WGPUBuffer, NumBuffers> const& gpuBuffers() const
	{
		return buffers_;
	}

	template <class T>
	[[nodiscard]] WGPUBuffer gpuBuffer() const
	{
		return buffers_[index_v<T, Ts...>];
	}

	void gpuUpdateBuffers() { (gpuUpdateBuffer<Ts>(), ...); }

	template <class T>
	void gpuUpdateBuffer()
	{
		// TODO: Implement

		WGPUBuffer buffer = buffers_[index_v<T, Ts...>];

		if (nullptr == buffer) {
			gpuInit();
		}

		std::size_t size                = map_.block_.serializedBucketSize<MapBlock<3, 8>>();
		std::size_t offset              = 0;
		std::size_t num_buckets_written = 0;
		for (auto it = map_.block_.beginBucket<MapBlock<3, 8>>();
		     map_.block_.endBucket<MapBlock<3, 8>>() != it; ++it, offset += size) {
			if (it->modified) {
				wgpuQueueWriteBuffer(queue_, buffer, offset, it->data.data(), size);
				it->modified = false;
				++num_buckets_written;
			}
		}
	}

	void swap(TreeData& other)
	{
		using std::swap;
		swap(static_cast<Base&>(*this), static_cast<Base&>(other));
	}

 private:
	WGPURequiredLimits requiredLimits(WGPUAdapter adapter) const
	{
		// TODO: Implement

		// Get adapter supported limits, in case we need them
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

		required.limits.maxBindGroups = 2;

		required.limits.maxBufferSize               = 2'147'483'648;
		required.limits.maxStorageBufferBindingSize = 2'147'483'648;

		required.limits.maxComputeWorkgroupSizeX          = 32;
		required.limits.maxComputeWorkgroupSizeY          = 4;
		required.limits.maxComputeWorkgroupSizeZ          = 1;
		required.limits.maxComputeInvocationsPerWorkgroup = 32;
		required.limits.maxComputeWorkgroupsPerDimension  = 31250;

		required.limits.maxUniformBuffersPerShaderStage = 1;
		// TODO: required.limits.maxUniformBufferBindingSize     = sizeof(uniform_);

		required.limits.maxTextureDimension1D            = 4096;
		required.limits.maxTextureDimension2D            = 4096;
		required.limits.maxTextureDimension3D            = 1;
		required.limits.maxTextureArrayLayers            = 1;
		required.limits.maxSampledTexturesPerShaderStage = 1;
		required.limits.maxSamplersPerShaderStage        = 1;

		// // We use at most 2 vertex attributes
		// required.limits.maxVertexAttributes = 2;
		// // We should also tell that we use 1 vertex buffers
		// required.limits.maxVertexBuffers = 1;
		// // Maximum size of a buffer is 6 vertices of 5 float each
		// // Maximum stride between 2 consecutive vertices in the vertex buffer
		// required.limits.maxVertexBufferArrayStride = 5 * sizeof(float);

		// // There is a maximum of 3 float forwarded from vertex to fragment shader
		// required.limits.maxInterStageShaderComponents = 3;

		// // We use at most 1 bind group for now
		// required.limits.maxBindGroups = 1;
		// // We use at most 1 uniform buffer per stage
		// required.limits.maxUniformBuffersPerShaderStage = 1;
		// // Uniform structs have a size of maximum 16 float (more than what we need)
		// required.limits.maxUniformBufferBindingSize = 16 * 4;

		return required;
	}

 protected:
	WGPUInstance                       instance_        = nullptr;
	WGPUAdapter                        adapter_         = nullptr;
	bool                               borrowed_device_ = false;
	WGPUDevice                         device_          = nullptr;
	WGPUQueue                          queue_           = nullptr;
	std::array<WGPUBuffer, NumBuffers> buffers_{};
};

template <bool GPU, class... Ts>
void swap(TreeData<GPU, Ts...>& lhs, TreeData<GPU, Ts...>& rhs)
{
	lhs.swap(rhs);
}

#endif
}  // namespace ufo

#endif  // UFO_CONTAINER_TREE_DATA_HPP