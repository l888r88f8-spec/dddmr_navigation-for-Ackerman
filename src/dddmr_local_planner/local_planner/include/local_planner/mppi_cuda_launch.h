#pragma once

// CUDA-neutral host-side launch geometry selection for the independent-thread
// MPPI kernels (one thread per candidate/control step, no block-wide shared
// reduction). Kernels that use __syncthreads() or fixed-size shared arrays must
// keep their own fixed block size and are deliberately not served by this
// helper.
//
// The selection only changes how the same independent threads are grouped into
// blocks; it does not alter indexing, arithmetic, evaluation order, or result
// validity. The SM count comes from the backend's cached
// cudaDeviceProp::multiProcessorCount.

#include <algorithm>
#include <cstddef>

namespace local_planner
{

struct MppiCudaLaunchGeometry
{
  unsigned int block_dim;
  unsigned int grid;
};

// Picks a power-of-two block size in [1, 64] that spreads `count` independent
// threads over approximately 3x multiProcessorCount blocks (inside the intended
// 2x-4x window when `count` allows it). Smaller blocks are used for small
// batches so a 96-candidate launch becomes several blocks instead of one; for
// very large counts the largest candidate block (64) is used.
inline MppiCudaLaunchGeometry mppiCudaSelectLaunch(
  const std::size_t count, const int multi_processor_count)
{
  if (count == 0u) {
    return MppiCudaLaunchGeometry{1u, 0u};
  }
  const std::size_t sm_count = static_cast<std::size_t>(
    std::max(1, multi_processor_count));
  const std::size_t target_blocks = sm_count * 3u;

  std::size_t best_block = 1u;
  std::size_t best_grid = count;
  std::size_t best_distance =
    best_grid > target_blocks ? best_grid - target_blocks :
    target_blocks - best_grid;
  std::size_t best_padding = 0u;

  for (std::size_t block = 1u; block <= 64u; block <<= 1) {
    const std::size_t grid = (count + block - 1u) / block;
    const std::size_t distance =
      grid > target_blocks ? grid - target_blocks : target_blocks - grid;
    const std::size_t padding = grid * block - count;
    if (distance < best_distance ||
      (distance == best_distance &&
      (padding < best_padding ||
      (padding == best_padding && block > best_block))))
    {
      best_block = block;
      best_grid = grid;
      best_distance = distance;
      best_padding = padding;
    }
  }

  return MppiCudaLaunchGeometry{
    static_cast<unsigned int>(best_block),
    static_cast<unsigned int>(best_grid)};
}

}  // namespace local_planner
