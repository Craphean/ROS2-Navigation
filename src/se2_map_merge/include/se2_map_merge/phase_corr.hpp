#pragma once

#include "se2_map_merge/types.hpp"

namespace se2_map_merge
{

struct PhaseCorrParams
{
  int occupied_threshold{65};
  int min_occupied_cells{80};
  int max_fft_dim{256};          // caps FFT grid size for <3 ms latency
  double max_shift_m{3.5};
  double min_response{0.08};
  double overlap_margin_m{3.5};
};

struct PhaseCorrResult
{
  bool valid{false};
  double dx{0.0};
  double dy{0.0};
  double response{0.0};
  int overlap_ref_cells{0};
  int overlap_src_cells{0};
};

/// Estimate coarse 2D translation (dx, dy) aligning ``src`` (pre-warped by
/// ``init_pose``) onto ``ref`` via Hanning-windowed 2D FFT phase correlation.
PhaseCorrResult estimate_coarse_translation(
  const OccupancyGrid2D & ref,
  const OccupancyGrid2D & src,
  const Pose2D & init_pose,
  const PhaseCorrParams & params = PhaseCorrParams{});

}  // namespace se2_map_merge
