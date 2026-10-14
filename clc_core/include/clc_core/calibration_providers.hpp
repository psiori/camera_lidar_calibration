#pragma once

namespace clc {

/// Tag type: initial guess from manual 2D–3D correspondences.
struct ManualGuessProvider {};
/// Tag type: NID / vlcal optimizer path.
struct NidOptimizerProvider {};

/// Maps provider tags to compile-time capability flags for the UI.
template <typename Provider>
struct InitialGuessTraits {};

template <typename Provider>
struct OptimizerTraits {};

template <>
struct InitialGuessTraits<ManualGuessProvider> {
  static constexpr bool supports_multi_dataset = true;
};

template <>
struct OptimizerTraits<NidOptimizerProvider> {
  static constexpr bool supports_multi_dataset = true;
};

}  // namespace clc
