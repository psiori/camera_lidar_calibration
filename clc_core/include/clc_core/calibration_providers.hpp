#pragma once

namespace clc {

struct ManualGuessProvider {};
struct NidOptimizerProvider {};

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
