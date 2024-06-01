/**
 * \file math/sampling_functions.hpp
 **/
#ifndef PBR_SAMPLING_FUNCTIONS_HPP
#define PBR_SAMPLING_FUNCTIONS_HPP

#include <cstdint>
#include <span>

#include "math/vecmath.hpp"

namespace pbr {
 
  /// the functions below are for sampling from only two sampling distributions

  float BalanceHeuristic(int32_t nf , float fpdf , int32_t ng , float gpdf);

  float PowerHeuristic(int32_t nf , float fpdf , int32_t ng , float gpdf);

  int32_t SampleDiscrete(const std::span<const float>& weights , float u , float* pmf , float* uremapped);

  float LinearPdf(float x , float a , float b);

  float SampleLinear(float u , float a , float b);

  float InvertLinearSample(float x , float a , float b);

  float BilinearPdf(const glm::vec2& p , const std::span<const float>& w);

  glm::vec2 SampleBilinear(const glm::vec2& u , const std::span<const float>& w);

  glm::vec2 InvertBilinearSample(const glm::vec2& p , const std::span<const float>& w);

  Point2 SampleUniformDiskConcentric(const Point2& u);

  float SampleVisibleWavelengths(float u);

  float VisibleWavelengthPDF(float lambda);

  glm::vec3 SampleUniformSphere(const Point2& u);

} // namespace pbr

#endif // !PBR_SAMPLING_FUNCTIONS_HPP
