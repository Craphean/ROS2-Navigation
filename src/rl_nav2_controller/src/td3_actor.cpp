#include "rl_nav2_controller/td3_actor.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>

namespace rl_nav2_controller
{
namespace
{

uint32_t read_u32(std::ifstream & in)
{
  uint32_t v = 0;
  in.read(reinterpret_cast<char *>(&v), sizeof(v));
  if (!in) {
    throw std::runtime_error("Unexpected EOF while reading TD3 actor weights");
  }
  return v;
}

std::vector<float> read_tensor(std::ifstream & in, const std::vector<uint32_t> & expect_dims)
{
  const uint32_t ndim = read_u32(in);
  if (ndim != expect_dims.size()) {
    throw std::runtime_error("TD3 actor weight rank mismatch");
  }
  size_t count = 1;
  for (uint32_t i = 0; i < ndim; ++i) {
    const uint32_t d = read_u32(in);
    if (d != expect_dims[i]) {
      throw std::runtime_error("TD3 actor weight shape mismatch");
    }
    count *= d;
  }
  std::vector<float> data(count);
  in.read(reinterpret_cast<char *>(data.data()), static_cast<std::streamsize>(count * sizeof(float)));
  if (!in) {
    throw std::runtime_error("Unexpected EOF while reading TD3 actor tensor data");
  }
  return data;
}

inline float relu(float x)
{
  return x > 0.0f ? x : 0.0f;
}

}  // namespace

void Td3Actor::load(const std::string & weights_path)
{
  std::ifstream in(weights_path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("Failed to open TD3 actor weights: " + weights_path);
  }

  char magic[4] = {};
  in.read(magic, 4);
  if (std::string(magic, 4) != "TD3A") {
    throw std::runtime_error("Bad TD3 actor weights magic in " + weights_path);
  }
  const uint32_t version = read_u32(in);
  if (version != 1) {
    throw std::runtime_error("Unsupported TD3 actor weights version");
  }

  w1_ = read_tensor(in, {kHidden1, kStateDim});
  b1_ = read_tensor(in, {kHidden1});
  w2_ = read_tensor(in, {kHidden2, kHidden1});
  b2_ = read_tensor(in, {kHidden2});
  w3_ = read_tensor(in, {kActionDim, kHidden2});
  b3_ = read_tensor(in, {kActionDim});
}

std::array<float, Td3Actor::kActionDim> Td3Actor::act(
  const std::array<float, kStateDim> & state) const
{
  std::array<float, kHidden1> h1{};
  for (int i = 0; i < kHidden1; ++i) {
    float sum = b1_[static_cast<size_t>(i)];
    const float * row = &w1_[static_cast<size_t>(i * kStateDim)];
    for (int j = 0; j < kStateDim; ++j) {
      sum += row[j] * state[static_cast<size_t>(j)];
    }
    h1[static_cast<size_t>(i)] = relu(sum);
  }

  std::array<float, kHidden2> h2{};
  for (int i = 0; i < kHidden2; ++i) {
    float sum = b2_[static_cast<size_t>(i)];
    const float * row = &w2_[static_cast<size_t>(i * kHidden1)];
    for (int j = 0; j < kHidden1; ++j) {
      sum += row[j] * h1[static_cast<size_t>(j)];
    }
    h2[static_cast<size_t>(i)] = relu(sum);
  }

  std::array<float, kActionDim> action{};
  for (int i = 0; i < kActionDim; ++i) {
    float sum = b3_[static_cast<size_t>(i)];
    const float * row = &w3_[static_cast<size_t>(i * kHidden2)];
    for (int j = 0; j < kHidden2; ++j) {
      sum += row[j] * h2[static_cast<size_t>(j)];
    }
    action[static_cast<size_t>(i)] = std::tanh(sum);
  }
  return action;
}

}  // namespace rl_nav2_controller
