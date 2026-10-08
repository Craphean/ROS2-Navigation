#ifndef RL_NAV2_CONTROLLER__TD3_ACTOR_HPP_
#define RL_NAV2_CONTROLLER__TD3_ACTOR_HPP_

#include <array>
#include <string>
#include <vector>

namespace rl_nav2_controller
{

/**
 * @brief Minimal TD3 actor: Linear(25,800)-ReLU-Linear(800,600)-ReLU-Linear(600,2)-Tanh
 *
 * Weights are loaded from a compact binary dump extracted from the exported ONNX actor.
 */
class Td3Actor
{
public:
  static constexpr int kStateDim = 25;
  static constexpr int kActionDim = 2;
  static constexpr int kHidden1 = 800;
  static constexpr int kHidden2 = 600;

  void load(const std::string & weights_path);
  std::array<float, kActionDim> act(const std::array<float, kStateDim> & state) const;

private:
  std::vector<float> w1_;  // [800, 25]
  std::vector<float> b1_;
  std::vector<float> w2_;  // [600, 800]
  std::vector<float> b2_;
  std::vector<float> w3_;  // [2, 600]
  std::vector<float> b3_;
};

}  // namespace rl_nav2_controller

#endif  // RL_NAV2_CONTROLLER__TD3_ACTOR_HPP_
