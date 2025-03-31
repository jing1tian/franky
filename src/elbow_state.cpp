#include "franky/elbow_state.hpp"

#include <iostream>

namespace franky {

std::ostream &operator<<(std::ostream &os, const ElbowState &elbow_state) {
  os << "ElbowState(joint_3_pos=" << elbow_state.joint_3_pos_;
  if (elbow_state.joint_4_flip_.has_value())
    os << ", joint_4_flip=" << elbow_state.joint_4_flip_.value();
  os << ")";
  return os;
}

std::ostream &operator<<(std::ostream &os, const FlipDirection &flip_direction) {
  switch (flip_direction) {
    case FlipDirection::kNegative:
      os << "FlipDirection.Negative";
    case FlipDirection::kNeutral:
      os << "FlipDirection.Neutral";
    default:
      os << "FlipDirection.Positive";
  }
  return os;
}

}  // namespace franky
