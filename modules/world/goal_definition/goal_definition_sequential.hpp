// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_GOAL_DEFINITION_SEQUENTIAL_HPP_
#define MODULES_WORLD_GOAL_DEFINITION_SEQUENTIAL_HPP_

#include "modules/world/goal_definition/goal_definition.hpp"
#include "modules/geometry/polygon.hpp"

namespace modules {
namespace world {
namespace objects {
class Agent;
}
namespace goal_definition {

constexpr int NO_GOAL_REACHED = -1;

class GoalDefinitionSequential : public GoalDefinition  {
 public:
  GoalDefinitionSequential() : GoalDefinition(),
                               sequential_goals_(),
                               last_sequential_goal_reached_(NO_GOAL_REACHED),
                               do_lane_change_(0) {}
  GoalDefinitionSequential(const std::vector<GoalDefinitionPtr>&
                           sequential_goals) :
                           GoalDefinition(),
                           sequential_goals_(sequential_goals),
                           last_sequential_goal_reached_(NO_GOAL_REACHED),
                           do_lane_change_(0) {}

  void AddSequentialGoal(const GoalDefinitionPtr& sequential_goal) {
    sequential_goals_.push_back(sequential_goal);
  }

  GoalDefinitionPtr GetNextGoal() const;
  GoalDefinitionPtr GetCurrentGoal() const;

  std::vector<GoalDefinitionPtr> get_sequential_goals() const {
    return sequential_goals_;
  }
  int GetCurrentId() const { return last_sequential_goal_reached_; }
  void SetLaneChange(int lane_change) { do_lane_change_ = lane_change; }
  int GetLaneChange() const { return do_lane_change_; }
  virtual const modules::geometry::Polygon& get_shape() const {
    return GetCurrentGoal()->get_shape();
  }
  virtual bool AtGoal(const modules::world::objects::Agent& agent);
  int LastSequentialGoal() const { return last_sequential_goal_reached_; }

 private:
  std::vector<GoalDefinitionPtr> sequential_goals_;
  int last_sequential_goal_reached_;
  int do_lane_change_;
};


}  // namespace goal_definition
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_GOAL_DEFINITION_SEQUENTIAL_HPP_