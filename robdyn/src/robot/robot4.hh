
#ifndef ROBOT_robot4_HPP
#define ROBOT_robot4_HPP
#include "robot/robot.hh"

namespace robot
{
  /// a basic hylos-like quadruped hybrid
  class robot4 : public Robot
  {
  public:
    robot4(ode::Environment& env, const Eigen::Vector3d& pos) { _build(env, pos); }
  protected:
    void _build(ode::Environment& env, const Eigen::Vector3d& pos);
  };
}

#endif
