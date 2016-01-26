
#ifndef ROBOT_FRS2BOT_HPP
#define ROBOT_FRS2BOT_HPP
#include "robot/robot.hh"

namespace robot
{
  /// a basic hylos-like quadruped hybrid
  class frs2bot : public Robot
  {
  public:
    frs2bot(ode::Environment& env, const Eigen::Vector3d& pos) { _build(env, pos); }
  protected:
    void _build(ode::Environment& env, const Eigen::Vector3d& pos);
  };
}

#endif
