#ifndef ROBOT_myrob_HPP
#define ROBOT_myrob_HPP
#include "robot/robot.hh"
#include "ode/environment_hexa.hh"
namespace robot
{
    class myrob : public Robot
    {
    public:
        myrob(ode::Environment_hexa & env, const Eigen::Vector3d & pos,std::vector<int> brokenLegs):_brokenLegs(brokenLegs)
        {

            _build(env, pos);

	    /*std::cout<<_bodies.size()<<std::endl;
	    for(int i=0; i<_bodies.size();i++)
	    std::cout<< _bodies[i]->get_pos()<<std::endl;*/

        }
        myrob(const myrob &o, ode::Environment_hexa & env) :
                Robot(o, env)
        {

	  for(int i=1;i<_bodies.size();i++)
	    {
	      env.add_leg_object((i-1)/3,*_bodies[i]);
	    }
        }
        boost::shared_ptr<myrob> clone(ode::Environment_hexa& env) const
        {

            return boost::shared_ptr<myrob>(new myrob(*this, env));
        }
    protected:
        std::vector<int> _brokenLegs;
        void _build(ode::Environment_hexa& env, const Eigen::Vector3d& pos);
    };
}

#endif
