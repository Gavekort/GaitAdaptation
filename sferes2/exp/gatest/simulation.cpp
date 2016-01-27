#include "simulation.hh"

#include <iostream>

#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>
#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"
#include <ode/environment.hh>
#include <robot/robot4.hh>
#include <ode/box.hh>
#include <ode/object.hh>
#include <renderer/osg_visitor.hh>

Simulation::Simulation(const float tilt, const bool headless) : env(tilt, 0.0f, 0.0f), rob(env, Eigen::Vector3d(0, 0, 0.5)){
    this->headless = headless;

    if(!headless){
        v = new renderer::OsgVisitor;
        rob.accept(*v);
    }

    env.set_gravity(0, 0, -9.81);
}


Simulation::~Simulation(){
    if(!headless){
    delete v;
    }
}
/*
   void Simulation::add_blocks(int count, int size, ode::Environment& env){
   typedef boost::mt19937 RNGType;
   RNGType rng( time(0) );
   boost::uniform_real<> loc_range(-3,3);
   boost::variate_generator<boost::mt19937&, boost::uniform_real<> > rlocation(rng,loc_range);
   boost::uniform_real<> size_range(0.001, (float) size/100);
   boost::variate_generator<boost::mt19937&, boost::uniform_real<> > rsize(rng,size_range);
   std::vector<ode::Object::ptr_t> g;
   double bsize;
   for(int i = 0; i < count; ++i){
   bsize = rsize();
   ode::Object::ptr_t b
   (new ode::Box(env, Eigen::Vector3d(rlocation(), rlocation(), bsize/2),
   10, bsize, bsize, bsize));
   g.push_back(b);
   b->accept(v);
   b->fix();
   env.add_to_ground(*b);
   }
   }
   */


