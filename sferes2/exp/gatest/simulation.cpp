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

Simulation::Simulation(const float tilt, const int count, const int size,
        const bool headless) : env(0.0f, tilt, 0.0f), rob(env, Eigen::Vector3d(0, 0, 0.5)){
    this->headless = headless;
    this->tilt = tilt;

    if(!headless){
        this->v.reset(new renderer::OsgVisitor()); //assures that v is updated
        rob.accept(*v);
    }

    env.set_gravity(0, 0, -9.81);
    add_blocks(count, size);
}


Simulation::~Simulation(){
}
/* Uses a 2D gaussian to spread blocks on the surface
 * https://en.wikipedia.org/wiki/Gaussian_function#Two-dimensional_Gaussian_function
 */
void Simulation::add_blocks(int count, int size){
    typedef boost::mt19937 RNGType;
    RNGType rng( time(0) );
    boost::uniform_real<> loc_range(-3,3);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > rlocation(rng,loc_range);
    boost::uniform_real<> size_range(0.001, (float) size/100);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > rsize(rng,size_range);
    double bsize;
    for(int i = 0; i < count; ++i){
        bsize = rsize();
        float x = rlocation();
        float y = rlocation();
        ode::Object::ptr_t b
            (new ode::Box(env, Eigen::Vector3d(x, y, bsize/2 + (tan(tilt)*-x)),
                          10, bsize*2, bsize*2, bsize));
        b->set_rotation(0.0f, -tilt, 0.0f);
        boxes.push_back(b);
        if(!headless){
            b->accept(*v);
        }
        b->fix();
        env.add_to_ground(*b);
    }
}
