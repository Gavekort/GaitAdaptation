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

    float xc = -3; //skew gauss and location
    float yc = 0;
    float s = 4; //spread gauss and location

    typedef boost::mt19937 RNGType;
    RNGType rng( time(0) );
    /* s-1 is the location based on how spread it is, which wraps the gaussian bell over
     * the relevant parts making the gauss and the locations in the same range.
     * The reason we subtract 1 is to chop off the "skirts" of the gauss, prevent the creation
     * of miniscule boxes which does nothing but impact performance.
     */
    boost::uniform_real<> loc_range(-(s-1),s-1);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > rlocation(rng,loc_range);
    boost::uniform_real<> size_range(0.03, (float) size/100);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > rsize(rng,size_range);

    for(int i = 0; i < count; ++i){
        //Gaussian gaussian amplitudes
        float a = rsize();

        float x = rlocation() + xc; //random + gaussian skew
        float y = rlocation() + yc;

        //2D gaussian
        float bsize = a*exp( -( (pow((x-xc), 2) / (2*pow(s, 2)) ) +
                    (pow((y-yc), 2) / (2*pow(s, 2)) ) ));


        /*tan(angle) is conversion from degrees to slope (relationship between rise and run)
         *Multiplying this with -x will find the correct height of the block based on the
         *rise. This means that you can place boxes along x and y in a slope, and the boxes will
         *follow the slope
         */
        ode::Object::ptr_t b
            (new ode::Box(env, Eigen::Vector3d(x, y, bsize/2 + (tan(tilt)*-x)),
                          10, bsize*4, bsize*4, bsize)); //multiply by 4 to stretch out

        b->set_rotation(0.0f, -tilt, 0.0f);
        boxes.push_back(b);
        if(!headless){
            b->accept(*v);
        }
        b->fix();
        env.add_to_ground(*b);
    }
}

