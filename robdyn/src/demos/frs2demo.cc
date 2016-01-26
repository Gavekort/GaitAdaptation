
#include <iostream>
#include <cmath>

#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>
#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"
#include "ode/environment.hh"
#include "robot/quadruped.hh"
#include "robot/frs2bot.hh"
#include "ode/box.hh"
#include "ode/object.hh"
#include "renderer/osg_visitor.hh"

int main(int argc, char **argv)
{
    dInitODE();

    typedef boost::mt19937 RNGType;
    //RNGType rng;
    RNGType rng( time(0) );
    boost::uniform_real<> loc_range(-3,3);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > rlocation(rng,loc_range);
    boost::uniform_real<> size_range(0.1,0.5);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > rsize(rng,size_range);
    std::cout << "RNG sanity: " << rlocation() << std::endl;

    renderer::OsgVisitor v;
    ode::Environment env(25.0f, 0.0f, 0.0f);
    env.set_gravity(0, 0, -9.81);
    std::cout << env.get_pitch() << " " << env.get_roll() << " " << env.get_z() << std::endl;
    robot::frs2bot hyb(env, Eigen::Vector3d(0, 0, 0.5));
    static const float step = atof(argv[4]);
    hyb.accept(v);

    //    float init = 0;
    //    float ampl = 0.3;
    //    float center = 0.6;
    //    static const int clockwise = -1;
    //    static const int counter_clockwise = 1;

    using namespace boost::assign;
    std::vector<size_t> b_servos = list_of(0)(3)(4)(7);

    float x = 0;
    float y = 0;
    dGeomID _ground = env.get_ground();
    std::vector<ode::Object::ptr_t> g;
    double bsize;
    for(int i = 0; i < 10; ++i){
        //ode::Box::ptr_t
        //    b(new ode::Box(env, Eigen::Vector3d(rlocation(),rlocation(),0), 10,
        //                rsize(), rsize(), rsize;
        bsize = rsize();
        ode::Object::ptr_t b
            (new ode::Box(env, Eigen::Vector3d(rlocation(), rlocation(), 0.5),
                          10, bsize, bsize, bsize));
        g.push_back(b);
        //        b->set_rotation(-25.0f, 0.0f, 0.0f);
        b->accept(v);
        b->fix();
        //env.add_to_ground(*b);
        //env._collision(b.get_geom(), _ground);
        g.push_back(b);
    }
    while(!v.done())
    {
        x += step;
        v.update();
        env.next_step(step);
        hyb.next_step(step);
        double a = atof(argv[1]);
        double b = atof(argv[2]);
        double c = atof(argv[3]);
        double phase = a*tanh(4*sin((2*M_PI)*(x+b))) + c;
        std::cout << phase << " " << -cos(x) << std::endl;

        for (size_t i = 0; i < hyb.servos().size(); ++i){
            if(i % 2){
                hyb.servos()[i]->set_angle(ode::Servo::SWEEP, phase/100);
            }else{
                hyb.servos()[i]->set_angle(ode::Servo::DIHEDRAL, phase/100);
            }
        }
        /*
           for(auto b : g){
        //b->set_in_contact(false);
        }
        for (size_t i = 0; i < hyb.servos().size(); ++i)
        {
        //float phase = (i < 4 ? 0 : 1) * M_PI / 2.0f;
        //bool bot = std::find(b_servos.begin(), b_servos.end(), i) != b_servos.end();
        //int c_shift = (bot ? -1 : 1);
        //hyb.servos()[i]->set_angle(ode::Servo::TWIST, center * c_shift);
        //				     + (i % 2 ? -1 : 1) * sin(x + phase) * ampl);
        }
        if(x < 2){
        hyb.servos()[0]->set_angle(ode::Servo::DIHEDRAL, -cos(x));
        }
        if(x == 0 || (x > 2 && x < 5)){
        for (size_t i = 1; i < hyb.servos().size(); ++i){
        if(i % 2){
        hyb.servos()[i]->set_angle(ode::Servo::SWEEP, -cos(y));
        }else{
        hyb.servos()[i]->set_angle(ode::Servo::DIHEDRAL, -cos(y));
        }
        }
        y += step;
        }
        x += step;
        }
        */

}
return 0;
}

void obstacles(ode::Environment* env, int size, int density){

}

