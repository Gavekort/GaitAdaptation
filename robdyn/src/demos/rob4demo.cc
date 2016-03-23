
#include <iostream>
#include <cmath>

#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>
#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"
#include "ode/environment.hh"
#include "robot/quadruped.hh"
#include "robot/robot4.hh"
#include "ode/box.hh"
#include "ode/object.hh"
#include "renderer/osg_visitor.hh"

int main(int argc, char **argv)
{
    dInitODE();

    typedef boost::mt19937 RNGType;
    RNGType rng( time(0) );
    boost::uniform_real<> loc_range(-3,3);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > rlocation(rng,loc_range);
    boost::uniform_real<> size_range(0.1,0.5);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > rsize(rng,size_range);
    std::cout << "RNG sanity: " << rlocation() << std::endl;

    renderer::OsgVisitor v;
    ode::Environment env(0.0f, 0.0f, 0.0f);
    env.set_gravity(0, 0, -9.81);
    std::cout << env.get_pitch() << " " << env.get_roll() << " " << env.get_z() << std::endl;
    robot::robot4 hyb(env, Eigen::Vector3d(0, 0, 0.2));
    static const float step = atof(argv[4]);
    hyb.accept(v);

    using namespace boost::assign;
    std::vector<size_t> b_servos = list_of(0)(3)(4)(7);

    float x = 0;
    float y = 0;
    dGeomID _ground = env.get_ground();
    std::vector<ode::Object::ptr_t> g;
    double bsize;
    for(int i = 0; i < 10; ++i){
        bsize = rsize();
        ode::Object::ptr_t b
            (new ode::Box(env, Eigen::Vector3d(rlocation(), rlocation(), bsize/2),
                          10, bsize, bsize, bsize));
        g.push_back(b);
        b->accept(v);
        b->fix();
        g.push_back(b);
    }
    while(1)
    {
        Eigen::Vector3d pos = hyb.pos();
        double dist = std::abs(pos(0)) + std::abs(pos(1)) + std::abs(pos(2));
        std::cout << dist << std::endl;
        x += step;
        v.update();
        env.next_step(step);
        hyb.next_step(step);
        double a = atof(argv[1]);
        double b = atof(argv[2]);
        double c = atof(argv[3]);
        double phase = a*tanh(4*sin((1*M_PI)*(x+b))) + c;

       /* for (size_t i = 0; i < hyb.servos().size(); ++i){
            if(i % 2){
                hyb.servos()[i]->set_angle(ode::Servo::SWEEP, phase/100);
            }else{
                hyb.servos()[i]->set_angle(ode::Servo::DIHEDRAL, phase/100);
            }
        }*/
                std::cout << phase << " " << phase * M_PI/180 << " " << cos(x) << std::endl;
        for (size_t i = 0; i < hyb.servos().size(); ++i){
        if(i <= 1){ //body joints only
            hyb.servos()[i]->set_angle(ode::Servo::DIHEDRAL, phase * M_PI/180);
        }else{
            if(i == 3 || i == 5 || i == 7 || i == 9){
                hyb.servos()[i]->set_angle(ode::Servo::DIHEDRAL, (phase * 1.8) * M_PI/180);
            }else{
                hyb.servos()[i]->set_angle(ode::Servo::DIHEDRAL, phase * M_PI/180);
            }
        }
        }
}
return 0;
}

void obstacles(ode::Environment* env, int size, int density){

}

