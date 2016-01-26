#include <iostream>

#include <boost/foreach.hpp>
#include "ode/environment.hh"
#include "ode/capped_cyl.hh"
#include "renderer/osg_visitor.hh"
#include "ode/mx28.hh"
#include "ode/box.hh"
int main()
{

    static const double groundoffset = 0.5;

    static const double bodyjoint_m = 0.5;
    static const double bodyjoint_l = 0.5;
    static const double bodyjoint_w = 0.25;
    static const double bodyjoint_h = 0.1;

    static const double legjoint_m = 0.5;
    static const double legjoint_l = 0.5;
    static const double legjoint_w = 0.2;
    static const double legjoint_h = 0.1;

    dInitODE();

/*
    ode::Object::ptr_t p3
        (new ode::Box(env, Eigen::Vector3d(0, 0, 1.5 + 0.5),
                      0.1, 0.2,0.1, 0.5));

    robot.push_back(p3);


*/

    renderer::OsgVisitor v;
    ode::Environment env;
    //env.set_gravity(0, 0, -9.81);
    env.set_gravity(0, 0, 0);
    std::vector<ode::Object::ptr_t> robot;
    std::vector<ode::Servo::ptr_t> servos;

    //Main body

    ode::Object::ptr_t fbody
        (new ode::Box(env,Eigen::Vector3d(0, 0, groundoffset), //Vector(x, y, z)
                      bodyjoint_m, bodyjoint_l, bodyjoint_w, bodyjoint_h)); //mass, length, width, height
    robot.push_back(fbody);



    /*  fbody->set_rotation(Eigen::Vector3d(0,1 , 0),
        Eigen::Vector3d(0, 0, 1));*/

    ode::Object::ptr_t bbody
        (new ode::Box(env, Eigen::Vector3d(bodyjoint_l, 0, groundoffset),
                      bodyjoint_m, bodyjoint_l, bodyjoint_w, bodyjoint_h)); //mass, length, width, height

    robot.push_back(bbody);

    ode::Servo::ptr_t s1
        (new ode::Mx28(env, Eigen::Vector3d(0.25, 0, 0.5), *fbody, *bbody));
    servos.push_back(s1);
    s1->set_axis(0, Eigen::Vector3d(0,0,1));

    //Legs

    //Front-Right Upper Join leg
    ode::Object::ptr_t frujleg //                      Rotational offset
        (new ode::Box(env,Eigen::Vector3d(0, bodyjoint_l - bodyjoint_w/2, groundoffset), //Vector(x, y, z
                      bodyjoint_m, bodyjoint_l, bodyjoint_w, bodyjoint_h)); //mass, length, width, height
    robot.push_back(frujleg);
    frujleg->set_rotation(0,0,1,M_PI/2);


    ode::Servo::ptr_t sfruj
        (new ode::Mx28(env, Eigen::Vector3d(0.25, -bodyjoint_w/2, 0.5), *fbody, *frujleg));
    servos.push_back(sfruj);
    sfruj->set_axis(0, Eigen::Vector3d(0,0,1));

    ode::Object::ptr_t flujleg //                      Rotational offset
        (new ode::Box(env,Eigen::Vector3d(0, -bodyjoint_l + bodyjoint_w/2, groundoffset), //Vector(x, y, z
                     bodyjoint_m, bodyjoint_l, bodyjoint_w, bodyjoint_h)); //mass, length, width, height
    robot.push_back(flujleg);
    flujleg->set_rotation(0,0,1,M_PI/2);

    ode::Servo::ptr_t sfluj
        (new ode::Mx28(env, Eigen::Vector3d(0.25, bodyjoint_w/2, 0.5), *fbody, *flujleg));
    servos.push_back(sfluj);
    sfluj->set_axis(0, Eigen::Vector3d(0,0,1));

/*
    ode::Servo::ptr_t s2
        (new ode::Mx28(env, Eigen::Vector3d(0, 0, 1 + 0.75), *bbody, *p3));
    servos.push_back(s2);
    s2->set_axis(0, Eigen::Vector3d(1,0,0));
    s2->set_axis(2, Eigen::Vector3d(0,1,0));
    */
   // fbody->set_rotation(M_PI,1,0,M_PI/4);
   bbody->fix();
    float x = 0;
    while(!v.done())
    {
        v.visit(robot);
        v.update();
        env.next_step(0.001);
        //BOOST_FOREACH(ode::Servo::ptr_t s, servos)
        for(auto s : servos){
            s->next_step(0.001);
        }
        sfruj->set_angle(ode::Servo::DIHEDRAL, cos(x));
        sfluj->set_angle(ode::Servo::DIHEDRAL, -cos(x));
        s1->set_angle(ode::Servo::DIHEDRAL, cos(x));
     //   s2->set_angle(ode::Servo::DIHEDRAL, -cos(x));

        x += 0.01;
    }

    return 0;
}

