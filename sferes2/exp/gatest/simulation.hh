#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <stdlib.h> //For EXIT_SUCCESS

#include <ode/environment.hh>
#include <robot/robot4.hh>
#include <ode/box.hh>
#include <ode/object.hh>
#include <renderer/osg_visitor.hh>

class Simulation{
    private:
        std::unique_ptr<renderer::OsgVisitor> v;
        //renderer::OsgVisitor v;
        boost::shared_ptr<ode::Environment> env;
        boost::shared_ptr<robot::Robot> rob;
        std::vector<ode::Object::ptr_t> boxes;
        bool headless;
        float tilt;
        float x = 0;
    public:
        typedef boost::shared_ptr<robot::robot4> robot_t;
        typedef boost::shared_ptr<ode::Environment> env_t;

        Simulation(const robot_t&, float, int, int, bool);
        void add_blocks(int, int);
        //template<typename Indiv, typename Robot, typename Environment>
        template<typename Indiv>
            float run(Indiv, float, int);
        template<typename Indiv>
            void procedure(Indiv, float);
};

/* Robot4 servos
 * (F)ront/(R)ear
 * (U)pper/(L)ower
 * (R)ight/(L)eft
 * 0 - main/mid SWEEP
 * 1 - mid/rear SWEEP
 * 2 - F U R SWEEP
 * 3 - F L R DIHED
 * 4 - R U R SWEEP
 * 5 - R L R DIHED
 * 6 - F U L SWEEP
 * 7 - F L L DIHED
 * 8 - R U L SWEEP
 * 9 - R L L DIHED
 */
template<typename Indiv>
float Simulation::run(Indiv ind, const float step, const int step_limit){

    while(x < step_limit) {
        if(!headless){
            if(v->done()){ //If user presses escape in window
                exit(EXIT_SUCCESS); //abort everything including sferes-backend
            }
        }
        procedure(ind, step);
    }

    Eigen::Vector3d pos = rob->pos();
    //std::cout << "Fitness: " << -pos(0) << std::endl;
    return -pos(0);
}

template<typename Indiv>
void Simulation::procedure(Indiv ind, const float step){
    x += step;
    //rob.bodies()[1]->fix();
    if(!headless) {
        v->update();
    }
    rob->next_step(step);
    env->next_step(step);
    int genptr = 0;
    for (size_t i = 0; i < rob->servos().size() - 4; ++i){
        float a = ind.data(genptr++) * 40.0f;
        float theta = ind.data(genptr++) * 1.0f;
        float b = ind.data(genptr++) * 20.0f;;
        //std::cout << "Servo(" << i << "): " << a << " " << theta << " " << b << std::endl;
        float h = 4;
        float f = 2;

        double phase = a*tanh(h*sin((f*M_PI)*(x+theta))) + b;
        if(i <= 1){ //body joints only
            rob->servos()[i]->set_angle(ode::Servo::DIHEDRAL, phase * M_PI/180);
        }else{
            rob->servos()[i]->set_angle(ode::Servo::DIHEDRAL, phase * M_PI/180);
            rob->servos()[i+4]->set_angle(ode::Servo::DIHEDRAL, phase * M_PI/180); //and opposite for other side
        }
    }
}
#endif
