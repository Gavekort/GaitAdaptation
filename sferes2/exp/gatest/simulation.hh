#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <sferes/gen/evo_float.hpp>
#include <sferes/ea/nsga2.hpp>

#include <ode/environment.hh>
#include <robot/robot4.hh>
#include <ode/box.hh>
#include <ode/object.hh>
#include <renderer/osg_visitor.hh>

class Simulation{
    private:
        renderer::OsgVisitor* v;
        ode::Environment* env;
        robot::robot4* rob;
    public:
        Simulation();
        ~Simulation();
        //void add_blocks(int, int, ode::Environment&);
        template<typename Indiv>
            float run(Indiv, float);
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
float Simulation::run(Indiv ind, const float step){
    std::cout << ind.size() << std::endl;
    float x = 0;
    while(!v->done() || x < 2) //2 is stepping, TODO: Make parameter
    {
        x += step;
        v->update();
        env->next_step(step);
        rob->next_step(step);
        int genptr = 0;
        for (size_t i = 0; i < rob->servos().size(); ++i){
            float a = ind.data(genptr++) * 20.0f;
            float theta = ind.data(genptr++) * 1.0f;
            float b = ind.data(genptr++) * 20.0f;;
            std::cout << "Servo(" << i << "): " << a << " " << theta << " " << b << std::endl;
            float h = 4;
            float f = 2;
            double phase = a*tanh(h*sin((f*M_PI)*(x+theta))) + b;
            if(i <= 1){
                rob->servos()[i]->set_angle(ode::Servo::SWEEP, phase * M_PI/180); // -2 to 2 is approx 180 deg
            }else{
                rob->servos()[i]->set_angle(ode::Servo::DIHEDRAL, phase * M_PI/180);
            }
        }
    }
    Eigen::Vector3d pos = rob->pos();
    return std::abs(pos(0)) + std::abs(pos(1)) + std::abs(pos(2)); //Dirty?

}


#endif
