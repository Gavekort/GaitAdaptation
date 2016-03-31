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
            float run_ind(Indiv, float, int);
        float run_conf(std::vector<float>, float, int);
        void procedure(std::vector<float>, float);
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
float Simulation::run_ind(Indiv ind, const float step, const int step_limit){
    std::vector<float> data = ind.data();

    Eigen::Vector3d rotation;
    bool flipped = false;

    while(x < step_limit && !flipped) {
        if(!headless){
            if(v->done()){ //If user presses escape in window
                exit(EXIT_SUCCESS); //abort everything including sferes-backend
            }
        }
        procedure(data, step);
        rotation = rob->rot();

//        std::cout << "Rot: " << rotation(0) * (180/M_PI) << std::endl;
        if(abs(rotation(0) * (180/M_PI)) > 90){
            flipped = true;
            break;
        }
    }

    Eigen::Vector3d pos = rob->pos();

    //std::cout << "Fitness: " << -pos(0) << std::endl;
    if(!flipped){
        return -pos(0);
    }else{
        return 0.0f;
    }
}


#endif
