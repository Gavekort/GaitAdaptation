#include <iostream>
#include <sferes/phen/parameters.hpp>
#include <sferes/gen/evo_float.hpp>
#include <sferes/ea/nsga2.hpp>
#include <sferes/eval/eval.hpp>
#include <sferes/stat/pareto_front.hpp>
#include <sferes/stat/best_fit.hpp>
#include <sferes/stat/mean_fit.hpp>
#include <sferes/modif/dummy.hpp>
#include <sferes/run.hpp>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>
#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

#include <sferes/stat/stat.hpp>

#include "simulation.hh"

#include "map_elites.hpp"
#include "fit_map.hpp"
#include "stat_map.hpp"
#include <sferes/gen/sampled.hpp>

//#include "stat_progress_archive.hpp"

using namespace sferes;
using namespace sferes::gen::evo_float;

boost::shared_ptr<robot::robot4> orob;
boost::shared_ptr<ode::Environment> oenv;

struct Params {
    struct ea {
        SFERES_CONST size_t behav_dim = 2;
        SFERES_CONST double epsilon = 0;//0.05;
        SFERES_ARRAY(size_t, behav_shape, 256, 256);
    };
    struct pop {
        // number of initial random points
        SFERES_CONST size_t init_size = 300;
        // size of a batch
        SFERES_CONST size_t size = 300;
        SFERES_CONST size_t nb_gen = 100000;
        SFERES_CONST size_t dump_period = 5;
    };
    struct parameters {
        SFERES_CONST float min = 0.0f;
        SFERES_CONST float max = 1.0f;
    };
    struct evo_float {
        SFERES_CONST float cross_rate = 0.25f;
        SFERES_CONST float mutation_rate = 0.1f;
        SFERES_CONST float eta_m = 15.0f;
        SFERES_CONST float eta_c = 10.0f;
        SFERES_CONST mutation_t mutation_type = polynomial;
        SFERES_CONST cross_over_t cross_over_type = sbx;
    };
};

FIT_MAP(GaitOpt){
    public :
        GaitOpt()  {}
        template<typename Indiv>
            void eval(Indiv& ind) {
                if (this->mode() == sferes::fit::mode::view){
                    Simulation sim(orob, 0.00f, 100, 30, false);
                    sim.run_ind(ind, 0.004f, 8);
                }else{
                    Simulation sim(orob, 0.00f, 100, 10, true);
                    float result = sim.run_ind(ind, 0.008f, 8);
                    this->_value = result;

                    std::vector<float> data;
                    data.push_back((ind.gen().data(6)+ind.gen().data(12))/2);//amplitudes of joints that lift
                    data.push_back((ind.gen().data(9)+ind.gen().data(15))/2);//amplitudes of joints that sweep

                    this->set_desc(data);
                }
            }

        bool dead(){
            return false;
        }
};

int main(int argc, char **argv) {
    std::cout<<"running "<<argv[0]<<" ... try --help for options (verbose)"<<std::endl;
    dInitODE2(0);
    oenv = boost::shared_ptr<ode::Environment>(new ode::Environment(0.0f, 0.0f, 0.0f));
    orob = boost::shared_ptr<robot::robot4>(new robot::robot4(*oenv, Eigen::Vector3d(0, 0, 0.5)));
    typedef gen::EvoFloat<18, Params> gen_t;
    typedef phen::Parameters<gen_t, GaitOpt<Params>, Params> phen_t;
    typedef eval::Parallel<Params> eval_t;
    typedef boost::fusion::vector<stat::Map<phen_t, Params>, stat::BestFit<phen_t, Params> > stat_t;
    typedef modif::Dummy<> modifier_t;
    typedef ea::MapElites<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
    ea_t ea;

    run_ea(argc, argv, ea);
    dCloseODE();
    return 0;
}
