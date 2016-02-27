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

#include "simulation.hh"

using namespace sferes;
using namespace sferes::gen::evo_float;


struct Params {
    struct evo_float {
        SFERES_CONST float cross_rate = 0.5f;
        SFERES_CONST float mutation_rate = 0.1f;
        SFERES_CONST float eta_m = 15.0f;
        SFERES_CONST float eta_c = 10.0f;
        SFERES_CONST mutation_t mutation_type = polynomial;
        SFERES_CONST cross_over_t cross_over_type = sbx;
    };
    struct pop {
        SFERES_CONST unsigned size = 300;
        SFERES_CONST unsigned nb_gen = 100;
        SFERES_CONST int dump_period = 5;
        SFERES_CONST int initial_aleat = 1;
        SFERES_CONST float coeff = 1.1f;
        SFERES_CONST float keep_rate = 0.6f;
    };
    struct parameters {
        SFERES_CONST float min = 0.0f;
        SFERES_CONST float max = 1.0f;
    };
};


SFERES_FITNESS(FitZDT2, sferes::fit::Fitness) {
    public:
        FitZDT2()  {}
        template<typename Indiv>
            void eval(Indiv& ind) {
                if (this->mode() == sferes::fit::mode::view)
                {
                    Simulation sim(0.00f, 100, 3, false);
                    sim.run(ind, 0.004f, 30);
                }else{
                    this->_objs.resize(1);
                    Simulation sim(0.00f, 100, 3, true);
                    this->_objs[0] = sim.run(ind, 0.008f, 8);
                }
            }
};

int main(int argc, char **argv) {
    std::cout<<"running "<<argv[0]<<" ... try --help for options (verbose)"<<std::endl;
    //headless = atoi(argv[1]);
    typedef gen::EvoFloat<18, Params> gen_t;
    typedef phen::Parameters<gen_t, FitZDT2<Params>, Params> phen_t;
    typedef eval::Parallel<Params> eval_t;
    typedef boost::fusion::vector<stat::ParetoFront<phen_t, Params> >  stat_t;
    typedef modif::Dummy<> modifier_t;
    typedef ea::Nsga2<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
    ea_t ea;

    dInitODE2(0);
    run_ea(argc, argv, ea);
    //  std::cout<<"==> mean fitness ="<<ea.stat<1>().mean()<<std::endl;

    dCloseODE();
    return 0;
}
