//| This file is a part of the ERC ResiBots project.
//| Copyright 2015, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jean-Baptiste Mouret, mouret@isir.fr
//|                      Antoine Cully, cully@isir.upmc.fr
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software.  You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.

//#define SHOW_TIMER
#include <limbo/limbo.hpp>
#include <limbo/inner_cmaes.hpp>
#include "exhaustiveSearchMap.hpp"
#include "meanMap.hpp"
#include "statTransferts.hpp"

#ifdef GRAPHIC
//#define NO_PARALLEL
#include "renderer/osg_visitor.hh"
#endif

//#include "hexapod.hh"
//#include "simu.hpp"
#include "robot4.hh"
#include "simulation.hh"

#define NO_PARALLEL
#include "limbo/parallel.hpp"

using namespace limbo;

struct Params {
    struct boptimizer {
        BO_PARAM(double, noise, 0.001);
        BO_PARAM(int, dump_period, 1);
    };
    struct maxiterations {
        BO_DYN_PARAM(int, n_iterations);
    };
    struct maxpredictedvalue {
        BO_PARAM(float, ratio, 0.9);
    };

    struct kf_maternfivehalfs {
        BO_PARAM(float, sigma, 1);
        BO_DYN_PARAM(float, l);
    };
    struct ucb {
        BO_DYN_PARAM(float, alpha);
    };

struct archiveparams {

        struct elem_archive {
            std::vector<float> params;
            float fit;
            std::vector<float> controller;
        };

        struct classcomp {
            bool operator()(const std::vector<float>& lhs, const std::vector<float>& rhs) const
            {
                assert(lhs.size() == 2 && rhs.size() == 2);
                int i = 0;
                while (i < 1 && round(lhs[i] * 4) == round(rhs[i] * 4)) //lhs[i]==rhs[i])
                    i++;
                return round(lhs[i] * 4) < round(rhs[i] * 4); //lhs[i]<rhs[i];
            }
        };
        typedef std::map<std::vector<float>, elem_archive, classcomp> archive_t;
        static std::map<std::vector<float>, elem_archive, classcomp> archive;
    };
};

Params::archiveparams::archive_t load_archive(std::string archive_name);
Params::archiveparams::archive_t create_random_map(int size);

namespace global {
    struct timeval timev_selection; // Initial absolute time (static)
    std::string res_dir;
    boost::shared_ptr<robot::robot4> orob;
    boost::shared_ptr<ode::Environment> oenv;
};
///---------------------------

//hexa_control::Transfert srv;

template <typename Params>
struct fit_eval_map {

    BOOST_STATIC_CONSTEXPR int dim = 2;
    fit_eval_map()
    {
        timerclear(&global::timev_selection);
        gettimeofday(&global::timev_selection, NULL);
    }

    float operator()(Eigen::VectorXd x) const
    {
        std::cout << "start eval" << std::endl;
        std::vector<float> key(x.size(), 0);
        for (int i = 0; i < x.size(); i++)
            key[i] = x[i];
        if (Params::archiveparams::archive.count(key) == 0)
            return -1000;

        for(auto data : Params::archiveparams::archive.at(key).params){
        std::cout << data << " ";
        }
        std::cout << std::endl;

        //Simu simu = Simu(Params::archiveparams::archive.at(key).controller, global::global_robot, global::brokenLegs, false, 5, 1, global::global_env->angle);
        #ifdef GRAPHIC
        Simulation sim(global::orob, -0.05f, 100, 10, false);
        #else
        Simulation sim(global::orob, 0.00f, 10, 6, true);
        #endif
        float result = sim.run_conf(Params::archiveparams::archive.at(key).params, 0.008f, 4);
        //if (simu.covered_distance() < 0 || simu.covered_distance() > 2.5) {

        //    std::cout << simu.covered_distance() << " measurement seems wrong, set to zero" << std::endl;
            return result;
        //}

        //return simu.covered_distance() * limbo::misc::gaussian_rand(0.95, 0.1);
    }
};

std::map<std::vector<float>, Params::archiveparams::elem_archive, Params::archiveparams::classcomp>
load_archive(std::string archive_name){

    std::map<std::vector<float>, Params::archiveparams::elem_archive, Params::archiveparams::classcomp> archive;

    std::ifstream monFlux(archive_name.c_str()); //Ouverture d'un fichier en lecture
    if (monFlux) {
        while (!monFlux.eof()) {
            Params::archiveparams::elem_archive elem;
            std::vector<float> candidate(2);
            //std::cout << "Data: ";
            for (int i = 0; i < 23; i++) { //43 is amount of elements in columns
                if (monFlux.eof())
                    break;
                float data;
                monFlux >> data;
               // std::cout << data << " ";
                if (i == 1 || i == 2){
                    candidate[i-1] = data; //I on the other hand have duplicated the candidate data
                }
                if (i == 3) {
                    elem.fit = data;
                }
                if (i >= 4)
                    elem.params.push_back(data);
            }
            //std::cout << " |Size| " << candidate << std::endl;
            if (elem.params.size() == 19) {
                std::cout << "Candidate: " << candidate.at(0) << " " << candidate.at(1) << " : " << elem.fit << std::endl;
                archive[candidate] = elem;
            }
        }
    }
    else {
        std::cout << "ERREUR: Impossible d'ouvrir le fichier en lecture." << std::endl;
        return archive;
    }
    std::cout << archive.size() << " elements loaded" << std::endl;
    return archive;
}


Params::archiveparams::archive_t Params::archiveparams::archive;
BO_DECLARE_DYN_PARAM(float, Params::kf_maternfivehalfs, l);
BO_DECLARE_DYN_PARAM(int, Params::maxiterations, n_iterations);
BO_DECLARE_DYN_PARAM(float, Params::ucb, alpha);

int main(int argc, char** argv)
{

    if (argc < 2) {
        std::cout << "please provide a map" << std::endl;
        return -1;
    }
    Params::archiveparams::archive = load_archive(argv[1]);

    if (argc > 2)
        Params::kf_maternfivehalfs::set_l(atof(argv[2]));
    else
        Params::kf_maternfivehalfs::set_l(0.4); //0.4 (antoine value)

    Params::ucb::set_alpha(0.05);
    Params::maxiterations::set_n_iterations(20);
    srand(time(NULL));

    typedef kernel_functions::MaternFiveHalfs<Params> Kernel_t;
    typedef inner_optimization::ExhaustiveSearchArchive<Params> InnerOpt_t;
    typedef boost::fusion::vector<stopping_criterion::MaxIterations<Params>, stopping_criterion::MaxPredictedValue<Params>> Stop_t;
    typedef mean_functions::MeanArchive_Map<Params> Mean_t;
    typedef boost::fusion::vector<stat::Acquisitions<Params>, stat::StatTransferts<Params>> Stat_t;

    typedef init_functions::NoInit<Params> Init_t;
    typedef model::GP<Params, Kernel_t, Mean_t> GP_t;
    typedef acquisition_functions::UCB<Params, GP_t> Acqui_t;

    dInitODE();

    global::oenv = boost::shared_ptr<ode::Environment>(new ode::Environment(0.0f, 0.0f, 0.0f));
    global::orob = boost::shared_ptr<robot::robot4>(new robot::robot4(*global::oenv, Eigen::Vector3d(0, 0, 0.5)));

    BOptimizer<Params, model_fun<GP_t>, init_fun<Init_t>, acq_fun<Acqui_t>, inneropt_fun<InnerOpt_t>, stat_fun<Stat_t>, stop_fun<Stop_t>> opt;
    global::res_dir = opt.res_dir();

    Eigen::VectorXd result(1);

    opt.optimize(fit_eval_map<Params>());
    float val = opt.best_observation();
    result = opt.best_sample().transpose();

        Simulation sim(global::orob, 0.00f, 10, 6, false);
        sim.run_conf(Params::archiveparams::archive.at(result).params, 0.008f, 4);

    std::cout << val << " res  " << result.transpose() << std::endl;

    dCloseODE();

    std::cout << "fin" << std::endl;

    return 0;
}
