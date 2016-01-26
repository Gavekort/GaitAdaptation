
#include "frs2bot.hh"
#include "ode/box.hh"
#include "ode/capped_cyl.hh"
#include "ode/sphere.hh"
#include "ode/motor.hh"
#include "ode/mx28.hh"
#include "ode/ax12.hh"




using namespace ode;
using namespace Eigen;

namespace robot
{
    void frs2bot :: _build(Environment& env, const Vector3d& pos)
    {
        static const double body_mass = 1;
        static const double body_length = 0.75;
        static const double body_width = 0.2;
        static const double body_height = 0.2;
        static const double leg_w = 0.1;
        static const double leg_length = 0.6;
        static const double leg_dist = 0.2;
        static const double leg_mass = 0.2;
        static const double wheel_rad = 0.1;
        static const double wheel_mass = 0.05;

        _main_body = Object::ptr_t
            (new Box(env, pos + Vector3d(0, 0, leg_length), //pos is Eigen::Vector3d in Box
                     body_mass, body_length, body_width, body_height));
        _bodies.push_back(_main_body);


        Object::ptr_t tail_body
            (new Box(env, pos + Vector3d(body_length, 0, leg_length),
                     body_mass, body_length, body_width, body_height));
        _bodies.push_back(tail_body);

        ode::Ax12::ptr_t s0
            (new ode::Ax12(env, pos + Vector3d(body_length/2, 0, leg_length), *_main_body, *tail_body));
        _servos.push_back(s0);
        s0->set_axis(ode::Ax12::DIHEDRAL, Eigen::Vector3d(0,0,1));

        for (size_t i = 0; i < 4; i += 2)
        {
            int left_right = i > 1 ? -1 : 1;
            int back_front = i % 2 == 0 ? -1 : 1;

            Object::ptr_t l1
                (new CappedCyl(env, pos + Vector3d(0,
                                                   left_right * (body_width/2 + leg_length/2),
                                                   leg_length),
                               leg_mass, leg_w, leg_length));
            l1->set_rotation(M_PI/2, 0, 0);
            _bodies.push_back(l1);

            Ax12::ptr_t s1
                (new Ax12(env, pos + Vector3d(0,
                                               left_right * (body_width / 2),
                                               leg_length),
                           *_main_body, *l1));
            _servos.push_back(s1);

            Object::ptr_t l11
                (new CappedCyl(env, pos + Vector3d(0,
                                                   left_right * (body_width / 2 + leg_length),
                                                   leg_length  / 2 + wheel_rad),
                               leg_mass, leg_w, leg_length));
            _bodies.push_back(l11);

            Ax12::ptr_t s2
                (new Ax12(env, pos + Vector3d(0,
                                               left_right * (body_width / 2 + leg_length),
                                               leg_length),
                           *l1, *l11));
            _servos.push_back(s2);

            //tail-segment
            Object::ptr_t bl1
                (new CappedCyl(env, pos + Vector3d(body_length, //X (which is 0) + skew down body_length
                                                   left_right * (body_width/2 + leg_length/2),
                                                   leg_length), //Z
                               leg_mass, leg_w, leg_length));
            bl1->set_rotation(M_PI/2, 0, 0);
            _bodies.push_back(bl1);

            Ax12::ptr_t bs1
                (new Ax12(env, pos + Vector3d(body_length,
                                               left_right * (body_width / 2),
                                               leg_length),
                           *tail_body, *bl1));
            _servos.push_back(bs1);

            Object::ptr_t bl11
                (new CappedCyl(env, pos + Vector3d(body_length,
                                                   left_right * (body_width / 2 + leg_length),
                                                   leg_length  / 2 + wheel_rad),
                               leg_mass, leg_w, leg_length));
            _bodies.push_back(bl11);

            Ax12::ptr_t bs2
                (new Ax12(env, pos + Vector3d(body_length,
                                               left_right * (body_width / 2 + leg_length),
                                               leg_length),
                           *bl1, *bl11));
            _servos.push_back(bs2);
        }
        //for (size_t i = 0; i < _servos.size(); ++i)
            //for (size_t j = 0; j < 3; ++j)
                //_servos[i]->set_lim(j, -M_PI / 3, M_PI / 3);
    }
}

