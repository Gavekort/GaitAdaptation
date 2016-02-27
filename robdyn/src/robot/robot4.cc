
#include "robot4.hh"
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
    void robot4 :: _build(Environment& env, const Vector3d& pos)
    {
        static const double body_mass = 0.25;
        static const double leg_mass = 0.15;
        /*static const double body_length = 0.75;
        static const double body_width = 0.2;
        static const double body_height = 0.2;
        static const double leg_w = 0.1;
        static const double leg_length = 0.6;
        static const double leg_dist = 0.2;
        static const double leg_mass = 0.2;
        static const double wheel_rad = 0.1;
        static const double wheel_mass = 0.05;*/

        //Robot dim definitions
        static const double segment_width = 0.1; //width of all segments

        static const double head_length = 0.25; //length of head
        static const double mid_length = 0.25; //mid segment
        static const double rear_length = 0.25; //rear segment

        static const double ufront_length = 0.22; //upper front leg
        static const double lfront_length = 0.22; //lower front leg

        static const double urear_length = 0.2; //upper rear leg
        static const double lrear_length = 0.2; //lower rear leg

        static const double ufront_stance = 2; //rotation of upper front joint
        static const double lfront_stance = -5; //lower front

        static const double urear_stance = 0; //upper rear
        static const double lrear_stance = 0; //lower rear

        static const double tfront_length = ufront_length + lfront_length;
        static const double trear_length = urear_length + lrear_length;

        _main_body = Object::ptr_t
            (new Box(env, pos + Vector3d(0, 0, 0), //pos is Eigen::Vector3d in Box
                     body_mass, head_length, segment_width, segment_width));
        _bodies.push_back(_main_body);


        Object::ptr_t _mid
            (new Box(env, pos + Vector3d(head_length/2 + mid_length/2, 0, 0),
                     body_mass, mid_length, segment_width, segment_width));
        _bodies.push_back(_mid);

        Object::ptr_t _rear
            (new Box(env, pos + Vector3d(head_length + mid_length, 0, 0),
                     body_mass, rear_length, segment_width, segment_width));
        _bodies.push_back(_rear);

        ode::Ax12::ptr_t s0_1
            (new ode::Ax12(env, pos + Vector3d(head_length/2, 0, 0), *_main_body, *_mid));
        _servos.push_back(s0_1);
        s0_1->set_axis(ode::Ax12::DIHEDRAL, Eigen::Vector3d(0,0,1));

        ode::Ax12::ptr_t s0_2
            (new ode::Ax12(env, pos + Vector3d(head_length/2 + mid_length, 0, 0), *_mid, *_rear));
        _servos.push_back(s0_2);
        s0_2->set_axis(ode::Ax12::DIHEDRAL, Eigen::Vector3d(0,0,1));

        for (size_t i = 0; i < 4; i += 2)
        {
            int left_right = i > 1 ? -1 : 1;

            Object::ptr_t l1
                (new CappedCyl(env, pos + Vector3d(0,
                                                   left_right * (segment_width/2 + ufront_length/2),
                                                   0),
                               leg_mass, segment_width/2, ufront_length));
            l1->set_rotation(M_PI/2, 0, 0);
            _bodies.push_back(l1);

            Ax12::ptr_t s1
                (new Ax12(env, pos + Vector3d(0,
                                               left_right * (segment_width / 2),
                                               0),
                           *_main_body, *l1));
            _servos.push_back(s1);
            s1->set_axis(ode::Ax12::DIHEDRAL, Eigen::Vector3d(0,0,1));

            Object::ptr_t l11
                (new CappedCyl(env, pos + Vector3d(0,
                                                   left_right *  ufront_length,
                                                   -lfront_length/2),
                               leg_mass, segment_width/2, lfront_length));
            _bodies.push_back(l11);


            Ax12::ptr_t s2
                (new Ax12(env, pos + Vector3d(0,
                                               left_right * ufront_length,
                                               0),
                           *l1, *l11));
            _servos.push_back(s2);
            s2->set_axis(ode::Ax12::SWEEP, Eigen::Vector3d(0,0,1));

            //tail-segment
            Object::ptr_t bl1
                (new CappedCyl(env, pos + Vector3d(head_length + mid_length, //X (which is 0) + skew down body_length
                                                   left_right * (segment_width/2 + urear_length/2),
                                                   0), //Z
                               leg_mass, segment_width/2, urear_length));
            bl1->set_rotation(M_PI/2, 0, 0);
            _bodies.push_back(bl1);

            Ax12::ptr_t bs1
                (new Ax12(env, pos + Vector3d(head_length + mid_length,
                                               left_right * (segment_width / 2),
                                               0),
                           *_rear, *bl1));
            _servos.push_back(bs1);
            bs1->set_axis(ode::Ax12::DIHEDRAL, Eigen::Vector3d(0,0,1));

            Object::ptr_t bl11
                (new CappedCyl(env, pos + Vector3d(head_length + mid_length,
                                                   left_right * urear_length,
                                                   -lrear_length/2),
                               leg_mass, segment_width/2, lrear_length));
            _bodies.push_back(bl11);

            Ax12::ptr_t bs2
                (new Ax12(env, pos + Vector3d(head_length + mid_length,
                                               left_right * (segment_width / 2 + urear_length/2),
                                               0),
                           *bl1, *bl11));
            _servos.push_back(bs2);
            bs2->set_axis(ode::Ax12::SWEEP, Eigen::Vector3d(0,0,1));
        }
        //for (size_t i = 0; i < _servos.size(); ++i)
            //for (size_t j = 0; j < 3; ++j)
                //_servos[i]->set_lim(j, -M_PI / 3, M_PI / 3);
    }
}

