#ifndef SYSTEMS_KINEMAIC_BICYCLE_MODEL_STEERING_DELAY_H_
#define SYSTEMS_KINEMAIC_BICYCLE_MODEL_STEERING_DELAY_H_

#include <mpc_local_planner/systems/kinematic_bicycle_model.h>

#include <cmath>

namespace mpc_local_planner {

class KinematicBicycleModelSteeringDelay : public KinematicBicycleModelVelocityInput 
{
    public:
        KinematicBicycleModelSteeringDelay() = default;

        // construct from wheelbase and steering delay
        KinematicBicycleModelSteeringDelay(double lr, double lf, double tau) :
            _lr(lr),
            _lf(lf),
            _tau(tau) {}
        
        SystemDynamicsInterface::Ptr getInstance() const override 
        {
            return std::make_shared<KinematicBicycleModelSteeringDelay>();
        }

        int getStateDimension() const override { return 4;}

        // new model
        void dynamics(const Eigen::Ref<const StateVector>& x, 
                      const Eigen::Ref<const ControlVector>& u,
                      Eigen::Ref<StateVector> f) const override
        {
            assert(x.size() == getStateDimension());
            assert(u.size() == getInputDimension());
            assert(x.size() == f.size());

            // state: [ x, y, theta, measured_steering_angle]
            // control: [vel_x, cmd_steering_angle]
            double beta = std::atan(_lr / (_lf + _lr) * std::tan(x[3])); 

            f[0] = u[0] * std::cos(x[2] + beta);
            f[1] = u[0] * std::sin(x[2] + beta);
            f[2] = u[0] * std::sin(beta) / _lr;
            f[3] = - (x[3] - u[1]) / _tau;

        }

        void setParameters(double lr, double lf, double tau)
        {
            _lf = lf;
            _lr = lr;
            _tau = tau;
        }

        double getSteeringDelay() const {return _tau;}
        
    protected:
        double _lr = 1.0;
        double _lf = 1.0;
        double _tau = 0.7;
};

} // end of namespace mpc_local_planner

#endif
