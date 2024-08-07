#include <controller/pid_controller.h>

using namespace cell_world;
using namespace std;

#define RATIO 1.0

namespace controller{

    Pid_outputs Pid_controller::process(const Pid_inputs &inputs, Behavior behavior) {
        double speed, P_value, I_value, D_value;
        double t = timer.to_seconds();
        timer.reset();
        if (behavior==Explore){
            speed = parameters.explore_speed;
            P_value = parameters.P_explore;
            I_value = parameters.I_explore;
            D_value = parameters.D_explore;
        } else {
            speed = parameters.pursue_speed;
            P_value = parameters.P_pursue;
            I_value = parameters.I_pursue;
            D_value = parameters.D_pursue;
        }
        in = inputs;
        auto dist = inputs.location.dist(inputs.destination);
        double destination_theta = inputs.location.atan(inputs.destination);
        auto theta = to_radians(inputs.rotation);
        error = angle_difference(theta, destination_theta) * direction(theta, destination_theta);

        normalized_error = normalize_error(error);
        error_derivative = (last_error - error)/t;

        // TODO: tune this
        last_error = error;
        error_integral += error * t;
//        cout << "Integral error: " << error_integral << endl;

        // eliminate integral windup (100)
        if (error_integral > 10) {
            error_integral = 10;
        } else if (error_integral < -10){
            error_integral = -10;
        }


        double adjustment = error * P_value - error_derivative * D_value + error_integral * I_value;
//        cout << "ADJUSTMENT: "<< adjustment << endl;
//        out.left =  normalized_error * speed * ( dist + 1 ) - adjustment;
//        out.right = normalized_error * speed * ( dist + 1 ) + adjustment;
        double dist_param = 1.0;
        out.left =  normalized_error * speed * ( dist + dist_param) - adjustment;
        out.right = normalized_error * speed * ( dist + dist_param) + adjustment;
        // catches outliers
        float max = abs(out.left);
        if (abs(out.right)>max) max = abs(out.right);
        if (max > 1){
            out.right = out.right / max;
            out.left = out.left / max;
        }
        if ((out.right < 0 && out.left > 0) || (out.right > 0 && out.left <0 )) {
            out.right *= RATIO;
            out.left *= RATIO;
        }
//        cout << out.left << " " << out.right << " " << theta << " " << destination_theta << " " << dist << endl;
        return out;
    }

    double Pid_controller::normalize_error(double error){
        double pi_err =  M_PI * error / 2;

//        double pi_err =  2 *M_PI * error ;
//        return 0.5 / ( (pi_err * pi_err) + 1 );

        return 1 / ( (pi_err * pi_err) + 1 );
    }

    Pid_controller::Pid_controller(const Pid_parameters &parameters): parameters(parameters) {
        timer.reset();
    }
}