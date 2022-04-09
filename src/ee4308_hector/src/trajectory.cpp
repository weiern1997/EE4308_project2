#include "trajectory.hpp"

std::vector<Position> generate_trajectory(Position pos_begin, Position pos_end, Position pos_future,double cur_speed, double average_speed, double target_dt, Grid & grid)
{
    // (1) estimate total duration
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double duration = sqrt(Dx*Dx + Dy*Dy) / average_speed;

    // (2) generate cubic / quintic trajectory
    // done by students
    std::vector<double> a(4,0);
    std::vector<double> b(4,0);
    double xi_prime, yi_prime, xf_prime, yf_prime;
    double angle = atan2(pos_end.y-pos_begin.y, pos_end.x-pos_begin.x);
    xi_prime = cur_speed * cos(angle);
    yi_prime = cur_speed * sin(angle);
    angle = atan2(pos_future.y-pos_end.y, pos_future.x-pos_end.x);
    xf_prime = average_speed * cos(angle);
    yf_prime = average_speed * sin(angle);

    a[0] = pos_begin.x;
    a[1] = xi_prime;
    a[2] = (-3/(duration*duration))*pos_begin.x + (-2/duration)*xi_prime
            + (3/(duration*duration))*pos_end.x + (-1/duration)*xf_prime;
    a[3] = (2/(duration*duration*duration))*pos_begin.x + (1/(duration*duration))*xi_prime
            + (-2/(duration*duration*duration))*pos_end.x + (1/(duration*duration))*xf_prime;
            
    b[0] = pos_begin.y;
    b[1] = yi_prime;
    b[2] = (-3/(duration*duration))*pos_begin.y + (-2/duration)*yi_prime
            + (3/(duration*duration))*pos_end.y + (-1/duration)*yf_prime;
    b[3] = (2/(duration*duration*duration))*pos_begin.y + (1/(duration*duration))*yi_prime
            + (-2/(duration*duration*duration))*pos_end.y + (1/(duration*duration))*yf_prime;

    // OR (2) generate targets for each target_dt
    std::vector<Position> trajectory = {pos_begin};
    for (double time = 0; time < duration; time += target_dt)
    {
        trajectory.emplace_back(
            a[0] + a[1]*time + a[2]*time*time + a[3]*time*time*time,
            b[0] + b[1]*time + b[2]*time*time + b[3]*time*time*time
        );
    }

    return trajectory; 
}

