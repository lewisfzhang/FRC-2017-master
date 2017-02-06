package com.team254.lib.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * Constant-time setpoint generator so that a feedback controller can follow a trajectory.
 */
public class TrajectoryFollower {
    public static class GoalState {
        public double pos;
        public double max_vel;
        public double pos_tolerance = 1E-3;
    }

    public static class MotionState {
        public double t;
        public double pos;
        public double vel;
        public double acc;

        @Override
        public String toString() {
            return "t: " + t + ", pos: " + pos + ", vel: " + vel + ", acc: " + acc;
        }
    }

    private MotionState setpoint_ = new MotionState();
    private GoalState goal_state_ = new GoalState();

    public TrajectoryFollower() {
    }

    public void setGoal(GoalState goal_state) {
        goal_state_ = goal_state;
    }

    public GoalState getGoal() {
        return goal_state_;
    }

    public MotionState getSetpoint() {
        return setpoint_;
    }

    /*
     * public MotionState updateSetpoint(double max_vel, double max_acc, final MotionState prev_state, double
     * current_time) { max_vel = Math.abs(max_vel); max_acc = Math.abs(max_acc); setpoint_ = new MotionState();
     * setpoint_.t = current_time; final double dt = Math.max(0.0, current_time - prev_state.t); if
     * (isFinishedTrajectory()) { setpoint_.pos = goal_state_.pos; setpoint_.vel = goal_state_.vel; setpoint_.acc = 0;
     * return setpoint_; } // Compute the new commanded position, velocity, and acceleration. double distance_to_go =
     * goal_state_.pos - prev_state.pos; double cur_vel = prev_state.vel; double cur_vel2 = cur_vel * cur_vel; double
     * goal_vel = goal_state_.vel; // In cases where the goal is opposite the current direction of travel, it's easiest
     * just to invert everything and solve the positive distance case. boolean inverted = false; if (distance_to_go < 0)
     * { inverted = true; distance_to_go *= -1; cur_vel *= -1; goal_vel *= -1; } // Compute discriminants of the minimum
     * and maximum reachable // velocities over the remaining distance. double max_reachable_velocity_disc = cur_vel2 /
     * 2.0 + max_acc * distance_to_go; double min_reachable_velocity_disc = cur_vel2 / 2.0 - max_acc * distance_to_go;
     * double cruise_vel = cur_vel; if (min_reachable_velocity_disc < 0 || cruise_vel < 0) { cruise_vel =
     * Math.min(max_vel, Math.sqrt(max_reachable_velocity_disc)); } double t_start = (cruise_vel - cur_vel) / max_acc;
     * // Accelerate // to // cruise_vel double x_start = cur_vel * t_start + .5 * config_.max_acc * t_start * t_start;
     * double t_end = Math.abs(cruise_vel / config_.max_acc); // Decelerate // to zero // vel. double x_end = cruise_vel
     * * t_end - .5 * config_.max_acc * t_end * t_end; double x_cruise = Math.max(0, distance_to_go - x_start - x_end);
     * double t_cruise = Math.abs(x_cruise / cruise_vel); // Figure out where we should be one dt along this trajectory.
     * if (t_start >= dt) { next_state_.pos = cur_vel * dt + .5 * config_.max_acc * dt * dt; next_state_.vel = cur_vel +
     * config_.max_acc * dt; next_state_.acc = config_.max_acc; } else if (t_start + t_cruise >= dt) { next_state_.pos =
     * x_start + cruise_vel * (dt - t_start); next_state_.vel = cruise_vel; next_state_.acc = 0; } else if (t_start +
     * t_cruise + t_end >= dt) { double delta_t = dt - t_start - t_cruise; next_state_.pos = x_start + x_cruise +
     * cruise_vel * delta_t - .5 * config_.max_acc * delta_t * delta_t; next_state_.vel = cruise_vel - config_.max_acc *
     * delta_t; next_state_.acc = -config_.max_acc; } else { // Trajectory ends this cycle. next_state_.pos =
     * distance_to_go; next_state_.vel = 0; next_state_.acc = 0; } if (inverted) { next_state_.pos *= -1;
     * next_state_.vel *= -1; next_state_.acc *= -1; } setpoint_.pos += next_state_.pos; setpoint_.vel =
     * next_state_.vel; setpoint_.acc = next_state_.acc;
     * 
     * double error = setpoint_.pos - position; if (reset_) { // Prevent jump in derivative term when we have been
     * reset. reset_ = false; last_error_ = error; error_sum_ = 0; } double output = kp_ * error + kd_ * ((error -
     * last_error_) / dt - setpoint_.vel) + (kv_ * setpoint_.vel + ka_ * setpoint_.acc); if (output < 1.0 && output >
     * -1.0) { // Only integrate error if the output isn't already saturated. error_sum_ += error * dt; } output += ki_
     * * error_sum_;
     * 
     * last_error_ = error; return output; }
     */

    public boolean isFinishedTrajectory() {
        return Math.abs(setpoint_.pos - goal_state_.pos) < goal_state_.pos_tolerance;// && (Math.abs(setpoint_.vel) <
                                                                                     // goal_state_.vel_tolerance ;
    }
}
