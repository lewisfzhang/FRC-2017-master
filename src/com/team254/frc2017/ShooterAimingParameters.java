package com.team254.frc2017;

import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;

/**
 * A container class to specify the shooter angle. It contains the desired
 * range, the field_to_goal_angle
 */
public class ShooterAimingParameters {
    double range;
    Rotation2d robot_to_goal_in_field;


    public ShooterAimingParameters(double range, Rotation2d robot_to_goal_in_field) {
        this.range = range;
        this.robot_to_goal_in_field = robot_to_goal_in_field;
    }


    public double getRange() {
        return range;
    }

    public Rotation2d getRobotToGoalInField() {
        return robot_to_goal_in_field;
    }

}
