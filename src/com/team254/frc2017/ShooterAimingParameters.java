package com.team254.frc2017;

import com.team254.lib.util.Rotation2d;

/**
 * A container class to specify the shooter angle. It contains the desired
 * range, the field_to_goal_angle
 */
public class ShooterAimingParameters {
    double range;
    Rotation2d field_to_goal;

    public ShooterAimingParameters(double range, Rotation2d field_to_goal) {
        this.range = range;
        this.field_to_goal = field_to_goal;
    }

    public double getRange() {
        return range;
    }

    public Rotation2d getFieldToGoal() {
        return field_to_goal;
    }

}
