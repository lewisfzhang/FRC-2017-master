package com.team254.frc2017;

import com.team254.frc2017.GoalTracker.TrackReport;
import com.team254.frc2017.vision.TargetInfo;
import com.team254.lib.util.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * RobotState keeps track of the poses of various coordinate frames throughout the match. A coordinate frame is simply a
 * point and direction in space that defines an (x,y) coordinate system. Transforms (or poses) keep track of the spatial
 * relationship between different frames.
 *
 * Robot frames of interest (from parent to child):
 *
 * 1. Field frame: origin is where the robot is turned on
 *
 * 2. Vehicle frame: origin is the center of the robot wheelbase, facing forwards
 *
 * 3. Camera frame: origin is the center of the camera imager relative to the robot base.
 *
 * 4. Goal frame: origin is the center of the boiler (note that orientation in this frame is arbitrary). Also note that
 * there can be multiple goal frames.
 *
 * As a kinematic chain with 4 frames, there are 3 transforms of interest:
 *
 * 1. Field-to-vehicle: This is tracked over time by integrating encoder and gyro measurements. It will inevitably
 * drift, but is usually accurate over short time periods.
 *
 * 2. Vehicle-to-camera: This is a constant.
 *
 * 3. Camera-to-goal: This is a pure translation, and is measured by the vision system.
 */

public class RobotState {
    private static RobotState instance_ = new RobotState();

    public static RobotState getInstance() {
        return instance_;
    }

    private static final int kObservationBufferSize = 100;

    private static final RigidTransform2d kVehicleToCamera = new RigidTransform2d(
            new Translation2d(Constants.kCameraXOffset, Constants.kCameraYOffset), new Rotation2d());

    // FPGATimestamp -> RigidTransform2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d> field_to_vehicle_;
    private RigidTransform2d.Delta vehicle_velocity_predicted_;
    private RigidTransform2d.Delta vehicle_velocity_measured_;
    private double distance_driven_;
    private GoalTracker goal_tracker_;
    private Rotation2d camera_pitch_correction_;
    private Rotation2d camera_yaw_correction_;
    private double differential_height_;
    private ShooterAimingParameters cached_shooter_aiming_params_ = null;
    private double cached_shooter_aiming_params_ts_ = 0;
    private static final double kCacheTime = 1.0 / 20.0;

    private RobotState() {
        reset(0, new RigidTransform2d());
    }

    public synchronized void reset(double start_time, RigidTransform2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        vehicle_velocity_predicted_ = RigidTransform2d.Delta.identity();
        vehicle_velocity_measured_ = RigidTransform2d.Delta.identity();
        goal_tracker_ = new GoalTracker();
        camera_pitch_correction_ = Rotation2d.fromDegrees(-Constants.kCameraPitchAngleDegrees);
        camera_yaw_correction_ = Rotation2d.fromDegrees(-Constants.kCameraYawAngleDegrees);
        differential_height_ = Constants.kBoilerTargetTopHeight - Constants.kCameraZOffset;
        distance_driven_ = 0.0;
    }

    public synchronized RigidTransform2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, RigidTransform2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized RigidTransform2d getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle().getValue()
                .transformBy(RigidTransform2d.fromVelocity(vehicle_velocity_predicted_.scaled(lookahead_time)));
    }

    public synchronized RigidTransform2d getFieldToCamera(double timestamp) {
        return getFieldToVehicle(timestamp).transformBy(kVehicleToCamera);
    }

    public synchronized List<RigidTransform2d> getCaptureTimeFieldToGoal() {
        List<RigidTransform2d> rv = new ArrayList<>();
        for (TrackReport report : goal_tracker_.getTracks()) {
            rv.add(RigidTransform2d.fromTranslation(report.field_to_goal));
        }
        return rv;
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, RigidTransform2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, RigidTransform2d.Delta measured_velocity,
            RigidTransform2d.Delta predicted_velocity) {
        addFieldToVehicleObservation(timestamp,
                Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), measured_velocity));
        vehicle_velocity_measured_ = measured_velocity;
        vehicle_velocity_predicted_ = predicted_velocity;
    }

    public void addVisionUpdate(double timestamp, List<TargetInfo> vision_update) {
        List<Translation2d> field_to_goals = new ArrayList<>();
        RigidTransform2d field_to_camera = getFieldToCamera(timestamp);
        if (!(vision_update == null || vision_update.isEmpty())) {
            for (TargetInfo target : vision_update) {
                double ydeadband = (target.getY() > -Constants.kCameraDeadband
                        && target.getY() < Constants.kCameraDeadband) ? 0.0 : target.getY();

                // Compensate for camera yaw
                double xyaw = target.getX() * camera_yaw_correction_.cos() + ydeadband * camera_yaw_correction_.sin();
                double yyaw = ydeadband * camera_yaw_correction_.cos() - target.getX() * camera_yaw_correction_.sin();
                double zyaw = target.getZ();

                // Compensate for camera pitch
                double xr = zyaw * camera_pitch_correction_.sin() + xyaw * camera_pitch_correction_.cos();
                double yr = yyaw;
                double zr = zyaw * camera_pitch_correction_.cos() - xyaw * camera_pitch_correction_.sin();

                // find intersection with the goal
                if (zr > 0) {
                    double scaling = differential_height_ / zr;
                    double distance = Math.hypot(xr, yr) * scaling + Constants.kBoilerRadius;
                    Rotation2d angle = new Rotation2d(xr, yr, true);
                    field_to_goals.add(field_to_camera
                            .transformBy(RigidTransform2d
                                    .fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin())))
                            .getTranslation());
                }
            }
        }
        synchronized (this) {
            goal_tracker_.update(timestamp, field_to_goals);
        }
    }

    public synchronized Optional<ShooterAimingParameters> getCachedAimingParameters() {
        return cached_shooter_aiming_params_ == null ? Optional.empty() : Optional.of(cached_shooter_aiming_params_);
    }

    public synchronized Optional<ShooterAimingParameters> getAimingParameters(double currentTimestamp,
            boolean forceUpdate) {
        if (forceUpdate) {
            cached_shooter_aiming_params_ = null;
        }
        return getAimingParameters(currentTimestamp);
    }

    public synchronized Optional<ShooterAimingParameters> getAimingParameters(double currentTimestamp) {
        if (cached_shooter_aiming_params_ != null
                && (currentTimestamp - cached_shooter_aiming_params_ts_ < kCacheTime)) {
            return Optional.of(cached_shooter_aiming_params_);
        }

        List<TrackReport> reports = goal_tracker_.getTracks();
        if (!reports.isEmpty()) {
            TrackReport report = reports.get(0);
            Translation2d robot_to_goal = getLatestFieldToVehicle().getValue().getTranslation().inverse()
                    .translateBy(report.field_to_goal);
            Rotation2d robot_to_goal_rotation = Rotation2d
                    .fromRadians(Math.atan2(robot_to_goal.getY(), robot_to_goal.getX()));

            ShooterAimingParameters params = new ShooterAimingParameters(robot_to_goal.norm(), robot_to_goal_rotation);
            cached_shooter_aiming_params_ = params;
            cached_shooter_aiming_params_ts_ = currentTimestamp;
            return Optional.of(params);
        } else {
            return Optional.empty();
        }
    }

    public synchronized void resetVision() {
        goal_tracker_.reset();
    }

    public synchronized RigidTransform2d.Delta generateOdometryFromSensors(double left_encoder_delta_distance,
            double right_encoder_delta_distance, Rotation2d current_gyro_angle) {
        final RigidTransform2d last_measurement = getLatestFieldToVehicle().getValue();
        final RigidTransform2d.Delta delta = Kinematics.forwardKinematics(last_measurement.getRotation(),
                left_encoder_delta_distance, right_encoder_delta_distance, current_gyro_angle);
        distance_driven_ += delta.dx;
        return delta;
    }

    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

    public synchronized RigidTransform2d.Delta getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized RigidTransform2d.Delta getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public void outputToSmartDashboard() {
        RigidTransform2d odometry = getLatestFieldToVehicle().getValue();
        SmartDashboard.putNumber("robot_pose_x", odometry.getTranslation().getX());
        SmartDashboard.putNumber("robot_pose_y", odometry.getTranslation().getY());
        SmartDashboard.putNumber("robot_pose_theta", odometry.getRotation().getDegrees());
        List<RigidTransform2d> poses = getCaptureTimeFieldToGoal();
        for (RigidTransform2d pose : poses) {
            // Only output first goal
            SmartDashboard.putNumber("goal_pose_x", pose.getTranslation().getX());
            SmartDashboard.putNumber("goal_pose_y", pose.getTranslation().getY());
            break;
        }
        Optional<ShooterAimingParameters> aiming_params = getCachedAimingParameters();
        if (aiming_params.isPresent()) {
            SmartDashboard.putNumber("goal_range", aiming_params.get().getRange());
            SmartDashboard.putNumber("goal_theta", aiming_params.get().getRobotToGoal().getDegrees());
        } else {
            SmartDashboard.putNumber("goal_range", 0.0);
            SmartDashboard.putNumber("goal_theta", 0.0);
        }
    }
}
