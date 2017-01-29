package com.team254.lib.util;

import java.util.Optional;
import java.util.Set;

/**
 * Implements an adaptive pure pursuit controller. See:
 * https://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_4/kelly_alonzo_1994_4 .pdf
 * 
 * Basically, we find a spot on the path we'd like to follow and calculate the wheel speeds necessary to make us land on
 * that spot. The target spot is a specified distance ahead of us, and we look further ahead the greater our tracking
 * error.
 */
 
public class AdaptivePurePursuitController {
    private static final double kEpsilon = 1E-9;

    //double mFixedLookahead;
    Path mPath;
    SpeedController mSpeedController;
    //RigidTransform2d.Delta mLastCommand;
    //double mLastTime;
    //double mMaxAccel;
    //double mDt;
    boolean mReversed;
    String filepath;
    int counter = 0;
    //double mPathCompletionTolerance;

//    public AdaptivePurePursuitController(double fixed_lookahead, double max_accel, double nominal_dt, Path path,
//            boolean reversed, double path_completion_tolerance) {
//        mFixedLookahead = fixed_lookahead;
//        mMaxAccel = max_accel;
//        mPath = path;
//        mDt = nominal_dt;
//        mLastCommand = null;
//        mReversed = reversed;
//        mPathCompletionTolerance = path_completion_tolerance;
//        mSpeedController = new SpeedController(mPath);
//    }
    
    public static void main(String[] args) {
        RigidTransform2d pose = new RigidTransform2d(new Translation2d(100,100), Rotation2d.fromDegrees(220));
        Translation2d point = new Translation2d(0, 200);
        getDirection(pose, point);
    }
    
    public AdaptivePurePursuitController(String filepath) {
        mPath = new Path(filepath);
        mReversed = false;
        mSpeedController = new SpeedController(mPath);
        this.filepath = filepath;
    }

    public RigidTransform2d.Delta update(RigidTransform2d pose) {
        if (mReversed) {
            pose = new RigidTransform2d(pose.getTranslation(),
                    pose.getRotation().rotateBy(Rotation2d.fromRadians(Math.PI)));
        }

        Translation2d lookaheadPoint = mPath.getTargetPoint(pose.getTranslation());
        if(counter == 60) {
            System.out.println(lookaheadPoint);
            counter = 0; 
        }
        counter++;
        if(isFinished())
            return new RigidTransform2d.Delta(0, 0, 0);
        
        double speed = mSpeedController.getSpeed(pose.getTranslation());
        
        RigidTransform2d.Delta rv;
        rv = new RigidTransform2d.Delta(speed, 0, getDirection(pose, lookaheadPoint) * Math.abs(speed) / getRadius(pose, lookaheadPoint));
        return rv;
    }
    
    public static double getRadius(RigidTransform2d pose, Translation2d point) {
        Translation2d poseToPoint = new Translation2d(pose.getTranslation(), point);
        Line perpendicularBisector = new Line(pose.getTranslation().translateBy(poseToPoint.scale(0.5)),
                new Translation2d(-poseToPoint.getY(), poseToPoint.getX()));
        Line radiusLine = new Line(pose.getTranslation(), 
                new Translation2d(pose.getRotation().sin(), pose.getRotation().cos()));
        Translation2d center = Line.intersection(perpendicularBisector, radiusLine);
        return new Translation2d(center, point).norm();
    }
    
    public static int getDirection(RigidTransform2d pose, Translation2d point) {
        Translation2d poseToPoint = new Translation2d(pose.getTranslation(), point);
        Translation2d robotDirection = new Translation2d(pose.getRotation());
        double poseToPointAngle = Math.toDegrees(Math.atan2(-poseToPoint.getY(), poseToPoint.getX()));
        double robotAngle = Math.toDegrees(Math.atan2(robotDirection.getY(), robotDirection.getX()));
        return (robotAngle < poseToPointAngle) ? 1 : -1; //if robot < pose turn left
    }
    
    public boolean isFinished() {
        return mPath.segments.size() == 0;
    }
    
    public void reset() {
        mPath = new Path(filepath);
        mReversed = false;
        mSpeedController = new SpeedController(mPath);
    }
    
    public static class Line {
        public final Translation2d point;
        public final Translation2d slope;
        
        public Line(Translation2d point, Translation2d slope) {
            this.point = point;
            this.slope = slope;
        }
        
        public void print() {
            System.out.println("Slope: " + slope);
            System.out.println("Point: " + point);
        }
        
        public double getSlope() {
            return slope.getY() / ((slope.getX() == 0) ? kEpsilon : slope.getX());
        }
        
        public double getYIntercept() {
            return point.getY() - point.getX() * getSlope();
        }
        
        public Translation2d getPoint(double x) {
            return new Translation2d(x, x * getSlope() + getYIntercept());
        }
        
        public static Translation2d intersection(Line l1, Line l2) {
            double x = (l1.getYIntercept() - l2.getYIntercept()) / (l2.getSlope() - l1.getSlope());
            return l1.getPoint(x);
        }
    }


    public static class Circle {
        public final Translation2d center;
        public final double radius;
        public final boolean turn_right;

        public Circle(Translation2d center, double radius, boolean turn_right) {
            this.center = center;
            this.radius = radius;
            this.turn_right = turn_right;
        }
    }

    public static Optional<Circle> joinPath(RigidTransform2d robot_pose, Translation2d lookahead_point) {
        double x1 = robot_pose.getTranslation().getX();
        double y1 = robot_pose.getTranslation().getY();
        double x2 = lookahead_point.getX();
        double y2 = lookahead_point.getY();

        Translation2d pose_to_lookahead = new Translation2d(robot_pose.getTranslation(), lookahead_point);
        double cross_product = pose_to_lookahead.getX() * robot_pose.getRotation().sin()
                - pose_to_lookahead.getY() * robot_pose.getRotation().cos();
        if (Math.abs(cross_product) < kEpsilon) {
            return Optional.empty();
        }

        double dx = pose_to_lookahead.getX();
        double dy = pose_to_lookahead.getY();
        double my = (cross_product > 0 ? -1 : 1) * robot_pose.getRotation().cos();
        double mx = (cross_product > 0 ? 1 : -1) * robot_pose.getRotation().sin();

        double cross_term = mx * dx + my * dy;

        if (Math.abs(cross_term) < kEpsilon) {
            // Points are colinear
            return Optional.empty();
        }

        return Optional.of(new Circle(
                new Translation2d((mx * (x1 * x1 - x2 * x2 - dy * dy) + 2 * my * x1 * dy) / (2 * cross_term),
                        (-my * (-y1 * y1 + y2 * y2 + dx * dx) + 2 * mx * y1 * dx) / (2 * cross_term)),
                .5 * Math.abs((dx * dx + dy * dy) / cross_term), cross_product > 0));
    }

}