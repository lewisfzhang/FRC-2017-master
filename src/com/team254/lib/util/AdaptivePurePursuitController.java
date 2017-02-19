package com.team254.lib.util;

import com.team254.frc2017.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private static final double kReallyBigNumber = 1E9;

    Path mPath;
    boolean mReversed;
    String filepath;
    
    public static void main(String[] args) {
        RigidTransform2d pose = new RigidTransform2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
        Translation2d point = new Translation2d(0, 100);
        System.out.println(getLength(pose,point));
    }
    
    /**
     * Creates a new Adaptive Pure Pursuit Controller with the path specified in <code>filepath</code>
     * @param filepath path to the text file containing the autonomous path
     */
    public AdaptivePurePursuitController(String filepath) {
        mPath = new Path(filepath);
        mReversed = false;
        this.filepath = filepath;
    }
    
    /**
     * Creates a new Adaptive Pure Pursuit Controller from the path object
     * @param path
     *      path for the Adaptive Pure Pursuit Controller to follow
     */
    public AdaptivePurePursuitController(Path path, boolean reversed) {       
        mPath = path;     
        mReversed = reversed;              
    }     

    /**
     * Gives the RigidTransform2d.Delta that the robot should take to follow the path
     * @param pose 
     *      robot pose
     * @return RigidTransform2d movement command for the robot to follow
     */
    public RigidTransform2d.Delta update(RigidTransform2d pose) {
        if (mReversed) {
            pose = new RigidTransform2d(pose.getTranslation(),
                    pose.getRotation().rotateBy(Rotation2d.fromRadians(Math.PI)));
        }

        Translation2d lookaheadPoint = mPath.getTargetPoint(pose.getTranslation());
        if(isFinished())
            return new RigidTransform2d.Delta(0, 0, 0);
        
        double speed = mPath.getSpeed(pose.getTranslation());
        if(speed < Constants.kMinSpeed)
            speed = Constants.kMinSpeed;
        if (mReversed)
            speed *= -1;
        //SmartDashboard.putNumber("Desired Speed", speed);
         
        
        RigidTransform2d.Delta rv;
        rv = new RigidTransform2d.Delta(speed, 0, getDirection(pose, lookaheadPoint) * Math.abs(speed) / getRadius(pose, lookaheadPoint));
        return rv;
    }
    
    /**
     * Gives the center of the circle joining the lookahead point and robot pose
     * @param pose
     *      robot pose
     * @param point
     *      lookahead point
     * @return center of the circle joining the lookahead point and robot pose
     */
    public static Translation2d getCenter(RigidTransform2d pose, Translation2d point) {
        Translation2d poseToPoint = new Translation2d(pose.getTranslation(), point);
        Line perpendicularBisector = new Line(pose.getTranslation().translateBy(poseToPoint.scale(0.5)),
                new Translation2d(-poseToPoint.getY(), poseToPoint.getX()));
        Line radiusLine = new Line(pose.getTranslation(), 
                new Translation2d(-pose.getRotation().sin(), pose.getRotation().cos()));
        return Line.intersection(perpendicularBisector, radiusLine);
    }
    
    /**
     * Gives the radius of the circle joining the lookahead point and robot pose
     * @param pose
     *      robot pose
     * @param point
     *      lookahead point
     * @return radius of the circle joining the lookahead point and robot pose
     */
    public static double getRadius(RigidTransform2d pose, Translation2d point) {
        Translation2d center = getCenter(pose, point);
        return new Translation2d(center, point).norm();
    }
    
    /**
     * Gives the length of the arc joining the lookahead point and robot pose
     * @param pose
     *      robot pose
     * @param point
     *      lookahead point
     * @return the length of the arc joining the lookahead point and robot pose
     */
    public static double getLength(RigidTransform2d pose, Translation2d point) {
        double radius = getRadius(pose, point);
        if(radius < kReallyBigNumber) {
            Translation2d center = getCenter(pose, point);
            Translation2d centerToPoint = new Translation2d(center, point);
            Translation2d centerToPose = new Translation2d(center, pose.getTranslation());
            double dotProduct = centerToPoint.getX() * centerToPose.getX() + centerToPoint.getY() * centerToPose.getY();
            double angle = Math.acos(dotProduct / (centerToPoint.norm() * centerToPose.norm()));
            return radius * angle;
        } else {
            return new Translation2d(pose.getTranslation(), point).norm();
        }
    }
    
    /**
     * Gives the direction the robot should turn to stay on the path
     * @param pose
     *      robot pose
     * @param point
     *      lookahead point
     * @return the direction the robot should turn: -1 is left, +1 is right
     */
    public static int getDirection(RigidTransform2d pose, Translation2d point) {
        Translation2d poseToPoint = new Translation2d(pose.getTranslation(), point);        
        Translation2d robot = pose.getRotation().toTranslation();  
        double cross = robot.getX() * poseToPoint.getY() - robot.getY() * poseToPoint.getX();
        return (cross < 0) ? -1 : 1; //if robot < pose turn left
    }
    
    /**
     * @return has the robot reached the end of the path
     */
    public boolean isFinished() {
        return mPath.segments.size() == 0;
    }
    
    public void reset() {
        mPath = new Path(filepath);
        mReversed = false;
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
}