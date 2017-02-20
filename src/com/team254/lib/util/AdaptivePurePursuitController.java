package com.team254.lib.util;

/**
 * Implements an adaptive pure pursuit controller. See:
 * https://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_4/kelly_alonzo_1994_4 .pdf
 * 
 * Basically, we find a spot on the path we'd like to follow and calculate the arc necessary to make us land on that
 * spot. The target spot is a specified distance ahead of us, and we look further ahead the greater our tracking error.
 * We also return the maximum speed we'd like to be going when we reach the target spot.
 */

public class AdaptivePurePursuitController {
    private static final double kEpsilon = 1E-4;
    private static final double kReallyBigNumber = 1E6;

    public static class Command {
        public RigidTransform2d.Delta delta = RigidTransform2d.Delta.identity();
        public double cross_track_error;
        public double max_velocity;
        public double end_velocity;

        public Command() {
        }

        public Command(RigidTransform2d.Delta delta, double cross_track_error, double max_velocity,
                double end_velocity) {
            this.delta = delta;
            this.cross_track_error = cross_track_error;
            this.max_velocity = max_velocity;
            this.end_velocity = end_velocity;
        }
    }

    Path mPath;
    boolean mReversed;
    double mFixedLookahead;

    /**
     * Creates a new Adaptive Pure Pursuit Controller from the path object
     * 
     * @param path
     *            path for the Adaptive Pure Pursuit Controller to follow
     */
    public AdaptivePurePursuitController(Path path, boolean reversed, double fixed_lookahead) {
        mPath = path;
        mReversed = reversed;
        mFixedLookahead = fixed_lookahead;
    }

    /**
     * Gives the RigidTransform2d.Delta that the robot should take to follow the path
     * 
     * @param pose
     *            robot pose
     * @return movement command for the robot to follow
     */
    public Command update(RigidTransform2d pose) {
        if (mReversed) {
            pose = new RigidTransform2d(pose.getTranslation(),
                    pose.getRotation().rotateBy(Rotation2d.fromRadians(Math.PI)));
        }

        final Path.TargetPointReport report = mPath.getTargetPoint(pose.getTranslation(), mFixedLookahead);
        // System.out.println("Lookahead point " + report.lookahead_point + ", vel " + report.lookahead_point_speed);
        if (isFinished()) {
            // Stop.
            return new Command(RigidTransform2d.Delta.identity(), report.closest_point_distance, report.max_speed, 0.0);
        }

        final Arc arc = new Arc(pose, report.lookahead_point);
        double scale_factor = 1.0;
        if (report.lookahead_point_speed < kEpsilon) {
            scale_factor = report.remaining_segment_distance / arc.length;
        }
        if (mReversed) {
            scale_factor *= -1;
        }

        return new Command(
                new RigidTransform2d.Delta(scale_factor * arc.length, 0.0,
                        arc.length * getDirection(pose, report.lookahead_point) * Math.abs(scale_factor) / arc.radius),
                report.closest_point_distance, report.max_speed,
                report.lookahead_point_speed * Math.signum(scale_factor));
    }

    public static class Arc {
        public Translation2d center;
        public double radius;
        public double length;

        public Arc(RigidTransform2d pose, Translation2d point) {
            center = getCenter(pose, point);
            radius = new Translation2d(center, point).norm();
            length = getLength(pose, point, center, radius);
        }
    }

    /**
     * Gives the center of the circle joining the lookahead point and robot pose
     * 
     * @param pose
     *            robot pose
     * @param point
     *            lookahead point
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
     * 
     * @param pose
     *            robot pose
     * @param point
     *            lookahead point
     * @return radius of the circle joining the lookahead point and robot pose
     */
    public static double getRadius(RigidTransform2d pose, Translation2d point) {
        Translation2d center = getCenter(pose, point);
        return new Translation2d(center, point).norm();
    }

    /**
     * Gives the length of the arc joining the lookahead point and robot pose
     * 
     * @param pose
     *            robot pose
     * @param point
     *            lookahead point
     * @return the length of the arc joining the lookahead point and robot pose
     */
    public static double getLength(RigidTransform2d pose, Translation2d point) {
        final double radius = getRadius(pose, point);
        final Translation2d center = getCenter(pose, point);
        return getLength(pose, point, center, radius);
    }

    public static double getLength(RigidTransform2d pose, Translation2d point, Translation2d center, double radius) {
        if (radius < kReallyBigNumber) {
            final Translation2d centerToPoint = new Translation2d(center, point);
            final Translation2d centerToPose = new Translation2d(center, pose.getTranslation());
            final double dotProduct = centerToPoint.getX() * centerToPose.getX()
                    + centerToPoint.getY() * centerToPose.getY();
            final double angle = Math.acos(dotProduct / (centerToPoint.norm() * centerToPose.norm()));
            return radius * angle;
        } else {
            return new Translation2d(pose.getTranslation(), point).norm();
        }
    }

    /**
     * Gives the direction the robot should turn to stay on the path
     * 
     * @param pose
     *            robot pose
     * @param point
     *            lookahead point
     * @return the direction the robot should turn: -1 is left, +1 is right
     */
    public static int getDirection(RigidTransform2d pose, Translation2d point) {
        Translation2d poseToPoint = new Translation2d(pose.getTranslation(), point);
        Translation2d robot = pose.getRotation().toTranslation();
        double cross = robot.getX() * poseToPoint.getY() - robot.getY() * poseToPoint.getX();
        return (cross < 0) ? -1 : 1; // if robot < pose turn left
    }

    /**
     * @return has the robot reached the end of the path
     */
    public boolean isFinished() {
        return mPath.segments.size() == 0;
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