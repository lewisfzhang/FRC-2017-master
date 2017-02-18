package com.team254.lib.util;

import java.util.Optional;

import com.team254.frc2017.Constants;
import com.team254.lib.util.motion.MotionProfile;
import com.team254.lib.util.motion.MotionProfileConstraints;
import com.team254.lib.util.motion.MotionProfileGenerator;
import com.team254.lib.util.motion.MotionProfileGoal;
import com.team254.lib.util.motion.MotionState;

/**
 * Class representing a segment of the robot's autonomous path.  There are
 * two types of segments: Translation (line or arc), and Rotation (turn in place)
 *   
 * @author MarioRuiz
 */

public abstract class PathSegment {
    
    abstract boolean isTurn();
    abstract double getMaxSpeed();
    
    /**
     * Subclass representing a robot movement (line or arc) on the autonomous path
     */
    public static class Translation extends PathSegment {
        private Translation2d start;
        private Translation2d end;
        private Translation2d center;
        private boolean extrapolateLookahead;
        private double curvature;
        private double maxSpeed;
        private double startAngle;
        private double endAngle;
        private MotionProfile speedController;
        
        /**
         * Constructor for a linear segment
         * @param x1 start x
         * @param y1 start y
         * @param x2 end x
         * @param y2 end y
         * @param maxSpeed maximum speed allowed on the segment
         */
        public Translation(double x1, double y1, double x2, double y2, double maxSpeed) {
            this.start = new Translation2d(x1, y1);
            this.end = new Translation2d(x2, y2);
            this.center = null;
            this.curvature = 0;
            this.maxSpeed = maxSpeed;
            this.startAngle = 0;
            this.endAngle = 0;
            extrapolateLookahead = false;
            createMotionProfiler(new MotionState(0.0, 0.0, 0.0, 0.0));
        }
        
        /**
         * Constructor for a linear segment
         * @param x1 start x
         * @param y1 start y
         * @param x2 end x
         * @param y2 end y
         * @param maxSpeed maximum speed allowed on the segment
         */
        public Translation(double x1, double y1, double x2, double y2, double maxSpeed, MotionState startState) {
            this.start = new Translation2d(x1, y1);
            this.end = new Translation2d(x2, y2);
            this.center = null;
            this.curvature = 0;
            this.maxSpeed = maxSpeed;
            this.startAngle = 0;
            this.endAngle = 0;
            extrapolateLookahead = false;
            createMotionProfiler(startState);
        }
        
        /**
         * Constructor for an arc segment
         * @param x1 start x
         * @param y1 start y
         * @param x2 end x
         * @param y2 end y
         * @param cx center x
         * @param cy center y
         * @param maxSpeed maximum speed allowed on the segment 
         */
        public Translation(double x1, double y1, double x2, double y2, double cx, double cy, double maxSpeed) {
            this.start = new Translation2d(x1, y1);
            this.end = new Translation2d(x2, y2);
            this.center = new Translation2d(cx, cy);
            this.maxSpeed = maxSpeed;
            extrapolateLookahead = false;
            calcArc();
            createMotionProfiler(new MotionState(0.0, 0.0, 0.0, 0.0));
        }
        
        /**
         * Constructor for an arc segment
         * @param x1 start x
         * @param y1 start y
         * @param x2 end x
         * @param y2 end y
         * @param cx center x
         * @param cy center y
         * @param maxSpeed maximum speed allowed on the segment 
         */
        public Translation(double x1, double y1, double x2, double y2, double cx, double cy, double maxSpeed, MotionState startState) {
            this.start = new Translation2d(x1, y1);
            this.end = new Translation2d(x2, y2);
            this.center = new Translation2d(cx, cy);
            this.maxSpeed = maxSpeed;
            extrapolateLookahead = false;
            calcArc();
            createMotionProfiler(startState);
        }
        
        /**
         * @return is this segment a rotation
         */
        public boolean isTurn() {
            return false;
        }
        
        /**
         * @return max speed of the segment
         */
        public double getMaxSpeed() {
            return maxSpeed;
        }
        
        private void createMotionProfiler(MotionState start_state) {
            MotionProfileConstraints motionConstraints = new MotionProfileConstraints(maxSpeed, Constants.kMaxAccel);
            MotionProfileGoal goal_state = new MotionProfileGoal(getLength());
            speedController = MotionProfileGenerator.generateProfile(motionConstraints, goal_state, start_state);
            System.out.println(speedController);
        }
        
        private void calcArc() {
            Translation2d deltaS = new Translation2d(center, start);
            Translation2d deltaE = new Translation2d(center, end);
            this.startAngle = Math.atan2(deltaS.getY(), deltaS.getX());
            startAngle = (startAngle < 0) ? startAngle + Math.PI*2 : startAngle;
            this.endAngle = Math.atan2(deltaE.getY(), deltaE.getX());
            endAngle = (endAngle < 0) ? endAngle + Math.PI*2 : endAngle;
            this.curvature = 1/deltaS.norm();
        }
        
        /**
         * @return starting point of the segment
         */
        public Translation2d getStart() {
            return start;
        }
        
        /**
         * @return end point of the segment
         */
        public Translation2d getEnd() {
            return end;
        }
        
        /**
         * @return the total length of the segment
         */
        public double getLength() {
            if(curvature == 0) {
                return new Translation2d(start, end).norm();
            } else {
                Double a = Math.acos(Math.cos(endAngle) * Math.cos(startAngle) + Math.sin(endAngle) * Math.sin(startAngle));
                return Math.PI * (1/curvature) * a / Math.PI;
            }
        }
        
        /**
         * Set whether or not to extrapolate the lookahead point.  Should only be true
         * for the last segment in the path
         * @param val 
         */
        public void extrapolateLookahead(boolean val) {
            extrapolateLookahead = val;
        }
        
        /**
         * Gets the point on the segment closest to the robot
         * @param position the current position of the robot
         * @return the point on the segment closest to the robot
         */
        public Translation2d getClosestPoint(Translation2d position) {
            if(curvature == 0) {
                Translation2d delta = new Translation2d(start, end);
                double u = ((position.getX() - start.getX()) * delta.getX() + (position.getY() - start.getY()) * delta.getY()) / (delta.getX() * delta.getX() + delta.getY() * delta.getY());
                if (u < 0)
                    return start;
                else if (u > 1)
                    return end;
                else
                    return new Translation2d(start.getX() + u * delta.getX(), start.getY() + u * delta.getY());
            } else {
                Translation2d delta = new Translation2d(center, position);
                double scale = (1/curvature) / delta.norm();
                delta = delta.scale(scale);
                Translation2d s = new Translation2d(center, start);
                Translation2d e = new Translation2d(center, end);
                if(Translation2d.Cross(delta, s) * Translation2d.Cross(delta, e) < 0) {
                    return center.translateBy(delta);
                } else {
                    Translation2d startDist = new Translation2d(position, start);
                    Translation2d endDist = new Translation2d(position, end);
                    if(endDist.norm() < startDist.norm()) {
                        return end;
                    } else {
                        return start;
                    }
                }
            } 
        }
        
        /**
         * Calculates the point on the segment <code>dist</code> distance from the starting point
         * @param dist distance from the starting point (lookahead distance)
         * @return point on the segment <code>dist</code> distance from the starting point
         */
        public Translation2d getLookAheadPoint(double dist) {
            double length = getLength();
            if(extrapolateLookahead) {
                if(curvature == 0) {
                    Translation2d delta = new Translation2d(start, end);
                    return start.translateBy( delta.scale(dist / length));
                } else {
                    Translation2d s = new Translation2d(center, start);
                    Translation2d e = new Translation2d(center, end);
                    double deltaAngle = Translation2d.GetAngle(s, e) * ((Translation2d.Cross(s, e) >= 0) ? 1 : -1);
                    deltaAngle *= dist / length;
                    Translation2d t = new Translation2d(Math.cos(startAngle + deltaAngle), Math.sin(startAngle + deltaAngle)).scale(1/curvature);
                    return center.translateBy(t);
                }
            } else {
                if(dist > length)
                    dist = length;
                if(curvature == 0) {
                    Translation2d delta = new Translation2d(start, end);
                    return start.translateBy( delta.scale(dist / length));
                } else {
                    Translation2d s = new Translation2d(center, start);
                    Translation2d e = new Translation2d(center, end);
                    double deltaAngle = Translation2d.GetAngle(s, e) * ((Translation2d.Cross(s, e) >= 0) ? 1 : -1);
                    deltaAngle *= dist / length;
                    Translation2d t = new Translation2d(Math.cos(startAngle + deltaAngle), Math.sin(startAngle + deltaAngle)).scale(1/curvature);
                    return center.translateBy(t);
                }
            }
        }
        
        /**
         * Gets the remaining distance left on the segment from point <code>point</code>
         * @param point result of <code>getClosestPoint()</code>
         * @return distance remaining
         */
        public double getRemainingDistance(Translation2d point) {
            if(curvature == 0) {
                return new Translation2d(end, point).norm();
            } else {
                Translation2d e = new Translation2d(center, end);
                Translation2d s = new Translation2d(center, start);
                Translation2d p = new Translation2d(center, point);
                double angle = Translation2d.GetAngle(e, p);
                double totalAngle = Translation2d.GetAngle(s, e);
                return angle/totalAngle * getLength();
            }
        }
        
        private double getDistanceTravelled(Translation2d robotPosition) {
            Translation2d pathPosition = getClosestPoint(robotPosition); 
            double remainingDist = getRemainingDistance(pathPosition);
            return getLength() - remainingDist;
            
        }
        
        public double getSpeed(Translation2d robotPosition) {
            double dist = getDistanceTravelled(robotPosition);
            Optional<MotionState> state = speedController.firstStateByPos(dist);
            if(state.isPresent()) {
                return state.get().vel();
            } else {
                System.out.println("Velocity does not exist at that position!");
                return 0.0;
            }
        }
        
        public double getSpeed(double t) {
            System.out.println("time: " + t);
            Optional<MotionState> state = speedController.stateByTime(t);
            if(state.isPresent()) {
                System.out.println(state.get());
                return state.get().vel();
            } else {
                System.out.println("ripipipip");
                return Constants.kMinSpeed; //TODO idk what this should return if state isn't present
            }
        }
        
        public MotionState getEndState() {
            return speedController.endState();
        }
    }
}
