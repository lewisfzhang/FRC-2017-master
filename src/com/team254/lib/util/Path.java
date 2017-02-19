package com.team254.lib.util;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import com.team254.frc2017.Constants;
import com.team254.lib.util.motion.MotionState;

/**
 * Class representing the robot's autonomous path.
 * 
 * Field Coordinate System:
 * Uses a right hand coordinate system.  Positive x is right, positive y is up,
 * and the origin is at the bottom left corner of the field.  For angles, 0 degrees
 * is facing right (1, 0) and angles increase as you turn counter clockwise.
 * 
 * @author MarioRuiz
 */

public class Path {
     List<PathSegment> segments;
     PathSegment prevSegment;
     
     public void extrapolateLast() {
         PathSegment.Translation last = ((PathSegment.Translation) segments.get(segments.size() - 1));
         last.extrapolateLookahead(true);
     }
     
     public Translation2d getEndPosition() {
         PathSegment.Translation last = ((PathSegment.Translation) segments.get(segments.size() - 1));
         return last.getEnd();
     }
     
     public Path() { segments = new ArrayList<PathSegment>(); }
     
     
     /**
      * add a segment to the Path
      * @param segment
      *     the segment to add
      */
     public void addSegment(PathSegment segment) {      
         segments.add(segment);       
     }
     
    /**
     * @return the last MotionState in the path
     */
     public MotionState getLastMotionState() {
         if(segments.size() > 0) {
             MotionState endState = ((PathSegment.Translation) segments.get(segments.size() - 1)).getEndState();
             //return endState;
             return new MotionState(0.0, 0.0, endState.vel(), endState.acc());
         } else {
             return new MotionState(0, 0, 0, 0);
         }
     }
     
     /**
      * get the remaining distance left for the robot to travel on the current segment
      * @param robotPos
      *     robot position
      * @return remaining distance on current segment
      */
     public double getSegmentRemainingDist(Translation2d robotPos) {
         PathSegment.Translation currentSegment = (PathSegment.Translation) segments.get(0);
         return currentSegment.getRemainingDistance(currentSegment.getClosestPoint(robotPos));
     }
     
     /**
      * @return the length of the current segment
      */
     public double getSegmentLength() {
         PathSegment.Translation currentSegment = (PathSegment.Translation) segments.get(0);
         return currentSegment.getLength();
     }
     
     /**
      * @return the starting pose of the robot
      */
     public RigidTransform2d getStartPose() {
         if(segments.get(0) == null)
             return new RigidTransform2d();
         else {
             PathSegment.Translation currentSegment = (PathSegment.Translation) segments.get(0);
             return new RigidTransform2d(currentSegment.getStart(), new Rotation2d());
         }
     }
     
     
     /**
      * Gives the position of the lookahead point
      * @param robotPos
      *     Robot position
      * @return lookahead point position
      */
     public Translation2d getTargetPoint(Translation2d robotPos) {
         Translation2d target;
         PathSegment.Translation currentSegment = (PathSegment.Translation) segments.get(0);
         double lookAheadDist = Constants.kAutoLookAhead;
         Translation2d closest = currentSegment.getClosestPoint(robotPos);
         double remainingDist = currentSegment.getRemainingDistance(closest);
         lookAheadDist -= remainingDist;
         int i = 1;
         while(lookAheadDist > 0 && i < segments.size()) {
             currentSegment = (PathSegment.Translation)segments.get(i);
             lookAheadDist -= currentSegment.getLength();
             i++;
         }
         target = currentSegment.getLookAheadPoint(currentSegment.getLength() + lookAheadDist);
         checkSegmentDone(robotPos);
         return target;
     }
     
     /**
      * Gives the speed the robot should be traveling at the given position
      * @param robotPos
      *     position of the robot
      * @return speed robot should be traveling
      */
     public double getSpeed(Translation2d robotPos) {
         PathSegment.Translation currentSegment = (PathSegment.Translation) segments.get(0);
         return currentSegment.getSpeed(robotPos);
     }
     
     /**
      * Checks if the robot has finished traveling along the current segment then removes it from
      * the path if it has
      * @param robotPos
      *     robot position
      */
     public void checkSegmentDone(Translation2d robotPos) {
         PathSegment.Translation currentSegment = (PathSegment.Translation) segments.get(0); 
         double remainingDist = currentSegment.getRemainingDistance(currentSegment.getClosestPoint(robotPos));
         if(remainingDist < Constants.kSegmentCompletionTolerance) {
             prevSegment = segments.remove(0);
         }
     }
     
     /**
      * Ensures that all speeds in the path are attainable and robot can slow down in time
      */
     public void verifySpeeds() {
         double maxStartSpeed = 0.0;
         for (int i=segments.size() - 1; i>=0; i--) {
             PathSegment.Translation segment = (PathSegment.Translation)(segments.get(i));
             maxStartSpeed += Math.sqrt(maxStartSpeed*maxStartSpeed + 2*Constants.kMaxAccel*segment.getLength());
             System.out.println(maxStartSpeed + ", "+ segment.getStartVel());
             if(segment.getStartVel() > maxStartSpeed) {
                 segment.setStartVel(maxStartSpeed);
                 System.out.println("Segment starting speed is too high!");
             }
             maxStartSpeed = segment.getStartVel();
         }
         for(int i=0; i<segments.size(); i++) {
             PathSegment.Translation segment = (PathSegment.Translation)(segments.get(i));
             double endSpeed = (i < segments.size()-1) ? ((PathSegment.Translation)(segments.get(i+1))).getStartVel() : 0.0;
             MotionState startState = (i > 0) ? ((PathSegment.Translation)(segments.get(i-1))).getEndState() : new MotionState(0,0,0,0);
             startState = new MotionState(0, 0, startState.vel(), startState.vel());
             segment.createMotionProfiler(startState, endSpeed);
         }
     }
     
     public String toString() {
         String str = "";
         for(PathSegment s : segments) {
             str += ((PathSegment.Translation) s).toString() + "\n";
         }
         return str;
     }
}
