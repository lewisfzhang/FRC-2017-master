package com.team254.lib.util;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import com.team254.frc2017.Constants;

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
     
     /**
      * Creates a new Path with the path segment specified in <code>filepath</code>
      * @param filepath path to the text file containing the autonomous path
      */
     public Path(String filepath) {
         segments = new ArrayList<PathSegment>();
         loadFile(filepath);
         //Odometer.getInstance().setPose(getStartPose());
     }
     
     public Path() { segments = new ArrayList<PathSegment>(); }
     
     private void loadFile(String filepath) {
         filepath = filepath.replaceFirst("^~", System.getProperty("user.home"));
         try {
             File file = new File(filepath);
             Scanner sc = new Scanner(file);
             while (sc.hasNextLine()) {
                 String[] s = sc.nextLine().split(",");
                 for(String a: s)
                     System.out.print(a + " ");
                 System.out.print("\n");
                 if(s[0].equals("LINE"))
                     segments.add(new PathSegment.Translation(Double.parseDouble(s[1]), Double.parseDouble(s[2]), Double.parseDouble(s[3]), Double.parseDouble(s[4]), Double.parseDouble(s[5])));
                 else if(s[0].equals("ARC"))
                     segments.add(new PathSegment.Translation(Double.parseDouble(s[1]), Double.parseDouble(s[2]), Double.parseDouble(s[3]), Double.parseDouble(s[4]), Double.parseDouble(s[5]), Double.parseDouble(s[6]), Double.parseDouble(s[7])));
                 else if(s[0].equals("TURN"))
                     segments.add(new PathSegment.Turn(Double.parseDouble(s[1]), Double.parseDouble(s[2]), Double.parseDouble(s[3]), Double.parseDouble(s[4])));
             }
             sc.close();
         } catch (Exception e) {
             e.printStackTrace(); 
         }
     }
     
     public double getSegmentRemainingDist(Translation2d robotPos) {
         PathSegment.Translation currentSegment = (PathSegment.Translation) segments.get(0);
         return currentSegment.getRemainingDistance(currentSegment.getClosestPoint(robotPos));
     }
     
     public double getSegmentLength() {
         PathSegment.Translation currentSegment = (PathSegment.Translation) segments.get(0);
         return currentSegment.getLength();
     }
     
     public RigidTransform2d getStartPose() {
         if(segments.get(0) == null)
             return new RigidTransform2d();
         else {
             PathSegment.Translation currentSegment = (PathSegment.Translation) segments.get(0);
             return new RigidTransform2d(currentSegment.getStart(), new Rotation2d());
         }
     }
     
     public Translation2d getTargetPoint(Translation2d robotPos) {
         Translation2d target;
         PathSegment.Translation currentSegment = (PathSegment.Translation) segments.get(0);
         double segmentLength = currentSegment.getLength();
         double lookAheadDist = Constants.kAutoLookAhead;
         Translation2d closest = currentSegment.getClosestPoint(robotPos);
         double remainingDist = currentSegment.getRemainingDistance(closest);
         lookAheadDist -= remainingDist;
         int i = 1;
         while(lookAheadDist > 0 && i < segments.size()) {
//                 if(segments.get(i).isTurn()) {
//                     return ((PathSegment.Turn)segments.get(i)).getCenter();
//                 }
             currentSegment = (PathSegment.Translation)segments.get(i);
             lookAheadDist -= currentSegment.getLength();
             i++;
         }
         target = currentSegment.getLookAheadPoint(currentSegment.getLength() + lookAheadDist);
         checkSegmentDone(robotPos);
         return target;
     }
     
     public void checkSegmentDone(Translation2d robotPos) {
         PathSegment.Translation currentSegment = (PathSegment.Translation) segments.get(0);
         double length = currentSegment.getLength();
         double remainingDist = currentSegment.getRemainingDistance(currentSegment.getClosestPoint(robotPos));
         if((length - remainingDist) / length > Constants.kAutoSegmentThreshold) {
             prevSegment = segments.remove(0);
         }
     }
     
     public double getMaxSpeed() {
         return (segments.size() == 0) ? 0.0 : segments.get(0).getMaxSpeed();
     }
     
     public double getStartSpeed() {
         if(prevSegment == null)
             return Constants.kMaxAccel;
         return (segments.size() == 0) ? 0.0 : prevSegment.getMaxSpeed();
     }
     
     public double getEndSpeed() {
         return (segments.size() < 2) ? 0.0 : segments.get(1).getMaxSpeed();
     }
     
     public static void main(String[] args) {
         PathSegment.Translation curve = new PathSegment.Translation(10.0, 0.0, 0.0, 10.0, 0.0, 0.0, 1.0);
         
         Translation2d p = new Translation2d(2.0, 1.0);
         System.out.println(curve.getRemainingDistance(curve.getClosestPoint(p)));
         System.out.println(curve.getLength());
         System.out.println(curve.getLookAheadPoint(15.707963267948964/4));
         PathSegment.Translation line = new PathSegment.Translation(0.0, 0.0, 20.0, 20.0, 1.0);
         p = new Translation2d(15.0, 10.0);
         System.out.println(line.getClosestPoint(p));
         System.out.println(line.getLength());
         
         //PathSegment.Segment curve = new PathSegment.Segment(43.533040796611594, 200.4199794406889, 100, 250.0, 100, 193.05472389242087, 100);
         //System.out.println(curve.getLookAheadPoint(18.05));
         //Path test = new Path("~/path.txt");
         //System.out.println(test.getTargetPoint(new Translation2d(20, 20)));
         Path mPath = new Path();
         mPath.segments.add(new PathSegment.Translation(0.0, 0.0, 117.677669, 0.0, 100.0));
         mPath.segments.add(new PathSegment.Translation(50.0, 0.0, 60.0, 0.0, 1000.0));
         mPath.segments.add(new PathSegment.Translation(60.0, 0.0, 150.0, 0.0, 20.0));
         SpeedController mSpeed = new SpeedController(mPath);
         mPath.getTargetPoint(new Translation2d(49.6, 0));
         System.out.println(mSpeed.getSpeed(new Translation2d(55, 0)));
     }     
}
