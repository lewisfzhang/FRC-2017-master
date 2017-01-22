package com.team254.lib.util;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import com.team254.frc2017.Constants;

public class Path {
     List<PathSegment> segments;
     
     public Path(String filepath) {
         segments = new ArrayList<PathSegment>();
         loadFile(filepath);
     }
     
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
                     segments.add(new PathSegment.Segment(Double.parseDouble(s[1]), Double.parseDouble(s[2]), Double.parseDouble(s[3]), Double.parseDouble(s[4]), Double.parseDouble(s[5])));
                 else if(s[0].equals("ARC"))
                     segments.add(new PathSegment.Segment(Double.parseDouble(s[1]), Double.parseDouble(s[2]), Double.parseDouble(s[3]), Double.parseDouble(s[4]), Double.parseDouble(s[5]), Double.parseDouble(s[6]), Double.parseDouble(s[7])));
                 else if(s[0].equals("TURN"))
                     segments.add(new PathSegment.Turn(Double.parseDouble(s[1]), Double.parseDouble(s[2]), Double.parseDouble(s[3]), Double.parseDouble(s[4])));
             }
             sc.close();
         } catch (Exception e) {
             e.printStackTrace(); 
         }
     }
     
     public Translation2d getTargetPoint(Translation2d robotPos) {
         Translation2d target;
         PathSegment.Segment currentSegment = (PathSegment.Segment) segments.get(0);
         double segmentLength = currentSegment.getLength();
         double lookAheadDist = Constants.kAutoLookAhead;
         Translation2d closest = currentSegment.getClosestPoint(robotPos);
         double remainingDist = currentSegment.getRemainingDistance(closest);
         if(lookAheadDist - remainingDist <= 0) {
             target = currentSegment.getLookAheadPoint(segmentLength + lookAheadDist - remainingDist);
         } else {
             lookAheadDist -= remainingDist;
             int i = 1;
             while(lookAheadDist > 0 && i < segments.size()) {
//                 if(segments.get(i).isTurn()) {
//                     return ((PathSegment.Turn)segments.get(i)).getCenter();
//                 }
                 currentSegment = (PathSegment.Segment)segments.get(i);
                 lookAheadDist -= currentSegment.getLength();
                 i++;
             }
             target = currentSegment.getLookAheadPoint(currentSegment.getLength() + lookAheadDist);
         }
         checkSegmentDone(robotPos);
         return target;
     }
     
     public void checkSegmentDone(Translation2d robotPos) {
         PathSegment.Segment currentSegment = (PathSegment.Segment) segments.get(0);
         double length = currentSegment.getLength();
         double remainingDist = currentSegment.getRemainingDistance(currentSegment.getClosestPoint(robotPos));
         if((length - remainingDist) / length > Constants.kAutoSegmentThreshold) {
             segments.remove(0);
         }
     }
     
     public static void main(String[] args) {
//         PathSegment.Segment curve = new PathSegment.Segment(10.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.1, 0, 3.0 * Math.PI/2, 1.0);
//         Translation2d p = new Translation2d(1.0, 1.0);
//         System.out.println(curve.getRemainingDistance(curve.getClosestPoint(p)));
//         System.out.println(curve.getLength());
//         System.out.println(curve.getLookAheadPoint(15.707963267948964/2));
//         PathSegment.Segment line = new PathSegment.Segment(0.0, 0.0, 20.0, 20.0, 1.0);
//         p = new Translation2d(15.0, 10.0);
//         System.out.println(line.getClosestPoint(p));
//         System.out.println(line.getLength());
         
         //PathSegment.Segment curve = new PathSegment.Segment(43.533040796611594, 200.4199794406889, 100, 250.0, 100, 193.05472389242087, 100);
         //System.out.println(curve.getLookAheadPoint(18.05));
         Path test = new Path("~/path.txt");
         System.out.println(test.getTargetPoint(new Translation2d(20, 20)));
     }
     
}
