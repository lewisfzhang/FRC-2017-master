package com.team254.lib.util;

import java.io.File;
import java.util.LinkedList;
import java.util.List;
import java.util.Scanner;

public class Path {
     List<PathSegment> segments;
     
     public Path(String filepath) {
         segments = new LinkedList<PathSegment>();
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
         
         
         
         //Path test = new Path("~/path.txt");
     }
     
}
