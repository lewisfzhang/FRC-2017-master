package com.team254.lib.util;

public class PathSegment {
    
    class Point {
        double x;
        double y;
        
        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
    
    static class Segment extends PathSegment {
        Point start;
        Point end;
        double curvature;
        double maxSpeed;
        
        public Segment(Point start, Point end, double curvature, double maxSpeed) {
            this.start = start;
            this.end = end;
            this.curvature = curvature;
            this.maxSpeed = maxSpeed;
        }
        
        public Segment(double x1, double y1, double x2, double y2, double curvature, double maxSpeed) {
            this.start = new Point(x1, y1);
            this.end = new Point(x2, y2);
            this.curvature = curvature;
            this.maxSpeed = maxSpeed;
        }
    }
    
    static class Turn extends PathSegment {
        Point center;
        double turnAmount;
        double turnSpeed;
        
        public Turn(Point center, Double turnAmount, Double turnSpeed) {
            this.center = center;
            this.turnAmount = turnAmount;
            this.turnSpeed = turnSpeed;
        }
        
        public Turn(double x, double y, double turnAmount, double turnSpeed) {
            this.center = new Point(x, y);
            this.turnAmount = turnAmount;
            this.turnSpeed = turnSpeed;
        }
    }
}