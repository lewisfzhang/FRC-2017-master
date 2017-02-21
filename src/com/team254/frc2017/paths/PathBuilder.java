package com.team254.frc2017.paths;

import java.util.List;

import com.team254.lib.util.AdaptivePurePursuitController;
import com.team254.lib.util.Path;
import com.team254.lib.util.PathSegment;
import com.team254.lib.util.Translation2d;

public class PathBuilder {
    private static final double kEpsilon = 1E-9;
    private static final double kReallyBigNumber = 1E9;
    
    static Path buildPathFromWaypoints(List<Waypoint> w) {
        Path p = new Path();
        if(w.size() < 2)
            throw new Error("Path must contain at least 2 waypoints");
        int i=0;
        if(w.size() > 2) {
            do {
                new Arc(getPoint(w, i), getPoint(w, i+1), getPoint(w, i+2)).addToPath(p);
                i++;
            } while(i < w.size() - 2);
        }
        new Line(w.get(w.size() - 2), w.get(w.size() - 1)).addToPath(p, 0);
        p.extrapolateLast();
        p.verifySpeeds();
        System.out.println(p);
        return p;
    }

    private static Waypoint getPoint(List<Waypoint> w, int i) {
        if(i > w.size())
            return w.get(w.size() -1);
        return w.get(i);
    }
    
    static class Waypoint {
        Translation2d position;
        double radius;
        double speed;
        String marker;
        
        public Waypoint(double x, double y, double r, double s) {
            position = new Translation2d(x, y);
            radius = r;
            speed = s;
        }
        
        public Waypoint(double x, double y, double r, double s, String m) {
            position = new Translation2d(x, y);
            radius = r;
            speed = s;
            marker = m;
        }
    }
    
    static class Line {
        Waypoint a;
        Waypoint b;
        Translation2d start;
        Translation2d end;
        Translation2d slope;
        double speed;
        
        public Line(Waypoint a, Waypoint b) {
            this.a = a;
            this.b = b;
            slope = new Translation2d(a.position, b.position);
            speed = b.speed;
            start = a.position.translateBy(slope.scale(a.radius / slope.norm()));
            end = b.position.translateBy(slope.scale(-b.radius / slope.norm()));
        }
        
        private void addToPath(Path p, double endSpeed) {
            double pathLength = new Translation2d(end, start).norm();
            if(pathLength > kEpsilon) {
                if(b.marker != null) {
                    p.addSegment(new PathSegment(start.getX(), start.getY(), end.getX(), end.getY(), b.speed, p.getLastMotionState(), endSpeed, b.marker));
                } else {
                    p.addSegment(new PathSegment(start.getX(), start.getY(), end.getX(), end.getY(), b.speed, p.getLastMotionState(), endSpeed));
                }
            }

        }
    }
    
    static class Arc {
        Line a;
        Line b;
        Translation2d center;
        double radius;
        double speed;
        
        public Arc(Waypoint a, Waypoint b, Waypoint c) {
            this(new Line(a, b), new Line(b, c));
        }
        
        public Arc(Line a, Line b) {
            this.a = a;
            this.b = b;
            this.speed = (a.speed + b.speed) / 2;
            this.center = intersect(a, b);
            this.radius = new Translation2d(center, a.end).norm();
        }
        
        private void addToPath(Path p) {
            a.addToPath(p, speed);
            if(radius > kEpsilon && radius < kReallyBigNumber) {
                p.addSegment(new PathSegment(a.end.getX(), a.end.getY(), b.start.getX(), b.start.getY(), center.getX(), center.getY(), speed, p.getLastMotionState(), b.speed));
            }
        }
        
        private static Translation2d intersect(Line l1, Line l2) {
            AdaptivePurePursuitController.Line lineA = new AdaptivePurePursuitController.Line(l1.end, l1.slope.perpendicular());
            AdaptivePurePursuitController.Line lineB = new AdaptivePurePursuitController.Line(l2.start, l2.slope.perpendicular());
            return AdaptivePurePursuitController.Line.intersection(lineA, lineB);
        }
    }
}
