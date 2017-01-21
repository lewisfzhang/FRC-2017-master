package com.team254.lib.util;

public class PathSegment {
    
    static class Segment extends PathSegment {
        private Translation2d start;
        private Translation2d end;
        private Translation2d center;
        private double curvature;
        private double maxSpeed;
        private double startAngle;
        private double endAngle;
        
        
        public Segment(Translation2d start, Translation2d end, double maxSpeed) {
            this.start = start;
            this.end = end;
            this.center = null;
            this.curvature = 0;
            this.maxSpeed = maxSpeed;
            this.startAngle = 0;
            this.endAngle = 0;
        }
        
        public Segment(Double x1, Double y1, Double x2, Double y2, double maxSpeed) {
            this.start = new Translation2d(x1, y1);
            this.end = new Translation2d(x2, y2);
            this.center = null;
            this.curvature = 0;
            this.maxSpeed = maxSpeed;
            this.startAngle = 0;
            this.endAngle = 0;
        }
        
        public Segment(Translation2d start, Translation2d end, Translation2d center, double curvature, double startAngle, double endAngle, double maxSpeed) {
            this.start = start;
            this.end = end;
            this.center = center;
            this.curvature = curvature;
            this.maxSpeed = maxSpeed;
            this.startAngle = startAngle;
            this.endAngle = endAngle;
        }
        
        public Segment(double x1, double y1, double x2, double y2, double cx, double cy, double curvature, double startAngle, double endAngle, double maxSpeed) {
            this.start = new Translation2d(x1, y1);
            this.end = new Translation2d(x2, y2);
            this.center = new Translation2d(cx, cy);
            this.curvature = curvature;
            this.maxSpeed = maxSpeed;
            this.startAngle = startAngle;
            this.endAngle = endAngle;
        }
        
        public Segment(double x1, double y1, double x2, double y2, double cx, double cy, double maxSpeed) {
            this.start = new Translation2d(x1, y1);
            this.end = new Translation2d(x2, y2);
            this.center = new Translation2d(cx, cy);
            this.maxSpeed = maxSpeed;
            calcArc();
        }
        
        public Segment(Translation2d start, Translation2d end, Translation2d center, double maxSpeed) {
            this.start = start;
            this.end = end;
            this.center = center;
            this.maxSpeed = maxSpeed;
            calcArc();
        }
        
        private void calcArc() {
            Translation2d deltaS = new Translation2d(center, start);
            Translation2d deltaE = new Translation2d(center, end);
            this.startAngle = Math.atan2(-deltaS.getY(), deltaS.getX());
            startAngle = (startAngle < 0) ? startAngle + Math.PI : startAngle;
            this.endAngle = Math.atan2(-deltaE.getY(), deltaE.getX());
            endAngle = (endAngle < 0) ? endAngle + Math.PI : endAngle;
            this.curvature = 1/deltaS.norm();
        }
        
        public double getMaxSpeed() {
            return maxSpeed;
        }
        
        public Translation2d getStart() {
            return start;
        }
        
        public Translation2d getEnd() {
            return end;
        }
        
        public double getLength() {
            if(curvature == 0) {
                return new Translation2d(start, end).norm();
            } else {
                Double a = endAngle - startAngle;
                a = (a > Math.PI) ? Math.PI *2 - a : a;
                return Math.PI * (1/curvature) * a / Math.PI;
            }
        }
        
        public Translation2d getClosest(Translation2d from) {
            if(curvature == 0) {
                Translation2d delta = new Translation2d(start, end);
                double u = ((from.getX() - start.getX()) * delta.getX() + (from.getY() - start.getY()) * delta.getY()) / (delta.getX() * delta.getX() + delta.getY() * delta.getY());
                if (u < 0)
                    return start;
                else if (u > 1)
                    return end;
                else
                    return new Translation2d(start.getX() + u * delta.getX(), start.getY() + u * delta.getY());
            } else {
                Translation2d delta = new Translation2d(center, from);
                System.out.println("Center to Robot: " + delta);
                double s = (1/curvature) / delta.norm();
                System.out.println("Scalar: " + s);
                delta = delta.scale(s);
                double a = Math.atan2(-delta.getY(), delta.getX());
                a = (a < 0) ? a + Math.PI*2 : a;
                System.out.println("Angle: " + a);
                if(((endAngle - startAngle) <= Math.PI && a < startAngle && a > endAngle) || ((endAngle - startAngle) > Math.PI && a > startAngle && a < endAngle)) {
                    Translation2d startDist = new Translation2d(from, start);
                    Translation2d endDist = new Translation2d(from, start);
                    if(endDist.norm() < startDist.norm()) {
                        return end;
                    } else {
                        return start;
                    }
                }
                return center.translateBy(delta);
            }
            
        }
        
        
    }
    
    static class Turn extends PathSegment {
        Translation2d center;
        double turnAmount;
        double turnSpeed;
        
        public Turn(Translation2d center, Double turnAmount, Double turnSpeed) {
            this.center = center;
            this.turnAmount = turnAmount;
            this.turnSpeed = turnSpeed;
        }
        
        public Turn(double x, double y, double turnAmount, double turnSpeed) {
            this.center = new Translation2d(x, y);
            this.turnAmount = turnAmount;
            this.turnSpeed = turnSpeed;
        }
        
        public Translation2d getCenter() {
            return center;
        }
        
        public double getTurnAmount() {
            return turnAmount;
        }
        
        public double getTurnSpeed() {
            return turnSpeed;
        }
    }
}