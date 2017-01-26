package com.team254.lib.util;

public abstract class PathSegment {
    
    abstract boolean isTurn();
    abstract double getSpeed();
    
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
        
        public boolean isTurn() {
            return false;
        }
        
        public double getSpeed() {
            return maxSpeed;
        }
        
        private void calcArc() {
            Translation2d deltaS = new Translation2d(center, start);
            Translation2d deltaE = new Translation2d(center, end);
            this.startAngle = Math.atan2(-deltaS.getY(), deltaS.getX());
            startAngle = (startAngle < 0) ? startAngle + Math.PI*2 : startAngle;
            this.endAngle = Math.atan2(-deltaE.getY(), deltaE.getX());
            endAngle = (endAngle < 0) ? endAngle + Math.PI*2 : endAngle;
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
        
        public Translation2d getClosestPoint(Translation2d from) {
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
                double s = (1/curvature) / delta.norm();
                delta = delta.scale(s);
                double a = Math.atan2(-delta.getY(), delta.getX());
                a = (a < 0) ? a + Math.PI*2 : a;
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
        
        public Translation2d getLookAheadPoint(double dist) throws Error {
            double length = getLength();
            if(dist > length)
                dist = length;
            if(curvature == 0) {
                Translation2d delta = new Translation2d(start, end);
                return start.translateBy( delta.scale(dist / length));
            } else {
                Double deltaAngle;
                if(endAngle - startAngle <= Math.PI) {
                    deltaAngle = (dist / length) * (endAngle - startAngle);
                } else {
                    deltaAngle = (dist / length) * (Math.PI * 2 - endAngle + startAngle);
                }
                Translation2d t = new Translation2d(Math.cos(deltaAngle + startAngle), -Math.sin(deltaAngle + startAngle)).scale(1/curvature);
                return center.translateBy(t);
            }
        }
        
        public double getRemainingDistance(Translation2d point) {
            if(curvature == 0) {
                return new Translation2d(end, point).norm();
            } else {
                Double angle = Math.atan2(center.getY()-point.getY(), point.getX()-center.getX());
                angle = (angle < 0) ? angle + Math.PI*2 : angle;
                double totalAngle;
                if(endAngle - startAngle <= Math.PI) {
                    totalAngle = (endAngle - startAngle);
                } else {
                    totalAngle = (Math.PI * 2 - endAngle + startAngle);
                }
                double deltaAngle = Math.abs(endAngle - angle);
                deltaAngle = (deltaAngle > Math.PI) ? Math.PI*2 - deltaAngle : deltaAngle;
                return deltaAngle/totalAngle * getLength();
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
        
        public boolean isTurn() {
            return true;
        }
        
        public double getSpeed() {
            return 0.0;
        }
    }
    
    static class ClosestPointReport {
        private Translation2d point;
        private double remainingDistance;
        
        public ClosestPointReport(Translation2d point, double remainingDistance) {
            this.point = point;
            this.remainingDistance = remainingDistance;
        }
        
        public double getRemainingDistance(double totalDist) {
            return totalDist - remainingDistance;
        }
        
        public Translation2d getPoint() {
            return point;
        }
    }
}