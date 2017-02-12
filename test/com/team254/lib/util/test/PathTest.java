package com.team254.lib.util.test;

import static org.junit.Assert.*;

import com.team254.frc2017.Constants;
import com.team254.lib.util.AdaptivePurePursuitController;
import com.team254.lib.util.Path;
import com.team254.lib.util.PathSegment;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

import org.junit.Test;

public class PathTest {
    public static final double kTestEpsilon = 1E-9;

    @Test
    public void testLinearPathSegment() {
        PathSegment.Translation segment = new PathSegment.Translation(0.0, 0.0, 100.0, 0.0, 50.0);
        assertEquals(100, segment.getLength(), kTestEpsilon);

        // GetClosestPoint - point on path
        Translation2d closestPoint = segment.getClosestPoint(new Translation2d(50,0));
        assertEquals(50, closestPoint.getX(), kTestEpsilon);
        assertEquals(0, closestPoint.getY(), kTestEpsilon);
        double dist = segment.getRemainingDistance(closestPoint);
        assertEquals(50, dist, kTestEpsilon);
        Translation2d lookAheadPoint = segment.getLookAheadPoint(75.0);
        assertEquals(75, lookAheadPoint.getX(), kTestEpsilon);
        assertEquals(0, lookAheadPoint.getY(), kTestEpsilon);


        // GetClosestPoint - point off of path
        closestPoint = segment.getClosestPoint(new Translation2d(20,50));
        assertEquals(20, closestPoint.getX(), kTestEpsilon);
        assertEquals(0, closestPoint.getY(), kTestEpsilon);
        dist = segment.getRemainingDistance(closestPoint);
        assertEquals(80, dist, kTestEpsilon);


        // GetClosestPoint - point behind start
        closestPoint = segment.getClosestPoint(new Translation2d(-30,-30));
        assertEquals(0, closestPoint.getX(), kTestEpsilon);
        assertEquals(0, closestPoint.getY(), kTestEpsilon);
        dist = segment.getRemainingDistance(closestPoint);
        assertEquals(100, dist, kTestEpsilon);

        // GetClosestPoint - point after end
        closestPoint = segment.getClosestPoint(new Translation2d(120,150));
        assertEquals(100, closestPoint.getX(), kTestEpsilon);
        assertEquals(0, closestPoint.getY(), kTestEpsilon);
        dist = segment.getRemainingDistance(closestPoint);
        assertEquals(0, dist, kTestEpsilon);
    
        //Try a different linear segment
        segment = new PathSegment.Translation(10.0, -12.0, -30.0, -120.0, 100.0);
        assertEquals(115.169440391, segment.getLength(), kTestEpsilon);

        // GetClosestPoint - point on path
        closestPoint = segment.getClosestPoint(new Translation2d(-20,-93));
        assertEquals(-20, closestPoint.getX(), kTestEpsilon);
        assertEquals(-93, closestPoint.getY(), kTestEpsilon);
        dist = segment.getRemainingDistance(closestPoint);
        assertEquals(28.7923600978, dist, kTestEpsilon);
        lookAheadPoint = segment.getLookAheadPoint(75.0);
        assertEquals(-16.048576686769547, lookAheadPoint.getX(), kTestEpsilon);
        assertEquals(-82.33115705427778, lookAheadPoint.getY(), kTestEpsilon);

        // GetClosestPoint - point off of path
        closestPoint = segment.getClosestPoint(new Translation2d(30,-39));
        assertEquals(3.618817852834706, closestPoint.getX(), kTestEpsilon);
        assertEquals(-29.2291917973462, closestPoint.getY(), kTestEpsilon);
        dist = segment.getRemainingDistance(closestPoint);
        assertEquals(96.79651096803562, dist, kTestEpsilon);


        // GetClosestPoint - point behind start
        closestPoint = segment.getClosestPoint(new Translation2d(30,30));
        assertEquals(10, closestPoint.getX(), kTestEpsilon);
        assertEquals(-12, closestPoint.getY(), kTestEpsilon);
        dist = segment.getRemainingDistance(closestPoint);
        assertEquals(115.169440391, dist, kTestEpsilon);

        // GetClosestPoint - point after end
        closestPoint = segment.getClosestPoint(new Translation2d(-21,-150));
        assertEquals(-30, closestPoint.getX(), kTestEpsilon);
        assertEquals(-120, closestPoint.getY(), kTestEpsilon);
        dist = segment.getRemainingDistance(closestPoint);
        assertEquals(0, dist, kTestEpsilon);
    }
    
    public void testArcPathSegment() {
        PathSegment.Translation segment = new PathSegment.Translation(0.0, 0.0, 100.0, 0.0, 50.0);
        assertEquals(100, segment.getLength(), kTestEpsilon);

        // GetClosestPoint - point on path
        Translation2d closestPoint = segment.getClosestPoint(new Translation2d(50,0));
        assertEquals(50, closestPoint.getX(), kTestEpsilon);
        assertEquals(0, closestPoint.getY(), kTestEpsilon);
        double dist = segment.getRemainingDistance(closestPoint);
        assertEquals(50, dist, kTestEpsilon);
        Translation2d lookAheadPoint = segment.getLookAheadPoint(75.0);
        assertEquals(75, lookAheadPoint.getX(), kTestEpsilon);
        assertEquals(0, lookAheadPoint.getY(), kTestEpsilon);


        // GetClosestPoint - point off of path
        closestPoint = segment.getClosestPoint(new Translation2d(20,50));
        assertEquals(20, closestPoint.getX(), kTestEpsilon);
        assertEquals(0, closestPoint.getY(), kTestEpsilon);
        dist = segment.getRemainingDistance(closestPoint);
        assertEquals(80, dist, kTestEpsilon);


        // GetClosestPoint - point behind start
        closestPoint = segment.getClosestPoint(new Translation2d(-30,-30));
        assertEquals(0, closestPoint.getX(), kTestEpsilon);
        assertEquals(0, closestPoint.getY(), kTestEpsilon);
        dist = segment.getRemainingDistance(closestPoint);
        assertEquals(100, dist, kTestEpsilon);

        // GetClosestPoint - point after end
        closestPoint = segment.getClosestPoint(new Translation2d(120,150));
        assertEquals(100, closestPoint.getX(), kTestEpsilon);
        assertEquals(0, closestPoint.getY(), kTestEpsilon);
        dist = segment.getRemainingDistance(closestPoint);
        assertEquals(0, dist, kTestEpsilon);
    }
}
