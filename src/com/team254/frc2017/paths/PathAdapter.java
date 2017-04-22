package com.team254.frc2017.paths;

import java.util.ArrayList;

import com.team254.frc2017.Constants;
import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

/**
 * Takes field measurements and creates paths from them
 */
public class PathAdapter {
    // Values to measure on the field
    static final double kRedCenterToBoiler = 126.5;
    static final double kRedWallToAirship = 114;
    static final double kRedCenterToHopper = 162;

    static final double kBlueCenterToBoiler = 126.5;
    static final double kBlueWallToAirship = 114;
    static final double kBlueCenterToHopper = 162;
    
    // Path Variables
    static final double kRadius = 20;
    static final double kSpeed = 60;

    // Don't mess with these
    static final double kPegOffsetX = 17.77; // center of airship to boiler peg
    static final double kPegOffsetY = 30.66; // front of airship to boiler peg
    static final double kHopperX = 116.5; // x position of hopper
    static final double kHopperOffsetYOffset = 3; //how many inches into the hopper you want to go
    static final Rotation2d kRedPegHeading = Rotation2d.fromDegrees(240);
    static final Rotation2d kBluePegHeading = Rotation2d.fromDegrees(120);
    static final Rotation2d kRedHopperHeading = Rotation2d.fromDegrees(30);
    static final Rotation2d kBlueHopperHeading = Rotation2d.fromDegrees(330);
    static final Rotation2d kStartHeading = Rotation2d.fromDegrees(180);
    static final double kRearDist = Constants.kCenterToRearBumperDistance + 10;
    static final double kFrontDist = Constants.kCenterToIntakeDistance;
    static final double kSideDist = Constants.kCenterToSideBumperDistance;
    static final double kHopperTurnDistance = 24; 
    static final double kGearTurnDistance = 24; 
    static final double kFieldHeight = 324;
    
    public static Translation2d getRedHopperPosition() {
        Translation2d contactPoint = new Translation2d(kHopperX, kFieldHeight/2 - kRedCenterToHopper - kHopperOffsetYOffset);
        Translation2d robotOffset = new Translation2d(kRedHopperHeading.cos() * kRearDist,
                kRedHopperHeading.sin() * kRearDist);
        robotOffset.translateBy(new Translation2d(kRedHopperHeading.sin() * kSideDist,
                kRedHopperHeading.cos() * kSideDist));
        return contactPoint.translateBy(robotOffset);
    }
    
    public static Translation2d getRedHopperTurnPosition() {
        Translation2d hopperPosition = getRedHopperPosition();
        Translation2d turnOffset = new Translation2d(kRedHopperHeading.cos() * kHopperTurnDistance, kRedHopperHeading.sin() * kHopperTurnDistance);
        return hopperPosition.translateBy(turnOffset);
    }
    
    public static Translation2d getRedGearTurnPosition() {
        Translation2d gearPosition = getRedGearPosition();
        Translation2d turnOffset = new Translation2d(kRedPegHeading.cos() * kGearTurnDistance, kRedPegHeading.sin() * kGearTurnDistance);
        return gearPosition.translateBy(turnOffset);
    }

    private static Translation2d getRedGearPosition() {
        Translation2d pegPosition = new Translation2d(kRedWallToAirship + kPegOffsetX, kFieldHeight / 2 - kPegOffsetY);
        Translation2d robotOffset = new Translation2d(kRedPegHeading.cos() * kRearDist,
                kRedPegHeading.sin() * kRearDist);
        return pegPosition.translateBy(robotOffset);
    }

    public static RigidTransform2d getRedStartPose() {
        return new RigidTransform2d(new Translation2d(Constants.kCenterToFrontBumperDistance,
                kFieldHeight / 2 - kRedCenterToBoiler + Constants.kCenterToSideBumperDistance), kStartHeading);
    }

    private static Translation2d getRedCenterPosition() {
        RigidTransform2d end = new RigidTransform2d(getRedGearPosition(), kRedPegHeading);
        return getRedStartPose().intersection(end);
    }

    public static Path getRedGearPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(getRedStartPose().getTranslation(), 0, 0));
        sWaypoints.add(new Waypoint(getRedCenterPosition(), kRadius, kSpeed));
        sWaypoints.add(new Waypoint(getRedGearPosition(), 0, kSpeed));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    public static Path getRedHopperPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(getRedGearPosition(), 0, 0));
        sWaypoints.add(new Waypoint(getRedGearTurnPosition(), kRadius, kSpeed));
        sWaypoints.add(new Waypoint(getRedHopperTurnPosition(), kRadius, kSpeed));
        sWaypoints.add(new Waypoint(getRedHopperPosition(), 0, kSpeed));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    public static Translation2d getBlueHopperPosition() {
        Translation2d contactPoint = new Translation2d(kHopperX, kFieldHeight/2 + kBlueCenterToHopper + kHopperOffsetYOffset);
        Translation2d robotOffset = new Translation2d(kBlueHopperHeading.cos() * kRearDist,
                kBlueHopperHeading.sin() * kRearDist);
        robotOffset.translateBy(new Translation2d(kBlueHopperHeading.sin() * kSideDist,
                kBlueHopperHeading.cos() * kSideDist));
        return contactPoint.translateBy(robotOffset);
    }
    
    public static Translation2d getBlueHopperTurnPosition() {
        Translation2d hopperPosition = getBlueHopperPosition();
        Translation2d turnOffset = new Translation2d(kBlueHopperHeading.cos() * kHopperTurnDistance, kBlueHopperHeading.sin() * kHopperTurnDistance);
        return hopperPosition.translateBy(turnOffset);
    }
    
    public static Translation2d getBlueGearTurnPosition() {
        Translation2d gearPosition = getBlueGearPosition();
        Translation2d turnOffset = new Translation2d(kBluePegHeading.cos() * kGearTurnDistance, kBluePegHeading.sin() * kGearTurnDistance);
        return gearPosition.translateBy(turnOffset);
    }

    private static Translation2d getBlueGearPosition() {
        Translation2d pegPosition = new Translation2d(kBlueWallToAirship + kPegOffsetX, kFieldHeight / 2 + kPegOffsetY);
        Translation2d robotOffset = new Translation2d(kBluePegHeading.cos() * kRearDist,
                kBluePegHeading.sin() * kRearDist);
        return pegPosition.translateBy(robotOffset);
    }

    public static RigidTransform2d getBlueStartPose() {
        return new RigidTransform2d(new Translation2d(Constants.kCenterToFrontBumperDistance,
                kFieldHeight / 2 + kBlueCenterToBoiler - Constants.kCenterToSideBumperDistance), kStartHeading);
    }

    private static Translation2d getBlueCenterPosition() {
        RigidTransform2d end = new RigidTransform2d(getBlueGearPosition(), kBluePegHeading);
        return getBlueStartPose().intersection(end);
    }

    public static Path getBlueGearPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(getBlueStartPose().getTranslation(), 0, 0));
        sWaypoints.add(new Waypoint(getBlueCenterPosition(), kRadius, kSpeed));
        sWaypoints.add(new Waypoint(getBlueGearPosition(), 0, kSpeed));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    public static Path getBlueHopperPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(getBlueGearPosition(), 0, 0));
        sWaypoints.add(new Waypoint(getBlueGearTurnPosition(), kRadius, kSpeed));
        sWaypoints.add(new Waypoint(getBlueHopperTurnPosition(), kRadius, kSpeed));
        sWaypoints.add(new Waypoint(getBlueHopperPosition(), 0, kSpeed));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    public static void main(String[] args) {
        System.out.println("Red:\n" + getRedStartPose().getTranslation());
        System.out.println(getRedCenterPosition());
        System.out.println(getRedGearPosition() + "\n");
        System.out.println(getRedGearPosition());
        System.out.println(getRedGearTurnPosition());
        System.out.println(getRedHopperTurnPosition());
        System.out.println(getRedHopperPosition());

        
        System.out.println("\nBlue:\n" + getBlueStartPose().getTranslation());
        System.out.println(getBlueCenterPosition());
        System.out.println(getBlueGearPosition() + "\n");
        System.out.println(getBlueGearPosition());
        System.out.println(getBlueGearTurnPosition());
        System.out.println(getBlueHopperTurnPosition());
        System.out.println(getBlueHopperPosition());
    }

}
