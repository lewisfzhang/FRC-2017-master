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
    static final double kRedCenterToBoiler = 127.5; //distance from the center of the airship to the corner between the boiler and driver station wall
    static final double kRedWallToAirship = 116.5; //distance from driver station wall to front of the airship
    static final double kRedCenterToHopper = 160.66; //distance from the center of the airship to the hopper pad
    static final double kBlueCenterToBoiler = 126.5;
    static final double kBlueWallToAirship = 114;
    static final double kBlueCenterToHopper = 162;

    // Correction for actual robot driving.
    // Difference between actual robot position and optimal at the gear peg (in local robot coords at the gear peg).
    static final double kRedBoilerGearXCorrection = 3.0;
    static final double kRedBoilerGearYCorrection = 7.0;
    static final double kBlueBoilerGearXCorrection = 3.0;
    static final double kBlueBoilerGearYCorrection = -7.0;

    // Path Variables
    static final double kLargeRadius = 40;
    static final double kRadius = 20;
    static final double kSpeed = 72;

    // Don't mess with these
    static final double kPegOffsetX = 17.77; // center of airship to boiler peg
    static final double kPegOffsetY = 30.66; // front of airship to boiler peg
    static final double kHopperX = 116.5; // x position of hopper
    static final double kHopperOffsetYOffset = 1.5; //how many inches into the hopper you want to go
    static final Rotation2d kRedPegHeading = Rotation2d.fromDegrees(240);
    static final Rotation2d kBluePegHeading = Rotation2d.fromDegrees(120);
    static final Rotation2d kRedHopperHeading = Rotation2d.fromDegrees(40); //angle to hit the red hopper at
    static final Rotation2d kBlueHopperHeading = Rotation2d.fromDegrees(320); //angle to hit the blue hopper at
    static final Rotation2d kStartHeading = Rotation2d.fromDegrees(180); //start angle (backwards)
    static final double kGearPlacementDist = Constants.kCenterToRearBumperDistance + 10; //distance away from the airship wall to place the gear at
    static final double kFrontDist = Constants.kCenterToIntakeDistance;
    static final double kSideDist = Constants.kCenterToSideBumperDistance;
    static final double kHopperTurnDistance = 24; // how long the third segment in the hopper path should be
    static final double kGearTurnDistance = 24; // how long the first segment in the hopper path should be
    static final double kEndHopperPathY = 96; // Y position we want the hopper path to end at
    static final double kFieldHeight = 324; // total height of the field in inches (doesn't really have to be accurate, everything is relative)
    
    public static Translation2d getRedHopperPosition() {
        Translation2d contactPoint = new Translation2d(kHopperX, kFieldHeight/2 - kRedCenterToHopper - kHopperOffsetYOffset);
        Translation2d robotOffset = new Translation2d(kFrontDist, kSideDist);
        robotOffset = robotOffset.direction().rotateBy(kRedHopperHeading).toTranslation().scale(robotOffset.norm());
        return contactPoint.translateBy(robotOffset);
    }

    //third point in the hopper path
    public static Translation2d getRedHopperTurnPosition() {
        Translation2d hopperPosition = getRedHopperPosition();
        Translation2d turnOffset = new Translation2d(kRedHopperHeading.cos() * kHopperTurnDistance,
                kRedHopperHeading.sin() * kHopperTurnDistance);
        return hopperPosition.translateBy(turnOffset);
    }

    //second point in the hopper path
    public static Translation2d getRedGearTurnPosition() {
        Translation2d gearPosition = getRedGearPosition();
        Translation2d turnOffset = new Translation2d(kRedPegHeading.cos() * kGearTurnDistance,
                kRedPegHeading.sin() * kGearTurnDistance);
        return gearPosition.translateBy(turnOffset);
    }

    public static Translation2d getRedGearCorrection() {
        return RigidTransform2d.fromRotation(kRedPegHeading)
                .transformBy(RigidTransform2d
                        .fromTranslation((new Translation2d(kRedBoilerGearXCorrection, kRedBoilerGearYCorrection))))
                .getTranslation();
    }

    //final position in the gear path, first position in the hopper path
    private static Translation2d getRedGearPosition() {
        Translation2d pegPosition = new Translation2d(kRedWallToAirship + kPegOffsetX, kFieldHeight / 2 - kPegOffsetY);
        Translation2d robotOffset = new Translation2d(kRedPegHeading.cos() * kGearPlacementDist,
                kRedPegHeading.sin() * kGearPlacementDist);
        return pegPosition.translateBy(robotOffset).translateBy(getRedGearCorrection().inverse());
    }

    //first position in the gear path
    public static RigidTransform2d getRedStartPose() {
        return new RigidTransform2d(new Translation2d(Constants.kCenterToFrontBumperDistance,
                kFieldHeight / 2 - kRedCenterToBoiler + Constants.kCenterToSideBumperDistance), kStartHeading);
    }

    //second position in the gear path
    private static Translation2d getRedCenterPosition() {
        RigidTransform2d end = new RigidTransform2d(getRedGearPosition(), kRedPegHeading);
        return getRedStartPose().intersection(end);
    }

    public static Path getRedGearPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(getRedStartPose().getTranslation(), 0, 0));
        sWaypoints.add(new Waypoint(getRedCenterPosition(), kLargeRadius, kSpeed));
        sWaypoints.add(new Waypoint(getRedGearPosition(), 0, kSpeed));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    public static Path getRedHopperPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(getRedGearPosition(), 0, 0));
        sWaypoints.add(new Waypoint(getRedGearTurnPosition(), kRadius, kSpeed));
        sWaypoints.add(new Waypoint(getRedHopperTurnPosition(), kRadius, kSpeed));
        sWaypoints.add(new Waypoint(getRedHopperPosition(), 0, kSpeed));

        Translation2d redHopperEndPosition = getRedHopperPosition();
        redHopperEndPosition.setY(kEndHopperPathY); //move y position to desired place
        sWaypoints.add(new Waypoint(redHopperEndPosition, 0, kSpeed));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    public static Translation2d getBlueHopperPosition() {
        Translation2d contactPoint = new Translation2d(kHopperX, kFieldHeight/2 + kBlueCenterToHopper + kHopperOffsetYOffset);
        Translation2d robotOffset = new Translation2d(kFrontDist, -kSideDist);
        robotOffset = robotOffset.direction().rotateBy(kBlueHopperHeading).toTranslation().scale(robotOffset.norm());
        return contactPoint.translateBy(robotOffset);
    }

    public static Translation2d getBlueHopperTurnPosition() {
        Translation2d hopperPosition = getBlueHopperPosition();
        Translation2d turnOffset = new Translation2d(kBlueHopperHeading.cos() * kHopperTurnDistance,
                kBlueHopperHeading.sin() * kHopperTurnDistance);
        return hopperPosition.translateBy(turnOffset);
    }

    public static Translation2d getBlueGearTurnPosition() {
        Translation2d gearPosition = getBlueGearPosition();
        Translation2d turnOffset = new Translation2d(kBluePegHeading.cos() * kGearTurnDistance,
                kBluePegHeading.sin() * kGearTurnDistance);
        return gearPosition.translateBy(turnOffset);
    }
    
    public static Translation2d getBlueGearCorrection() {
        return RigidTransform2d.fromRotation(kBluePegHeading)
                .transformBy(RigidTransform2d
                        .fromTranslation((new Translation2d(kBlueBoilerGearXCorrection, kBlueBoilerGearYCorrection))))
                .getTranslation();
    }

    private static Translation2d getBlueGearPosition() {
        Translation2d pegPosition = new Translation2d(kBlueWallToAirship + kPegOffsetX, kFieldHeight / 2 + kPegOffsetY);
        Translation2d robotOffset = new Translation2d(kBluePegHeading.cos() * kGearPlacementDist,
                kBluePegHeading.sin() * kGearPlacementDist);
        return pegPosition.translateBy(robotOffset).translateBy(getBlueGearCorrection().inverse());
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
        sWaypoints.add(new Waypoint(getBlueCenterPosition(), kLargeRadius, kSpeed));
        sWaypoints.add(new Waypoint(getBlueGearPosition(), 0, kSpeed));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    public static Path getBlueHopperPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(getBlueGearPosition(), 0, 0));
        sWaypoints.add(new Waypoint(getBlueGearTurnPosition(), kRadius, kSpeed));
        sWaypoints.add(new Waypoint(getBlueHopperTurnPosition(), kRadius, kSpeed));
        sWaypoints.add(new Waypoint(getBlueHopperPosition(), 0, kSpeed));
        
        Translation2d blueHopperEndPosition = getBlueHopperPosition();
        blueHopperEndPosition.setY(kEndHopperPathY); //move y position to desired place
        sWaypoints.add(new Waypoint(blueHopperEndPosition, 0, kSpeed));

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
