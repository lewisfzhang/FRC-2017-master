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

    static final double kBlueCenterToBoiler = 126.5;
    static final double kBlueWallToAirship = 114;

    // Path Variables
    static final double kRadius = 48;
    static final double kSpeed = 60;

    // Don't mess with these
    static final double kPegOffsetX = 17.77; // center of airship to boiler peg
    static final double kPegOffsetY = 30.66; // front of airship to boiler peg
    static final Rotation2d kRedPegHeading = Rotation2d.fromDegrees(240);
    static final Rotation2d kBluePegHeading = Rotation2d.fromDegrees(120);
    static final Rotation2d kStartHeading = Rotation2d.fromDegrees(180);
    static final double kRearDist = Constants.kCenterToRearBumperDistance + 10;
    static final double kFieldHeight = 324;

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

    public static void main(String[] args) {
        System.out.println("Red:\n" + getRedStartPose().getTranslation());
        System.out.println(getRedCenterPosition());
        System.out.println(getRedGearPosition());
        System.out.println("\nBlue:\n" + getBlueStartPose().getTranslation());
        System.out.println(getBlueCenterPosition());
        System.out.println(getBlueGearPosition());
    }

}
