package com.team254.lib.util.pixy.constants;

public abstract class PixyNumberConstants {
    public double cx, cy, k1, k2, k3, fx, fy;

    public static PixyNumberConstants getThisRobotConstants() {
        return new PixyNumber1Constants();
    }
}
