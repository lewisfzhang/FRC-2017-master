package com.team254.lib.util.pixy;

import java.awt.Point;

import com.team254.frc2017.Constants;
import com.team254.lib.util.pixy.constants.*;

public class AdjustedBlock extends Frame.Block {
    public static double cx;
    public static double cy;
    public static double k1;
    public static double k2;
    public static double k3;

    public AdjustedBlock(Frame.Block block) {
        PixyNumberConstants constant = new PixyNumberConstants();
        cx = constant.cx;
        cy = constant.cy;
        k1 = constant.k1;
        k2 = constant.k2;
        k3 = constant.k3;
        Point center = transformCoordinates(block.centerX, block.centerY);
        this.centerX = center.x;
        this.centerY = center.y;
        undistortFourCorners();
    }

    private void undistortFourCorners() {
        // based on the inverted plane of a camera, you can calculate the four side x and y values of the rectangular
        // block
        double leftSideX = centerX - (width / 2);
        double rightSideX = centerX + (width / 2);
        double topSideY = centerY - (height / 2);
        double bottomSideY = centerY + (height / 2);

        // Undistort the two opposite points of the rectangular block
        Point bottomLeft = transformCoordinates(leftSideX, bottomSideY);
        Point topRight = transformCoordinates(rightSideX, topSideY);

        // Calculate the width and height based on opposite points of rectangular block
        width = topRight.x - bottomLeft.x;
        height = bottomLeft.y - topRight.y;
    }

    private static Point transformCoordinates(double xDistorted, double yDistorted) {
        // put in terms of cx and cy being the origin and normalize
        double xDistNormalized = (xDistorted - cx) / 400;
        double yDistNormalized = (yDistorted - cy) / 400;
        // apply undistortion
        double rDistorted = Math.hypot(xDistNormalized, yDistNormalized);
        double rUndistorted = radiusFuncInv(rDistorted);
        double xUndistorted = xDistNormalized * rUndistorted / rDistorted;
        double yUndistorted = yDistNormalized * rUndistorted / rDistorted;
        // convert back to pixel space and return
        Point undistort = new Point((int) (Math.round(xUndistorted * 400 + cx)),
                (int) (Math.round(yUndistorted * 400 + cy)));
        return undistort;
    }

    private static double radiusFuncInv(double rDistorted) { // Function for Undistortion
        double rSquared = rDistorted * rDistorted;
        double rUndistorted = rDistorted;
        rDistorted *= rSquared;
        rUndistorted -= k1 * rDistorted;
        rDistorted *= rSquared;
        rUndistorted += (3 * k1 * k1 - k2) * rDistorted;
        rDistorted *= rSquared;
        rUndistorted += (-12 * k1 * k1 * k1 + 8 * k1 * k2 - k3) * rDistorted;
        return rUndistorted;
    }
}
