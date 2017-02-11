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
    public AdjustedBlock(Frame.Block block){
        switch(Constants.kPixyNumber) {
        case 1:
            PixyNumber1Constants one = new PixyNumber1Constants();
            cx = one.cx;
            cy = one.cy;
            k1 = one.k1;
            k2 = one.k2;
            k3 = one.k3;
            break;
        case 2:
            PixyNumber2Constants two = new PixyNumber2Constants();
            cx = two.cx;
            cy = two.cy;
            k1 = two.k1;
            k2 = two.k2;
            k3 = two.k3;
            break;
        case 3:
            PixyNumber3Constants three = new PixyNumber3Constants();
            cx = three.cx;
            cy = three.cy;
            k1 = three.k1;
            k2 = three.k2;
            k3 = three.k3;
            break;
        case 4:
            PixyNumber4Constants four = new PixyNumber4Constants();
            cx = four.cx;
            cy = four.cy;
            k1 = four.k1;
            k2 = four.k2;
            k3 = four.k3;
            break;
        case 5:
            PixyNumber5Constants five = new PixyNumber5Constants();
            cx = five.cx;
            cy = five.cy;
            k1 = five.k1;
            k2 = five.k2;
            k3 = five.k3;
            break;
        case 6:
            PixyNumber6Constants six = new PixyNumber6Constants();
            cx = six.cx;
            cy = six.cy;
            k1 = six.k1;
            k2 = six.k2;
            k3 = six.k3;
            break;
        }
        Point center = transformCoordinates(block.centerX, block.centerY);
        this.centerX = center.x;
        this.centerY = center.y;
        undistortFourCorners(this);
    }
    private void undistortFourCorners(AdjustedBlock block) {
        // based on the inverted plane of a camera, you can calculate the four side x and y values of the rectangular block
        double leftSideX = block.centerX-(block.width/2);
        double rightSideX = block.centerX+(block.width/2);
        double topSideY = block.centerY-(block.height/2);
        double bottomSideY = block.centerY+(block.height/2);

        // Undistort the two opposite points of the rectangular block
        Point bottomLeft = transformCoordinates(leftSideX, bottomSideY);
        Point topRight = transformCoordinates(rightSideX, topSideY);

        // Calculate the width and height based on opposite points of rectangular block
        block.width = topRight.x - bottomLeft.x;
        block.height = bottomLeft.y - topRight.y;
    }
    private static Point transformCoordinates(double xDistorted, double yDistorted) {
        // put in terms of cx and cy being the origin and normalize
        double xDistNormalized = (xDistorted - cx)/400;
        double yDistNormalized = (yDistorted - cy)/400;
        // apply undistortion
        double rDistorted = Math.hypot(xDistNormalized, yDistNormalized);
        double rUndistorted = radiusFuncInv(rDistorted);
        double xUndistorted = xDistNormalized*rUndistorted/rDistorted;
        double yUndistorted = yDistNormalized*rUndistorted/rDistorted;
        // convert back to pixel space and return
        Point undistort = new Point((int) (Math.round(xUndistorted*400 + cx)), (int) (Math.round(yUndistorted*400 + cy)));
        return undistort;
    }
    private static double radiusFuncInv(double rDistorted) { // Function for Undistortion
        double rSquared = rDistorted*rDistorted;
        double rUndistorted = rDistorted;
        rDistorted *= rSquared;
        rUndistorted -= k1*rDistorted;
        rDistorted *= rSquared;
        rUndistorted += (3*k1*k1 - k2)*rDistorted;
        rDistorted *= rSquared;
        rUndistorted += (-12*k1*k1*k1 + 8*k1*k2 - k3)*rDistorted;
        return rUndistorted;
    }
}
