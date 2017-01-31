package com.team254.lib.util.pixy;

import java.awt.Point;

import com.team254.lib.util.pixy.Frame.Block; 

public class AdjustedBlock extends Block {
    static double cx = 163.019;
    static double cy = 96.8935;
    static double k1 = -0.431415; // radial distortion coefficient 1
    static double k2 = 0.260901; // radial distortion coefficient 2
    static double k3 = -0.134147; // radial distortion coefficient 3
    public AdjustedBlock(Block block){
        Point center = transformCoordinates(block.centerX, block.centerY);
        block.centerX = center.x;
        block.centerY = center.y;
        undistortFourCorners(block);
    }
    private void undistortFourCorners(Block block) {
        double leftSideX = block.centerX-(block.width/2);
        double rightSideX = block.centerX+(block.width/2);
        double topSideY = block.centerY-(block.height/2);
        double bottomSideY = block.centerY+(block.height/2);
        Point topLeft = transformCoordinates(leftSideX, topSideY);
        Point bottomLeft = transformCoordinates(leftSideX, bottomSideY);
        Point topRight = transformCoordinates(rightSideX, topSideY);
        if (topLeft.x >= 320 || topLeft.x < 0)
            topLeft.x = (int) leftSideX;
        if (topLeft.y >= 200 || topLeft.y < 0)
            topLeft.y = (int) topSideY;
        if (bottomLeft.x >= 320 || bottomLeft.x < 0)
            bottomLeft.x = (int) leftSideX;
        if (bottomLeft.y >= 200 || bottomLeft.y < 0)
            bottomLeft.y = (int) bottomSideY;
        if (topRight.x >= 320 || topRight.x < 0)
            topRight.x = (int) rightSideX;
        if (topRight.y >= 200 || topRight.y < 0)
            topRight.y = (int) rightSideX;
        block.width = topRight.x - topLeft.x;
        block.height = bottomLeft.y - topLeft.y;
    }
    private static Point transformCoordinates(double xDistorted, double yDistorted) {
        // put in terms of cx and cy being the orgin and normalize
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
    private static double radiusFuncInv(double rDistorted) {
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
