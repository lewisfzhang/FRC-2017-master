package com.team254.lib.util.pixy;

import java.awt.Point;

import com.team254.lib.util.pixy.constants.PixyNumberConstants;

public class BlockAdjuster {
    private PixyNumberConstants mConstants;

    public BlockAdjuster(PixyNumberConstants constants) {
        mConstants = constants;
    }

    public void adjustBlock(Frame.Block block) {
        Point center = transformCoordinates(block.centerX, block.centerY);
        block.centerX = center.x;
        block.centerY = center.y;
        undistortFourCorners(block);
    }

    private  void undistortFourCorners(Frame.Block block) {
        // based on the inverted plane of a camera, you can calculate the four side x and y values of the rectangular
        // block
        double leftSideX = block.centerX - (block.width / 2);
        double rightSideX = block.centerX + (block.width / 2);
        double topSideY = block.centerY - (block.height / 2);
        double bottomSideY = block.centerY + (block.height / 2);

        // Undistort the two opposite points of the rectangular block
        Point bottomLeft = transformCoordinates(leftSideX, bottomSideY);
        Point topRight = transformCoordinates(rightSideX, topSideY);

        // Calculate the width and height based on opposite points of rectangular block
        block.width = topRight.x - bottomLeft.x;
        block.height = bottomLeft.y - topRight.y;
    }

    private Point transformCoordinates(double xDistorted, double yDistorted) {
        // put in terms of cx and cy being the origin and normalize
        double xDistNormalized = (xDistorted - mConstants.cx) / 400;
        double yDistNormalized = (yDistorted - mConstants.cy) / 400;
        // apply undistortion
        double rDistorted = Math.hypot(xDistNormalized, yDistNormalized);
        double rUndistorted = radiusFuncInv(rDistorted);
        double xUndistorted = xDistNormalized * rUndistorted / rDistorted;
        double yUndistorted = yDistNormalized * rUndistorted / rDistorted;
        // convert back to pixel space and return
        Point undistort = new Point((int) (Math.round(xUndistorted * 400 + mConstants.cx)),
                (int) (Math.round(yUndistorted * 400 + mConstants.cy)));
        return undistort;
    }

    private  double radiusFuncInv(double rDistorted) { // Function for Undistortion
        double rSquared = rDistorted * rDistorted;
        double rUndistorted = rDistorted;
        rDistorted *= rSquared;
        rUndistorted -= mConstants.k1 * rDistorted;
        rDistorted *= rSquared;
        rUndistorted += (3 * mConstants.k1 * mConstants.k1 - mConstants.k2) * rDistorted;
        rDistorted *= rSquared;
        rUndistorted += (-12 * mConstants.k1 * mConstants.k1 * mConstants.k1 + 8 * mConstants.k1 * mConstants.k2 - mConstants.k3) * rDistorted;
        return rUndistorted;
    }
}
