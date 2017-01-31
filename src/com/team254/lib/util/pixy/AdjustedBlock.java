package com.team254.lib.util.pixy;

import java.awt.Point;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException; 

public class AdjustedBlock extends Frame.Block {
    static double cx, cy, k1, k2, k3;
//    static double k1 = -0.431415; // radial distortion coefficient 1
//    static double k2 = 0.260901; // radial distortion coefficient 2
//    static double k3 = -0.134147; // radial distortion coefficient 3
    public AdjustedBlock(Frame.Block block){
        String line = "";
        BufferedReader br = null;
        try {
            File filePath = new File("output.txt");
            br = new BufferedReader(new FileReader(filePath));
            line = br.readLine(); // Camera Matrix:
            line = br.readLine(); // fx:
            line = br.readLine(); // fy: 
            line = br.readLine(); // cx: 
            cx = Double.parseDouble(line.substring(4));
            line = br.readLine(); // cy: 
            cy = Double.parseDouble(line.substring(4));
            line = br.readLine(); // blank line
            line = br.readLine(); // Distortion Coefficients:
            line = br.readLine(); // k1: 
            k1 = Double.parseDouble(line.substring(4));
            line = br.readLine(); // k2: 
            k2 = Double.parseDouble(line.substring(4));
            line = br.readLine(); // k3: 
            k3 = Double.parseDouble(line.substring(4));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            if (br != null) {
                try {
                    br.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
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
