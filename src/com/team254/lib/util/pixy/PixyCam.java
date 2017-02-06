package com.team254.lib.util.pixy;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.util.pixy.Frame.Block;
import com.team254.lib.util.pixy.constants.PixyNumberConstants;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PixyCam {
    static double cx, cy, k1, k2, k3;
    public PixyCam() {
        this(500000, SPI.Port.kOnboardCS0);
    }
    
    class Point {
        int x;
        int y;
        public Point(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }

    public PixyCam(int clockRate, SPI.Port port) {
        SPI spi = new SPI(port);
        spi.setMSBFirst();
        spi.setClockActiveHigh();
        spi.setSampleDataOnRising();
        spi.setChipSelectActiveLow();
        spi.setClockRate(clockRate);
        pspi = new PeekableSPI(spi);
    }

    public Frame.Block parseBlock() {
        cx = PixyNumberConstants.cx;
        cy = PixyNumberConstants.cy;
        k1 = PixyNumberConstants.k1;
        k2 = PixyNumberConstants.k2;
        k3 = PixyNumberConstants.k3;
        Frame.Block block = new Frame.Block();
        // Wait for sync
        int lastByte = 0x00;
        while (true) {
            int curByte = pspi.readByte();
            if (lastByte == 0xaa && curByte == 0x55)
                break;
            lastByte = curByte;
        }

        // check if there's another sync word
        if (pspi.peekWord() == 0xaa55) {
            return null;
        }

        
        try {
         // read the block data
            block.checksum = pspi.readWord();
            block.signature = pspi.readWord();
            block.centerX = pspi.readWord();
            block.centerY = pspi.readWord();
            block.width = pspi.readWord();
            block.height = pspi.readWord();
            SmartDashboard.putDouble("Original X", block.centerX);
            SmartDashboard.putDouble("Original Y", block.centerY);
            Point center = transformCoordinates(block.centerX, block.centerY);
            block.centerX = center.x;
            block.centerY = center.y;
            undistortFourCorners(block);
            SmartDashboard.putDouble("New X", block.centerX);
            SmartDashboard.putDouble("New Y", block.centerY);
            int chk = block.signature + block.centerX + block.centerY + block.width + block.height;
            /*if (block.checksum != chk) {
                System.out.println("BLOCK HAD AN INVALID CHECKSUM (" + Integer.toHexString(block.checksum) + ", should be "
                        + Integer.toHexString(chk) + ")");
                return null;
            }*/

            return block;
        } catch(NoClassDefFoundError e) {
            e.printStackTrace();
        } catch(NullPointerException e) {
            e.printStackTrace();
        }
        return null;
        
    }

    public Frame getFrame() {
        List<Frame.Block> blocks = new ArrayList<>();
        for (int i = 0; i < 10; i++){
            Frame.Block b = parseBlock();
            if (b == null)
                break;
            blocks.add(b);

            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        return new Frame(blocks, frameCount++);
    }

    public void setServoPosition(int pan, int tilt) {
        pspi.writeWord(0xff00);
        pspi.writeWord(pan);
        pspi.writeWord(tilt);
    }

    public void setBrightness(int brightness) {
        pspi.writeWord(0xfe00);
        pspi.writeByte(brightness);
    }

    public void setLEDOptions(int r, int g, int b) {
        pspi.writeWord(0xfd00);
        pspi.writeByte(r);
        pspi.writeByte(g);
        pspi.writeByte(b);
    }

    private void undistortFourCorners(Block block) {
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

    private Point transformCoordinates(double xDistorted, double yDistorted) {
//        try {
            // put in terms of cx and cy being the origin and normalize
            double xDistNormalized = (xDistorted - cx)/400;
            double yDistNormalized = (yDistorted - cy)/400;
            // apply undistortion
            double rDistorted = Math.hypot(xDistNormalized, yDistNormalized);
            double rUndistorted = radiusFuncInv(rDistorted);
            double xUndistorted = xDistNormalized*rUndistorted/rDistorted;
            double yUndistorted = yDistNormalized*rUndistorted/rDistorted;
            // convert back to pixel space and return
            Point undistort = new Point((int) (Math.round(xUndistorted*400)), (int) (Math.round(yUndistorted*400)));
            return undistort;
//        } catch(NoClassDefFoundError e) {
//            System.out.println("You done goofed");
//            e.printStackTrace();
//            return new Point((int) xDistorted, (int) yDistorted);
//        }        
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

    @Override
    public String toString() {
        return "{ frameCount: " + frameCount + " }";
    }

    private PeekableSPI pspi;
    private int frameCount = 0;

}
