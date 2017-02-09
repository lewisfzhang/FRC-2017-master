package com.team254.lib.util.pixy;

import java.util.ArrayList;
import java.util.List;

import com.team254.frc2017.Constants;
import com.team254.lib.util.pixy.Frame.Block;
import com.team254.lib.util.pixy.constants.PixyNumberConstants;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PixyCam {
    static double cx, cy, k1, k2, k3;
    private PixyNumberConstants kPixyConstants;
    public PixyCam() {
        this(Constants.kPixySPIRefreshRate, SPI.Port.kOnboardCS0);
        kPixyConstants = new PixyNumberConstants();
        cx = kPixyConstants.cx;
        cy = kPixyConstants.cy;
        k1 = kPixyConstants.k1;
        k2 = kPixyConstants.k2;
        k3 = kPixyConstants.k3;
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
        kPixyConstants = new PixyNumberConstants();
        cx = kPixyConstants.cx;
        cy = kPixyConstants.cy;
        k1 = kPixyConstants.k1;
        k2 = kPixyConstants.k2;
        k3 = kPixyConstants.k3;
    }
    
    /**
     * Reads up to 10 bytes, looking for the sync word (<code>0xaa55</code>).
     * @return <code>true</code> if the sync word was encountered,
     *         <code>false</code> if not (no block available)
     */
    private boolean waitForSync() {
        for (int n = 0; n < 10; n++) {
            int curByte = pspi.readByte();
            if (lastByte == 0xaa && curByte == 0x55)
                return true;
            lastByte = curByte;
        }
        return false;
    }
    private int lastByte = 0x00;
    
    protected Frame.Block parseBlock() {
        Frame.Block block = new Frame.Block();
        
        // wait for the sync word
        if (!waitForSync()) {
            // no block is available yet; return nothing
            return null;
        }

        // check if there's another sync word
        if (pspi.peekWord() == 0xaa55) {
            // this is a frame boundary; save the sync for the next parseBlock and return
            wasFrameBoundary = true;
            return null;
        }


        // read the block data
        block.checksum = pspi.readWord();
        block.signature = pspi.readWord();
        block.centerX = pspi.readWord();
        block.centerY = pspi.readWord();
        block.width = pspi.readWord();
        block.height = pspi.readWord();
        SmartDashboard.putNumber("Distorted (Orig.) X", block.centerX);
        SmartDashboard.putNumber("Distorted (Orig.) Y", block.centerY);
        Point center = transformCoordinates(block.centerX, block.centerY);
        block.centerX = center.x;
        block.centerY = center.y;
        undistortFourCorners(block);
        SmartDashboard.putNumber("Undistorted (New) X", block.centerX);
        SmartDashboard.putNumber("Undistorted (New) Y", block.centerY);
        int chk = (block.signature + block.centerX + block.centerY + block.width + block.height) & 0xFFFF;
        if (block.checksum != chk) {
            System.out.println("BLOCK HAD AN INVALID CHECKSUM (" + Integer.toHexString(block.checksum) + ", should be "
                                + Integer.toHexString(chk) + ")");
            return null; // skip this block
        }
        return block;
    }

    public Frame getFrame() {
        // get the next Block (if available)
        Frame.Block block = parseBlock();
        
        if (block != null) {
            // add the Block to the current frame's list
            blocksRead.add(block);
        } else if (wasFrameBoundary) {
            wasFrameBoundary = false;
            
            if (!blocksRead.isEmpty()) {
                // return a new Frame containing blocksRead
                List<Frame.Block> list = blocksRead;
                blocksRead = new ArrayList<>();
                return new Frame(list, frameCount++);
            }
        }
        
        // the next Frame isn't complete/ready yet
        return null;
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
            SmartDashboard.putNumber("Transform R value", rUndistorted);
            SmartDashboard.putNumber("Transform X value", xUndistorted*400);
            SmartDashboard.putNumber("Transform Y value", yUndistorted*400);
            SmartDashboard.putNumber("Camera cx value", cx);
            SmartDashboard.putNumber("Camera cy value", cy);
            return undistort;
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
    private List<Frame.Block> blocksRead = new ArrayList<>();
    private boolean wasFrameBoundary = false;

}
