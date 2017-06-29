package com.team254.lib.util.pixy;

import edu.wpi.first.wpilibj.SPI;

import java.util.ArrayList;
import java.util.List;

public class PixyCam {

    private Frame mCurrentFrame;
    private PeekableSPI pspi;
    private int frameCount = 0;
    private List<Frame.Block> blocksRead = new ArrayList<>();
    private boolean wasFrameBoundary = false;

    public PixyCam() {
        this(500000, SPI.Port.kOnboardCS0);
        mCurrentFrame = new Frame();
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

    /**
     * Reads up to 10 bytes, looking for the sync word (<code>0xaa55</code>).
     *
     * @return <code>true</code> if the sync word was encountered, <code>false</code> if not (no block available)
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
        int chk = (block.signature + block.centerX + block.centerY + block.width + block.height) & 0xFFFF;
        if (block.checksum != chk) {
            System.out.println("BLOCK HAD AN INVALID CHECKSUM (" + Integer.toHexString(block.checksum) + ", should be "
                    + Integer.toHexString(chk) + ")");
            return null; // skip this block
        }
        return block;
    }

    public Frame getFrame() {
        if (wasFrameBoundary) {
            wasFrameBoundary = false;
            blocksRead.clear();
        }
        // get the next Block (if available)
        Frame.Block block;
        while ((block = parseBlock()) != null) {
            blocksRead.add(block);
        }

        if (wasFrameBoundary) {
            if (!blocksRead.isEmpty()) {
                // return a new Frame containing blocksRead
                mCurrentFrame.setBlocks(blocksRead);
                return mCurrentFrame;
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

    @Override
    public String toString() {
        return "{ frameCount: " + frameCount + " }";
    }
}
