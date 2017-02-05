package com.team254.lib.util.pixy;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PixyCam {

    public PixyCam() {
        this(500000, SPI.Port.kOnboardCS0);
        SmartDashboard.getString("Got Pixy Mon");
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

        // read the block data
        block.checksum = pspi.readWord();
        block.signature = pspi.readWord();
        block.centerX = pspi.readWord();
        block.centerY = pspi.readWord();
        block.width = pspi.readWord();
        block.height = pspi.readWord();
        block = new AdjustedBlock(block);
        int chk = block.signature + block.centerX + block.centerY + block.width + block.height;
        if (block.checksum != chk) {
            System.out.println("BLOCK HAD AN INVALID CHECKSUM (" + Integer.toHexString(block.checksum) + ", should be "
                    + Integer.toHexString(chk) + ")");
            return null;
        }
        
        return block;
    }

    public Frame getFrame() {
        List<Frame.Block> blocks = new ArrayList<>();
        while (true) {
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

    @Override
    public String toString() {
        return "{ frameCount: " + frameCount + " }";
    }

    private PeekableSPI pspi;
    private int frameCount = 0;

}
