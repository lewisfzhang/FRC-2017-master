package com.team254.lib.util.pixy;

import java.util.List;

public class Frame {

    public static class Block {
        public static enum SyncType {
            NORMAL, COLOR_CODE
        }

        SyncType sync;
        public int checksum, signature, centerX, centerY, width, height;

        @Override
        public String toString() {
            return ":Block { signature: " + signature + ", center: (" + centerX + ", " + centerY + "), size: " + width
                    + "x" + height + " }";
        }
    }

    void setBlocks(List<Frame.Block> blocks) {
        this.blocks = blocks;
        ++id;

    }

    public List<Frame.Block> getBlocks() {
        return blocks;
    }

    public int getID() {
        return id;
    }

    @Override
    public String toString() {
        String str = "Frame { blockCount: " + blocks.size() + ", id: " + id;
        for (Frame.Block b : blocks) {
            str += "\n    " + b;
        }
        str += "\n}";
        return str;
    }

    List<Frame.Block> blocks;
    int id;
}
