package com.team254.lib.util.pixy;

import com.team254.lib.util.pixy.constants.PixyNumberConstants;

import java.util.List;

public class FrameAdjuster {

    private BlockAdjuster mBlockAdjuster;

    public FrameAdjuster(PixyNumberConstants c) {
        mBlockAdjuster = new BlockAdjuster(c);
    }
    public void adjustFrame(Frame frame) {
        List<Frame.Block> list = frame.getBlocks();
        for (Frame.Block block : list) {
            mBlockAdjuster.adjustBlock(block);
        }
    }
}
