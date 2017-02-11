package com.team254.lib.util.pixy;

import java.util.List;

public class FrameAdjuster {
    public static void adjustFrame(Frame frame) {
        List<Frame.Block> list = frame.getBlocks();
        for (Frame.Block block : list) {
            BlockAdjuster.adjustBlock(block);
        }
    }
}
