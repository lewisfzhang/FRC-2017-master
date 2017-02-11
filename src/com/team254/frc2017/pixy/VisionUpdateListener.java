package com.team254.frc2017.pixy;

import com.team254.lib.util.pixy.Frame;

public interface VisionUpdateListener {
    
    /**
     * This method gets called with every new frame from the pixy.
     * @param frame – the new <code>Frame</code>, which contains a list of detected
     *              <code>Block</code>s (objects)
     */
    public void onUpdate(Frame frame);
    
}