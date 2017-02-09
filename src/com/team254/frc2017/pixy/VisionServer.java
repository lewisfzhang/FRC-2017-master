package com.team254.frc2017.pixy;

import java.util.ArrayList;

import com.team254.lib.util.CrashTrackingRunnable;
import com.team254.lib.util.pixy.Frame;
import com.team254.lib.util.pixy.PixyCam;

public class VisionServer {
    
    public static final long VISION_THREAD_DELAY = 2;
    
    private static VisionServer _instance;
    
    /**
     * Gets the singleton <code>VisionServer</code> instance.
     * @return the instance
     */
    public static VisionServer getInstance() {
        if (_instance == null) {
            _instance = new VisionServer();
        }
        return _instance;
    }
    
    
    private PixyCam pixy;
    private ArrayList<VisionUpdateListener> listeners = new ArrayList<>();
    
    /**
     * Initializes the PixyCam and starts a new thread to poll it for data.
     */
    private VisionServer() {
        pixy = new PixyCam();
        new Thread(new CrashTrackingRunnable() {
            @Override
            public void runCrashTracked() {
                while (true) {
                    // get a Frame from the pixy (if available)
                    Frame frame = pixy.getFrame();
                    if (frame != null) {
                        for (VisionUpdateListener l : listeners) {
                            l.onUpdate(frame); // TODO: pass the update
                        }
                    }
                    
                    // sleep briefly to not hog the CPU
                    try {
                        Thread.sleep(VISION_THREAD_DELAY);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        }, "Vision server thread").start();
    }
    
    /**
     * Registers a new <code>VisionUpdateListener</code> to receive updates about detected objects.
     * @param listener – the listener
     * @throws IllegalArgumentException if <code>listener</code> is <code>null</code>
     */
    public void addListener(VisionUpdateListener listener) {
        if (listener == null) throw new IllegalArgumentException("Cannot add a null listener!");
        listeners.add(listener);
    }
    
}
