package com.team254.frc2017.pixy;

import com.team254.lib.util.CrashTrackingRunnable;
import com.team254.lib.util.pixy.Frame;
import com.team254.lib.util.pixy.PixyCam;

import java.util.ArrayList;

public class PixyVisionServer {

    public static final long VISION_THREAD_DELAY = 2;

    private static PixyVisionServer _instance;
    boolean mRunning = false;
    Thread mThread = null;

    /**
     * Gets the singleton <code>VisionServer</code> instance.
     *
     * @return the instance
     */
    public static PixyVisionServer getInstance() {
        if (_instance == null) {
            _instance = new PixyVisionServer();
        }
        return _instance;
    }

    private PixyCam pixy;
    private ArrayList<PixyUpdateListener> listeners = new ArrayList<>();

    /**
     * Initializes the PixyCam and starts a new thread to poll it for data.
     */
    private PixyVisionServer() {
        pixy = new PixyCam();
    }

    public void start() {
        mRunning = true;
        if (mThread != null) {
            return;
        }

        mThread = new Thread(new CrashTrackingRunnable() {
            @Override
            public void runCrashTracked() {
                while (mRunning) {
                    // get a Frame from the pixy (if available)
                    Frame frame = pixy.getFrame();
                    if (frame != null) {
                        for (PixyUpdateListener l : listeners) {
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
        }, "Vision server thread");
        mThread.start();
    }

    public void stop() {
        mRunning = false;
        mThread = null;
    }

    /**
     * Registers a new <code>VisionUpdateListener</code> to receive updates about detected objects.
     *
     * @param listener
     *            – the listener
     * @throws IllegalArgumentException
     *             if <code>listener</code> is <code>null</code>
     */
    public void addListener(PixyUpdateListener listener) {
        if (listener == null)
            throw new IllegalArgumentException("Cannot add a null listener!");
        listeners.add(listener);
    }

}
