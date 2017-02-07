package com.team254.frc2017.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.pixy.Frame;
import com.team254.lib.util.pixy.PixyCam;

import edu.wpi.first.wpilibj.SPI;

public class PixyCamSystem extends Subsystem {

    private boolean mClosedLoop = false;

    private Map<PixyCam, Frame> mLastFrames = new HashMap<>();
    private List<PixyCamSystem_Loop> mLooperThreads = new ArrayList<>();

    public void connectToPixy(SPI.Port port, int hz) {
        PixyCam newCam = new PixyCam(hz, port);
        // Start a looper
        PixyCamSystem_Loop camLooper = new PixyCamSystem_Loop(newCam);
        mLooperThreads.add(camLooper);
    }

    public void connectToPixy() {
        connectToPixy(SPI.Port.kOnboardCS0, 500000);
    }

    @Override
    public void outputToSmartDashboard() {
        // TODO Auto-generated method stub
        // TODO: Log pixy data?
    }

    @Override
    public void stop() {
        mClosedLoop = false;

    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        throw new UnsupportedOperationException(
                "Camera loops will be automatically created by registering cameras with connectToPixy().");
    }

    public class PixyCamSystem_Loop implements Loop {

        private PixyCam mCamera;

        public PixyCamSystem_Loop(PixyCam camera) {
            mCamera = camera;
        }

        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (PixyCamSystem.this) {
                if (mClosedLoop) {
                    // update camera, read data
                    Frame lastFrame = mCamera.getFrame();
                    // save most recent frame
                    mLastFrames.put(mCamera, lastFrame);
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            // TODO: Disable pixy?
        }

    }

}
