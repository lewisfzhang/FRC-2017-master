package com.team254.frc2017.subsystems;

import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;

public class ConnectionMonitor extends Subsystem {

    public static double kConnectionTimeoutSec = 1.0;

    private static ConnectionMonitor mInstance = null;
    public static ConnectionMonitor getInstance() {
        if (mInstance == null) {
            mInstance = new ConnectionMonitor();
        }
        return mInstance;
    }

    private double mLastPacketTime;

    ConnectionMonitor() {
        mLastPacketTime = 0.0;
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(new Loop() {
           @Override
           public void onStart(double timestamp) {
               synchronized (ConnectionMonitor.this) {
                   mLastPacketTime = timestamp;
               }
           }


           @Override
           public void onLoop(double timestamp) {
               synchronized (ConnectionMonitor.this) {
                   if (timestamp - mLastPacketTime > kConnectionTimeoutSec) {
                       LED.getInstance().setWantedState(LED.WantedState.BLINK);
                   }
               }
           }

           @Override
           public  void onStop(double timestamp) {

           }
        });
    }

    @Override
    public void outputToSmartDashboard() {
    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }

    public synchronized void setLastPacketTime(double timestamp) {
        mLastPacketTime = timestamp;
    }
}
