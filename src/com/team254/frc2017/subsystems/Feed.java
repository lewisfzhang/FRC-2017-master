package com.team254.frc2017.subsystems;

import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Feed extends Subsystem{

    AnalogInput mFeedCounter = new AnalogInput(0);
    
    private Double mLastSharpVoltage = 0.0;
    private Integer ballcount = 0;
    
    public Feed() {
        
    }
    
    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("SharpVoltage", mFeedCounter.getVoltage());  
        SmartDashboard.putNumber("BallCount", ballcount);
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void zeroSensors() {
        // TODO Auto-generated method stub
        
    }

    protected class Feed_Loop implements Loop {
        public void onStart(double timestamp) {

        }

        public void onLoop(double timestamp) {
            if (mLastSharpVoltage > Constants.kFeedSensorThreshold && mFeedCounter.getVoltage() < Constants.kFeedSensorThreshold) {
                ballcount++;
            }
            mLastSharpVoltage = mFeedCounter.getVoltage();
            outputToSmartDashboard();
        }

        public void onStop(double timestamp) {
            stop();
        }
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(new Feed_Loop());
    }
    
    private Double getSharpReading() {
        return mFeedCounter.getVoltage();
    }

}
