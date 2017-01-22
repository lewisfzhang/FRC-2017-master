package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Proto_Feeder extends Subsystem{
    private static Proto_Feeder mInstance;
    
    private CANTalon mFeed, mFeedSlave;
    
    private int kVelocityControlSlot = 0;
    
    public static Proto_Feeder getInstance() {
        if (mInstance == null) {
            mInstance = new Proto_Feeder();
        }
        return mInstance;
    }
    
    public Proto_Feeder() {
        mFeed = new CANTalon(3);
        mFeed.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
        mFeed.setVoltageCompensationRampRate(10000.0);
        mFeed.enableBrakeMode(false);

        mFeedSlave = new CANTalon(4);
        mFeedSlave.changeControlMode(TalonControlMode.Voltage);
        mFeedSlave.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
        mFeedSlave.setVoltageCompensationRampRate(10000.0);
        mFeedSlave.enableBrakeMode(false);

        updateConstants();
    }
    
    /*public class Proto_Feeder_Loop implements Loop {


        public void onStart(double timestamp) {
            
        }

        public void onLoop(double timestamp) {
            
        }

        public void onStop(double timestamp) {
            
        }
    }*/
    
    public synchronized void updateConstants() {
        mFeed.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (mFeed.isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect feeder encoder!", false);
        }
        mFeed.setPID(Constants.kFeedKp, Constants.kFeedKi, Constants.kFeedKd,
                Constants.kFeedKf, Constants.kFeedIZone, Constants.kFeedRampRate,
                kVelocityControlSlot);
        mFeed.changeControlMode(CANTalon.TalonControlMode.Speed);
        mFeed.setProfile(kVelocityControlSlot);
        mFeed.setAllowableClosedLoopErr(Constants.kFeedVelocityAllowableError);
        mFeed.configPeakOutputVoltage(-12.0, 12.0);
    }
    
    public synchronized void setSetpoint(double setpoint) {
        mFeed.setSetpoint(setpoint);
    }

    @Override
    public synchronized void outputToSmartDashboard() {
        // TODO Auto-generated method stub
        SmartDashboard.putNumber("Feed Speed", mFeed.getSpeed());
    }

    @Override
    public synchronized void stop() {
        // TODO Auto-generated method stub
        mFeed.setSetpoint(0);
        
    }

    @Override
    public void zeroSensors() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        // TODO Auto-generated method stub
        
    }

}
