package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;

public class Proto_Shooter extends Subsystem {

    private static Proto_Shooter mInstance = null;

    public static Proto_Shooter getInstance() {
        if (mInstance == null)
            mInstance = new Proto_Shooter();
        return mInstance;
    }

    private CANTalon mMaster, mSlave;

    private Proto_Shooter() {
        mMaster = new CANTalon(5);
        mMaster.changeControlMode(TalonControlMode.Voltage);
        mMaster.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
        mMaster.setStatusFrameRateMs(StatusFrameRate.General, 5); // 5ms (200 Hz)
        mMaster.setVoltageCompensationRampRate(10000.0);
        mMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);

        mSlave = new CANTalon(6);
        mSlave.changeControlMode(TalonControlMode.Follower);
        mSlave.setVoltageCompensationRampRate(10000.0);
    }

    public void setVelocity(double velocity_rpm) {
        // TODO implement!
    }

    @Override
    public void outputToSmartDashboard() {
    }

    @Override
    public void stop() {
        mMaster.set(0.0);
    }

    @Override
    public void zeroSensors() {
    }

}
