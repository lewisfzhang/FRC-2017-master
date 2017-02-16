package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Looper;

public class Intake extends Subsystem {
    private static Intake sInstance = new Intake();
    public static Intake getInstance() { return sInstance; };

    private CANTalon mMasterTalon, mSlaveTalon;

    private Intake() {
        mMasterTalon = new CANTalon(Constants.kIntakeMasterId);
        mSlaveTalon = new CANTalon(Constants.kIntakeSlaveId);
        mMasterTalon.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mSlaveTalon.changeControlMode(CANTalon.TalonControlMode.Follower);
        mSlaveTalon.set(Constants.kIntakeMasterId);
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

    @Override
    public void registerEnabledLoops(Looper in) {

    }


    public void setOn() {
        setOpenLoop(1.0);
    }

    public void setOff() {
        setOpenLoop(0.0);
    }

    public void setReverse() {
        setOpenLoop(-1.0);
    }

    private void setOpenLoop(double value) {

    }

}
