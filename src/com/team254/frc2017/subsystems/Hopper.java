package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Looper;

public class Hopper extends Subsystem {
    private static Hopper sInstance = null;
    public static Hopper getInstance() {
        if (sInstance == null) {
            sInstance = new Hopper();
        }
        return sInstance;
    }

    private CANTalon mMasterTalon, mSlaveTalon;

    private Hopper() {
        mMasterTalon = new CANTalon(Constants.kHopperMasterId);
        mSlaveTalon = new CANTalon(Constants.kHopperSlaveId);
        mMasterTalon.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mSlaveTalon.changeControlMode(CANTalon.TalonControlMode.Follower);
        mSlaveTalon.set(Constants.kHopperSlaveId);
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

}
