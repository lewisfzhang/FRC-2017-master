package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Looper;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends Subsystem {
    private static Intake sInstance = null;

    public static Intake getInstance() {
        if (sInstance == null) {
            sInstance = new Intake();
        }
        return sInstance;
    }


    private CANTalon mMasterTalon, mSlaveTalon;
    private Solenoid mDeploySolenoid;

    private boolean mDeployed = false;

    private Intake() {
        mMasterTalon = new CANTalon(Constants.kIntakeMasterId);
        mMasterTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 15);
        mSlaveTalon = new CANTalon(Constants.kIntakeSlaveId);
        mSlaveTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 15);
        mMasterTalon.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mSlaveTalon.changeControlMode(CANTalon.TalonControlMode.Follower);
        mSlaveTalon.set(Constants.kIntakeMasterId);
        mDeploySolenoid = new Solenoid(Constants.kDeploySolenoidId);
    }

    @Override
    public void outputToSmartDashboard() {

    }

    @Override
    public void stop() {
        setOff();
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerEnabledLoops(Looper in) {

    }

    private void deploy() {
        mDeploySolenoid.set(true);
        mDeployed = true;
    }

    public void setOn() {
        deploy();
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
