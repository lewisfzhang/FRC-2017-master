package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.drivers.LazyCANTalon;

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

    private Intake() {
        mMasterTalon = new LazyCANTalon(Constants.kIntakeMasterId);
        mMasterTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 15);
        mMasterTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);
        mSlaveTalon = new LazyCANTalon(Constants.kIntakeSlaveId);
        mSlaveTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 15);
        mSlaveTalon.changeControlMode(CANTalon.TalonControlMode.Follower);
        mSlaveTalon.set(Constants.kIntakeMasterId);
        mSlaveTalon.reverseOutput(true);
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

    public synchronized void deploy() {
        mDeploySolenoid.set(true);
    }

    public synchronized void setOn() {
        deploy();
        setOpenLoop(Constants.kIntakeVoltage);
    }

    public synchronized void setOff() {
        setOpenLoop(0.0);
    }

    public synchronized void setReverse() {
        setOpenLoop(-Constants.kIntakeVoltage);
    }

    private void setOpenLoop(double voltage) {
        voltage = -voltage; // Flip so +V = intake
        mMasterTalon.set(voltage);
    }

}
