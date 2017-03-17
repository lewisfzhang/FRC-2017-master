package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.drivers.CANTalonFactory;

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
        mMasterTalon = CANTalonFactory.createDefaultTalon(Constants.kIntakeMasterId);
        mMasterTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 15);
        mMasterTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 500);
        mMasterTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);

        mSlaveTalon = CANTalonFactory.createPermanentSlaveTalon(Constants.kIntakeSlaveId, Constants.kIntakeMasterId);
        mSlaveTalon.reverseOutput(true);
        mDeploySolenoid = new Solenoid(Constants.kIntakeDeploySolenoidId);
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
    
    public synchronized void reset() {  // only use this in autoInit to reset the intake
        mDeploySolenoid.set(false);
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
        // voltage = -voltage; // Flip so +V = intake
        mMasterTalon.set(-voltage);
    }

}
