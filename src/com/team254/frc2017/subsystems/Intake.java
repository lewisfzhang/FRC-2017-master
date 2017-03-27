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
    private double mCurrentThrottle;

    private Intake() {
        mMasterTalon = CANTalonFactory.createDefaultTalon(Constants.kIntakeMasterId);
        mMasterTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 15);
        mMasterTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 500);
        mMasterTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);

        mSlaveTalon = CANTalonFactory.createPermanentSlaveTalon(Constants.kIntakeSlaveId, Constants.kIntakeMasterId);
        mSlaveTalon.reverseOutput(true);
        mDeploySolenoid = new Solenoid(Constants.kIntakeDeploySolenoidId);

        // Set current throttle to 0.0;
        mCurrentThrottle = 0.0;
    }

    @Override
    public void outputToSmartDashboard() {

    }

    @Override
    public synchronized void stop() {
        mCurrentThrottle = 0.0;
        setOff();
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerEnabledLoops(Looper in) {

    }

    public synchronized void setCurrentThrottle(double currentThrottle) {
        mCurrentThrottle = currentThrottle;
    }

    public synchronized void deploy() {
        mDeploySolenoid.set(true);
    }
    
    public synchronized void reset() {  // only use this in autoInit to reset the intake
        mDeploySolenoid.set(false);
    }

    public synchronized void setOn() {
        deploy();
        setOpenLoop(getScaledIntakeVoltage());
    }

    public synchronized void setOff() {
        setOpenLoop(0.0);
    }

    public synchronized void setReverse() {
        setOpenLoop(-Constants.kIntakeVoltageMax);
    }

    private double getScaledIntakeVoltage() {
        // Perform a linear interpolation from the Abs of throttle. Keep in mind we want to run at
        // full throttle when in reverse.

        final double scale = Math.min(0.0, mCurrentThrottle);

        return Constants.kIntakeVoltageMax - scale * Constants.kIntakeVoltageDifference;
    }

    private void setOpenLoop(double voltage) {
        // voltage = -voltage; // Flip so +V = intake
        mMasterTalon.set(-voltage);
    }

}
