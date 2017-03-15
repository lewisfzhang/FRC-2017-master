package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.MovingAverage;
import com.team254.lib.util.drivers.CANTalonFactory;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

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

    private MovingAverage mThrottleAverage = new MovingAverage(50);

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
        mThrottleAverage.clear();
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
        mThrottleAverage.addNumber(currentThrottle);
    }

    public synchronized void deploy() {
        mDeploySolenoid.set(true);
    }

    public synchronized void reset() { // only use this in autoInit to reset the intake
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

        double scale;
        if (mThrottleAverage.getSize() > 0) {
            scale = Math.min(0.0, Math.max(0.0, mThrottleAverage.getAverage()));
        } else {
            scale = 0.0;
        }

        return Constants.kIntakeVoltageMax - scale * Constants.kIntakeVoltageDifference;
    }

    private void setOpenLoop(double voltage) {
        // voltage = -voltage; // Flip so +V = intake
        mMasterTalon.set(-voltage);
    }

    public boolean checkSystem() {
        final double kCurrentThres = 0.5;

        setOn();

        Timer.delay(2.0);

        final double currentMaster = mMasterTalon.getOutputCurrent();
        final double currentSlave = mSlaveTalon.getOutputCurrent();

        setOff();

        System.out.println("Intake Master Current: " + currentMaster + " Slave current: " + currentSlave);

        boolean failure = false;

        if (currentMaster < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!!! Intake Master Current Low !!!!!!!!!!!!!!");
        }

        if (currentSlave < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!!! Intake Slave Current Low !!!!!!!!!!!!!!!!");
        }

        return !failure;
    }

}
