package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.SynchronousPIDF;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Proto_Shooter extends Subsystem {
    private static Proto_Shooter mInstance = null;

    public static Proto_Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Proto_Shooter();
        }
        return mInstance;
    }

    private CANTalon mShooterMaster, mShooterTwo, mIntakeMaster, mIntakeSlave;
    private SynchronousPIDF mController;
    private boolean mClosedLoop = false;

    private double mVelocityRpm = 0;

    private Proto_Shooter() {
        mShooterMaster = new CANTalon(1);
        mShooterMaster.changeControlMode(TalonControlMode.Speed);
        mShooterMaster.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
        mShooterMaster.setStatusFrameRateMs(StatusFrameRate.General, 1); // 1ms (1 KHz)
        mShooterMaster.setVoltageCompensationRampRate(10000.0);
        mShooterMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        mShooterMaster.setProfile(0);

        mShooterTwo = new CANTalon(2);
        mShooterTwo.changeControlMode(TalonControlMode.Speed);
        mShooterTwo.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
        mShooterTwo.setStatusFrameRateMs(StatusFrameRate.General, 1); // 1ms (1 KHz)
        mShooterTwo.setVoltageCompensationRampRate(10000.0);
        mShooterTwo.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        mShooterTwo.setProfile(0);

        mIntakeMaster = new CANTalon(3);
        mIntakeMaster.changeControlMode(TalonControlMode.PercentVbus);
        mIntakeMaster.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
        mIntakeMaster.setVoltageCompensationRampRate(10000.0);
        
        mIntakeSlave = new CANTalon(4);
        mIntakeSlave.changeControlMode(TalonControlMode.Follower);
        mIntakeSlave.set(3);

        mController = new SynchronousPIDF(Constants.kFlywheelKp, Constants.kFlywheelKi, Constants.kFlywheelKd,
                Constants.kFlywheelKf);
    }

    public class Proto_Shooter_Loop implements Loop {
        //private double mPrevTimestamp = 0;
        //private double mPrevRotations = 0;

        public void onStart(double timestamp) {
            synchronized (Proto_Shooter.this) {
                mController.reset();
                //mPrevTimestamp = timestamp;
                //mPrevRotations = mShooterMaster.getPosition();
                mVelocityRpm = 0.0;
                
                mShooterMaster.setPID(Constants.kShooterOneKp, 0, 0, Constants.kShooterOneKf, 0, 10000, 0);
                mShooterTwo.setPID(Constants.kShooterTwoKp, 0, 0, Constants.kShooterTwoKf, 0, 10000, 0);
                
                mIntakeMaster.setPID(0, 0, 0, 0, 0, 10000, 0);
                
                mShooterMaster.set(0.0);
                mShooterTwo.set(0.0);
                mIntakeMaster.set(0.0);
            }
        }

        public void onLoop(double timestamp) {
            synchronized (Proto_Shooter.this) {
            	mShooterMaster.set(-9500);
            	mShooterTwo.set(-9500);
            	mIntakeMaster.set(0.5);
            	SmartDashboard.putNumber("shooter_master_speed", mShooterMaster.getSpeed());
            	SmartDashboard.putNumber("shooter_two_speed", mShooterTwo.getSpeed());
            	System.out.println("Shooter Speed: " + mShooterMaster.getSpeed() + "Second Speed: " + mShooterTwo.getSpeed());
//                double curr_rotations = mShooterMaster.getPosition();
//                double delta_t = (timestamp - mPrevTimestamp) * 1000.0; // ms to seconds
//                if (delta_t < 1E-3) {
//                    delta_t = 1E-3; // Prevent divide-by-zero
//                }
//                mVelocityRpm = ((curr_rotations - mPrevRotations) * 60.0) / delta_t;
//
//                if (mClosedLoop) {
//                    double voltage = mController.calculate(mVelocityRpm, delta_t);
//                    setVoltage(voltage);
//                } else {
//                    setVoltage(0.0);
//                }
//
//                mPrevTimestamp = timestamp;
//                mPrevRotations = curr_rotations;
            }
        }

        public void onStop(double timestamp) {
            synchronized (Proto_Shooter.this) {
                mClosedLoop = false;
                stop();
            }
        }
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(new Proto_Shooter_Loop());
    }

    public synchronized void setRpmSetpoint(double rpm) {
        mClosedLoop = true;
        mController.setSetpoint(rpm);
    }

    public synchronized void setManualVoltage(double voltage) {
        mClosedLoop = false;
        setVoltage(voltage);
    }

    public synchronized double getRpm() {
        return mVelocityRpm;
    }

    // This is protected since it should only ever be called by a public synchronized method or the loop.
    protected void setVoltage(double voltage) {
        mShooterMaster.set(voltage);
        mShooterTwo.set(voltage);
    }

    // TODO: Move this to its own subsystem.
    public synchronized void setFeedRoller(double voltage) {
        mIntakeMaster.set(voltage);
    }

    public synchronized double getSetpoint() {
        return mController.getSetpoint();
    }

    public synchronized boolean isOnTarget() {
        return (Math.abs(getRpm() - getSetpoint()) < Constants.kFlywheelOnTargetTolerance);
    }

    @Override
    public void outputToSmartDashboard() {
    	SmartDashboard.putNumber("shooter_master_speed", mShooterMaster.getSpeed());
    	SmartDashboard.putNumber("shooter_two_speed", mShooterTwo.getSpeed());
        SmartDashboard.putNumber("flywheel_rpm", getRpm());
        SmartDashboard.putNumber("flywheel_setpoint", getSetpoint());
        SmartDashboard.putBoolean("flywheel_on_target", isOnTarget());
        SmartDashboard.putNumber("flywheel_master_current", mShooterMaster.getOutputCurrent());
        SmartDashboard.putNumber("flywheel_slave_current", mShooterTwo.getOutputCurrent());
        SmartDashboard.putNumber("intake_voltage", mIntakeMaster.getOutputVoltage());
    }

    @Override
    public synchronized void stop() {
        mClosedLoop = false;
        mController.reset();
        mShooterMaster.set(0.0);
        mIntakeMaster.set(0.0);
    }

    @Override
    public synchronized void zeroSensors() {
        // Nothing to do.
    }
}