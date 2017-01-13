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

    private CANTalon mMaster, mSlave, mIntake;
    private SynchronousPIDF mController;
    private boolean mClosedLoop = false;

    private double mVelocityRpm = 0;

    private Proto_Shooter() {
        mMaster = new CANTalon(3);
        mMaster.changeControlMode(TalonControlMode.Voltage);
        mMaster.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
        mMaster.setStatusFrameRateMs(StatusFrameRate.General, 1); // 1ms (1 KHz)
        mMaster.setVoltageCompensationRampRate(10000.0);
        mMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);

        mSlave = new CANTalon(4);
        mSlave.changeControlMode(TalonControlMode.Voltage);
        mSlave.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
        mSlave.setVoltageCompensationRampRate(10000.0);

        mIntake = new CANTalon(5);
        mIntake.changeControlMode(TalonControlMode.Voltage);
        mIntake.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
        mIntake.setVoltageCompensationRampRate(10000.0);

        mController = new SynchronousPIDF(Constants.kFlywheelKp, Constants.kFlywheelKi, Constants.kFlywheelKd,
                Constants.kFlywheelKf);
    }

    public class Proto_Shooter_Loop implements Loop {
        private double mPrevTimestamp = 0;
        private double mPrevRotations = 0;

        public void onStart(double timestamp) {
            synchronized (Proto_Shooter.this) {
                mController.reset();
                mPrevTimestamp = timestamp;
                mPrevRotations = mMaster.getPosition();
                mVelocityRpm = 0.0;
            }
        }

        public void onLoop(double timestamp) {
            synchronized (Proto_Shooter.this) {
                double curr_rotations = mMaster.getPosition();
                double delta_t = (timestamp - mPrevTimestamp) * 1000.0; // ms to seconds
                if (delta_t < 1E-3) {
                    delta_t = 1E-3; // Prevent divide-by-zero
                }
                mVelocityRpm = ((curr_rotations - mPrevRotations) * 60.0) / delta_t;

                if (mClosedLoop) {
                    double voltage = mController.calculate(mVelocityRpm, delta_t);
                    setVoltage(voltage);
                } else {
                    setVoltage(0.0);
                }

                mPrevTimestamp = timestamp;
                mPrevRotations = curr_rotations;
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
        mMaster.set(voltage);
        mSlave.set(voltage);
    }

    // TODO: Move this to its own subsystem.
    public synchronized void setFeedRoller(double voltage) {
        mIntake.set(voltage);
    }

    public synchronized double getSetpoint() {
        return mController.getSetpoint();
    }

    public synchronized boolean isOnTarget() {
        return (Math.abs(getRpm() - getSetpoint()) < Constants.kFlywheelOnTargetTolerance);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("flywheel_rpm", getRpm());
        SmartDashboard.putNumber("flywheel_setpoint", getSetpoint());
        SmartDashboard.putBoolean("flywheel_on_target", isOnTarget());
        SmartDashboard.putNumber("flywheel_master_current", mMaster.getOutputCurrent());
        SmartDashboard.putNumber("flywheel_slave_current", mSlave.getOutputCurrent());
        SmartDashboard.putNumber("intake_voltage", mIntake.getOutputVoltage());
    }

    @Override
    public synchronized void stop() {
        mController.reset();
        mMaster.set(0.0);
        mSlave.set(0.0);
        mIntake.set(0.0);
    }

    @Override
    public synchronized void zeroSensors() {
        // Nothing to do.
    }
}
