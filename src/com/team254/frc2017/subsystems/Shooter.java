package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.CircularBuffer;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import com.team254.lib.util.drivers.CANTalonFactory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
    private static Shooter mInstance = null;

    public static class ShooterDebugOutput {
        public double timestamp;
        public double setpoint;
        public double rpm;
    }

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    private enum ControlMethod {
        OPEN_LOOP,
        SPIN_UP,
        HOLD_WHEN_READY,
        HOLD,
    }

    private final CANTalon mRightMaster, mRightSlave, mLeftSlave1, mLeftSlave2;

    private ControlMethod mControlMethod;
    private double mSetpointRpm;
    private CircularBuffer mKvEstimator = new CircularBuffer(Constants.kShooterKvBufferSize);

    // Used for transitioning from spin-up to hold loop.
    private boolean mOnTarget = false;
    private double mOnTargetStartTime = Double.POSITIVE_INFINITY;

    private ShooterDebugOutput mDebug = new ShooterDebugOutput();

    private final ReflectingCSVWriter<ShooterDebugOutput> mCSVWriter;

    private Shooter() {
        mRightMaster = CANTalonFactory.createDefaultTalon(Constants.kRightShooterMasterId);
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.Voltage);
        mRightMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        mRightMaster.reverseSensor(true);
        mRightMaster.reverseOutput(false);
        mRightMaster.enableBrakeMode(false);
        mRightMaster.SetVelocityMeasurementPeriod(CANTalon.VelocityMeasurementPeriod.Period_10Ms);
        mRightMaster.SetVelocityMeasurementWindow(32);
        mRightMaster.setNominalClosedLoopVoltage(12);

        mRightMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 2);
        mRightMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.AnalogTempVbat, 2);

        CANTalon.FeedbackDeviceStatus sensorPresent = mRightMaster
                .isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (sensorPresent != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect shooter encoder: " + sensorPresent, false);
        }

        mRightSlave = makeSlave(Constants.kRightShooterSlaveId, false);
        mLeftSlave1 = makeSlave(Constants.kLeftShooterSlave1Id, true);
        mLeftSlave2 = makeSlave(Constants.kLeftShooterSlave2Id, true);

        refreshControllerConsts();

        mControlMethod = ControlMethod.OPEN_LOOP;

        System.out.println("RPM Polynomial: " + Constants.kFlywheelAutoAimPolynomial);

        mCSVWriter = new ReflectingCSVWriter<ShooterDebugOutput>("/home/lvuser/SHOOTER-LOGS.csv",
                ShooterDebugOutput.class);
    }

    public void refreshControllerConsts() {
        mRightMaster.setProfile(0);
        mRightMaster.setP(Constants.kShooterTalonKP);
        mRightMaster.setI(Constants.kShooterTalonKI);
        mRightMaster.setD(Constants.kShooterTalonKD);
        mRightMaster.setF(Constants.kShooterTalonKF);
        mRightMaster.setIZone(Constants.kShooterTalonIZone);

        mRightMaster.setVoltageRampRate(Constants.kShooterRampRate);
    }

    @Override
    public synchronized void outputToSmartDashboard() {
        double current_rpm = getSpeedRpm();
        SmartDashboard.putNumber("shooter_speed_talon", current_rpm);
        SmartDashboard.putNumber("shooter_speed_error", mSetpointRpm - current_rpm);

        SmartDashboard.putBoolean("shooter on target", isOnTarget());
        // SmartDashboard.putNumber("shooter_talon_position", mRightMaster.getPosition());
        // SmartDashboard.putNumber("shooter_talon_enc_position", mRightMaster.getEncPosition());
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public void zeroSensors() {
        // Don't zero the flywheel, it'll make deltas screwy
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Shooter.this) {
                    mControlMethod = ControlMethod.OPEN_LOOP;
                    mKvEstimator.clear();
                    mOnTarget = false;
                    mOnTargetStartTime = Double.POSITIVE_INFINITY;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Shooter.this) {
                    if (mControlMethod != ControlMethod.OPEN_LOOP) {
                        handleClosedLoop(timestamp);
                        mCSVWriter.add(mDebug);
                    } else {
                        // Reset all state.
                        mKvEstimator.clear();
                        mOnTarget = false;
                        mOnTargetStartTime = Double.POSITIVE_INFINITY;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                mCSVWriter.flush();
            }
        });
    }

    public synchronized void setOpenLoop(double voltage) {
        if (mControlMethod != ControlMethod.OPEN_LOOP) {
            mControlMethod = ControlMethod.OPEN_LOOP;
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.Voltage);
            mRightMaster.setCurrentLimit(Constants.kShooterOpenLoopCurrentLimit);
            mRightMaster.EnableCurrentLimit(true);
        }
        mRightMaster.set(voltage);
    }

    public synchronized void setSpinUp(double setpointRpm) {
        if (mControlMethod != ControlMethod.SPIN_UP) {
            configureForSpinUp();
        }
        mSetpointRpm = setpointRpm;
    }
    
    public synchronized void setHoldWhenReady(double setpointRpm) {
        if (mControlMethod == ControlMethod.OPEN_LOOP || mControlMethod == ControlMethod.SPIN_UP) {
            configureForHoldWhenReady();
        }
        mSetpointRpm = setpointRpm;
    }

    private void configureForSpinUp() {
        mControlMethod = ControlMethod.SPIN_UP;
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
        mRightMaster.EnableCurrentLimit(false);
        mRightMaster.DisableNominalClosedLoopVoltage();
        mRightMaster.setVoltageRampRate(Constants.kShooterRampRate);
    }
    
    private void configureForHoldWhenReady() {
        mControlMethod = ControlMethod.HOLD_WHEN_READY;
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
        mRightMaster.EnableCurrentLimit(false);
        mRightMaster.DisableNominalClosedLoopVoltage();
        mRightMaster.setVoltageRampRate(Constants.kShooterRampRate);
    }

    private void configureForHold() {
        mControlMethod = ControlMethod.HOLD;
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.Voltage);
        mRightMaster.EnableCurrentLimit(false);
        mRightMaster.set(mKvEstimator.getAverage() * mSetpointRpm);
        mRightMaster.setVoltageRampRate(Constants.kShooterHoldRampRate);
        mRightMaster.setVoltageCompensationRampRate(Constants.kShooterHoldVoltageCompRate);
    }

    private void resetHold() {
        mKvEstimator.clear();
        mOnTarget = false;
    }

    private void handleClosedLoop(double timestamp) {
        final double speed = getSpeedRpm();

        // See if we should be spinning up or holding.
        if (mControlMethod == ControlMethod.SPIN_UP) {
            mRightMaster.set(mSetpointRpm);
            resetHold();
        } else if (mControlMethod == ControlMethod.HOLD_WHEN_READY) {
            final double abs_error = Math.abs(speed - mSetpointRpm);
            final boolean on_target_now = mOnTarget ? abs_error < Constants.kShooterStopOnTargetRpm
                    : abs_error < Constants.kShooterStartOnTargetRpm;
            if (on_target_now && !mOnTarget) {
                // First cycle on target.
                mOnTargetStartTime = timestamp;
                mOnTarget = true;
            } else if (!on_target_now) {
                resetHold();
            }

            if (mOnTarget) {
                // Update Kv.
                mKvEstimator.addValue(mRightMaster.getOutputVoltage() / speed);
            }
            if (mKvEstimator.getNumValues() >= Constants.kShooterMinOnTargetSamples) {
                configureForHold();
            } else {
                mRightMaster.set(mSetpointRpm);
            }
        }
        // No else because we may have changed control methods above.
        if (mControlMethod == ControlMethod.HOLD) {            
            // Update Kv if we exceed our target velocity.  As the system heats up, drag is reduced.
            if (speed > mSetpointRpm) {
                mKvEstimator.addValue(mRightMaster.getOutputVoltage() / speed);
                mRightMaster.set(mSetpointRpm * mKvEstimator.getAverage());
            }
        }
        mDebug.timestamp = timestamp;
        mDebug.rpm = speed;
        mDebug.setpoint = mSetpointRpm;
    }

    public synchronized double getLastSetpointRpm() {
        return mSetpointRpm;
    }

    private double getSpeedRpm() {
        return mRightMaster.getSpeed();
    }

    private static CANTalon makeSlave(int talonId, boolean flipOutput) {
        CANTalon slave = CANTalonFactory.createPermanentSlaveTalon(talonId, Constants.kRightShooterMasterId);
        slave.reverseOutput(flipOutput);
        slave.enableBrakeMode(false);
        return slave;
    }

    public synchronized boolean isOnTarget() {
        return mControlMethod == ControlMethod.HOLD;
    }

    @Override
    public void writeToLog() {
        mCSVWriter.write();
    }

    public boolean checkSystem() {
        final double kCurrentThres = 0.5;
        final double kRpmThres = 2000;

        setOpenLoop(8.0);

        Timer.delay(4.0);

        final double currentRightMaster = mRightMaster.getOutputCurrent();
        final double currentRightSlave = mRightSlave.getOutputCurrent();
        final double currentLeftSlave1 = mLeftSlave1.getOutputCurrent();
        final double currentLeftSlave2 = mLeftSlave2.getOutputCurrent();

        final double rpm = mRightMaster.getSpeed();

        setOpenLoop(0.0);

        System.out.println("Shooter Right Master Current: " + currentRightMaster + " Shooter Right Slave Current: " + currentRightSlave);
        System.out.println("Shooter Left Slave One Current: " + currentLeftSlave1 + " Shooter Left Slave Two Current: " + currentLeftSlave2);

        boolean failure = false;

        if (currentRightMaster < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Right Master Current Low !!!!!!!!!!");
        }

        if (currentRightSlave < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Right Slave Current Low !!!!!!!!!!");
        }

        if (currentLeftSlave1 < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Left Slave One Current Low !!!!!!!!!!");
        }

        if (currentLeftSlave2 < kCurrentThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter Left Slave Two Current Low !!!!!!!!!!");
        }

        if (rpm < kRpmThres) {
            failure = true;
            System.out.println("!!!!!!!!!!!!!!!!!! Shooter RPM Low !!!!!!!!!!!!!!!!!!!!!!!");
        }

        return !failure;
    }
}
