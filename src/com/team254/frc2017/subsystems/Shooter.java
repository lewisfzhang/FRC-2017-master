package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import com.team254.lib.util.drivers.CANTalonFactory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
    private final int kSpinUpProfile = 0;
    private final int kHoldProfile = 1;

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
        SPIN_UP_LOOP,
        HOLD_LOOP,
    }

    private final CANTalon mRightMaster, mRightSlave, mLeftSlave1, mLeftSlave2;

    private ControlMethod mControlMethod;
    private double mSetpointRpm;
    
    // Used for transitioning from spin-up to hold loop.
    private double mHoldKf = 0.0;
    private int mHoldKfSamples = 0;
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
        //mRightMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 5);

        CANTalon.FeedbackDeviceStatus sensorPresent =
                mRightMaster.isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (sensorPresent != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect shooter encoder: " + sensorPresent, false);
        }

        mRightSlave = makeSlave(Constants.kRightShooterSlaveId, false);
        mLeftSlave1 = makeSlave(Constants.kLeftShooterSlave1Id, true);
        mLeftSlave2 = makeSlave(Constants.kLeftShooterSlave2Id, true);

        refreshControllerConsts();

        mControlMethod = ControlMethod.OPEN_LOOP;

        System.out.println("RPM Polynomial: " + Constants.kFlywheelAutoAimPolynomial);

        mCSVWriter = new ReflectingCSVWriter<ShooterDebugOutput>("/home/lvuser/SHOOTER-LOGS.csv", ShooterDebugOutput.class);
    }

    public void refreshControllerConsts() {
        mRightMaster.setProfile(kHoldProfile);
        mRightMaster.setP(Constants.kShooterTalonHoldKP);
        mRightMaster.setI(Constants.kShooterTalonHoldKI);
        mRightMaster.setD(Constants.kShooterTalonHoldKD);
        mRightMaster.setIZone(Constants.kShooterTalonIZone);
        
        mRightMaster.setProfile(kSpinUpProfile);
        mRightMaster.setP(Constants.kShooterTalonKP);
        mRightMaster.setI(Constants.kShooterTalonKI);
        mRightMaster.setD(Constants.kShooterTalonKD);
        mRightMaster.setF(Constants.kShooterTalonKF);
        mRightMaster.setIZone(Constants.kShooterTalonIZone);

        mRightMaster.setVoltageCompensationRampRate(Constants.kShooterVoltageCompensationRampRate);
        mRightMaster.setVoltageRampRate(Constants.kShooterRampRate);
    }

    @Override
    public void outputToSmartDashboard() {
        double current_rpm = getSpeedRpm();
        SmartDashboard.putNumber("shooter_speed_talon", current_rpm);
        SmartDashboard.putNumber("shooter_speed_error", mRightMaster.getSetpoint() - current_rpm);

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
                    mHoldKf = 0.0;
                    mHoldKfSamples = 0;
                    mOnTarget = false;
                    mOnTargetStartTime = Double.POSITIVE_INFINITY;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Shooter.this) {
                    if (mControlMethod != ControlMethod.OPEN_LOOP) {
                        // TODO: UNCOMMENT FOR USING HOLD_LOOP
                        //handleClosedLoop(timestamp);
                        mCSVWriter.add(mDebug);
                    } else {
                        // Reset all state.
                        mHoldKf = 0.0;
                        mHoldKfSamples = 0;
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

    public synchronized void setClosedLoopRpm(double setpointRpm) {
        if (mControlMethod != ControlMethod.SPIN_UP_LOOP) {
            configureForSpinUp();
        }
        mSetpointRpm = setpointRpm;

        mRightMaster.set(setpointRpm);
    }
    
    private void configureForSpinUp() {
        mControlMethod = ControlMethod.SPIN_UP_LOOP;
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
        mRightMaster.EnableCurrentLimit(false);
        mRightMaster.setProfile(kSpinUpProfile);
        mRightMaster.setNominalClosedLoopVoltage(12.0);  // TODO delete
        // mRightMaster.DisableNominalClosedLoopVoltage();
        
        // TODO(jared): Tune these.
        mRightMaster.SetVelocityMeasurementPeriod(CANTalon.VelocityMeasurementPeriod.Period_10Ms);
        mRightMaster.SetVelocityMeasurementWindow(32);
    }
    
    private void configureForHold() {
        mControlMethod = ControlMethod.HOLD_LOOP;
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
        mRightMaster.EnableCurrentLimit(false);
        mRightMaster.setProfile(kHoldProfile);
        mRightMaster.setNominalClosedLoopVoltage(12.0);
        mRightMaster.setF(Constants.kShooterTalonKF);  // TODO use mHoldKf

        // TODO(jared): Change velocity measurement params
        // mRightMaster.SetVelocityMeasurementPeriod(CANTalon.VelocityMeasurementPeriod.Period_5Ms);
        // mRightMaster.SetVelocityMeasurementWindow(1);
    }
    
    private void resetHold() {
        mHoldKf = 0.0;
        mHoldKfSamples = 0;
        mOnTarget = false;
        mOnTargetStartTime = Double.POSITIVE_INFINITY;
    }
    
    private void handleClosedLoop(double timestamp) {
        final double speed = getSpeedRpm();

        final double abs_error = Math.abs(speed - mSetpointRpm);
        // See if we should be spinning up or holding.
        if (mControlMethod == ControlMethod.SPIN_UP_LOOP) {
            final boolean on_target_now = mOnTarget ? abs_error < Constants.kShooterStopOnTargetRpm : abs_error < Constants.kShooterStartOnTargetRpm;
            if (on_target_now && !mOnTarget) {
                // First cycle on target.
                mOnTargetStartTime = timestamp;
                mOnTarget = true;
            } else if (!on_target_now) {
                resetHold();
            }
            
            if (mOnTarget) {
                // Update Kf.
                // TODO(jared): Note to up the frame with bus voltage
                final double kf = 1023.0 * (mRightMaster.getOutputVoltage() / 12.0) / (10.0 * 4096.0 * speed);
                mHoldKf = (mHoldKf * mHoldKfSamples + kf) / (mHoldKfSamples + 1);
                ++mHoldKfSamples;
            }
            if (timestamp - mOnTargetStartTime > Constants.kShooterMinOnTargetDuration) {
                configureForHold();
            } else {
                mRightMaster.set(mSetpointRpm);
            }
        }
        if (mControlMethod == ControlMethod.HOLD_LOOP) {
            mRightMaster.set(mSetpointRpm);
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
        return Util.epsilonEquals(getSpeedRpm(), mSetpointRpm, 100);
        //return mControlMethod == ControlMethod.HOLD_LOOP;
    }
    
    @Override
    public void writeToLog() {
        mCSVWriter.write();
    }
}
