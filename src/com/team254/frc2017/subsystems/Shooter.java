package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.CSVWriter;
import com.team254.lib.util.ConstantsBase;
import com.team254.lib.util.MovingAverage;
import com.team254.lib.util.Util;
import com.team254.lib.util.drivers.CANTalonFactory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
    private final int kSpinUpProfile = 0;
    private final int kHoldProfile = 1;

    private static Shooter mInstance = null;

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    private enum ControlMethod {
        OPEN_LOOP,
        SPIN_UP_LOOP,
        HOLD_SPEED_LOOP,
    }

    private final CANTalon mRightMaster, mRightSlave, mLeftSlave1, mLeftSlave2;

    private ControlMethod mControlMethod;
    // The setpoint the talon currently has
    private double mCurTalonSetpointRpm;
    private double mSetpointRpm;

    private double mLastTimeRPMOff;
    private MovingAverage mMovingVperRpm = new MovingAverage(30);

    private final CSVWriter mCSVWriter;

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

        mCSVWriter = new CSVWriter("/home/lvuser/SHOOTER-LOGS.csv", new String[]{"time", "flywheel_rpm"});
    }

    public void refreshControllerConsts() {
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
        SmartDashboard.putNumber("shooter_speed_talon", getSpeedRpm());
        SmartDashboard.putNumber("shooter_speed_error", mRightMaster.getSetpoint() - getSpeedRpm());

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
                    mLastTimeRPMOff = timestamp;
                    mControlMethod = ControlMethod.OPEN_LOOP;
                }
            }

            @Override
            public void onLoop(double timestamp) {
//                mCSVWriter.addValue(0, Timer.getFPGATimestamp());
//                mCSVWriter.addValue(1, getSpeedRpm());
//                mCSVWriter.write();

                synchronized (Shooter.this) {
                    if (mControlMethod == ControlMethod.SPIN_UP_LOOP) {
                        handleSpinUp(timestamp);
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                mCSVWriter.flush();
            }
        });
    }


    private void handleSpinUp(double timestamp) {
        if (!Util.epsilonEquals(getSpeedRpm(), mSetpointRpm, Constants.kShooterAllowableErrorRpm)) {
            // Out of epsilon, mark time.
            mLastTimeRPMOff = timestamp;
        }

        // Convert RPM to Native Units per 100 ms.
        mMovingVperRpm.addNumber(mRightMaster.getOutputVoltage() /
                (mCurTalonSetpointRpm / 60.0 / 10.0 * 4096.0));

        // If we hold for a quarter second.  Push to hold profile.
        if (timestamp - mLastTimeRPMOff > 0.25) {
            mControlMethod = ControlMethod.HOLD_SPEED_LOOP;

            // For now, assume bus is 12.0 V.
            final double VAverage = mMovingVperRpm.getAverage();
            final double kF = VAverage / 12.0 * 1023;


            mRightMaster.setNominalClosedLoopVoltage(12.0);
            mRightMaster.setPID(Constants.kShooterTalonHoldKP, Constants.kShooterTalonHoldKI,
                    Constants.kShooterTalonHoldKD, kF, Constants.kShooterTalonIZone,
                    Constants.kShooterVoltageCompensationRampRate, kHoldProfile);
            mRightMaster.setProfile(kHoldProfile);
        }
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
        if (mControlMethod == ControlMethod.HOLD_SPEED_LOOP) {
            if (Util.epsilonEquals(setpointRpm, mCurTalonSetpointRpm, Constants.kShooterSetpointDeadbandRpm)) {
                return;
            }
        }

        if (mControlMethod != ControlMethod.SPIN_UP_LOOP) {
            mControlMethod = ControlMethod.SPIN_UP_LOOP;
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            mRightMaster.EnableCurrentLimit(false);
            mCurTalonSetpointRpm = Double.MIN_VALUE;

            mRightMaster.DisableNominalClosedLoopVoltage();
            mRightMaster.setProfile(kSpinUpProfile);

            mMovingVperRpm.clear();
            mLastTimeRPMOff = Timer.getFPGATimestamp();
        }

        mSetpointRpm = setpointRpm;

        if (!Util.epsilonEquals(setpointRpm, mCurTalonSetpointRpm, Constants.kShooterSetpointDeadbandRpm)) {
            // Talon speed is in rpm
            mRightMaster.set(setpointRpm);
            mCurTalonSetpointRpm = setpointRpm;
        }
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
        // HACKS
        return (mControlMethod == ControlMethod.HOLD_SPEED_LOOP);
        //return getSpeedRpm() > 1000;
        //return Util.epsilonEquals(getSpeedRpm(), mSetpointRpm, Constants.kShooterAllowableErrorRpm);
    }
}
