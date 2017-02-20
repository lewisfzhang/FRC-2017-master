package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.CSVWriter;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {

    private static Shooter mInstance = null;

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    private enum ControlMethod {
        OPEN_LOOP,
        CLOSED_LOOP,
    }

    private final CANTalon mRightMaster, mRightSlave, mLeftSlave1, mLeftSlave2;

    private ControlMethod mControlMethod;
    // The setpoint the talon currently has
    private double mCurTalonSetpointRpm;
    private double mSetpointRpm;


    private final CSVWriter mCSVWriter;

    private Shooter() {
        mRightMaster = new CANTalon(Constants.kRightShooterMasterId);
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.Voltage);
        mRightMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 5);
        mRightMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        mRightMaster.reverseSensor(true);
        mRightMaster.reverseOutput(false);
        mRightMaster.enableBrakeMode(false);
        mRightMaster.SetVelocityMeasurementPeriod(CANTalon.VelocityMeasurementPeriod.Period_2Ms);
        mRightMaster.SetVelocityMeasurementWindow(1);
        mRightMaster.setNominalClosedLoopVoltage(12);

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

        mCSVWriter = new CSVWriter("/home/lvuser/SHOOTER-LOGS.csv", new String[]{"time", "flywheel_rpm"});
    }

    public void refreshControllerConsts() {
        mRightMaster.setP(Constants.kShooterTalonKP);
        mRightMaster.setI(Constants.kShooterTalonKI);
        mRightMaster.setD(Constants.kShooterTalonKD);
        mRightMaster.setF(Constants.kShooterTalonKF);
        mRightMaster.setVoltageRampRate(Constants.kShooterRampRate);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("shooter_speed_talon", getSpeedRpm());
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

            }

            @Override
            public void onLoop(double timestamp) {
                mCSVWriter.addValue(0, Timer.getFPGATimestamp());
                mCSVWriter.addValue(1, getSpeedRpm());
                mCSVWriter.write();
            }

            @Override
            public void onStop(double timestamp) {
                mCSVWriter.flush();
            }
        });
    }

    public synchronized void setOpenLoop(double voltage) {
        if (mControlMethod == ControlMethod.CLOSED_LOOP) {
            mControlMethod = ControlMethod.OPEN_LOOP;
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.Voltage);
        }
        mRightMaster.set(voltage);
    }

    public synchronized void setClosedLoopRpm(double setpointRpm) {
        if (mControlMethod == ControlMethod.OPEN_LOOP) {
            mControlMethod = ControlMethod.CLOSED_LOOP;
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            mCurTalonSetpointRpm = Double.MIN_VALUE;
        }

        mSetpointRpm = setpointRpm;

        if (!Util.epsilonEquals(setpointRpm, mCurTalonSetpointRpm, Constants.kShooterSetpointDeadbandRpm)) {
            // Talon speed is in rpm
            mRightMaster.set(setpointRpm);
            mCurTalonSetpointRpm = setpointRpm;
        }
    }

    private double getSpeedRpm() {
        return mRightMaster.getSpeed();
    }

    private static CANTalon makeSlave(int talonId, boolean flipOutput) {
        CANTalon slave = new CANTalon(talonId);
        slave.changeControlMode(CANTalon.TalonControlMode.Follower);
        slave.reverseOutput(flipOutput);
        slave.set(Constants.kRightShooterMasterId);
        slave.enableBrakeMode(false);
        slave.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 5);
        return slave;
    }

    public boolean isOnTarget() {
        // HACKS
        return getSpeedRpm() > 1000; //Util.epsilonEquals(getSpeedRpm(), mSetpointRpm, Constants.kShooterAllowableErrorRpm);
    }
}
