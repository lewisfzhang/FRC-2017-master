package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import edu.wpi.first.wpilibj.DriverStation;
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

    private Shooter() {
        mRightMaster = new CANTalon(Constants.kRightShooterMasterId);
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.Voltage);
        mRightMaster.changeMotionControlFramePeriod(1);
        mRightMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 5);
        mRightMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        mRightMaster.reverseSensor(true);
        mRightMaster.reverseOutput(false);
        mRightMaster.enableBrakeMode(false);

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
    }

    public void refreshControllerConsts() {
        mRightMaster.setP(Constants.kShooterTalonKP);
        mRightMaster.setI(Constants.kShooterTalonKI);
        mRightMaster.setD(Constants.kShooterTalonKD);
        mRightMaster.setF(Constants.kShooterTalonKF);
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
        }
        // Talon speed is in ??? units
        mRightMaster.set(setpointRpm);
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
}
