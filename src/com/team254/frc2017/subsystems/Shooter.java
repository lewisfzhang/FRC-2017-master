package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.ShooterController;
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

    private final CANTalon mLeftMaster, mLeftSlave, mRightSlave1, mRightSlave2;
    private final ShooterController mController;

    private ControlMethod mControlMethod;
    private double mFlywheelRpm;

    private Shooter() {
        mLeftMaster = new CANTalon(Constants.kLeftShooterMasterId);
        mLeftMaster.changeControlMode(CANTalon.TalonControlMode.Voltage);
        mLeftMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 1);
        mLeftMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        mLeftMaster.reverseSensor(false);
        mLeftMaster.reverseOutput(false);
        mLeftMaster.enableBrakeMode(false);

        CANTalon.FeedbackDeviceStatus sensorPresent =
                mLeftMaster.isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (sensorPresent != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect shooter encoder: " + sensorPresent, false);
        }

        mLeftSlave = makeSlave(Constants.kLeftShooterSlaveId, false);
        mRightSlave1 = makeSlave(Constants.kRightShooterSlave1Id, true);
        mRightSlave2 = makeSlave(Constants.kRightShooterSlave2Id, true);

        mController = new ShooterController();
        refreshControllerConsts();

        mControlMethod = ControlMethod.OPEN_LOOP;
    }

    public void refreshControllerConsts() {
        mController.setConstants(
                Constants.kShooterKI,
                Constants.kShooterKF,
                Constants.kShooterBangLowThresholdFraction,
                Constants.kShooterSaturationDelay,
                Constants.kShooterMaxOutput,
                Constants.kShooterMinOutput,
                Constants.kShooterSaturationOutput);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("shooter_rpm", mFlywheelRpm);
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
            double mLastTimestamp;
            double mLastEncoderPositionTicks;

            @Override
            public void onStart(double timestamp) {
                mLastTimestamp = timestamp;
                mLastEncoderPositionTicks = mLeftMaster.getPosition();
                setOpenLoop(0);
            }

            @Override
            public void onLoop(double timestamp) {
                double dTime = timestamp - mLastTimestamp;
                mLastTimestamp = timestamp;

                double encoderPositionTicks = mLeftMaster.getPosition();
                mFlywheelRpm = (encoderPositionTicks - mLastEncoderPositionTicks) * 60.0 / 1024.0;
                mLastEncoderPositionTicks = encoderPositionTicks;

                synchronized (Shooter.this) {
                    if (mControlMethod == ControlMethod.OPEN_LOOP) {
                        // talon output is sticky
                        return;
                    }
                    mLeftMaster.set(mController.calculate(mFlywheelRpm, dTime));
                }
            }

            @Override
            public void onStop(double timestamp) {
                setOpenLoop(0);
                mFlywheelRpm = 0;
            }
        });
    }

    public synchronized void setOpenLoop(double voltage) {
        mControlMethod = ControlMethod.OPEN_LOOP;
        mLeftMaster.set(voltage);
    }

    public synchronized void setClosedLoopRpm(double setpointRpm) {
        if (mControlMethod == ControlMethod.OPEN_LOOP) {
            mController.resetState();
            mControlMethod = ControlMethod.CLOSED_LOOP;
        }
        mController.setSetpoint(setpointRpm);
    }

    private static CANTalon makeSlave(int talonId, boolean flipOutput) {
        CANTalon slave = new CANTalon(talonId);
        slave.changeControlMode(CANTalon.TalonControlMode.Follower);
        slave.reverseOutput(flipOutput);
        slave.set(Constants.kLeftShooterMasterId);
        slave.enableBrakeMode(false);
        slave.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 1);
        return slave;
    }
}
