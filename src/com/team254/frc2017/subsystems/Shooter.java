package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Looper;
import edu.wpi.first.wpilibj.DriverStation;

public class Shooter extends Subsystem {

    private static Shooter mInstance = new Shooter();

    public static Shooter getInstance() {
        return mInstance;
    }

    private final CANTalon mLeftMaster, mLeftSlave, mRightSlave1, mRightSlave2;

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
    }


    @Override
    public void outputToSmartDashboard() {

    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }


    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerEnabledLoops(Looper in) {

    }

    public void setOpenLoop(double voltage) {
        mLeftMaster.set(voltage);
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
