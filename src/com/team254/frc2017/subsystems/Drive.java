package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.ControlBoard;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.DriveSignal;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {
    CANTalon mLeftMaster, mRightMaster, mLeftSlave, mRightSlave; // Master and
                                                                 // slave motors

    private static Drive mInstance;
    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    ControlBoard mControlBoard = ControlBoard.getInstance();

    private Drive() {
        mLeftMaster = new CANTalon(0);
        mLeftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mLeftSlave = new CANTalon(1);
        mLeftSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
        mLeftSlave.set(0);

        mRightMaster = new CANTalon(2);
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mRightSlave = new CANTalon(3);
        mRightSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
        mRightSlave.set(2);
    }

    public static Drive getInstance() {
        if (mInstance == null)
            mInstance = new Drive();
        return mInstance;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Left Drive Motor RPM", mLeftMaster.getSpeed());
        SmartDashboard.putNumber("Right Drive Motor RPM", mRightMaster.getSpeed());
    }

    public void run() {
        DriveSignal driveSignal = mCheesyDriveHelper.cheesyDrive(mControlBoard.getThrottle(), mControlBoard.getTurn(),
                mControlBoard.getQuickTurn());
        mLeftMaster.set(driveSignal.leftMotor);
        mRightMaster.set(driveSignal.rightMotor);
    }

    public void stop() {
        mLeftMaster.set(0.0);
        mRightMaster.set(0.0);
    }

    public void zeroSensors() {

    }
}
