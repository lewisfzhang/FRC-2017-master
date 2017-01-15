package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.ControlBoard;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.DriveSignal;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {
    private static CANTalon mLeftMaster, mRightMaster, mLeftSlave, mRightSlave; // Master and slave motor

    private static Drive mInstance;
    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    ControlBoard mControlBoard = ControlBoard.getInstance();

    private Drive() {
        mLeftMaster = new CANTalon(11);
        mLeftSlave = new CANTalon(12);
        mRightMaster = new CANTalon(3);
        mRightSlave = new CANTalon(4);
    }

    public static Drive getInstance() {
        if (mInstance == null)
            mInstance = new Drive();
        return mInstance;
    }

    protected class Drive_Loop implements Loop {
        public void onStart(double timestamp) {

        }

        public void onLoop(double timestamp) {
            DriveSignal driveSignal = mCheesyDriveHelper.cheesyDrive(mControlBoard.getThrottle(),
                    mControlBoard.getTurn(), mControlBoard.getQuickTurn());
            mLeftMaster.set(driveSignal.getLeft());
            mLeftSlave.set(driveSignal.getLeft());
            mRightMaster.set(driveSignal.getRight());
            mRightSlave.set(driveSignal.getRight());

            outputToSmartDashboard();
        }

        public void onStop(double timestamp) {
            stop();
        }
    }

    public void registerEnabledLoops(Looper in) {
        in.register(new Drive_Loop());
    }

    public void stop() {
        mLeftMaster.stopMotor();
        mLeftSlave.stopMotor();
        mRightMaster.stopMotor();
        mRightSlave.stopMotor();
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Left Drive Motor RPM", mLeftMaster.getSpeed());
        SmartDashboard.putNumber("Right Drive Motor RPM", mRightMaster.getSpeed());
    }

    public void zeroSensors() {

    }

}
