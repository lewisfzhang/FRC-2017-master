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
        mLeftMaster = new CANTalon(3);
        mLeftSlave = new CANTalon(4);
        mRightMaster = new CANTalon(11);
        mRightSlave = new CANTalon(12);
        mRightMaster.reverseOutput(true);
    }

    public static Drive getInstance() {
        if (mInstance == null)
            mInstance = new Drive();
        return mInstance;
    }

    protected class Drive_Loop implements Loop {
        public void onStart(double timestamp) {
        	mRightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        	mRightSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
        	mRightSlave.set(11);
        	
        	mLeftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        	mLeftSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
        	mLeftSlave.set(3);
        }

        public void onLoop(double timestamp) {
            DriveSignal driveSignal = mCheesyDriveHelper.cheesyDrive(mControlBoard.getThrottle(),
                    mControlBoard.getTurn(), mControlBoard.getQuickTurn());
            mLeftMaster.set(driveSignal.getLeft());
            mRightMaster.set(driveSignal.getRight());

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
        mLeftMaster.set(0);
        mRightMaster.set(0);
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Left Drive Motor RPM", mLeftMaster.getSpeed());
        SmartDashboard.putNumber("Right Drive Motor RPM", mRightMaster.getSpeed());
    }

    public void zeroSensors() {

    }

}
