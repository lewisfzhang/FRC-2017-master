package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.ControlBoard;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.DriveSignal;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {
	Talon mLeft, mRight, sLeft, sRight; //Master and slave motor
	
	private static Drive mInstance;
	CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
	ControlBoard mControlBoard = ControlBoard.getInstance();
	
	private Drive()
	{
		mLeft = new Talon(0);
    	sLeft = new Talon(1);
    	mRight = new Talon(2);
    	sRight = new Talon(3);
	}
	
	public static Drive getInstance()
	{
		if (mInstance == null) mInstance = new Drive();
		return mInstance;
	}
	
    public void outputToSmartDashboard()
    {
    	SmartDashboard.putNumber("Left Drive Motor RPM", mLeft.getSpeed());
    	SmartDashboard.putNumber("Right Drive Motor RPM", mRight.getSpeed());
    }
    
    public void run()
    {
		DriveSignal driveSignal = mCheesyDriveHelper.cheesyDrive(
				mControlBoard.getThrottle(),
				mControlBoard.getTurn(),
				mControlBoard.getQuickTurn());
		mLeft.set(driveSignal.leftMotor);
		sLeft.set(driveSignal.leftMotor);
		mRight.set(driveSignal.rightMotor);
		sRight.set(driveSignal.rightMotor);
    }

    public void stop()
    {
    	mLeft.stopMotor();
    	sLeft.stopMotor();
    	mRight.stopMotor();
    	sRight.stopMotor();
    }

    public void zeroSensors()
    {
    	
    }
}
