package com.team254.frc2017.subsystems;

import com.team254.frc2017.ControlBoard;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.DriveSignal;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {
	private static Talon mLeftMaster, mRightMaster, mLeftSlave, mRightSlave; //Master and slave motor
	
	private static Drive mInstance;
	CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
	ControlBoard mControlBoard = ControlBoard.getInstance();
	
	private Drive()
	{
		mLeftMaster = new Talon(0);
    	mLeftSlave = new Talon(1);
    	mRightMaster = new Talon(2);
    	mRightSlave = new Talon(3);
	}
	
	public static Drive getInstance()
	{
		if (mInstance == null) mInstance = new Drive();
		return mInstance;
	} 
	
	protected class Drive_Looper implements Loop
	{
	    public void onStart()
	    {
	    	
	    }

	    public void onLoop()
	    {
			DriveSignal driveSignal = mCheesyDriveHelper.cheesyDrive(
					mControlBoard.getThrottle(),
					mControlBoard.getTurn(),
					mControlBoard.getQuickTurn());
			mLeftMaster.set(driveSignal.leftMotor);
			mLeftSlave.set(driveSignal.leftMotor);
			mRightMaster.set(driveSignal.rightMotor);
			mRightSlave.set(driveSignal.rightMotor);
			
	    	outputToSmartDashboard();
	    }

	    public void onStop()
	    {
	    	stop();
	    }
	}
	
	public void registerEnabledLoops(Looper in)
	{
		in.register(new Drive_Looper());
	}
	
	public void stop()
	{
		mLeftMaster.stopMotor();
    	mLeftSlave.stopMotor();
    	mRightMaster.stopMotor();
    	mRightSlave.stopMotor();
	}
	
	public void outputToSmartDashboard()
	{
    	SmartDashboard.putNumber("Left Drive Motor RPM", mLeftMaster.getSpeed());
    	SmartDashboard.putNumber("Right Drive Motor RPM", mRightMaster.getSpeed());
	}
	
	public void zeroSensors()
	{
		
	}
}
