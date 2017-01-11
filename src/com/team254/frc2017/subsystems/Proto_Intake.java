package com.team254.frc2017.subsystems;

import com.team254.frc2017.ControlBoard;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.DriveSignal;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Proto_Intake extends Subsystem {
	Talon mIntake;
	
	private static Proto_Intake mInstance;
	ControlBoard mControlBoard = ControlBoard.getInstance();
	
	private Proto_Intake()
	{
		mIntake = new Talon(7);
	}
	
	public static Proto_Intake getInstance()
	{
		if (mInstance == null) mInstance = new Proto_Intake();
		return mInstance;
	}
	
    public void outputToSmartDashboard()
    {
    	SmartDashboard.putNumber("Flywheel Motor RPM", mIntake.getSpeed());
    }
    
    public void run()
    {
		if (mControlBoard.getIntakeButton()) mIntake.set(1);
		else mIntake.set(0);
    }

    public void stop()
    {
    	mIntake.stopMotor();
    }

    public void zeroSensors()
    {
    	
    }
}
