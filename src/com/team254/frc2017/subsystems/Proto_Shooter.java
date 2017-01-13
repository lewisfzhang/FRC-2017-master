package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;
/*
 * TO-DO: Set other CANTalon as a follower
 * Actually make the thing vroom vroom
 */
import com.team254.frc2017.Constants;
import com.team254.frc2017.ControlBoard;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.SynchronousPIDF;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Proto_Shooter extends Subsystem {
	private static Proto_Shooter mInstance = null;
	private double velocity_rpm = 0;
	
	private CANTalon mMaster, mSlave, mIntake;
	private SynchronousPIDF pid;

	private Proto_Shooter() {
		mMaster = new CANTalon(3);
		mMaster.changeControlMode(TalonControlMode.Voltage);
		mMaster.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
		mMaster.setStatusFrameRateMs(StatusFrameRate.General, 1); // 1ms (1 KHz)
		mMaster.setVoltageCompensationRampRate(10000.0);
		mMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);

		mSlave = new CANTalon(4);
		mSlave.changeControlMode(TalonControlMode.Voltage);
		mSlave.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
		mSlave.setVoltageCompensationRampRate(10000.0);
		
		mIntake = new CANTalon(5);
		mIntake.changeControlMode(TalonControlMode.Voltage);
		mIntake.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
		mIntake.setVoltageCompensationRampRate(10000.0);
		
		pid = new SynchronousPIDF(Constants.kFlywheelKp, Constants.kFlywheelKi, Constants.kFlywheelKd, Constants.kFlywheelKf);
		pid.setSetpoint(Constants.kFlywheelTarget);

		//mMaster.setPID(Constants.kFlywheelKp, Constants.kFlywheelKi, Constants.kFlywheelKd);
	}

	public static Proto_Shooter getInstance() {
		if (mInstance == null)
			mInstance = new Proto_Shooter();
		return mInstance;
	}

	public class Proto_Shooter_Looper implements Loop {
		private double prev_timestamp = 0;
		private double prev_rotations = 0;
		
		public void onStart() {
			prev_timestamp = Timer.getFPGATimestamp();
			prev_rotations = mMaster.getPosition();
		}

		public void onLoop() {
			double curr_timestamp = Timer.getFPGATimestamp();
			double curr_rotations = mMaster.getPosition();
			velocity_rpm = ((curr_rotations - prev_rotations) * 60) / ((curr_timestamp - prev_timestamp) * 1000);
						
			if (ControlBoard.getInstance().getFireButton()) {
				double voltage = pid.calculate(velocity_rpm, (curr_timestamp - prev_timestamp) / 1000);
				setVoltage(voltage);
				mIntake.set(Constants.kFlywheelIntakeVoltage);
			} else {
				setRpm(0.0);
				mIntake.set(0.0);
			}	
								
			prev_timestamp = Timer.getFPGATimestamp();
			prev_rotations = mMaster.getPosition();
			outputToSmartDashboard();
		}

		public void onStop() {
			stop();
			velocity_rpm = 0; //loop is no longer calculating velocity so set it to 0 for now
		}
	}

	@Override
	public void registerEnabledLoops(Looper in) {
		in.register(new Proto_Shooter_Looper());
	}

	public synchronized double getRpm() {
		return velocity_rpm;
	}

	public synchronized void setRpm(double rpm) {
		mMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
		mMaster.set(rpm);
	}
	
	public synchronized void setVoltage(double voltage) {
		mMaster.changeControlMode(CANTalon.TalonControlMode.Voltage);
		mMaster.set(voltage);
	}

	public synchronized double getSetpoint() {
		return mMaster.getSetpoint();
	}

	public synchronized boolean isOnTarget() {
		return (Math.abs(getRpm() - getSetpoint()) < Constants.kFlywheelOnTargetTolerance);
	}

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("flywheel_rpm", getRpm());
		SmartDashboard.putNumber("flywheel_setpoint", mMaster.getSetpoint());
		SmartDashboard.putBoolean("flywheel_on_target", isOnTarget());
		SmartDashboard.putNumber("flywheel_master_current", mMaster.getOutputCurrent());
		SmartDashboard.putNumber("flywheel_slave_current", mSlave.getOutputCurrent());
		SmartDashboard.putNumber("intake_voltage", mIntake.getOutputVoltage());
	}

	@Override
	public void stop() {
		mMaster.set(0.0);
		mIntake.set(0.0);
	}

	@Override
	public void zeroSensors() {
	}
}
