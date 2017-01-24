package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.team254.frc2017.ControlBoard;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;

import edu.wpi.first.wpilibj.Talon;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Proto_Intake extends Subsystem {
    private static CANTalon mIntakeSlave, mIntakeMaster;

    private static Proto_Intake mInstance;
    ControlBoard mControlBoard = ControlBoard.getInstance();

    private Proto_Intake() {
        mIntakeMaster = new CANTalon(1);
        mIntakeMaster.changeControlMode(TalonControlMode.Voltage);
        mIntakeMaster.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
        mIntakeMaster.setVoltageCompensationRampRate(10000.0);
        mIntakeMaster.enableBrakeMode(false);
        
        mIntakeSlave = new CANTalon(2);
        mIntakeSlave.changeControlMode(TalonControlMode.Follower);
        mIntakeSlave.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
        mIntakeSlave.setVoltageCompensationRampRate(10000.0);
        mIntakeSlave.enableBrakeMode(false);
        mIntakeSlave.set(1);
    }

    public static Proto_Intake getInstance() {
        if (mInstance == null)
            mInstance = new Proto_Intake();
        return mInstance;
    }

    public class Proto_Intake_Loop implements Loop {
        public void onStart(double timestamp) {

        }

        public void onLoop(double timestamp) {
            
        }

        public void onStop(double timestamp) {
            stop();
        }
    }

    public void registerEnabledLoops(Looper in) {
        in.register(new Proto_Intake_Loop());
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Flywheel Motor RPM", mIntakeMaster.getSpeed());
    }
    
    public synchronized void setManualVoltage(double voltage) {
        setVoltage(voltage);
    }

 // This is protected since it should only ever be called by a public
    // synchronized method or the loop.
    protected void setVoltage(double voltage) {
        SmartDashboard.putNumber("PIDF (v)", voltage);
        mIntakeMaster.set(-voltage);
        mIntakeSlave.set(voltage);
    }
    
    public void stop() {
        mIntakeMaster.set(0.0);
        mIntakeSlave.set(0.0);
    }

    public void zeroSensors() {

    }
}
