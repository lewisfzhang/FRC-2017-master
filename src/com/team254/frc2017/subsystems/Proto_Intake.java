package com.team254.frc2017.subsystems;

import com.team254.frc2017.ControlBoard;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;

import edu.wpi.first.wpilibj.Talon;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Proto_Intake extends Subsystem {
    private static Talon mIntake;

    private static Proto_Intake mInstance;
    ControlBoard mControlBoard = ControlBoard.getInstance();

    private Proto_Intake() {
        mIntake = new Talon(7);
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
            if (mControlBoard.getIntakeButton())
                mIntake.set(1);
            else
                mIntake.set(0);
            outputToSmartDashboard();
        }

        public void onStop(double timestamp) {
            stop();
        }
    }

    public void registerEnabledLoops(Looper in) {
        in.register(new Proto_Intake_Loop());
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Intake RPM", mIntake.getSpeed());
    }

    public void run() {
        if (mControlBoard.getIntakeButton()) {
            mIntake.set(1.0);
        } else {
            mIntake.set(0.0);
        }
    }

    public void stop() {
        mIntake.set(0.0);
    }

    public void zeroSensors() {

    }
}
