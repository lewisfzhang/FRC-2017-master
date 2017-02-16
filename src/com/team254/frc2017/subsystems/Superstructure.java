package com.team254.frc2017.subsystems;

import com.team254.frc2017.loops.Looper;

public class Superstructure extends Subsystem {

    static Superstructure mInstance = null;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    private final Feeder mFeeder = Feeder.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Hopper mHopper = Hopper.getInstance();
    private final Shooter mShooter = Shooter.getInstance();


    @Override
    public void outputToSmartDashboard() {
    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        mFeeder.registerEnabledLoops(enabledLooper);
        mIntake.registerEnabledLoops(enabledLooper);
        mHopper.registerEnabledLoops(enabledLooper);
        mShooter.registerEnabledLoops(enabledLooper);
    }

    public void setWantIntakeStopped() {
        mIntake.setOff();
    }

    public void setWantIntakeOn() {
        mIntake.setOn();
    }

    public void setWantFeedOn() {
        mFeeder.setOn();
    }

    public void setWantFeedIdle() {
        mFeeder.setOff();
    }
}
