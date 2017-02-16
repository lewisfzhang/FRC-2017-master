package com.team254.frc2017.subsystems;

import com.team254.frc2017.loops.Looper;

public class Superstructure extends Subsystem {

    static Superstructure mInstance = new Superstructure();

    public static Superstructure getInstance() {
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
}
