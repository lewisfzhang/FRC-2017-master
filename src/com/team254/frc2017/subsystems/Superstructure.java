package com.team254.frc2017.subsystems;

import com.team254.frc2017.Constants;
import com.team254.frc2017.RobotState;
import com.team254.frc2017.ShooterAimingParameters;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.InterpolatingDouble;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;
import java.util.List;

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

    // Superstructure doesn't own the drive, but needs to access it
    private final Drive mDrive = Drive.getInstance();

    // Intenal state of the system
    public enum SystemState {
        IDLE,
        WAITING_FOR_AIM,
        SHOOTING,
        UNJAMMING,
        UNJAMMING_WITH_SHOOT,
        JUST_FEED,
    };

    // Desired function from user
    public enum WantedState {
        IDLE,
        SHOOT,
        UNJAM,
        UNJAM_SHOOT,
        MANUAL_FEED,
    }

    private SystemState  mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    public boolean isOnTargetToShoot() {
        return mDrive.isOnTarget() && mShooter.isOnTarget();
    }

    private Loop mLoop = new Loop() {

        // Every time we transition states, we update the current state start
        // time and the state changed boolean (for one cycle)
        private double mCurrentStateStartTime;
        private boolean mStateChanged;

        @Override
        public void onStart(double timestamp) {
            mWantedState = WantedState.IDLE;
            mCurrentStateStartTime = timestamp;
            mSystemState = SystemState.IDLE;
            mStateChanged = true;
        }

        @Override
        public void onLoop(double timestamp) {
            SystemState newState = mSystemState;
            switch (mSystemState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case WAITING_FOR_AIM:
                    newState = handleWaitingForAim();
                    break;
                case SHOOTING:
                    newState = handleShooting();
                    break;
                case UNJAMMING_WITH_SHOOT:
                    newState = handleUnjammingWithShoot();
                    break;
                case UNJAMMING:
                    newState = handleUnjamming();
                    break;
                case JUST_FEED:
                    newState = handleJustFeed();
                    break;
                default:
                    newState = SystemState.IDLE;
            }

            if (newState != mSystemState) {
                System.out.println("Superstructure state " + mSystemState + " to " + newState);
                mSystemState = newState;
                mCurrentStateStartTime = timestamp;
                mStateChanged = true;
            } else {
                mStateChanged = false;
            }

        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private SystemState handleIdle() {
        stop();
        mFeeder.setWantedState(Feeder.WantedState.IDLE);
        mHopper.setWantedState(Hopper.WantedState.IDLE);
        switch (mWantedState) {
            case UNJAM:
                return SystemState.UNJAMMING;
            case UNJAM_SHOOT:
                return SystemState.UNJAMMING_WITH_SHOOT;
            case SHOOT:
                return SystemState.WAITING_FOR_AIM;
            case MANUAL_FEED:
                return SystemState.JUST_FEED;
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handleWaitingForAim() {
        autoSpinShooter();
        mFeeder.setWantedState(Feeder.WantedState.IDLE);
        mHopper.setWantedState(Hopper.WantedState.IDLE);
        if (isOnTargetToShoot()) {
           return SystemState.SHOOTING;
        }
        return SystemState.WAITING_FOR_AIM;
    }

    private SystemState handleShooting() {
        autoSpinShooter();
        mFeeder.setWantedState(Feeder.WantedState.FEED);
        mHopper.setWantedState(Hopper.WantedState.FEED);
        if (!isOnTargetToShoot()) {
            mSystemState = SystemState.WAITING_FOR_AIM;
        }
        return SystemState.SHOOTING;
    }



    private SystemState handleUnjammingWithShoot() {
        autoSpinShooter();
        mFeeder.setWantedState(Feeder.WantedState.UNJAM);
        mHopper.setWantedState(Hopper.WantedState.UNJAM);
        switch (mWantedState) {
            case UNJAM:
                return SystemState.UNJAMMING;
            case UNJAM_SHOOT:
                return SystemState.UNJAMMING_WITH_SHOOT;
            case SHOOT:
                return SystemState.WAITING_FOR_AIM;
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handleUnjamming() {
        mShooter.stop();
        mFeeder.setWantedState(Feeder.WantedState.UNJAM);
        mHopper.setWantedState(Hopper.WantedState.UNJAM);
        switch (mWantedState) {
            case UNJAM:
                return SystemState.UNJAMMING;
            case UNJAM_SHOOT:
                return SystemState.UNJAMMING_WITH_SHOOT;
            case SHOOT:
                return SystemState.WAITING_FOR_AIM;
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handleJustFeed() {
        mFeeder.setWantedState(Feeder.WantedState.FEED);
        mHopper.setWantedState(Hopper.WantedState.FEED);
        switch (mWantedState) {
            case UNJAM:
                return SystemState.UNJAMMING;
            case UNJAM_SHOOT:
                return SystemState.UNJAMMING_WITH_SHOOT;
            case SHOOT:
                return SystemState.WAITING_FOR_AIM;
            case MANUAL_FEED:
                return SystemState.JUST_FEED;
            default:
                return SystemState.IDLE;
        }
    }

    private double getShootingSetpointRpm(double range) {
        return Constants.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
    }

    public void autoSpinShooter() {
        ShooterAimingParameters aim = RobotState.getInstance().getAimingParameters(Timer.getFPGATimestamp());
        mShooter.setClosedLoopRpm(getShootingSetpointRpm(aim.getRange()));
    }

    public void setmWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }

    public synchronized void setShooterOpenLoop(double voltage) {
        mShooter.setOpenLoop(voltage);
    }

    public synchronized void setClosedLoopRpm(double setpointRpm) {
        mShooter.setClosedLoopRpm(setpointRpm);
    }

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
        enabledLooper.register(mLoop);
    }

    public void setWantIntakeStopped() {
        mIntake.setOff();
    }

    public void setWantIntakeOn() {
        mIntake.setOn();
    }

    public void reloadConstants() {
        mShooter.refreshControllerConsts();
    }
}
