package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.drivers.LazyCANTalon;

public class Hopper extends Subsystem {
    private static final double kUnjamInPeriod = .1;
    private static final double kUnjamOutPeriod = .2;
    private static final double kUnjamInPower = .5;
    private static final double kUnjamOutPower = -.5;
    private static final double kFeedPower = 1.0;

    private static Hopper sInstance = null;
    public static Hopper getInstance() {
        if (sInstance == null) {
            sInstance = new Hopper();
        }
        return sInstance;
    }

    private CANTalon mMasterTalon, mSlaveTalon;

    public enum SystemState {
        FEEDING,
        UNJAMMING_IN,
        UNJAMMING_OUT,
        IDLE,
        EXHAUSTING,
        OPEN_LOOP_OVERRIDE,
    }

    public enum WantedState {
        IDLE,
        UNJAM,
        EXHAUST,
        FEED,
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;
    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            mSystemState = SystemState.IDLE;
            mCurrentStateStartTime = timestamp;
            mStateChanged = true;
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Hopper.this) {
                SystemState newState;
                switch (mSystemState) {
                    case IDLE:
                        newState = handleIdle();
                        break;
                    case UNJAMMING_OUT:
                        newState = handleUnjammingOut(timestamp, mCurrentStateStartTime);
                        break;
                    case UNJAMMING_IN:
                        newState = handleUnjammingIn(timestamp, mCurrentStateStartTime);
                        break;
                    case FEEDING:
                        newState = handleFeeding();
                        break;
                    case EXHAUSTING:
                        newState = handleExhaust();
                        break;
                    default:
                        newState = SystemState.IDLE;
                }
                if (newState != mSystemState) {
                    System.out.println("Hopper state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case FEED:
                return SystemState.FEEDING;
            case UNJAM:
                return SystemState.UNJAMMING_OUT;
            case EXHAUST:
                return SystemState.EXHAUSTING;
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handleIdle() {
        setOpenLoop(0);
        return defaultStateTransfer();
    }


    private SystemState handleUnjammingOut(double now, double startStartedAt) {
        setOpenLoop(kUnjamOutPower);
        SystemState newState = SystemState.UNJAMMING_OUT;
        if (now - startStartedAt > kUnjamOutPeriod) {
            newState = SystemState.UNJAMMING_IN;
        }
        switch (mWantedState) {
            case FEED:
                return SystemState.FEEDING;
            case UNJAM:
                return newState;
            case EXHAUST:
                return SystemState.EXHAUSTING;
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handleUnjammingIn(double now, double startStartedAt) {
        setOpenLoop(kUnjamInPower);
        SystemState newState = SystemState.UNJAMMING_IN;
        if (now - startStartedAt > kUnjamInPeriod) {
            newState = SystemState.UNJAMMING_OUT;
        }
        switch (mWantedState) {
            case FEED:
                return SystemState.FEEDING;
            case UNJAM:
                return newState;
            case EXHAUST:
                return SystemState.EXHAUSTING;
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handleFeeding() {
        setOpenLoop(kFeedPower);
        switch (mWantedState) {
            case FEED:
                return SystemState.FEEDING;
            case UNJAM:
                return SystemState.UNJAMMING_OUT;
            case EXHAUST:
                return SystemState.EXHAUSTING;
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handleExhaust() {
        setOpenLoop(-kFeedPower);
        switch (mWantedState) {
            case FEED:
                return SystemState.FEEDING;
            case UNJAM:
                return SystemState.UNJAMMING_OUT;
            case EXHAUST:
                return SystemState.EXHAUSTING;
            default:
                return SystemState.IDLE;
        }
    }

    private Hopper() {
        mMasterTalon = new LazyCANTalon(Constants.kHopperMasterId);
        mMasterTalon.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mMasterTalon.setVoltageRampRate(Constants.kHopperRampRate);

        mSlaveTalon = new LazyCANTalon(Constants.kHopperSlaveId);
        mSlaveTalon.changeControlMode(CANTalon.TalonControlMode.Follower);
        mSlaveTalon.set(Constants.kHopperMasterId);
        mSlaveTalon.reverseOutput(true);
    }

    public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }

    @Override
    public void outputToSmartDashboard() {

    }

    private void setOpenLoop(double openLoop) {
        mMasterTalon.set(openLoop);
    }

    @Override
    public void stop() {
        setWantedState(WantedState.IDLE);
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }
}
