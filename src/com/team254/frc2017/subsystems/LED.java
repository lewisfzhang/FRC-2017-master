package com.team254.frc2017.subsystems;

import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import edu.wpi.first.wpilibj.Relay;

public class LED extends Subsystem {
    private static final int kDefaultBlinkCount = 3;
    private static final double kBlinkDuration = 0.1; // seconds for full cycle
    private static final double kTotalBlinkDuration = kDefaultBlinkCount * kBlinkDuration;

    private static LED mInstance = null;

    public static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }
        return mInstance;
    }

    // Internal state of the system
    public enum SystemState {
        OFF, FIXED_ON, BLINKING
    }

    public enum WantedState {
        OFF, FIXED_ON, BLINK
    }

    private SystemState mSystemState = SystemState.OFF;
    private WantedState mWantedState = WantedState.OFF;

    private boolean mIsLEDOn;
    private Relay mLEDRelay;

    public LED() {
        mLEDRelay = new Relay(Constants.kLEDRelayId);

        // Force a relay change.
        mIsLEDOn = true;
        setLEDOff();
    }

    private Loop mLoop = new Loop() {
        private double mCurrentStateStartTime;

        @Override
        public void onStart(double timestamp) {
            mSystemState = SystemState.OFF;
            mWantedState = WantedState.OFF;

            mCurrentStateStartTime = timestamp;
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (LED.this) {
                SystemState newState;
                double timeInState = timestamp - mCurrentStateStartTime;
                switch (mSystemState) {
                    case OFF:
                        newState = handleOff();
                        break;
                    case BLINKING:
                        newState = handleBlinking(timeInState);
                        break;
                    case FIXED_ON:
                        newState = handleFixedOn();
                        break;
                    default:
                        System.out.println("Fell through on LED states!!1");
                        newState = SystemState.OFF;
                }
                if (newState != mSystemState) {
                    System.out.println("LED state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            setLEDOff();
        }
    };

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case OFF:
                return SystemState.OFF;
            case BLINK:
                return SystemState.BLINKING;
            case FIXED_ON:
                return SystemState.FIXED_ON;
            default:
                return SystemState.OFF;
        }
    }

    private synchronized SystemState handleOff() {
        setLEDOff();
        return defaultStateTransfer();
    }

    private synchronized SystemState handleFixedOn() {
        setLEDOn();
        return defaultStateTransfer();
    }

    private synchronized SystemState handleBlinking(double timeInState) {
        if (timeInState > kTotalBlinkDuration) {
            setLEDOff();
            // Transition to OFF state and clear wanted state.
            mWantedState = WantedState.OFF;
            return SystemState.OFF;
        }

        int cycleNum = (int)(timeInState / (kBlinkDuration / 2.0));
        if ((cycleNum % 2) == 0) {
            setLEDOn();
        } else {
            setLEDOff();
        }
        return SystemState.BLINKING;
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

    public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }

    private void setLEDOn() {
        if (!mIsLEDOn) {
            mIsLEDOn = true;
            mLEDRelay.set(Relay.Value.kForward);
        }
    }

    private void setLEDOff() {
        if (mIsLEDOn) {
            mIsLEDOn = false;
            mLEDRelay.set(Relay.Value.kOff);
        }
    }
}
