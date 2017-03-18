package com.team254.frc2017.subsystems;

import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;

public class LED extends Subsystem {
    private static final int kDefaultBlinkCount = 4;
    private static final double kBlinkDuration = 0.2; // seconds for full cycle
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
        OFF, FIXED_ON, BLINKING, RANGE_FINDING
    }

    public enum WantedState {
        OFF, FIXED_ON, BLINK, FIND_RANGE
    }

    private SystemState mSystemState = SystemState.OFF;
    private WantedState mWantedState = WantedState.OFF;

    private boolean mIsDisabled = false;

    private boolean mIsLEDOn;
    private double mDesiredRangeHz;
    private AnalogInput mCheckLightButton;
    private DigitalOutput mLED;

    public LED() {
        mLED = new DigitalOutput(9);
        mLED.set(false);

        mCheckLightButton = new AnalogInput(2);

        // Force a relay change.
        mIsLEDOn = true;
        setLEDOff();
    }

    private Loop mLoop = new Loop() {
        private double mCurrentStateStartTime;

        @Override
        public void onStart(double timestamp) {
            synchronized (LED.this) {
                mSystemState = SystemState.OFF;
                mWantedState = WantedState.OFF;
                mLED.set(false);
            }

            mDesiredRangeHz = 0.0;

            mCurrentStateStartTime = timestamp;
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (LED.this) {

                if (mIsDisabled) {
                    if (mCheckLightButton.getAverageVoltage() < 0.15) {
                        setWantedState(WantedState.FIXED_ON);
                    } else {
                        setWantedState(WantedState.OFF);
                    }
                }

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
                    case RANGE_FINDING:
                        newState =  handleRangeFinding(timeInState);
                        break;
                    default:
                        System.out.println("Fell through on LED states!!");
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
            case FIND_RANGE:
                return SystemState.RANGE_FINDING;
            case FIXED_ON:
                return SystemState.FIXED_ON;
            default:
                return SystemState.OFF;
        }
    }

    private synchronized SystemState handleOff() {
        setLEDOff();
        setRangeLEDOff();
        return defaultStateTransfer();
    }

    private synchronized SystemState handleFixedOn() {
        setLEDOn();
        setRangeLEDOff();
        return defaultStateTransfer();
    }

    public synchronized void setmDesiredRangeHz(double desiredHz) {
        mDesiredRangeHz = desiredHz;
    }

    private synchronized SystemState handleRangeFinding(double timeInState) {
        // Set main LED on.
        setLEDOn();

        // Flash the Range LED at the given Hz.
        if (mDesiredRangeHz < 1e9)  {
            setRangeLEDOff();
        } else if (mDesiredRangeHz > 1e6) {
            setRangeLEDOn();
        } else {
            double duration = 1.0 / mDesiredRangeHz;
            final int cycleNum = (int) (timeInState / duration);
            if ((cycleNum % 2) == 0) {
                setRangeLEDOn();
            } else {
                setRangeLEDOff();
            }
        }
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

    public synchronized void setLEDOn() {
        if (!mIsLEDOn) {
            mIsLEDOn = true;
            mLED.set(true);
        }
    }

    public synchronized void setLEDOff() {
        if (mIsLEDOn) {
            mIsLEDOn = false;
            mLED.set(false);
        }
    }

    public synchronized void setIsDisabled(boolean isDisabled) {
        mIsDisabled = isDisabled;
    }

    private void setRangeLEDOn() {
    }

    private void setRangeLEDOff() {
    }
}
