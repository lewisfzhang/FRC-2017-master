package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.ConstantsBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Feeder extends Subsystem {
    private static final double kReversing = -1.0;
    private static final double kUnjamInPeriod = .1 * kReversing;
    private static final double kUnjamOutPeriod = .2 * kReversing;
    private static final double kUnjamInPower = 6.0 * kReversing;
    private static final double kUnjamOutPower = -6.0 * kReversing;
    private static final double kFeedVoltage = 8.0;
    private static final double kExhaustVoltage = kFeedVoltage * kReversing;

    private static Feeder sInstance = null;
    public static Feeder getInstance() {
        if (sInstance == null) {
            sInstance = new Feeder();
        }
        return sInstance;
    }

    private final CANTalon mMasterTalon, mSlaveTalon;

    public Feeder() {
        mMasterTalon = new CANTalon(Constants.kFeederMasterId);

        mMasterTalon.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        mMasterTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);
        mMasterTalon.SetVelocityMeasurementWindow(64);
        mMasterTalon.SetVelocityMeasurementPeriod(CANTalon.VelocityMeasurementPeriod.Period_100Ms);

        mMasterTalon.setVoltageRampRate(0.0);
        mMasterTalon.reverseOutput(false);
        mMasterTalon.enableBrakeMode(true);

        mSlaveTalon = new CANTalon(Constants.kFeederSlaveId);
        mSlaveTalon.changeControlMode(CANTalon.TalonControlMode.Follower);
        mSlaveTalon.set(Constants.kFeederMasterId);
        mSlaveTalon.setVoltageRampRate(0.0);
        mSlaveTalon.reverseOutput(true);
        mSlaveTalon.enableBrakeMode(true);
    }

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
            synchronized (Feeder.this) {
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
                    System.out.println("Feeder state " + mSystemState + " to " + newState);
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
        setOpenLoop(0.0f);
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
        if (mStateChanged) {
            setOpenLoop(kFeedVoltage);
        }

        return defaultStateTransfer();
    }

    private SystemState handleExhaust() {
        setOpenLoop(kExhaustVoltage);
        return defaultStateTransfer();
    }

    public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }

    private void setOpenLoop(double voltage) {
        if (mStateChanged) {
            mMasterTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);
        }
        mMasterTalon.set(voltage);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("feeder_speed", mMasterTalon.getSpeed());
        SmartDashboard.putNumber("feeder_i_accum", mMasterTalon.GetIaccum());
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
