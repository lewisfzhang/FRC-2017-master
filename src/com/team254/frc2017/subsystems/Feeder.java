package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;

public class Feeder extends Subsystem {

    private static Feeder sInstance = null;
    public static Feeder getInstance() {
        if (sInstance == null) {
            sInstance = new Feeder();
        }
        return sInstance;
    }


    private final CANTalon mMasterTalon, mSlaveTalon;

    public enum States {
        NOOP,
        STOPPED,
        FEEDING,
        REVERSING,
    };

    private States mControlState = States.NOOP;

    public Feeder() {
        mMasterTalon = new CANTalon(Constants.kFeederMasterId);
        mMasterTalon.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mSlaveTalon = new CANTalon(Constants.kFeederSlaveId);
        mSlaveTalon.changeControlMode(CANTalon.TalonControlMode.Follower);
        mSlaveTalon.set(Constants.kFeederMasterId);
        mSlaveTalon.reverseOutput(true);
    }

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Feeder.this) {
                switch (mControlState) {
                    case NOOP:
                        stop();
                        return;
                    case STOPPED:
                        stop();
                        return;
                    case FEEDING:
                        setOpenLoop(-1);
                        return;
                    case REVERSING:
                        setOpenLoop(1);
                        return;
                    default:
                        System.err.println("Unexpected feeder drive control state: " + mControlState);
                        break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private void setOpenLoop(double openLoop) {
        mMasterTalon.set(openLoop);
    }

    @Override
    public void outputToSmartDashboard() {

    }

    @Override
    public void stop() {
        setOpenLoop(0);
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }


    public void setOn() {
        mControlState = States.FEEDING;
    }

    public void setOff() {
        mControlState = States.STOPPED;
    }

    public void setReverse() {
        mControlState = States.REVERSING;
    }
}
