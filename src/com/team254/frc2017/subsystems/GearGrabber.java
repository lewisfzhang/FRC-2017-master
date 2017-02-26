package com.team254.frc2017.subsystems;

import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class GearGrabber extends Subsystem {

    private static GearGrabber mInstance;
    public static GearGrabber getInstance() {
        if (mInstance == null) {
            mInstance = new GearGrabber();
        }
        return mInstance;
    }

    public enum WantedState {
        STOWED,
        DOWN_FOR_GEAR,
        PLACE,
        SCORE,
    }

    private enum SystemState {
        STOWED_WITH_GEAR,
        STOWED_WITHOUT_GEAR,

        PLACING_POSITION,
        RELEASING_GEAR,

        DOWN_FOR_GEAR,
        DOWN_GRABBING_GEAR,
    }

    private final Solenoid mGearPusher;
    private final Solenoid mAngleSolenoid;
    private final Solenoid mAssemblyPusher;

    private WantedState mWantedState;
    private SystemState mSystemState;

    private GearGrabber() {
        mGearPusher = Constants.makeSolenoidForId(Constants.kGearPusherSolenoidId);
        mAngleSolenoid = Constants.makeSolenoidForId(Constants.kAngleSolenoidId);
        mAssemblyPusher = Constants.makeSolenoidForId(Constants.kAssemblyPusherSolenoidId);

        mWantedState = WantedState.STOWED;
        mSystemState = SystemState.STOWED_WITH_GEAR;
    }

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
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
        enabledLooper.register(new Loop() {
            private double mCurrentStateStartTime;

            @Override
            public void onStart(double timestamp) {
                mCurrentStateStartTime = Timer.getFPGATimestamp();
            }

            @Override
            public void onLoop(double timestamp) {

                synchronized (GearGrabber.this) {
                    final SystemState newState;
                    double timeInState = Timer.getFPGATimestamp() - mCurrentStateStartTime;
                    switch (mSystemState) {
                        case STOWED_WITH_GEAR:
                            newState = handleStowedWithGear();
                            break;
                        case STOWED_WITHOUT_GEAR:
                            newState = handleStowedWithoutGear();
                            break;
                        case PLACING_POSITION:
                            newState = handlePlacingPosition();
                            break;
                        case RELEASING_GEAR:
                            newState = handleReleasingGear(timeInState);
                            break;
                        case DOWN_FOR_GEAR:
                            newState = handleDownForGear();
                            break;
                        case DOWN_GRABBING_GEAR:
                            newState = handleDownGrabbingGear(timeInState);
                            break;
                        default:
                            System.out.println("Unexpected gear grabber system state: " + mSystemState);
                            newState = mSystemState;
                            break;
                    }
                    if (newState != mSystemState) {
                        System.out.println("Changed state: " + mSystemState + " -> " + newState);
                        mSystemState = newState;
                        mCurrentStateStartTime = Timer.getFPGATimestamp();
                    }
                }


            }

            @Override
            public void onStop(double timestamp) {
                // Set the states to what the robot falls into when disabled.
                mWantedState = WantedState.STOWED;
                mSystemState = SystemState.STOWED_WITH_GEAR;
            }
        });
    }



    private SystemState handleStowedWithGear() {
        setExtended(false);
        setGrabberDown(false);
        setReleaseGrasp(false);

        switch (mWantedState) {
            case STOWED:
                return SystemState.STOWED_WITH_GEAR;
            case DOWN_FOR_GEAR:
                return SystemState.DOWN_FOR_GEAR;
            case PLACE:
                return SystemState.PLACING_POSITION;
            case SCORE:
                return SystemState.STOWED_WITHOUT_GEAR;
            default:
                System.out.println("Unexpected gear grabber wanted state: " + mWantedState);
                return mSystemState;
        }
    }

    private SystemState handleStowedWithoutGear() {
        setExtended(false);
        setGrabberDown(false);
        setReleaseGrasp(true);

        switch (mWantedState) {
            case STOWED:
                return SystemState.STOWED_WITHOUT_GEAR;
            case DOWN_FOR_GEAR:
                return SystemState.DOWN_FOR_GEAR;
            case PLACE:
                return SystemState.STOWED_WITHOUT_GEAR;
            case SCORE:
                return SystemState.STOWED_WITHOUT_GEAR;
            default:
                System.out.println("Unexpected gear grabber wanted state: " + mWantedState);
                return mSystemState;
        }
    }

    private SystemState handlePlacingPosition() {
        setExtended(true);
        setGrabberDown(false);
        setReleaseGrasp(false);

        switch (mWantedState) {
            case STOWED:
                return SystemState.STOWED_WITH_GEAR;
            case DOWN_FOR_GEAR:
                return SystemState.DOWN_FOR_GEAR;
            case PLACE:
                return SystemState.PLACING_POSITION;
            case SCORE:
                return SystemState.RELEASING_GEAR;
            default:
                System.out.println("Unexpected gear grabber wanted state: " + mWantedState);
                return mSystemState;

        }
    }

    private SystemState handleReleasingGear(double timeInState) {
        setExtended(true);
        setGrabberDown(false);
        setReleaseGrasp(true);

        if (timeInState < Constants.kGrabberPlaceTimeSeconds) {
            return SystemState.RELEASING_GEAR;
        } else {
            return SystemState.STOWED_WITHOUT_GEAR;
        }
    }

    private SystemState handleDownForGear() {
        setExtended(false);
        setGrabberDown(true);
        setReleaseGrasp(true);

        switch (mWantedState) {
            case STOWED:
                return SystemState.DOWN_GRABBING_GEAR;
            case DOWN_FOR_GEAR:
                return SystemState.DOWN_FOR_GEAR;
            case PLACE:
                return SystemState.DOWN_GRABBING_GEAR;
            case SCORE:
                return SystemState.DOWN_GRABBING_GEAR;
            default:
                System.out.println("Unexpected gear grabber wanted state: " + mWantedState);
                return mSystemState;
        }
    }

    private SystemState handleDownGrabbingGear(double timeInState) {
        setExtended(false);
        setGrabberDown(true);
        setReleaseGrasp(false);

        if (timeInState < Constants.kGrabberGrabTimeSeconds) {
            return SystemState.DOWN_GRABBING_GEAR;
        } else {
            return SystemState.STOWED_WITH_GEAR;
        }
    }

    private void setExtended(boolean isExtended) {
        mAssemblyPusher.set(isExtended);
    }

    private void setGrabberDown(boolean isDown) {
        mAngleSolenoid.set(isDown);
    }

    private void setReleaseGrasp(boolean isReleased) {
        mGearPusher.set(isReleased);
    }
}
