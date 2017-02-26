package com.team254.frc2017.subsystems;

import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OldGearGrabber extends Subsystem {
    private static OldGearGrabber mInstance = new OldGearGrabber();
    
    public static OldGearGrabber getInstance() {
        return mInstance;
    }
    
    public enum SystemState {
        STOWED,
        WAITING_FOR_GEAR,
        GEAR_GRABBED,
        STOWED_WITH_GEAR,
        EXTENDED_WITH_GEAR,
        EXTENDED_WITHOUT_GEAR
    }
    
    public enum WantedState {
        STOWED, GRAB_GEAR, SCORE_GEAR
    }
    
    private SystemState mSystemState = SystemState.STOWED;
    private WantedState mWantedState = WantedState.STOWED;
    
    // Declare solenoids
    private final Solenoid mGearPusher;
    private final Solenoid mAngleSolenoid;
    private final Solenoid mAssemblyPusher;
    private double kMechanicalDelay = 0.25; //seconds
    private double kScoreDelay = 2; // Delay for arm (scoring position w/ gear) to drop the gear
    private double kPickupDelay = 3; // Delay for driver to pick up gear; update with bumper switch once HW is ready
    
    private OldGearGrabber() {
        // Instantiate solenoids
        mGearPusher = Constants.makeSolenoidForId(Constants.kGearPusherSolenoidId);
        mAngleSolenoid = Constants.makeSolenoidForId(Constants.kAngleSolenoidId);
        mAssemblyPusher = Constants.makeSolenoidForId(Constants.kAssemblyPusherSolenoidId);
    }
    
    private double mCurrentStateStartTime;
    private boolean mIsSameState = true;
    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (OldGearGrabber.this) {
                mWantedState = WantedState.STOWED;
                mSystemState = SystemState.STOWED;
                handleStowed();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            SystemState newState = mSystemState;
            synchronized (OldGearGrabber.this) {
                switch (mSystemState) {
                case STOWED:
                    newState = handleStowed();
                    break;
                case WAITING_FOR_GEAR:
                    newState = handleWaitingForGear(mCurrentStateStartTime);
                    break;
                case GEAR_GRABBED:
                    newState = handleGearGrabbed(mCurrentStateStartTime);
                    break;
                case STOWED_WITH_GEAR:
                    newState = handleStowedWithGear(mCurrentStateStartTime);
                    break;
                case EXTENDED_WITH_GEAR:
                    newState = handleExtendedWithGear(mCurrentStateStartTime);
                    break;
                case EXTENDED_WITHOUT_GEAR:
                    newState = handleExtendedWithoutGear(mCurrentStateStartTime);
                    break;
                default:
                    System.out.println("Unexpected gear grabber control state: " + mWantedState);
                    break;
                }
                
                if (newState != mSystemState) {
                    System.out.println("Changed state: " + mSystemState + " -> " + newState);
                    mIsSameState = false;
                    mSystemState = newState;
                }
                else {
                    mIsSameState = true;
                }
                outputToSmartDashboard();
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };
    
    public synchronized void setWantedState(WantedState in) {
        mWantedState = in;
    }
    
    private synchronized SystemState handleStowed() {
        mGearPusher.set(false);
        mAngleSolenoid.set(false);
        mAssemblyPusher.set(false);
        
        switch (mWantedState) {
        case STOWED:
            return SystemState.STOWED;
        case GRAB_GEAR:
            return SystemState.WAITING_FOR_GEAR;
        case SCORE_GEAR:
            if (!mIsSameState)
                mCurrentStateStartTime = Timer.getFPGATimestamp();
            return SystemState.GEAR_GRABBED;
        default:
            return SystemState.STOWED;
        }
    }

    private synchronized SystemState handleWaitingForGear(double timestamp) {
        mGearPusher.set(false);
        mAngleSolenoid.set(true);
        mAssemblyPusher.set(false);
        
        switch (mWantedState) {
        case STOWED:
            return SystemState.STOWED;
        case GRAB_GEAR:
            if (!mIsSameState) {
                mCurrentStateStartTime = Timer.getFPGATimestamp();
            } else {
                if (timestamp - Timer.getFPGATimestamp() > kPickupDelay)
                    return SystemState.GEAR_GRABBED;
            }
            return SystemState.WAITING_FOR_GEAR;
        case SCORE_GEAR:
            return SystemState.STOWED; // why score when there's no gear?
        default:
            return SystemState.STOWED;
        }
    }
    
    private synchronized SystemState handleGearGrabbed(double timestamp) {
        mGearPusher.set(true);
        mAngleSolenoid.set(true);
        mAssemblyPusher.set(false);
        
        switch (mWantedState) {
        case STOWED:
            return SystemState.STOWED;
        case GRAB_GEAR:
            if (Timer.getFPGATimestamp() - timestamp > kMechanicalDelay)
                return SystemState.STOWED_WITH_GEAR;
            else
                return SystemState.GEAR_GRABBED;
        case SCORE_GEAR:
            if (Timer.getFPGATimestamp() - timestamp > kMechanicalDelay)
                return SystemState.STOWED_WITH_GEAR;
            else
                return SystemState.GEAR_GRABBED;
        default:
            return SystemState.STOWED;
        } 
    }
    
    private synchronized SystemState handleStowedWithGear(double timestamp) {
        mGearPusher.set(true);
        mAngleSolenoid.set(false);
        mAssemblyPusher.set(false);
        
        switch (mWantedState) {
        case STOWED:
            return SystemState.STOWED;
        case GRAB_GEAR:
            return SystemState.WAITING_FOR_GEAR;
        case SCORE_GEAR:
            if (Timer.getFPGATimestamp() - timestamp > kMechanicalDelay)
                return SystemState.EXTENDED_WITH_GEAR;
            else
                return SystemState.STOWED_WITH_GEAR;
        default:
            return SystemState.STOWED;
        }
    }
    
    private synchronized SystemState handleExtendedWithGear(double timestamp) {
        mGearPusher.set(true);
        mAngleSolenoid.set(false);
        mAssemblyPusher.set(true);
        
        switch (mWantedState) {
        case STOWED:
            return SystemState.STOWED;
        case GRAB_GEAR:
            return SystemState.WAITING_FOR_GEAR;
        case SCORE_GEAR:
            if (Timer.getFPGATimestamp() - timestamp > kScoreDelay)
                return SystemState.EXTENDED_WITHOUT_GEAR;
            else
                return SystemState.EXTENDED_WITH_GEAR;
        default:
            return SystemState.STOWED;
        }
    }
    
    private synchronized SystemState handleExtendedWithoutGear(double timestamp) {
        mGearPusher.set(true);
        mAngleSolenoid.set(false);
        mAssemblyPusher.set(true);
        
        switch (mWantedState) {
        case STOWED:
            return SystemState.STOWED;
        case GRAB_GEAR:
            return SystemState.WAITING_FOR_GEAR;
        case SCORE_GEAR:
            return SystemState.EXTENDED_WITHOUT_GEAR;
        default:
            return SystemState.STOWED;
        }
    }
    
    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Gear Grabber State", mSystemState.toString());
    }

    @Override
    public void stop() {
        
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }  
}