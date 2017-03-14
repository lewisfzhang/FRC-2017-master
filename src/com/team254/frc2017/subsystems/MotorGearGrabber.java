package com.team254.frc2017.subsystems;


import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.drivers.CANTalonFactory;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorGearGrabber extends Subsystem {

    public static boolean kWristDown = false;
    public static boolean kWristUp = !kWristDown;
    public static double kContainGearSetpoint = -3;
    public static double kScoreGearSetpoint = 12;
    public static double kIntakeGearSetpoint = -12;
    public static double kTransitionDelay = 0.5;
    public static double kExhaustDelay = 0.1;
    public static double kIntakeThreshold = 25;
    public static double kContainThreshold = 0.0;
    public static double kThresholdTime = 0.15;
    
    private static MotorGearGrabber mInstance;
    public static MotorGearGrabber getInstance() {
        if (mInstance == null) {
            mInstance = new MotorGearGrabber();
        }
        return mInstance;
    }

    public enum WantedState {
        IDLE,
        FORCE_LOWERED, // Mostly for auto mode
        ACQUIRE,
        SCORE,
    }

    private enum SystemState {
        IDLE,
        INTAKE,
        STOWING,
        STOWED,
        EXHAUSTING,
        LOWERING, // For auto only
        EXHAUST,
        DOWN,
    }

    private final Solenoid mWristSolenoid;
    private final CANTalon mMasterTalon;

    private WantedState mWantedState;
    private SystemState mSystemState;
    private double startTimeInThreshold;

    private MotorGearGrabber() {
        mWristSolenoid = Constants.makeSolenoidForId(Constants.kGearWristSolenoid);
        mMasterTalon = CANTalonFactory.createDefaultTalon(Constants.kGearGrabberId);
        mMasterTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 15);
        mMasterTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);

        mSystemState = SystemState.INTAKE;
        mWantedState = WantedState.ACQUIRE;
    }
    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Gear Grabber Current", mMasterTalon.getOutputCurrent());
    }

    @Override
    public synchronized void stop() {
        mSystemState = SystemState.STOWED;
        mWantedState = WantedState.IDLE;
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        Loop loop = new Loop() {
            private double mCurrentStateStartTime;

            @Override
            public void onStart(double timestamp) {
                mCurrentStateStartTime = Timer.getFPGATimestamp();
                mSystemState = SystemState.INTAKE;
                mWantedState = WantedState.ACQUIRE;
            }

            @Override
            public void onLoop(double timestamp) {

                synchronized (MotorGearGrabber.this) {
                    SystemState newState = mSystemState;
                    double timeInState = Timer.getFPGATimestamp() - mCurrentStateStartTime;
                    switch (mSystemState) {
                        case IDLE:
                            newState = handleIdle();
                            break;
                        case INTAKE:
                            newState = handleIntake(timeInState);
                            break;
                        case STOWING:
                            newState = handleStowing(timeInState);
                            break;
                        case STOWED:
                            newState = handleStowed(timeInState);
                            break;
                        case EXHAUSTING:
                            newState = handleExhausting(timeInState);
                            break;
                        case LOWERING:
                            newState = handleLowering(timeInState);
                            break;
                        case EXHAUST:
                            newState = handleExhaust(timeInState);
                            break;
                        case DOWN:
                            newState = handleLowered(timeInState);
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
                        startTimeInThreshold = 0;
                    }
                }


            }

            @Override
            public void onStop(double timestamp) {
                mWantedState = WantedState.IDLE;
                mSystemState = SystemState.IDLE;
                // Set the states to what the robot falls into when disabled.
                stop();
            }
        };
        enabledLooper.register(loop);
    }
    
    private SystemState handleIdle() {
        switch(mWantedState) {
            case ACQUIRE:
                return SystemState.INTAKE;
            default:
                setWristUp();
                mMasterTalon.set(0);
                return SystemState.IDLE;
        }
    }
    
    private SystemState handleIntake(double timeInState) {
        switch(mWantedState) {
            case IDLE:
                if(mMasterTalon.getOutputCurrent() < kIntakeThreshold) {
                    return SystemState.IDLE;
                }
            default:
                setWristDown();
                mMasterTalon.set(kIntakeGearSetpoint);
                if(mMasterTalon.getOutputCurrent() > kIntakeThreshold) {
                    if(startTimeInThreshold == 0.0) {
                        startTimeInThreshold = timeInState;
                    }
                } else {
                    startTimeInThreshold = 0.0;
                }
                if(timeInState - startTimeInThreshold > kThresholdTime && startTimeInThreshold != 0) {
                    LED.getInstance().setWantedState(LED.WantedState.BLINK);
                    return SystemState.STOWING;
                } else {
                    return SystemState.INTAKE;
                }
        }
    }
    
    private SystemState handleExhaust(double timeInState) {
        switch(mWantedState) {
            case SCORE:
                setWristDown();
                mMasterTalon.set(kScoreGearSetpoint);
                return SystemState.EXHAUST;
            case ACQUIRE:
                return SystemState.INTAKE;
            default:
                return SystemState.IDLE;
        }
    }
    
    public SystemState handleStowing(double timeInState) {
        setWristUp();
        mMasterTalon.set(kIntakeGearSetpoint);
        if(timeInState > kTransitionDelay) {
            return SystemState.STOWED;
        }
        return SystemState.STOWING;
    }
    
    private SystemState handleStowed(double timeInState) {
        switch(mWantedState) {
            case SCORE:
                return SystemState.EXHAUSTING;
            case FORCE_LOWERED:
                return SystemState.LOWERING;
            default:
//                if(mMasterTalon.getOutputCurrent() <= K_CONTAIN_THRESH) {
//                    if(startTimeInThreshold == 0.0) {
//                        startTimeInThreshold = timeInState;
//                    }
//                } else {
//                    startTimeInThreshold = 0.0;
//                }
//                if(timeInState - startTimeInThreshold > K_THRESH_TIME*2 && startTimeInThreshold != 0) {
//                    LED.getInstance().setWantedState(LED.WantedState.BLINK);
//                    return SystemState.IDLE;
//                }
                setWristUp();
                mMasterTalon.set(kContainGearSetpoint);
                return SystemState.STOWED;
        }
    }
    
    private SystemState handleExhausting(double timeInState) {
        setWristDown();
        if(timeInState > kExhaustDelay) {
            mMasterTalon.set(kScoreGearSetpoint);
        } else {
            mMasterTalon.set(kContainGearSetpoint);
        }
        if(timeInState > kTransitionDelay) {
            return SystemState.EXHAUST;
        }
        return SystemState.EXHAUSTING;
    }
    
    private SystemState handleLowering(double timeInState) {
        setWristDown();
        mMasterTalon.set(kIntakeGearSetpoint);
        if(timeInState > kTransitionDelay) {
            return SystemState.DOWN;
        }
        return SystemState.LOWERING;
    }
    
    private SystemState handleLowered(double timeInState) {
        switch(mWantedState) {
            case FORCE_LOWERED:
                setWristDown();
                mMasterTalon.set(kContainGearSetpoint);
                return SystemState.DOWN;
            default:
                return SystemState.IDLE;
        }
    }


    public boolean mWristUp = false;

    public void setOpenLoop(double value) {
        mMasterTalon.set(value);
    }

    public void setWristUp() {
        if(!mWristUp) {
            mWristUp  = true;
            mWristSolenoid.set(mWristUp);
        }
    }

    public void setWristDown() {
        if(mWristUp) {
            mWristUp  = false;
            mWristSolenoid.set(mWristUp);
        }
    }
    
    public void setWantedState(WantedState wanted) {
        mWantedState = wanted;
    }

    public synchronized void reset() {
        mWantedState = WantedState.ACQUIRE;
        mSystemState = SystemState.STOWED;
    }

}
