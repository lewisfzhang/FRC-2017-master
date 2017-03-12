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

    public static boolean K_WRIST_DOWN = false;
    public static boolean K_WRIST_UP = !K_WRIST_DOWN;
    public static double K_CONTAIN_GEAR_SETPOINT = -3;
    public static double K_SCORE_GEAR_SETPOINT = 12;
    public static double K_INTAKE_GEAR_SETPOINT = -12;
    public static double K_DELAY = 0.5;
    public static double K_EXHAUST_DELAY = 0.1;
    public static double K_INTAKE_THRESH = 25;
    public static double K_CONTAIN_THRESH = 0.0;
    public static double K_THRESH_TIME = 0.15;
    
    private static MotorGearGrabber mInstance;
    public static MotorGearGrabber getInstance() {
        if (mInstance == null) {
            mInstance = new MotorGearGrabber();
        }
        return mInstance;
    }

    public enum WantedState {
        IDLE,
        FORCE_PREP_SCORE, // Mostly for auto mode
        ACQUIRE,
        SCORE,
    }

    private enum SystemState {
        IDLE,
        INTAKE,
        STOWING,
        LOWERING,
        RAISING,
        STOWED,
        EXHAUSTING,
        EXHAUST,
    }

    private final Solenoid mWristSolenoid;
    private final CANTalon mMasterTalon;

    private WantedState mWantedState;
    private SystemState mSystemState;
    private double startTimeInThreshold;

    private MotorGearGrabber() {
        mWristSolenoid = new Solenoid(Constants.kGearWristSolenoid);
        mMasterTalon = CANTalonFactory.createDefaultTalon(Constants.kGearGrabberId);
        mMasterTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 15);
        mMasterTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);
        //mMasterTalon.setCurrentLimit(20);
        mMasterTalon.EnableCurrentLimit(true);
        mSystemState = SystemState.IDLE;
        mWantedState = WantedState.IDLE;
    }
    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Gear Grabber Current", mMasterTalon.getOutputCurrent());
    }

    @Override
    public void stop() {
        mMasterTalon.set(0);
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
                        case LOWERING:
                            newState = handleLowering(timeInState);
                            break;
                        case RAISING:
                            newState = handleRaising(timeInState);
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
                        case EXHAUST:
                            newState = handleExhaust(timeInState);
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
                // Set the states to what the robot falls into when disabled.
            }
        };
        enabledLooper.register(loop);
    }
    
    private SystemState handleIdle() {
        switch(mWantedState) {
            case ACQUIRE:
                return SystemState.LOWERING;
            default:
                setWristUp();
                mMasterTalon.set(0);
                return SystemState.IDLE;
        }
    }
    
    private SystemState handleIntake(double timeInState) {
        switch(mWantedState) {
            case IDLE:
                return SystemState.RAISING;
            default:
                setWristDown();
                mMasterTalon.set(K_INTAKE_GEAR_SETPOINT);
                if(mMasterTalon.getOutputCurrent() > K_INTAKE_THRESH) {
                    if(startTimeInThreshold == 0.0) {
                        startTimeInThreshold = timeInState;
                    }
                } else {
                    startTimeInThreshold = 0.0;
                }
                if(timeInState - startTimeInThreshold > K_THRESH_TIME && startTimeInThreshold != 0) {
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
                mMasterTalon.set(K_SCORE_GEAR_SETPOINT);
                return SystemState.EXHAUST;
            case ACQUIRE:
                return SystemState.INTAKE;
            default:
                return SystemState.RAISING;
        }
    }
    
    public SystemState handleStowing(double timeInState) {
        setWristUp();
        mMasterTalon.set(K_CONTAIN_GEAR_SETPOINT);
        if(timeInState > K_DELAY) {
            return SystemState.STOWED;
        }
        return SystemState.STOWING;
    }
    
    public SystemState handleLowering(double timeInState) {
        setWristDown();
        mMasterTalon.set(K_INTAKE_GEAR_SETPOINT);
        if(timeInState > K_DELAY) {
            return SystemState.INTAKE;
        }
        return SystemState.LOWERING;
    }
    
    public SystemState handleRaising(double timeInState) {
        setWristUp();
        mMasterTalon.set(0);
        if(timeInState > K_DELAY) {
            return SystemState.IDLE;
        }
        return SystemState.RAISING;
    }
    
    private SystemState handleStowed(double timeInState) {
        switch(mWantedState) {
            case SCORE:
                return SystemState.EXHAUSTING;
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
                mMasterTalon.set(K_CONTAIN_GEAR_SETPOINT);
                return SystemState.STOWED;
        }
    }
    
    private SystemState handleExhausting(double timeInState) {
        setWristDown();
        if(timeInState > K_EXHAUST_DELAY) {
            mMasterTalon.set(K_SCORE_GEAR_SETPOINT);
        } else {
            mMasterTalon.set(K_CONTAIN_GEAR_SETPOINT);
        }
        if(timeInState > K_DELAY) {
            return SystemState.EXHAUST;
        }
        return SystemState.EXHAUSTING;
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


}
