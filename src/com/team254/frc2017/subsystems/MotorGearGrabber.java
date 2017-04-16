package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.drivers.CANTalonFactory;
import com.team254.lib.util.drivers.UltrasonicSensor;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorGearGrabber extends Subsystem {

    public static boolean kWristDown = false;
    public static boolean kWristUp = !kWristDown;
    public static double kContainGearSetpoint = -3;
    public static double kBallClearSetpoint = 8;
    public static double kScoreGearSetpoint = 12;
    public static double kIntakeGearSetpoint = -12;
    public static double kTransitionDelay = 0.5;
    public static double kExhaustDelay = 0.1;
    public static double kIntakeThreshold = 15;
    public static double kContainThreshold = 0.0;
    public static double kThresholdTime = 0.15;

    private static MotorGearGrabber mInstance;
    private static UltrasonicSensor mUltrasonicSensor;

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
        CLEAR_BALLS
    }

    private enum SystemState {
        BALL_CLEARING,
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
        mUltrasonicSensor = new UltrasonicSensor(Constants.kUltrasonicSensorId);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Gear Grabber Current", mMasterTalon.getOutputCurrent());
        SmartDashboard.putNumber("Ultrasonic Distance", mUltrasonicSensor.getLatestDistance());
        System.out.println(mMasterTalon.getOutputCurrent());
    }

    @Override
    public void stop() {
        setWantedState(WantedState.IDLE);
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
                synchronized (MotorGearGrabber.this) {
                    mSystemState = SystemState.STOWING;
                    mWantedState = WantedState.ACQUIRE;
                }
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
                    case BALL_CLEARING:
                        newState = handleBallClearing();
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
                    
                    mUltrasonicSensor.update();

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
        switch (mWantedState) {
        case ACQUIRE:
            return SystemState.INTAKE;
        case CLEAR_BALLS:
            return SystemState.BALL_CLEARING;
        default:
            setWristUp();
            mMasterTalon.set(0);
            return SystemState.IDLE;
        }
    }

    private SystemState handleBallClearing() {
        setWristDown();
        mMasterTalon.set(kBallClearSetpoint);

        switch (mWantedState) {
        case ACQUIRE:
            return SystemState.INTAKE;
        case CLEAR_BALLS:
            return SystemState.BALL_CLEARING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleIntake(double timeInState) {
        switch (mWantedState) {
        case CLEAR_BALLS:
            return SystemState.BALL_CLEARING;
        case IDLE:
            if (mMasterTalon.getOutputCurrent() < kIntakeThreshold) {
                return SystemState.IDLE;
            }
            // Fall through intended.
        default:
            setWristDown();
            mMasterTalon.set(kIntakeGearSetpoint);
            if (mMasterTalon.getOutputCurrent() > kIntakeThreshold) {
                if (startTimeInThreshold == 0.0) {
                    startTimeInThreshold = timeInState;
                }
            } else {
                startTimeInThreshold = 0.0;
            }
            if (timeInState - startTimeInThreshold > kThresholdTime && startTimeInThreshold != 0) {
                LED.getInstance().setWantedState(LED.WantedState.BLINK);
                if (mWantedState == WantedState.IDLE) {
                    return SystemState.STOWING;
                } else {
                    return SystemState.INTAKE;
                }
            } else {
                return SystemState.INTAKE;
            }
        }
    }

    private SystemState handleExhaust(double timeInState) {
        setWristDown();
        mMasterTalon.set(kScoreGearSetpoint);

        switch (mWantedState) {
        case SCORE:
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
        if (timeInState > kTransitionDelay) {
            return SystemState.STOWED;
        }
        return SystemState.STOWING;
    }

    private SystemState handleStowed(double timeInState) {
        switch (mWantedState) {
        case SCORE:
            return SystemState.EXHAUSTING;
        case FORCE_LOWERED:
            return SystemState.LOWERING;
        default:
            // if(mMasterTalon.getOutputCurrent() <= K_CONTAIN_THRESH) {
            // if(startTimeInThreshold == 0.0) {
            // startTimeInThreshold = timeInState;
            // }
            // } else {
            // startTimeInThreshold = 0.0;
            // }
            // if(timeInState - startTimeInThreshold > K_THRESH_TIME*2 && startTimeInThreshold != 0) {
            // LED.getInstance().setWantedState(LED.WantedState.BLINK);
            // return SystemState.IDLE;
            // }
            setWristUp();
            mMasterTalon.set(kContainGearSetpoint);
            return SystemState.STOWED;
        }
    }

    private SystemState handleExhausting(double timeInState) {
        setWristDown();
        if (timeInState > kExhaustDelay) {
            mMasterTalon.set(kScoreGearSetpoint);
        } else {
            mMasterTalon.set(kContainGearSetpoint);
        }
        if (timeInState > kTransitionDelay) {
            return SystemState.EXHAUST;
        }
        return SystemState.EXHAUSTING;
    }

    private SystemState handleLowering(double timeInState) {
        setWristDown();
        mMasterTalon.set(kIntakeGearSetpoint);
        if (timeInState > kTransitionDelay) {
            return SystemState.DOWN;
        }
        return SystemState.LOWERING;
    }

    private SystemState handleLowered(double timeInState) {
        switch (mWantedState) {
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
    
    public double getDistance() {
        return mUltrasonicSensor.getLatestDistance();
    }

    public void setWristUp() {
        if (!mWristUp) {
            mWristUp = true;
            mWristSolenoid.set(mWristUp);
        }
    }

    public void setWristDown() {
        if (mWristUp) {
            mWristUp = false;
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
