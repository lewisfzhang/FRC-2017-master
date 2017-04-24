package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.drivers.CANTalonFactory;
import com.team254.lib.util.drivers.MB1043;
import com.team254.lib.util.drivers.UltrasonicSensor;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorGearGrabber extends Subsystem {

    public static boolean kWristDown = false;
    public static boolean kWristUp = !kWristDown;
    public static double kBallClearSetpoint = 8;
    public static double kScoreGearSetpoint = 12;
    public static double kIntakeGearSetpoint = -12;
    public static double kTransitionDelay = 0.5;
    public static double kExhaustDelay = 0.1;
    public static double kIntakeThreshold = 15;
    public static double kThresholdTime = 0.15;

    private static MotorGearGrabber mInstance;
    private static MB1043 mUltrasonicSensor;

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
        INTAKE,
        STOWING,
        STOWED,
        EXHAUSTING,
        EXHAUST,
    }

    private final Solenoid mWristSolenoid;
    private final CANTalon mMasterTalon;

    private WantedState mWantedState;
    private SystemState mSystemState;
    private double mThresholdStart;

    private MotorGearGrabber() {
        mWristSolenoid = Constants.makeSolenoidForId(Constants.kGearWristSolenoid);
        mMasterTalon = CANTalonFactory.createDefaultTalon(Constants.kGearGrabberId);
        mMasterTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 15);
        mMasterTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);
        mUltrasonicSensor = new MB1043(Constants.kUltrasonicSensorId);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Gear Grabber Current", mMasterTalon.getOutputCurrent());
        SmartDashboard.putNumber("Ultrasonic Distance", getLatestRawDistance());
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
                    mWantedState = WantedState.IDLE;
                }
                mCurrentStateStartTime = Timer.getFPGATimestamp();
            }

            @Override
            public void onLoop(double timestamp) {

                synchronized (MotorGearGrabber.this) {
                    SystemState newState = mSystemState;
                    double timeInState = Timer.getFPGATimestamp() - mCurrentStateStartTime;
                    switch (mSystemState) {
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
                    case EXHAUST:
                        newState = handleExhaust(timeInState);
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
                    }
                }

            }

            @Override
            public void onStop(double timestamp) {
                mWantedState = WantedState.IDLE;
                mSystemState = SystemState.STOWED;
                // Set the states to what the robot falls into when disabled.
                stop();
            }
        };
        enabledLooper.register(loop);
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
            return SystemState.STOWED;
        }
    }

    private SystemState handleIntake(double timeInState) {
        switch (mWantedState) {
        case CLEAR_BALLS:
            return SystemState.BALL_CLEARING;
        case IDLE:
            return SystemState.STOWING;
            // Fall through intended.
        default:
            setWristDown();
            mMasterTalon.set(kIntakeGearSetpoint);
            if (mMasterTalon.getOutputCurrent() > kIntakeThreshold) {
                if(timeInState - mThresholdStart > kThresholdTime) {
                    LED.getInstance().setWantedState(LED.WantedState.BLINK);
                } else {
                    if(mThresholdStart == Double.POSITIVE_INFINITY) {
                        mThresholdStart = timeInState;
                    }
                }
            } else {
                mThresholdStart = Double.POSITIVE_INFINITY;
                LED.getInstance().setWantedState(LED.WantedState.OFF);
            }
            return SystemState.INTAKE;
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
            return SystemState.STOWED;
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
        case ACQUIRE:
            return SystemState.INTAKE;
        default:
            setWristUp();
            if(timeInState < 1) {
                mMasterTalon.set(kIntakeGearSetpoint);
            } else {
                mMasterTalon.set(0.0);
            }
            return SystemState.STOWED;
        }
    }

    private SystemState handleExhausting(double timeInState) {
        setWristDown();
        if (timeInState > kExhaustDelay) {
            mMasterTalon.set(kScoreGearSetpoint);
        } else {
            mMasterTalon.set(0);
        }
        if (timeInState > kTransitionDelay) {
            return SystemState.EXHAUST;
        }
        return SystemState.EXHAUSTING;
    }

    private boolean mWristUp = false;

    public void setOpenLoop(double value) {
        mMasterTalon.set(value);
    }
    
    public double getLatestRawDistance() {
        return 0;//mUltrasonicSensor.getLatestDistanceInches();
    }
    
    public double getRawDistanceInches(){return 0;}

    public double getFilteredDistance() {
        return mUltrasonicSensor.getAverageDistance();
    }

    private void setWristUp() {
        mWristUp = true;
        mWristSolenoid.set(mWristUp);
    }

    private void setWristDown() {
        mWristUp = false;
        mWristSolenoid.set(mWristUp);
    }

    public synchronized void setWantedState(WantedState wanted) {
        mWantedState = wanted;
    }

    public synchronized void reset() {
        mWantedState = WantedState.ACQUIRE;
        mSystemState = SystemState.STOWED;
    }

    public boolean checkSystem() {
        System.out.println("Testing GearGrabber.--------------------------------");
        final double kCurrentThres = 0.5;

        mMasterTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);

        mMasterTalon.set(-6.0f);
        Timer.delay(4.0);
        final double current = mMasterTalon.getOutputCurrent();
        mMasterTalon.set(0.0);

        System.out.println("MotorGearGrabber Current: " + current);

        if (current < kCurrentThres) {
            System.out.println("!!!!!!!!!!!! MotorGear Grabber Current Low !!!!!!!!!!!");
            return false;
        }
        return true;
    }

}
