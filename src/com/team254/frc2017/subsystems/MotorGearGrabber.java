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
    public static double K_SCORE_GEAR_SETPOINT = -9;
    public static double K_INTAKE_GEAR_SETPOINT = 12;

    private static MotorGearGrabber mInstance;
    public static MotorGearGrabber getInstance() {
        if (mInstance == null) {
            mInstance = new MotorGearGrabber();
        }
        return mInstance;
    }

    public enum WantedState {
        IDLE,
        STOWED,
        FORCE_PREP_SCORE, // Mostly for auto mode
        ACQUIRE,
        SCORE,
    }

    private enum SystemState {
        STOWED_WITH_GEAR,
        STOWED_WITHOUT_GEAR,
        DOWN_ACQUIRING_GEAR,
        PLACING_POSITION,
        PLACING_POSITION_WITHOUT_GEAR,
        RELEASING_GEAR_WRIST,
        RELEASING_GEAR_MOTOR,
    }

    private final Solenoid mWristSolenoid;
    private final CANTalon mMasterTalon;

    private WantedState mWantedState;
    private SystemState mSystemState;

    private MotorGearGrabber() {
        mWristSolenoid = new Solenoid(Constants.kGearWristSolenoid);
        mMasterTalon = CANTalonFactory.createDefaultTalon(Constants.kGearGrabberId);
        mMasterTalon.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 15);
        mMasterTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);
        mMasterTalon.setCurrentLimit(20);
        mMasterTalon.EnableCurrentLimit(true);
        mSystemState = SystemState.STOWED_WITH_GEAR;
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
                        case STOWED_WITH_GEAR:
                            newState = handleStowedWithGear();
                            break;
                        case STOWED_WITHOUT_GEAR:
                            newState = handleStowedWithoutGear();
                            break;
                        case DOWN_ACQUIRING_GEAR:
                            newState = handleDownAcquiringGear();
                            break;
                        case PLACING_POSITION:
                            newState = handlePlacingPosition();
                            break;
                        case PLACING_POSITION_WITHOUT_GEAR:
                            newState = handlePlacingPositionWithoutGear();
                            break;
                        case RELEASING_GEAR_MOTOR:
                            newState = handleReleasingWithMotor(timeInState);
                            break;
                        case RELEASING_GEAR_WRIST:
                            newState = handleReleasingWrist(timeInState);
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
        };
    }

    private SystemState handleStowedWithGear() {
        mMasterTalon.set(K_CONTAIN_GEAR_SETPOINT);
        mWristSolenoid.set(K_WRIST_DOWN);
        switch (mWantedState) {
            case ACQUIRE:
                return SystemState.DOWN_ACQUIRING_GEAR;
            case FORCE_PREP_SCORE:
                return SystemState.PLACING_POSITION;
        }
        return SystemState.STOWED_WITH_GEAR;
    }

    private SystemState handleStowedWithoutGear() {
        mMasterTalon.set(0);
        mWristSolenoid.set(K_WRIST_DOWN);
        switch (mWantedState) {
            case ACQUIRE:
                return SystemState.DOWN_ACQUIRING_GEAR;
            case FORCE_PREP_SCORE: // Use placing position WITH gear since clearly this broke
                return SystemState.PLACING_POSITION;
        }
        return SystemState.STOWED_WITHOUT_GEAR;
    }

    private SystemState handleDownAcquiringGear() {
        mMasterTalon.set(K_INTAKE_GEAR_SETPOINT);
        mWristSolenoid.set(K_WRIST_DOWN);
        switch (mWantedState) {
            case STOWED:
                return SystemState.STOWED_WITHOUT_GEAR;
            case FORCE_PREP_SCORE:
                return SystemState.PLACING_POSITION;
            case IDLE:
                return SystemState.STOWED_WITHOUT_GEAR;
        }
        if (false) { // TODO: Implement "we got a gear" detector
            return SystemState.PLACING_POSITION;
        }
        return SystemState.DOWN_ACQUIRING_GEAR;
    }

    private SystemState handlePlacingPosition() {
        mMasterTalon.set(K_CONTAIN_GEAR_SETPOINT);
        mWristSolenoid.set(K_WRIST_UP);
        switch (mWantedState) {
            case STOWED:
                return SystemState.STOWED_WITH_GEAR;
            case FORCE_PREP_SCORE:
                return SystemState.PLACING_POSITION;
            case IDLE:
                return SystemState.STOWED_WITHOUT_GEAR;
        }
        if (false) { // TODO: Implement "we got a gear" detector
            return SystemState.PLACING_POSITION;
        }
        return SystemState.DOWN_ACQUIRING_GEAR;
    }

    private SystemState handlePlacingPositionWithoutGear() {
        mMasterTalon.set(0);
        mWristSolenoid.set(K_WRIST_UP);
        switch (mWantedState) {
            case STOWED:
                return SystemState.STOWED_WITH_GEAR;
            case FORCE_PREP_SCORE:
                return SystemState.PLACING_POSITION;
            case ACQUIRE:
                return SystemState.DOWN_ACQUIRING_GEAR;
        }
        return SystemState.PLACING_POSITION_WITHOUT_GEAR;
    }


    private SystemState handleReleasingWithMotor(double timeInState) {
        mMasterTalon.set(K_SCORE_GEAR_SETPOINT);
        mWristSolenoid.set(K_WRIST_DOWN);
        if (timeInState > .25) {
            return SystemState.STOWED_WITHOUT_GEAR;
        }
        return SystemState.RELEASING_GEAR_MOTOR;
    }

    private SystemState handleReleasingWrist(double timeInState) {
        mMasterTalon.set(K_CONTAIN_GEAR_SETPOINT);
        mWristSolenoid.set(K_WRIST_DOWN);
        if (timeInState > .25) {
            return SystemState.RELEASING_GEAR_MOTOR;
        }
        return SystemState.RELEASING_GEAR_WRIST;
    }

    public boolean mWristUp = false;

    public void setOpenLoop(double value) {
        mMasterTalon.set(value);
    }

    public void setWristUp() {
        mWristUp  = true;
        mWristSolenoid.set(mWristUp);
    }

    public void setWristDown() {
        mWristUp  = false;
        mWristSolenoid.set(mWristUp);
    }
    



}
