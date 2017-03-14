package com.team254.frc2017.subsystems;

import com.team254.frc2017.Constants;
import com.team254.frc2017.RobotState;
import com.team254.frc2017.ShooterAimingParameters;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.drivers.RevRoboticsAirPressureSensor;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class Superstructure extends Subsystem {

    static Superstructure mInstance = null;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    private final Feeder mFeeder = Feeder.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Hopper mHopper = Hopper.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final LED mLED = LED.getInstance();
    //private final Solenoid mLeftHopperSolenoid = new Solenoid(Constants.kLeftHopperSolenoidId);
    private final Solenoid mRightHopperSolenoid = Constants.makeSolenoidForId(Constants.kRightHopperSolenoidId);
    private final Compressor mCompressor = new Compressor(0);
    private final RevRoboticsAirPressureSensor mAirPressureSensor = new RevRoboticsAirPressureSensor(3);
    private boolean mIsTeleop = false;

    // Superstructure doesn't own the drive, but needs to access it
    private final Drive mDrive = Drive.getInstance();

    // Intenal state of the system
    public enum SystemState {
        IDLE, WAITING_FOR_AIM, SHOOTING, UNJAMMING, UNJAMMING_WITH_SHOOT, JUST_FEED, EXHAUSTING, HANGING
    };

    // Desired function from user
    public enum WantedState {
        IDLE, SHOOT, UNJAM, UNJAM_SHOOT, MANUAL_FEED, EXHAUST, HANG
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    public synchronized void isTeleop(boolean teleop) {
        mIsTeleop = teleop;
    }
    
    public boolean isOnTargetToShoot() {
        return (mDrive.isOnTarget() && mDrive.isAutoAiming()) && mShooter.isOnTarget();
    }

    public boolean isOnTargetToKeepShooting() {
        return true;
    }

    public synchronized boolean isShooting() {
        return mSystemState == SystemState.SHOOTING;
    }

    private Loop mLoop = new Loop() {

        // Every time we transition states, we update the current state start
        // time and the state changed boolean (for one cycle)
        private double mCurrentStateStartTime;
        private boolean mStateChanged;

        @Override
        public void onStart(double timestamp) {
            synchronized (Superstructure.this) {
                mWantedState = WantedState.IDLE;
                mCurrentStateStartTime = timestamp;
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Superstructure.this) {
                SystemState newState = mSystemState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle(mStateChanged);
                    break;
                case WAITING_FOR_AIM:
                    newState = handleWaitingForAim();
                    break;
                case SHOOTING:
                    newState = handleShooting();
                    break;
                case UNJAMMING_WITH_SHOOT:
                    newState = handleUnjammingWithShoot();
                    break;
                case UNJAMMING:
                    newState = handleUnjamming();
                    break;
                case JUST_FEED:
                    newState = handleJustFeed();
                    break;
                case EXHAUSTING:
                    newState = handleExhaust();
                    break;
                case HANGING:
                    newState = handleHang();
                    break;
                default:
                    newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    System.out.println("Superstructure state " + mSystemState + " to " + newState + " Timestamp: " + Timer.getFPGATimestamp());
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

    private SystemState handleIdle(boolean stateChanged) {
        if (stateChanged) {
            stop();
            mLED.setWantedState(LED.WantedState.OFF);
        }
        mFeeder.setWantedState(Feeder.WantedState.IDLE);
        mHopper.setWantedState(Hopper.WantedState.IDLE);
        mCompressor.setClosedLoopControl(mIsTeleop);

        switch (mWantedState) {
        case UNJAM:
            return SystemState.UNJAMMING;
        case UNJAM_SHOOT:
            return SystemState.UNJAMMING_WITH_SHOOT;
        case SHOOT:
            return SystemState.WAITING_FOR_AIM;
        case MANUAL_FEED:
            return SystemState.JUST_FEED;
        case EXHAUST:
            return SystemState.EXHAUSTING;
        case HANG:
            return SystemState.HANGING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleWaitingForAim() {

        mCompressor.setClosedLoopControl(false);
        mFeeder.setWantedState(Feeder.WantedState.IDLE);
        mHopper.setWantedState(Hopper.WantedState.IDLE);

        if (autoSpinShooter()) {
            return SystemState.SHOOTING;
        }
        switch (mWantedState) {
        case UNJAM:
            return SystemState.UNJAMMING;
        case UNJAM_SHOOT:
            return SystemState.UNJAMMING_WITH_SHOOT;
        case SHOOT:
            return SystemState.WAITING_FOR_AIM;
        case MANUAL_FEED:
            return SystemState.JUST_FEED;
        case EXHAUST:
            return SystemState.EXHAUSTING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleShooting() {
        autoSpinShooter();
        mCompressor.setClosedLoopControl(false);
        mFeeder.setWantedState(Feeder.WantedState.FEED);
        mHopper.setWantedState(Hopper.WantedState.FEED);
        mLED.setWantedState(LED.WantedState.FIXED_ON);
        setWantIntakeOn();
        switch (mWantedState) {
        case UNJAM:
            return SystemState.UNJAMMING;
        case UNJAM_SHOOT:
            return SystemState.UNJAMMING_WITH_SHOOT;
        case SHOOT:
            if (!isOnTargetToKeepShooting()) {
                return SystemState.WAITING_FOR_AIM;
            }
            return SystemState.SHOOTING;
        case MANUAL_FEED:
            return SystemState.JUST_FEED;
        case EXHAUST:
            return SystemState.EXHAUSTING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleUnjammingWithShoot() {
        autoSpinShooter();
        mCompressor.setClosedLoopControl(false);
        mFeeder.setWantedState(Feeder.WantedState.UNJAM);
        mHopper.setWantedState(Hopper.WantedState.UNJAM);
        mLED.setWantedState(LED.WantedState.FIXED_ON);
        switch (mWantedState) {
        case UNJAM:
            return SystemState.UNJAMMING;
        case UNJAM_SHOOT:
            return SystemState.UNJAMMING_WITH_SHOOT;
        case SHOOT:
            return SystemState.WAITING_FOR_AIM;
        case EXHAUST:
            return SystemState.EXHAUSTING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleUnjamming() {
        mShooter.stop();
        mCompressor.setClosedLoopControl(false);
        mFeeder.setWantedState(Feeder.WantedState.UNJAM);
        mHopper.setWantedState(Hopper.WantedState.UNJAM);
        mLED.setWantedState(LED.WantedState.FIXED_ON);
        switch (mWantedState) {
        case UNJAM:
            return SystemState.UNJAMMING;
        case UNJAM_SHOOT:
            return SystemState.UNJAMMING_WITH_SHOOT;
        case SHOOT:
            return SystemState.WAITING_FOR_AIM;
        case EXHAUST:
            return SystemState.EXHAUSTING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleJustFeed() {
        mCompressor.setClosedLoopControl(false);
        mFeeder.setWantedState(Feeder.WantedState.FEED);
        mHopper.setWantedState(Hopper.WantedState.FEED);
        switch (mWantedState) {
        case UNJAM:
            return SystemState.UNJAMMING;
        case UNJAM_SHOOT:
            return SystemState.UNJAMMING_WITH_SHOOT;
        case SHOOT:
            return SystemState.WAITING_FOR_AIM;
        case MANUAL_FEED:
            return SystemState.JUST_FEED;
        case EXHAUST:
            return SystemState.EXHAUSTING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleExhaust() {
        mCompressor.setClosedLoopControl(false);
        mFeeder.setWantedState(Feeder.WantedState.EXHAUST);
        mHopper.setWantedState(Hopper.WantedState.EXHAUST);

        switch (mWantedState) {
        case UNJAM:
            return SystemState.UNJAMMING;
        case UNJAM_SHOOT:
            return SystemState.UNJAMMING_WITH_SHOOT;
        case SHOOT:
            return SystemState.WAITING_FOR_AIM;
        case MANUAL_FEED:
            return SystemState.JUST_FEED;
        case EXHAUST:
            return SystemState.EXHAUSTING;
        default:
            return SystemState.IDLE;
        }

    }
    
    private SystemState handleHang() {
        mCompressor.setClosedLoopControl(false);
        mFeeder.setWantedState(Feeder.WantedState.IDLE);
        mHopper.setWantedState(Hopper.WantedState.IDLE);
        mShooter.setOpenLoop(-12.0);

        switch (mWantedState) {
        case HANG:
            return SystemState.HANGING;
        default:
            return SystemState.IDLE;
        }
    }

    private double getShootingSetpointRpm(double range) {
        return Constants.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
    }

    public synchronized boolean autoSpinShooter() {
        final Optional<ShooterAimingParameters> aimOptional = RobotState.getInstance()
                .getAimingParameters(Timer.getFPGATimestamp());
        if (aimOptional.isPresent()) {
            final ShooterAimingParameters aim = aimOptional.get();
            double range = aim.getRange();
            mShooter.setClosedLoopRpm(getShootingSetpointRpm(range));

            if (range < Constants.kFlywheelAutoAimMap.firstKey().value
                    || range > Constants.kFlywheelAutoAimMap.lastKey().value) {
                // We cannot make this shot
                mLED.setWantedState(LED.WantedState.BLINK);
                return false;
            }

            mLED.setWantedState(LED.WantedState.FIXED_ON);
            // mShooter.setClosedLoopRpm(Constants.kShooterTuningRpm);

            return isOnTargetToShoot();

        } else if (Superstructure.getInstance().isShooting()) {
            // Keep the previous setpoint.
            mLED.setWantedState(LED.WantedState.BLINK);
            return false;
        } else {
            mLED.setWantedState(LED.WantedState.BLINK);
            mShooter.setClosedLoopRpm(getShootingSetpointRpm(Constants.kDefaultShootingDistanceInches));
            return false;
        }
    }

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }

    public synchronized void setShooterOpenLoop(double voltage) {
        mShooter.setOpenLoop(voltage);
    }

    public synchronized void setClosedLoopRpm(double setpointRpm) {
        mShooter.setClosedLoopRpm(setpointRpm);
    }
    
    public synchronized void setActuateHopper(boolean extended) {
        //mLeftHopperSolenoid.set(extended);
        mRightHopperSolenoid.set(extended);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Air Pressure psi", mAirPressureSensor.getAirPressurePsi());
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

    public void setWantIntakeReversed() {
        mIntake.setReverse();
    }

    public void setWantIntakeStopped() {
        mIntake.setOff();
    }

    public void setWantIntakeOn() {
        mIntake.setOn();
    }

    public void reloadConstants() {
        mShooter.refreshControllerConsts();
    }
}
