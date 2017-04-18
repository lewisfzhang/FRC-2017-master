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
    private final Solenoid mHopperSolenoid = Constants.makeSolenoidForId(Constants.kHopperSolenoidId);
    private final Compressor mCompressor = new Compressor(0);
    private final RevRoboticsAirPressureSensor mAirPressureSensor = new RevRoboticsAirPressureSensor(3);

    // Superstructure doesn't own the drive, but needs to access it
    private final Drive mDrive = Drive.getInstance();

    // Intenal state of the system
    public enum SystemState {
        IDLE,
        WAITING_FOR_AIM,
        SHOOTING,
        UNJAMMING,
        UNJAMMING_WITH_SHOOT,
        JUST_FEED,
        EXHAUSTING,
        HANGING,
        RANGE_FINDING
    };

    // Desired function from user
    public enum WantedState {
        IDLE, SHOOT, UNJAM, UNJAM_SHOOT, MANUAL_FEED, EXHAUST, HANG, RANGE_FINDING
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    private double mCurrentTuningRpm = Constants.kShooterTuningRpmFloor;
    private double mLastGoalRange = 0.0;

    private boolean mCompressorOverride = false;

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
                case RANGE_FINDING:
                    newState = handleRangeFinding();
                    break;
                default:
                    newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    System.out.println("Superstructure state " + mSystemState + " to " + newState + " Timestamp: "
                            + Timer.getFPGATimestamp());
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

    private SystemState handleRangeFinding() {
        autoSpinShooter(false);
        mLED.setWantedState(LED.WantedState.FIND_RANGE);
        mFeeder.setWantedState(Feeder.WantedState.FEED);

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
        case RANGE_FINDING:
            return SystemState.RANGE_FINDING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleIdle(boolean stateChanged) {
        if (stateChanged) {
            stop();
            mLED.setWantedState(LED.WantedState.OFF);
        }
        mFeeder.setWantedState(Feeder.WantedState.IDLE);
        mHopper.setWantedState(Hopper.WantedState.IDLE);
        mCompressor.setClosedLoopControl(!mCompressorOverride);

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
        case RANGE_FINDING:
            return SystemState.RANGE_FINDING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleWaitingForAim() {

        mCompressor.setClosedLoopControl(false);
        mFeeder.setWantedState(Feeder.WantedState.FEED);
        mHopper.setWantedState(Hopper.WantedState.IDLE);

        if (autoSpinShooter(true)) {

            System.out.println(Timer.getFPGATimestamp() + ": making shot: Range: " + mLastGoalRange + " setpoint: "
                    + mShooter.getLastSetpointRpm());

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
        // Don't auto spin anymore - just hold the last setpoint
        mCompressor.setClosedLoopControl(false);
        mFeeder.setWantedState(Feeder.WantedState.FEED);
        mHopper.setWantedState(Hopper.WantedState.FEED);
        mLED.setWantedState(LED.WantedState.FIND_RANGE);
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
        case RANGE_FINDING:
            return SystemState.RANGE_FINDING;
        case MANUAL_FEED:
            return SystemState.JUST_FEED;
        case EXHAUST:
            return SystemState.EXHAUSTING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleUnjammingWithShoot() {
        // Don't auto spin anymore - just hold the last setpoint
        mCompressor.setClosedLoopControl(false);
        mFeeder.setWantedState(Feeder.WantedState.FEED);
        mHopper.setWantedState(Hopper.WantedState.EXHAUST);
        mLED.setWantedState(LED.WantedState.FIND_RANGE);
        setWantIntakeOn();
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
        mLED.setWantedState(LED.WantedState.FIND_RANGE);
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
        if (Constants.kUseFlywheelAutoAimPolynomial) {
            return Constants.kFlywheelAutoAimPolynomial.predict(range);
        } else {
            return Constants.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    public synchronized boolean autoSpinShooter(boolean allow_shooting) {
        final double timestamp = Timer.getFPGATimestamp();
        final Optional<ShooterAimingParameters> aimOptional = RobotState.getInstance()
                .getAimingParameters(timestamp);
        mLED.setWantedState(LED.WantedState.FIND_RANGE);
        if (aimOptional.isPresent()) {
            final ShooterAimingParameters aim = aimOptional.get();
            double range = aim.getRange();
            final boolean range_valid = Constants.kIsShooterTuning || (range >= Constants.kShooterAbsoluteRangeFloor
                    && range <= Constants.kShooterAbsoluteRangeCeiling);
            if (!range_valid) {
                range = Math.max(Constants.kShooterAbsoluteRangeFloor,
                        Math.min(Constants.kShooterAbsoluteRangeCeiling, range));
            }
            if (!Constants.kIsShooterTuning) {

                mLastGoalRange = range;
                final double setpoint = getShootingSetpointRpm(range);
                if (allow_shooting) {
                    mShooter.setHoldWhenReady(setpoint);
                } else {
                    mShooter.setSpinUp(setpoint);
                }

                boolean is_optimal_range = false;
                if (range < Constants.kShooterOptimalRangeFloor) {
                    mLED.setRangeBlicking(true);
                } else if (range > Constants.kShooterOptimalRangeCeiling) {
                    mLED.setRangeBlicking(true);
                } else {
                    mLED.setRangeBlicking(false);
                    mLED.setRangeLEDOn();

                    is_optimal_range = true;
                }

                SmartDashboard.putBoolean("optimal range", is_optimal_range);
            } else {
                // We are shooter tuning find current RPM we are tuning for.
                mShooter.setHoldWhenReady(mCurrentTuningRpm);
                mLastGoalRange = aimOptional.get().getRange();
            }

            return range_valid && isOnTargetToShoot()
                    && (timestamp - aim.getLastSeenTimestamp()) < Constants.kMaxGoalTrackAge;
        } else if (Superstructure.getInstance().isShooting()) {
            mLED.setRangeBlicking(true);
            // Keep the previous setpoint.
            return false;
        } else {
            mLED.setRangeBlicking(true);
            mShooter.setSpinUp(getShootingSetpointRpm(Constants.kDefaultShootingDistanceInches));
            return false;

            // We are shooter tuning fine current RPM we are tuning for.
            /*
             * mShooter.setClosedLoopRpm(mCurrentTuningRpm); return isOnTargetToShoot();
             */
        }
    }

    public synchronized void incrementTuningRpm() {
        if (mCurrentTuningRpm <= Constants.kShooterTuningRpmCeiling) {
            mCurrentTuningRpm += Constants.kShooterTuningRpmStep;
            System.out.println("Changing RPM to: " + mCurrentTuningRpm);
        }
    }

    public synchronized double getCurrentTuningRpm() {
        return mCurrentTuningRpm;
    }

    public synchronized double getCurrentRange() {
        return mLastGoalRange;
    }

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }

    public synchronized void setShooterOpenLoop(double voltage) {
        mShooter.setOpenLoop(voltage);
    }

    public synchronized void setClosedLoopRpm(double setpointRpm) {
        mShooter.setSpinUp(setpointRpm);
    }

    public synchronized void setActuateHopper(boolean extended) {
        mHopperSolenoid.set(extended);
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

    public void setOverrideCompressor(boolean force_off) {
        mCompressorOverride = force_off;
    }

    public void reloadConstants() {
        mShooter.refreshControllerConsts();
    }
}
