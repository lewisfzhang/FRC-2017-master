package com.team254.frc2017.subsystems;

import java.util.Collection;
import java.util.LinkedList;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.ControlBoard;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.*;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {

    private static Drive mInstance = new Drive();

    public static Drive getInstance() {
        return mInstance;
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP
    }

    private final CANTalon mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
    private final Solenoid mShifter;
    private DriveControlState mDriveControlstate;

    private boolean mIsHighGear;
    private boolean mIsBreakMode;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            setOpenLoop(DriveSignal.NEUTRAL);
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (mDriveControlstate) {
                    case OPEN_LOOP:
                        return;
                    default:
                        System.out.println("Unexpected drive control state: " + mDriveControlstate);
                        break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private Drive() {
        // Start all Talons in open loop mode.
        mLeftMaster = new CANTalon(Constants.kLeftDriveMasterId);
        mLeftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);

        mLeftSlave = new CANTalon(Constants.kLeftDriveSlaveId);
        mLeftSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
        mLeftSlave.set(Constants.kLeftDriveMasterId);

        mRightMaster = new CANTalon(Constants.kRightDriveMasterId);
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mRightMaster.setInverted(true);

        mRightSlave = new CANTalon(Constants.kRightDriverSlaveId);
        mRightSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
        mRightSlave.setInverted(true);
        mRightSlave.set(Constants.kRightDriveMasterId);

        mShifter = Constants.makeSolenoidForId(Constants.kShifterSolenoidId);

        setHighGear(true);
        setOpenLoop(DriveSignal.NEUTRAL);

        // Force a CAN message across.
        mIsBreakMode = true;
        setBreakMode(false);
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlstate != DriveControlState.OPEN_LOOP) {
            mLeftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            mRightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            mDriveControlstate = DriveControlState.OPEN_LOOP;
        }
        mRightMaster.set(signal.getRight());
        mLeftMaster.set(signal.getLeft());
    }

    public boolean isHighGear() {
        return mIsHighGear;
    }

    public synchronized void setHighGear(boolean highGear) {
        mIsHighGear = highGear;
        mShifter.set(!highGear);
    }

    public synchronized void setBreakMode(boolean on) {
        if (mIsBreakMode != on) {
            mIsBreakMode = on;
            mRightMaster.enableBrakeMode(on);
            mRightSlave.enableBrakeMode(on);
            mLeftMaster.enableBrakeMode(on);
            mLeftSlave.enableBrakeMode(on);
        }
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputToSmartDashboard() {
    }

    @Override
    public void zeroSensors() {

    }
}
