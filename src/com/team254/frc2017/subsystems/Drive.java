package com.team254.frc2017.subsystems;

import java.util.Collection;
import java.util.LinkedList;

import com.ctre.CANTalon;
import com.team254.frc2017.Constants;
import com.team254.frc2017.ControlBoard;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.CollisionDetectionListener;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {
    private static CANTalon mLeftMaster, mRightMaster, mLeftSlave, mRightSlave; // Master and slave motor

    private static Drive mInstance;
    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    ControlBoard mControlBoard = ControlBoard.getInstance();
    
    private Collection<CollisionDetectionListener> mCollisionListeners = new LinkedList<CollisionDetectionListener>();

    private Drive() {
        mLeftMaster = new CANTalon(11);
        mLeftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mLeftSlave = new CANTalon(12);
        mLeftSlave.changeControlMode(CANTalon.TalonControlMode.PercentVbus);


        mRightMaster = new CANTalon(3);
        mRightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mRightMaster.setInverted(true);
        mRightMaster.enableBrakeMode(false);

        mRightSlave = new CANTalon(4);
        mRightSlave.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        mRightSlave.setInverted(true);
        mRightSlave.enableBrakeMode(false);

    }

    public static Drive getInstance() {
        if (mInstance == null)
            mInstance = new Drive();
        return mInstance;
    }

    protected class Drive_Loop implements Loop {
        public void onStart(double timestamp) {

        }

        public void onLoop(double timestamp) {
            DriveSignal driveSignal = mCheesyDriveHelper.cheesyDrive(mControlBoard.getThrottle(),
                    mControlBoard.getTurn(), mControlBoard.getQuickTurn());
            mLeftMaster.set(driveSignal.getLeft());
            mLeftSlave.set(driveSignal.getLeft());
            mRightMaster.set(driveSignal.getRight());
            mRightSlave.set(driveSignal.getRight());
        }

        public void onStop(double timestamp) {
            stop();
        }
    }

    public void registerEnabledLoops(Looper in) {
        in.register(new Drive_Loop());
    }

    public void stop() {
        mLeftMaster.stopMotor();
        mLeftSlave.stopMotor();
        mRightMaster.stopMotor();
        mRightSlave.stopMotor();
    }
    
    

    public void outputToSmartDashboard() {
    }

    public void zeroSensors() {

    }


}
