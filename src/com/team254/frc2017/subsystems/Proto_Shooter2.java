package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.MovingAverage;
import com.team254.lib.util.SynchronousPIDF;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Proto_Shooter2 extends Subsystem {
    private static Proto_Shooter2 mInstance = null;

    private CANTalon mMaster, mSlave, mIntake;
    Encoder mRPMEncoder = new Encoder(2, 3, true, EncodingType.k4X);
    
    private SynchronousPIDF mController;
    private boolean mClosedLoop = false;

    private double mVelocityRpm = 0;
    
    public static Proto_Shooter2 getInstance() {
        if (mInstance == null) {
            mInstance = new Proto_Shooter2();
        }
        return mInstance;
    }

    private Proto_Shooter2() {
        mMaster = new CANTalon(4);
        mMaster.changeControlMode(TalonControlMode.Voltage);
        mMaster.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
        mMaster.setStatusFrameRateMs(StatusFrameRate.Feedback, 1); // 1ms (1 KHz)
        mMaster.setVoltageCompensationRampRate(10000.0);
        //mMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative); Mag Encoder connected directly to RoboRIO
        mMaster.enableBrakeMode(false);

        mSlave = new CANTalon(5);
        mSlave.changeControlMode(TalonControlMode.Voltage);
        mSlave.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
        mSlave.setVoltageCompensationRampRate(10000.0);
        mSlave.enableBrakeMode(false);

        mIntake = new CANTalon(6);
        mIntake.changeControlMode(TalonControlMode.Voltage);
        mIntake.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
        mIntake.setVoltageCompensationRampRate(10000.0);
        mIntake.enableBrakeMode(false);
        
        mRPMEncoder.setDistancePerPulse(0.0009765625); // 1/1024 (1024 counts per revolution)
        mRPMEncoder.setSamplesToAverage(50);
        
        mController = new SynchronousPIDF(Constants.kFlywheelKp, Constants.kFlywheelKi, Constants.kFlywheelKd, Constants.kFlywheelKf);
        this.setRpmSetpoint(Constants.kFlywheelTarget);
        mController.setOutputRange(-12.0, 12.0);
    }

    public class Proto_Shooter_Loop implements Loop {
        private double mPrevTimestamp = 0;
        private double mPrevRotations = 0;
        boolean mIsFirstLoop;

        public void onStart(double timestamp) {
            synchronized (Proto_Shooter2.this) {
                mController.reset();
                mPrevTimestamp = timestamp;
                mPrevRotations = mMaster.getPosition();
                mVelocityRpm = 0.0;
                mIsFirstLoop = true;
            }
        }

        public void onLoop(double timestamp) {
            synchronized (Proto_Shooter2.this) {
                if (mIsFirstLoop) {
                    mPrevTimestamp = timestamp;
                    mIsFirstLoop = false;
                }
                
                double mTimestamp = Timer.getFPGATimestamp();
                //double curr_rotations = mMaster.getPosition();
                //System.out.println("Position " + curr_rotations);
                mVelocityRpm = mRPMEncoder.getRate();
                System.out.println("Encoder rate: " + mRPMEncoder.getRate());
                double delta_t = mTimestamp - mPrevTimestamp;
                if (delta_t < 1E-6) {
                    delta_t = 1E-6; // Prevent divide-by-zero
                }
                
                //mVelocityRpm = ((curr_rotations - mPrevRotations) * 60.0) / delta_t;
                
                //mMovingAverageRPM.addNumber(mVelocityRpm);

                if (mClosedLoop) {
                    //if (mMovingAverageRPM.isUnderMaxSize()){
                        double voltage = mController.calculate(mVelocityRpm, delta_t);
                        setVoltage(voltage);
                    //}
                    //else
                    //{
                        //double voltage = mController.calculate(mMovingAverageRPM.getAverage(), delta_t);
                        //setVoltage(voltage);
                    //}                
                    
                }

                mPrevTimestamp = mTimestamp;
                //mPrevRotations = curr_rotations;
                outputToSmartDashboard();
            }
        }

        public void onStop(double timestamp) {
            synchronized (Proto_Shooter2.this) {
                mClosedLoop = false;
                stop();
            }
        }
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(new Proto_Shooter_Loop());
    }

    public synchronized void setRpmSetpoint(double rpm) {
        mClosedLoop = true;
        mController.setSetpoint(Constants.kFlywheelReduction * rpm);
    }

    public synchronized void setManualVoltage(double voltage) {
        mClosedLoop = false;
        setVoltage(voltage);
    } 

    public synchronized double getRpm() {
        return mRPMEncoder.getRate()*60 / Constants.kFlywheelReduction;
    }

    // This is protected since it should only ever be called by a public synchronized method or the loop.
    protected void setVoltage(double voltage) {
        mMaster.set(voltage);
        mSlave.set(voltage);
    }

    // TODO: Move this to its own subsystem.
    public synchronized void setFeedRoller(double voltage) {
        mIntake.set(voltage);
    }

    public synchronized double getSetpoint() {
        return mController.getSetpoint();
    }

    public synchronized boolean isOnTarget() {
        return (Math.abs(getRpm() - getSetpoint()) < Constants.kFlywheelOnTargetTolerance);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Flywheel RPM (2)", getRpm());
        SmartDashboard.putNumber("Encoder Count (2)", mRPMEncoder.get());
    }

    @Override
    public synchronized void stop() {
        mClosedLoop = false;
        mController.reset();
        mMaster.set(0.0);
        mSlave.set(0.0);
        mIntake.set(0.0);
    }

    @Override
    public synchronized void zeroSensors() {
        // Nothing to do.
    }
}
