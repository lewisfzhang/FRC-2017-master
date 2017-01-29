package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.JRadFlywheelController;
import com.team254.lib.util.SynchronousPIDF;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Proto_Shooter extends Subsystem {
    private CANTalon mMaster, mSlave;
    private Encoder mRPMEncoder;
    
    private String name;

    private JRadFlywheelController mController;
    private boolean mClosedLoop = false;

    private double mVelocityRpm = 0;

    private double mLastOutput = 0;

    public Proto_Shooter(int FlywheelMasterID, int FlywheelSlaveID, boolean invertedMaster, boolean invertedSlave, int quadEncoderA, int quadEncoderB, boolean encoderInverted, double kj, double kLoadRatio, String name) {
        this.name = name;
        
        mMaster = new CANTalon(FlywheelMasterID);
        mMaster.changeControlMode(TalonControlMode.Voltage);
        mMaster.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
        mMaster.setVoltageCompensationRampRate(10000.0);
        mMaster.enableBrakeMode(false);
        mMaster.setInverted(invertedMaster);

        mSlave = new CANTalon(FlywheelSlaveID);
        mSlave.changeControlMode(TalonControlMode.Voltage);
        mSlave.changeMotionControlFramePeriod(5); // 5ms (200 Hz)
        mSlave.setVoltageCompensationRampRate(10000.0);
        mSlave.enableBrakeMode(false);
        mSlave.setInverted(invertedSlave);
        

        mRPMEncoder = new Encoder(quadEncoderA, quadEncoderB, encoderInverted, EncodingType.k4X);
        mRPMEncoder.setDistancePerPulse(1.0 / 1024.0);

        mController = new JRadFlywheelController(kj, kLoadRatio);
        // mController.enableLogging(true);
        mController.setOutputRange(-12.0, 12.0);
    }
    
    public void setPIDF(double j, double loadRatio) {
        mController.setKj(j);
        mController.setKLoadRatio(loadRatio);
    }

    public class Proto_Shooter_Loop implements Loop {
        private double mPrevTimestamp = 0;
        private double mPrevRotations = 0;

        public void onStart(double timestamp) {
            synchronized (Proto_Shooter.this) {
                mController.reset();
                mPrevTimestamp = Timer.getFPGATimestamp();
                mPrevRotations = mRPMEncoder.getDistance();
                mVelocityRpm = 0.0;
            }
        }

        public void onLoop(double timestamp) {
            synchronized (Proto_Shooter.this) {
                //mController.setGains(Constants.kFlywheelKp, Constants.kFlywheelKi, Constants.kFlywheelKd, Constants.kFlywheelKf);
                double now = Timer.getFPGATimestamp();
                double distance_now = mRPMEncoder.getDistance();
                double delta_t = now - mPrevTimestamp;
                if (delta_t < 1E-6) {
                    delta_t = 1E-6; // Prevent divide-by-zero
                }
                mVelocityRpm = (distance_now - mPrevRotations) / delta_t * 60.0;
                if (mClosedLoop) {
                    double voltage = mController.calculate(mVelocityRpm, delta_t, now);
                    setVoltage(voltage);
                }

                mPrevTimestamp = now;
                mPrevRotations = distance_now;
            }
        }

        public void onStop(double timestamp) {
            synchronized (Proto_Shooter.this) {
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
        // Whoops: this actually turns the JRad controller into the Kp version
        if (mClosedLoop) {
            mController.setInitialOutput(8);
        }
        mClosedLoop = true;
        mController.setSetpoint(Constants.kFlywheelReduction * rpm);
    }

    public synchronized void setManualVoltage(double voltage) {
        mClosedLoop = false;
        setVoltage(voltage);
    }

    public synchronized double getRpm() {
        return mVelocityRpm / Constants.kFlywheelReduction;
    }

    // This is protected since it should only ever be called by a public
    // synchronized method or the loop.
    protected void setVoltage(double voltage) {
        //SmartDashboard.putNumber("PIDF (v)", voltage);
        mLastOutput = voltage;
        mMaster.set(voltage);
        mSlave.set(voltage);
    }

    public synchronized double getSetpoint() {
        return mController.getSetpoint();
    }

    public synchronized boolean isOnTarget() {
        return (Math.abs(getRpm() - getSetpoint()) < Constants.kFlywheelOnTargetTolerance);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Flywheel RPM (" + name + ")", getRpm());
        SmartDashboard.putNumber("Encoder Count (" + name + ")", mRPMEncoder.get());
        SmartDashboard.putNumber("Flywheel Voltage (" + name + ")", mLastOutput);

        double errorLimited = Math.max(-200, Math.min(200, mVelocityRpm - getSetpoint()));
        SmartDashboard.putNumber("Flywheel error limited (" + name + ")", errorLimited);
    }

    @Override
    public synchronized void stop() {
        mClosedLoop = false;
        mController.reset();
        mMaster.set(0.0);
        mSlave.set(0.0);
    }

    @Override
    public synchronized void zeroSensors() {
        // Nothing to do.
    }
}
