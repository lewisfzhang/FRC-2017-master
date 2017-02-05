package com.team254.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.JRadFlywheelController;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Proto_Shooter extends Subsystem {
    private final CANTalon mLeft1, mLeft2, mRight1, mRight2;
    private final Encoder mRpmEncoder;

    private final String mName;

    private final JRadFlywheelController mController;

    private boolean mClosedLoop = false;
    private double mLastOutput = 0;
    private double mLastFlywheelRpm = 0;

    private static CANTalon makeTalon(int deviceNumber, boolean isInverted) {
        CANTalon talon = new CANTalon(deviceNumber);
        talon.changeControlMode(TalonControlMode.Voltage);
        talon.changeMotionControlFramePeriod(5);
        talon.setVoltageCompensationRampRate(10000);
        talon.enableBrakeMode(false);
        talon.setInverted(isInverted);
        return talon;
    }

    public Proto_Shooter(
            double kj,
            double kLoadRatio,
            String name) {
        this.mName = name;

        mLeft1 = makeTalon(10, true);
        // mLeft1.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        // mLeft1.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 1);

        mLeft2 = makeTalon(9, false);
        mRight1 = makeTalon(5, false);
        mRight2 = makeTalon(6, false);

        mRpmEncoder = new Encoder(1, 2);
        mRpmEncoder.setDistancePerPulse(1.0/1024);

        // mRPMEncoder = new Encoder(quadEncoderA, quadEncoderB, encoderInverted, EncodingType.k4X);
        // mRPMEncoder.setDistancePerPulse(1.0 / 1024.0);

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
        private double mPrevPosition = 0;

        public void onStart(double timestamp) {
            synchronized (Proto_Shooter.this) {
                mController.reset();
                mPrevTimestamp = Timer.getFPGATimestamp();
                mPrevPosition = mRpmEncoder.get();
                mLastFlywheelRpm = 0;
            }
        }

        public void onLoop(double timestamp) {
            synchronized (Proto_Shooter.this) {
                double now = Timer.getFPGATimestamp();
                double delta_t = now - mPrevTimestamp;
                double newPosition = mRpmEncoder.getDistance();

                if (delta_t < 1E-6) {
                    delta_t = 1E-6; // Prevent divide-by-zero
                }

                mLastFlywheelRpm = (newPosition - mPrevPosition) / delta_t * 60.0;
                mPrevPosition = newPosition;
                mPrevTimestamp = now;
                if (mClosedLoop) {
                    double voltage = mController.calculate(mLastFlywheelRpm, delta_t, now);
                    setVoltage(voltage);
                }
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
        if (!mClosedLoop) {
            mController.setInitialOutput(Constants.kFlywheelAKfv * rpm);
        }
        mClosedLoop = true;
        mController.setSetpoint(Constants.kFlywheelReduction * rpm);
    }

    public synchronized void setManualVoltage(double voltage) {
        mClosedLoop = false;
        setVoltage(voltage);
    }

    // This is protected since it should only ever be called by a public
    // synchronized method or the loop.
    protected void setVoltage(double voltage) {
        //SmartDashboard.putNumber("PIDF (v)", voltage);
        mLastOutput = voltage;
        mLeft1.set(voltage);
        mLeft2.set(voltage);
        mRight1.set(voltage);
        mRight2.set(voltage);
    }

    public synchronized double getSetpoint() {
        return mController.getSetpoint();
    }

    public synchronized boolean isOnTarget() {
        return (Math.abs(mLastFlywheelRpm - getSetpoint()) < Constants.kFlywheelOnTargetTolerance);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Flywheel RPM (" + mName + ")", mLastFlywheelRpm);
        SmartDashboard.putNumber("Encoder Count (" + mName + ")", mRpmEncoder.getDistance());
        SmartDashboard.putNumber("Flywheel Voltage (" + mName + ")", mLastOutput);

        double errorLimited = Math.max(-200, Math.min(200, mLastFlywheelRpm - getSetpoint()));
        SmartDashboard.putNumber("Flywheel error limited (" + mName + ")", errorLimited);
    }

    @Override
    public synchronized void stop() {
        setVoltage(0);
        mClosedLoop = false;
        mController.reset();
    }

    @Override
    public synchronized void zeroSensors() {
        // Nothing to do.
    }
}
