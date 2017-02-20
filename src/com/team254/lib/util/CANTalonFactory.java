package com.team254.lib.util;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.MotorSafety;

/**
 * Creates CANTalon objects and configures all the parameters we control to factory defaults.
 */
public class CANTalonFactory {

    private static final int DEFAULT_CONTROL_PERIOD_MS = 5;
    private static final int DEFAULT_ENABLE_PERIOD_MS = 5;
    private static final int DEFAULT_MOTION_CONTROL_FRAME_PERIOD_MS = 5;
    private static final int DEFAULT_ENCODER_CODES_PER_REV = 1024;
    private static final boolean DEFAULT_LIMIT_SWITCH_NORMALLY_OPEN = true;
    private static final double DEAFAULT_MAX_OUTPUT_VOLTAGE = 12;
    private static final double DEFAULT_NOMINAL_VOLTAGE = 12;
    private static final double DEFAULT_PEAK_VOLTAGE = 12;
    private static final boolean DEFAULT_ENABLE_BRAKE = false;
    private static final boolean DEFAULT_ENABLE_CURRENT_LIMIT = false;
    private static final boolean DEFAULT_ENABLE_SOFT_LIMIT = false;
    private static final boolean DEFAULT_ENABLE_LIMIT_SWITCH = false;
    private static final int DEFAULT_ALLOWABLE_CLOSED_LOOP_ERROR = 0;
    private static final double DEFAULT_CLOSED_LOOP_RAMP_RATE = 0;
    private static final int DEFAULT_CURRENT_LIMIT = 0;
    private static final double DEFAULT_EXPIRATION_TIMEOUT_SECONDS = MotorSafety.DEFAULT_SAFETY_EXPIRATION;
    private static final CANTalon.FeedbackDevice DEFAULT_FEEDBACK_DEVICE = CANTalon.FeedbackDevice.CtreMagEncoder_Relative;
    private static final double DEFAULT_FORWARD_SOFT_LIMIT = 0;
    private static final boolean DEFAULT_INVERTED = false;
    private static final int DEFAULT_I_ZONE = 0;
    private static final double DEFAULT_MOTION_MAGIC_ACCELERATION = 0;
    private static final double DEFAULT_MOTION_MAGIC_CRUISE_VELOCITY = 0;
    private static final double DEFAULT_NOMINAL_CLOSED_LOOP_VOLTAGE = 0;
    private static final double DEFAULT_REVERSE_SOFT_LIMIT = 0;
    private static final boolean DEFAULT_SAFETY_ENABLED = false;

    private static final int DEFAULT_GENERAL_STATUS_FRAME_RATE_MS = 5;
    private static final int DEFAULT_FEEDBACK_STATUS_FRAME_RATE_MS = 5;
    private static final int DEFAULT_QUAD_ENCODER_STATUS_FRAME_RATE_MS = 5;
    private static final int DEFAULT_ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 5;
    private static final int DEFAULT_PULSE_WIDTH_STATUS_FRAME_RATE_MS = 5;
    private static final CANTalon.VelocityMeasurementPeriod DEFAULT_VELOCITY_MEASUREMENT_PERIOD = CANTalon.VelocityMeasurementPeriod.Period_100Ms;
    private static final int DEFAULT_VELOCITY_MEASUREMENT_WINDOW = 5;
    private static final double DEFAULT_VOLTAGE_COMPENSATION_RAMP_RATE = 0;
    private static final double DEFAULT_VOLTAGE_RAMP_RATE = 0;

    /**
     * Don't use me yet, we need to pull "good" values from a talon using getFullTalonInfo to populate the default
     * values first.
     */
    public static CANTalon createTalon(int id) {
        CANTalon talon = new CANTalon(id, DEFAULT_CONTROL_PERIOD_MS, DEFAULT_ENABLE_PERIOD_MS);
        talon.changeControlMode(CANTalon.TalonControlMode.Voltage);
        talon.changeMotionControlFramePeriod(DEFAULT_MOTION_CONTROL_FRAME_PERIOD_MS);
        talon.clearIAccum();
        talon.ClearIaccum();
        talon.clearMotionProfileHasUnderrun();
        talon.clearMotionProfileTrajectories();
        talon.clearStickyFaults();
        talon.configEncoderCodesPerRev(DEFAULT_ENCODER_CODES_PER_REV);
        talon.ConfigFwdLimitSwitchNormallyOpen(DEFAULT_LIMIT_SWITCH_NORMALLY_OPEN);
        talon.configMaxOutputVoltage(DEAFAULT_MAX_OUTPUT_VOLTAGE);
        talon.configNominalOutputVoltage(DEFAULT_NOMINAL_VOLTAGE, -DEFAULT_NOMINAL_VOLTAGE);
        talon.configPeakOutputVoltage(DEFAULT_PEAK_VOLTAGE, -DEFAULT_PEAK_VOLTAGE);
        talon.configPotentiometerTurns(0);
        talon.ConfigRevLimitSwitchNormallyOpen(DEFAULT_LIMIT_SWITCH_NORMALLY_OPEN);
        talon.enableBrakeMode(DEFAULT_ENABLE_BRAKE);
        talon.EnableCurrentLimit(DEFAULT_ENABLE_CURRENT_LIMIT);
        talon.enableForwardSoftLimit(DEFAULT_ENABLE_SOFT_LIMIT);
        talon.enableLimitSwitch(DEFAULT_ENABLE_LIMIT_SWITCH, DEFAULT_ENABLE_LIMIT_SWITCH);
        talon.enableReverseSoftLimit(DEFAULT_ENABLE_SOFT_LIMIT);
        talon.enableZeroSensorPositionOnForwardLimit(false);
        talon.enableZeroSensorPositionOnIndex(false, false);
        talon.enableZeroSensorPositionOnReverseLimit(false);
        talon.reverseOutput(false);
        talon.reverseSensor(false);
        talon.set(0);
        talon.setAllowableClosedLoopErr(DEFAULT_ALLOWABLE_CLOSED_LOOP_ERROR);
        talon.setAnalogPosition(0);
        talon.setCloseLoopRampRate(DEFAULT_CLOSED_LOOP_RAMP_RATE);
        talon.setCurrentLimit(DEFAULT_CURRENT_LIMIT);
        talon.setEncPosition(0);
        talon.setExpiration(DEFAULT_EXPIRATION_TIMEOUT_SECONDS);
        talon.setFeedbackDevice(DEFAULT_FEEDBACK_DEVICE);
        talon.setForwardSoftLimit(DEFAULT_FORWARD_SOFT_LIMIT);
        talon.setInverted(DEFAULT_INVERTED);
        talon.setIZone(DEFAULT_I_ZONE);
        talon.setMotionMagicAcceleration(DEFAULT_MOTION_MAGIC_ACCELERATION);
        talon.setMotionMagicCruiseVelocity(DEFAULT_MOTION_MAGIC_CRUISE_VELOCITY);
        talon.setNominalClosedLoopVoltage(DEFAULT_NOMINAL_CLOSED_LOOP_VOLTAGE);
        talon.setPosition(0);
        talon.setPID(0, 0, 0, 0, DEFAULT_I_ZONE, DEFAULT_CLOSED_LOOP_RAMP_RATE, 0);
        talon.setPID(0, 0, 0, 0, DEFAULT_I_ZONE, DEFAULT_CLOSED_LOOP_RAMP_RATE, 1);
        talon.setProfile(0);
        talon.setPulseWidthPosition(0);
        talon.setReverseSoftLimit(DEFAULT_REVERSE_SOFT_LIMIT);
        talon.setSafetyEnabled(DEFAULT_SAFETY_ENABLED);

        talon.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, DEFAULT_GENERAL_STATUS_FRAME_RATE_MS);
        talon.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, DEFAULT_FEEDBACK_STATUS_FRAME_RATE_MS);
        talon.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, DEFAULT_QUAD_ENCODER_STATUS_FRAME_RATE_MS);
        talon.setStatusFrameRateMs(CANTalon.StatusFrameRate.AnalogTempVbat, DEFAULT_ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS);
        talon.setStatusFrameRateMs(CANTalon.StatusFrameRate.PulseWidth, DEFAULT_PULSE_WIDTH_STATUS_FRAME_RATE_MS);

        talon.SetVelocityMeasurementPeriod(DEFAULT_VELOCITY_MEASUREMENT_PERIOD);
        talon.SetVelocityMeasurementWindow(DEFAULT_VELOCITY_MEASUREMENT_WINDOW);
        talon.setVoltageCompensationRampRate(DEFAULT_VOLTAGE_COMPENSATION_RAMP_RATE);
        talon.setVoltageRampRate(DEFAULT_VOLTAGE_RAMP_RATE);

        // questionable
        talon.DisableNominalClosedLoopVoltage();
        talon.stopLiveWindowMode();

        return talon;
    }

    /**
     * Run this on a fresh talon to produce good values for the defaults.
     */
    public static String getFullTalonInfo(CANTalon talon) {
        StringBuilder sb = new StringBuilder()
                .append("isRevLimitSwitchClosed").append(talon.isRevLimitSwitchClosed()).append("\n")
                .append("getBusVoltage").append(talon.getBusVoltage()).append("\n")
                .append("isForwardSoftLimitEnabled").append(talon.isForwardSoftLimitEnabled()).append("\n")
                .append("getFaultRevSoftLim").append(talon.getFaultRevSoftLim()).append("\n")
                .append("getStickyFaultOverTemp").append(talon.getStickyFaultOverTemp()).append("\n")
                .append("isZeroSensorPosOnFwdLimitEnabled").append(talon.isZeroSensorPosOnFwdLimitEnabled()).append("\n")
                .append("getMotionProfileTopLevelBufferCount").append(talon.getMotionProfileTopLevelBufferCount()).append("\n")
                .append("getNumberOfQuadIdxRises").append(talon.getNumberOfQuadIdxRises()).append("\n")
                .append("getInverted").append(talon.getInverted()).append("\n")
                .append("getPulseWidthRiseToRiseUs").append(talon.getPulseWidthRiseToRiseUs()).append("\n")
                .append("getError").append(talon.getError()).append("\n")
                .append("isSensorPresent").append(talon.isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative)).append("\n")
                .append("isControlEnabled").append(talon.isControlEnabled()).append("\n")
                .append("getTable").append(talon.getTable()).append("\n")
                .append("isEnabled").append(talon.isEnabled()).append("\n")
                .append("isZeroSensorPosOnRevLimitEnabled").append(talon.isZeroSensorPosOnRevLimitEnabled()).append("\n")
                .append("isSafetyEnabled").append(talon.isSafetyEnabled()).append("\n")
                .append("getOutputVoltage").append(talon.getOutputVoltage()).append("\n")
                .append("getTemperature").append(talon.getTemperature()).append("\n")
                .append("getSmartDashboardType").append(talon.getSmartDashboardType()).append("\n")
                .append("getPulseWidthPosition").append(talon.getPulseWidthPosition()).append("\n")
                .append("getOutputCurrent").append(talon.getOutputCurrent()).append("\n")
                .append("get").append(talon.get()).append("\n")
                .append("isZeroSensorPosOnIndexEnabled").append(talon.isZeroSensorPosOnIndexEnabled()).append("\n")
                .append("getMotionMagicCruiseVelocity").append(talon.getMotionMagicCruiseVelocity()).append("\n")
                .append("getStickyFaultRevSoftLim").append(talon.getStickyFaultRevSoftLim()).append("\n")
                .append("getFaultRevLim").append(talon.getFaultRevLim()).append("\n")
                .append("getEncPosition").append(talon.getEncPosition()).append("\n")
                .append("getIZone").append(talon.getIZone()).append("\n")
                .append("getAnalogInPosition").append(talon.getAnalogInPosition()).append("\n")
                .append("getFaultUnderVoltage").append(talon.getFaultUnderVoltage()).append("\n")
                .append("getCloseLoopRampRate").append(talon.getCloseLoopRampRate()).append("\n")
                .append("toString").append(talon.toString()).append("\n")
                .append("getMotionMagicActTrajPosition").append(talon.getMotionMagicActTrajPosition()).append("\n")
                .append("getF").append(talon.getF()).append("\n")
                .append("getClass").append(talon.getClass()).append("\n")
                .append("getAnalogInVelocity").append(talon.getAnalogInVelocity()).append("\n")
                .append("getI").append(talon.getI()).append("\n")
                .append("isReverseSoftLimitEnabled").append(talon.isReverseSoftLimitEnabled()).append("\n")
                .append("getPIDSourceType").append(talon.getPIDSourceType()).append("\n")
                .append("getEncVelocity").append(talon.getEncVelocity()).append("\n")
                .append("GetVelocityMeasurementPeriod").append(talon.GetVelocityMeasurementPeriod()).append("\n")
                .append("getP").append(talon.getP()).append("\n")
                .append("GetVelocityMeasurementWindow").append(talon.GetVelocityMeasurementWindow()).append("\n")
                .append("getDeviceID").append(talon.getDeviceID()).append("\n")
                .append("getStickyFaultRevLim").append(talon.getStickyFaultRevLim()).append("\n")
                .append("getMotionMagicActTrajVelocity").append(talon.getMotionMagicActTrajVelocity()).append("\n")
                .append("getReverseSoftLimit").append(talon.getReverseSoftLimit()).append("\n")
                .append("getD").append(talon.getD()).append("\n")
                .append("getFaultOverTemp").append(talon.getFaultOverTemp()).append("\n")
                .append("getForwardSoftLimit").append(talon.getForwardSoftLimit()).append("\n")
                .append("GetFirmwareVersion").append(talon.GetFirmwareVersion()).append("\n")
                .append("getLastError").append(talon.getLastError()).append("\n")
                .append("isAlive").append(talon.isAlive()).append("\n")
                .append("getPinStateQuadIdx").append(talon.getPinStateQuadIdx()).append("\n")
                .append("getAnalogInRaw").append(talon.getAnalogInRaw()).append("\n")
                .append("getFaultForLim").append(talon.getFaultForLim()).append("\n")
                .append("getSpeed").append(talon.getSpeed()).append("\n")
                .append("getStickyFaultForLim").append(talon.getStickyFaultForLim()).append("\n")
                .append("getFaultForSoftLim").append(talon.getFaultForSoftLim()).append("\n")
                .append("getStickyFaultForSoftLim").append(talon.getStickyFaultForSoftLim()).append("\n")
                .append("getClosedLoopError").append(talon.getClosedLoopError()).append("\n")
                .append("getSetpoint").append(talon.getSetpoint()).append("\n")
                .append("isMotionProfileTopLevelBufferFull").append(talon.isMotionProfileTopLevelBufferFull()).append("\n")
                .append("getDescription").append(talon.getDescription()).append("\n")
                .append("hashCode").append(talon.hashCode()).append("\n")
                .append("isFwdLimitSwitchClosed").append(talon.isFwdLimitSwitchClosed()).append("\n")
                .append("getPinStateQuadA").append(talon.getPinStateQuadA()).append("\n")
                .append("getPinStateQuadB").append(talon.getPinStateQuadB()).append("\n")
                .append("GetIaccum").append(talon.GetIaccum()).append("\n")
                .append("getFaultHardwareFailure").append(talon.getFaultHardwareFailure()).append("\n")
                .append("pidGet").append(talon.pidGet()).append("\n")
                .append("getBrakeEnableDuringNeutral").append(talon.getBrakeEnableDuringNeutral()).append("\n")
                .append("getStickyFaultUnderVoltage").append(talon.getStickyFaultUnderVoltage()).append("\n")
                .append("getPulseWidthVelocity").append(talon.getPulseWidthVelocity()).append("\n")
                .append("GetNominalClosedLoopVoltage").append(talon.GetNominalClosedLoopVoltage()).append("\n")
                .append("getPosition").append(talon.getPosition()).append("\n")
                .append("getExpiration").append(talon.getExpiration()).append("\n")
                .append("getPulseWidthRiseToFallUs").append(talon.getPulseWidthRiseToFallUs()).append("\n")
                .append("createTableListener").append(talon.createTableListener()).append("\n")
                .append("getControlMode").append(talon.getControlMode()).append("\n")
                .append("getMotionMagicAcceleration").append(talon.getMotionMagicAcceleration()).append("\n")
                .append("getControlMode").append(talon.getControlMode());

        return sb.toString();
    }
}
