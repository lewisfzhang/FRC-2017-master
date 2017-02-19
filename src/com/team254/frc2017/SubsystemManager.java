package com.team254.frc2017;

import com.team254.frc2017.loops.Looper;
import com.team254.frc2017.subsystems.Subsystem;

import java.util.List;

/**
 * Updates all the subsystems
 */
public class SubsystemManager {

    private final List<Subsystem> mAllSubsystems;

    public SubsystemManager(List<Subsystem> allSubsystems) {
        mAllSubsystems = allSubsystems;
    }

    public void outputToSmartDashboard() {
        mAllSubsystems.forEach((s)->s.outputToSmartDashboard());
    }

    public void stop() {
        mAllSubsystems.forEach((s)->s.stop());
    }

    public void zeroSensors() {
        mAllSubsystems.forEach((s)->s.zeroSensors());
    }

    public void registerEnabledLoops(Looper enabledLooper) {
        mAllSubsystems.forEach((s)->s.registerEnabledLoops(enabledLooper));
    }
}
