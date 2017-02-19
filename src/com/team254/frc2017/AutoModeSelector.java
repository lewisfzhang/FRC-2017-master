package com.team254.frc2017;

import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.modes.GearThenHopperShootMode;

public class AutoModeSelector {

    public AutoModeBase getSelectedAutoMode() {
        return new GearThenHopperShootMode();
    }
}
