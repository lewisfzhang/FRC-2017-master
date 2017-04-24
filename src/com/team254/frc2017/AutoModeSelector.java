package com.team254.frc2017;

import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.modes.ActuateHopperShootModeBlue;
import com.team254.frc2017.auto.modes.ActuateHopperShootModeRed;
import com.team254.frc2017.auto.modes.BoilerGearThenShootModeBlue;
import com.team254.frc2017.auto.modes.BoilerGearThenShootModeRed;
import com.team254.frc2017.auto.modes.CenterGearThenShootModeBlue;
import com.team254.frc2017.auto.modes.CenterGearThenShootModeRed;
import com.team254.frc2017.auto.modes.GearThenHopperShootModeBlue;
import com.team254.frc2017.auto.modes.GearThenHopperShootModeRed;
import com.team254.frc2017.auto.modes.GearThenIntakeModeBlue;
import com.team254.frc2017.auto.modes.GearThenIntakeModeRed;
import com.team254.frc2017.auto.modes.RamHopperShootModeBlue;
import com.team254.frc2017.auto.modes.RamHopperShootModeRed;
import com.team254.frc2017.auto.modes.StandStillMode;
import com.team254.frc2017.auto.modes.TestLineMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.json.simple.JSONArray;

import java.util.function.Supplier;

public class AutoModeSelector {

    public static final String AUTO_OPTIONS_DASHBOARD_KEY = "auto_options";
    public static final String SELECTED_AUTO_MODE_DASHBOARD_KEY = "selected_auto_mode";

    private static class AutoModeCreator {
        private final String mDashboardName;
        private final Supplier<AutoModeBase> mCreator;

        private AutoModeCreator(String dashboardName, Supplier<AutoModeBase> creator) {
            mDashboardName = dashboardName;
            mCreator = creator;
        }
    }

    private static final AutoModeCreator mDefaultMode = new AutoModeCreator("Stand Still", () -> new StandStillMode());
    private static final AutoModeCreator[] mAllModes = {
            new AutoModeCreator("Boiler Gear then Intake Shoot Blue", () -> new GearThenIntakeModeBlue()),
            new AutoModeCreator("Boiler Gear then Intake Shoot Red", () -> new GearThenIntakeModeRed()),
            new AutoModeCreator("Boiler Gear then 10 Ball Shoot Red", () -> new BoilerGearThenShootModeRed()),
            new AutoModeCreator("Boiler Gear then 10 Ball Shoot Blue", () -> new BoilerGearThenShootModeBlue()),
            new AutoModeCreator("Center Gear then Shoot Red", () -> new CenterGearThenShootModeRed()),
            new AutoModeCreator("Center Gear then Shoot Blue", () -> new CenterGearThenShootModeBlue()),
            new AutoModeCreator("Ram Hopper Blue", () -> new RamHopperShootModeBlue()),
            new AutoModeCreator("Ram Hopper Red", () -> new RamHopperShootModeRed()),
            new AutoModeCreator("Gear then Hopper Shoot Blue", () -> new GearThenHopperShootModeBlue()),
            new AutoModeCreator("Gear then Hopper Shoot Red", () -> new GearThenHopperShootModeRed()),
            new AutoModeCreator("TEST Straight line", () -> new TestLineMode()),
    };

    public static void initAutoModeSelector() {
        JSONArray modesArray = new JSONArray();
        for (AutoModeCreator mode : mAllModes) {
            modesArray.add(mode.mDashboardName);
        }
        SmartDashboard.putString(AUTO_OPTIONS_DASHBOARD_KEY, modesArray.toString());
        SmartDashboard.putString(SELECTED_AUTO_MODE_DASHBOARD_KEY, mDefaultMode.mDashboardName);
    }

    public static AutoModeBase getSelectedAutoMode() {
        String selectedModeName = SmartDashboard.getString(
                SELECTED_AUTO_MODE_DASHBOARD_KEY,
                "NO SELECTED MODE!!!!");
        for (AutoModeCreator mode : mAllModes) {
            if (mode.mDashboardName.equals(selectedModeName)) {
                return mode.mCreator.get();
            }
        }
        DriverStation.reportError("Failed to select auto mode: " + selectedModeName, false);
        return mDefaultMode.mCreator.get();
    }
}
