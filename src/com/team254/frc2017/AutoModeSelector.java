package com.team254.frc2017;

import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.modes.GearThenHopperShootModeBlue;
import com.team254.frc2017.auto.modes.StandStillMode;
import com.team254.frc2017.auto.modes.TestLineMode;
import com.team254.frc2017.auto.modes.TestTurnInPlaceMode;

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

    private static final AutoModeCreator mDefaultMode =
            new AutoModeCreator("Gear then Hopper Shoot", ()->new GearThenHopperShootModeBlue());
    private static final AutoModeCreator[] mAllModes = {
            new AutoModeCreator("Test Line", ()->new TestLineMode()),
            new AutoModeCreator("Test Turn In Place", ()->new TestTurnInPlaceMode()),
            new AutoModeCreator("Stand Still", ()->new StandStillMode()),
            mDefaultMode,
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
