package com.team254.frc2017;

import com.team254.frc2017.Constants.RobotName;

public abstract class ConstantsModifier {    
    
    /**
     * Initializes robot constants
     * @param name the name of the robot, PRAC_BOT, PROG_BOT, or COMP_BOT
     */
    public static void initConstants(RobotName name) {
        switch (name) {
            case PRAC_BOT : ConstantsModifier.setPracBotConstants();
            case PROG_BOT : ConstantsModifier.setProgBotConstants();
            case COMP_BOT : break;
            default : break;
        }
    }
   
    private static void setPracBotConstants() {
        // UPDATE VALUES HERE!
        // e.g. Constants.kFlywheelKi = 0;
    }
    
    private static void setProgBotConstants() {
        // UPDATE VALUES HERE!
        // e.g. Constants.kFlywheelKi = 0;
    }
}
