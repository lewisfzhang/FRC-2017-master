package com.team254.lib.util.pixy.constants;

import com.team254.frc2017.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PixyNumberConstants {
    public static double cx, cy, k1, k2, k3;
    public PixyNumberConstants() {
        switch(Constants.kPixyNumber) {
        case 1:
            cx = PixyNumber1Constants.cx;
            cy = PixyNumber1Constants.cy;
            k1 = PixyNumber1Constants.k1;
            k2 = PixyNumber1Constants.k2;
            k3 = PixyNumber1Constants.k3;
            break;
        case 2:
            cx = PixyNumber2Constants.cx;
            cy = PixyNumber2Constants.cy;
            k1 = PixyNumber2Constants.k1;
            k2 = PixyNumber2Constants.k2;
            k3 = PixyNumber2Constants.k3;
            break;
        case 3:
            cx = PixyNumber3Constants.cx;
            cy = PixyNumber3Constants.cy;
            k1 = PixyNumber3Constants.k1;
            k2 = PixyNumber3Constants.k2;
            k3 = PixyNumber3Constants.k3;
            break;
        case 4:
            cx = PixyNumber4Constants.cx;
            cy = PixyNumber4Constants.cy;
            k1 = PixyNumber4Constants.k1;
            k2 = PixyNumber4Constants.k2;
            k3 = PixyNumber4Constants.k3;
            break;
        case 5:
            cx = PixyNumber5Constants.cx;
            cy = PixyNumber5Constants.cy;
            k1 = PixyNumber5Constants.k1;
            k2 = PixyNumber5Constants.k2;
            k3 = PixyNumber5Constants.k3;
            break;
        case 6:
            cx = PixyNumber6Constants.cx;
            cy = PixyNumber6Constants.cy;
            k1 = PixyNumber6Constants.k1;
            k2 = PixyNumber6Constants.k2;
            k3 = PixyNumber6Constants.k3;
            break;
        default:
            cx = PixyNumber1Constants.cx;
            cy = PixyNumber1Constants.cy;
            k1 = PixyNumber1Constants.k1;
            k2 = PixyNumber1Constants.k2;
            k3 = PixyNumber1Constants.k3;
            break;
        }
    }
}
