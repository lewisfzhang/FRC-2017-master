package com.team254.lib.util.pixy.constants;

import com.team254.frc2017.Constants;

public class PixyNumberConstants {
    public static double cx, cy, k1, k2, k3;
    public PixyNumberConstants() {
        switch(Constants.kPixyNumber) {
        case 1:
            PixyNumber1Constants _pixy1 = new PixyNumber1Constants();
            cx = _pixy1.cx;
            cy = _pixy1.cy;
            k1 = _pixy1.k1;
            k2 = _pixy1.k2;
            k3 = _pixy1.k3;
            break;
        case 2:
            PixyNumber2Constants _pixy2 = new PixyNumber2Constants();
            cx = _pixy2.cx;
            cy = _pixy2.cy;
            k1 = _pixy2.k1;
            k2 = _pixy2.k2;
            k3 = _pixy2.k3;
            break;
        case 3:
            PixyNumber3Constants _pixy3 = new PixyNumber3Constants();
            cx = _pixy3.cx;
            cy = _pixy3.cy;
            k1 = _pixy3.k1;
            k2 = _pixy3.k2;
            k3 = _pixy3.k3;
            break;
        case 4:
            PixyNumber4Constants _pixy4 = new PixyNumber4Constants();
            cx = _pixy4.cx;
            cy = _pixy4.cy;
            k1 = _pixy4.k1;
            k2 = _pixy4.k2;
            k3 = _pixy4.k3;
            break;
        case 5:
            PixyNumber5Constants _pixy5 = new PixyNumber5Constants();
            cx = _pixy5.cx;
            cy = _pixy5.cy;
            k1 = _pixy5.k1;
            k2 = _pixy5.k2;
            k3 = _pixy5.k3;
            break;
        case 6:
            PixyNumber6Constants _pixy6 = new PixyNumber6Constants();
            cx = _pixy6.cx;
            cy = _pixy6.cy;
            k1 = _pixy6.k1;
            k2 = _pixy6.k2;
            k3 = _pixy6.k3;
            break;
        }
    }
}
