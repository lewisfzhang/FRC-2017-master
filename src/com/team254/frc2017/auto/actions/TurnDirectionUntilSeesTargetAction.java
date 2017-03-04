package com.team254.frc2017.auto.actions;

import java.util.Optional;

import com.team254.frc2017.RobotState;
import com.team254.frc2017.ShooterAimingParameters;
import com.team254.lib.util.math.Rotation2d;

import edu.wpi.first.wpilibj.Timer;

public class TurnDirectionUntilSeesTargetAction extends TurnToHeadingAction {

    RobotState mState = RobotState.getInstance();
    
    public TurnDirectionUntilSeesTargetAction(Rotation2d heading) {
        super(heading);
        // TODO Auto-generated constructor stub
    }
    
    @Override
    public boolean isFinished() {
        double now = Timer.getFPGATimestamp();
        Optional<ShooterAimingParameters> aimParams = mState.getAimingParameters(now);
        if(aimParams.isPresent() && Math.abs(now - aimParams.get().getLastSeenTimestamp()) < 0.5) {
            return true;
        }
        return super.isFinished(); 
    }

}
