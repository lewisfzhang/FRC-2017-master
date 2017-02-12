package com.team254.frc2017;

import com.team254.frc2017.vision.TargetInfo;
import com.team254.lib.util.*;

import java.util.ArrayList;
import java.util.List;

public class RobotState {
  private static RobotState instance_ = new RobotState();

  public static RobotState getInstance() {
    return instance_;
  }

  protected RobotState() {
    reset(0, new RigidTransform2d());
  }


  public synchronized void reset(double start_time, RigidTransform2d initial_field_to_vehicle) {
  }

  public void addVisionUpdate(double timestamp, List<TargetInfo> vision_update) {

  }
}
