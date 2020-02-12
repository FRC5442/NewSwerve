/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class PiVisionTable extends SubsystemBase {

  NetworkTable visionTable;

  /**
   * Creates a new VisionTable.
   */
  public PiVisionTable() {
    visionTable = RobotContainer.visionTable;
  }

  public double getYawOffset() {
    double yawOffset = visionTable.getEntry("Yaw").getDouble(0);
    return yawOffset;
  }

  public boolean isTapeDetected() {
    boolean tapeDetected = visionTable.getEntry("tapeDetected").getBoolean(false);
    return tapeDetected;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
