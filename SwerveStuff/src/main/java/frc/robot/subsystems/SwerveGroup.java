/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SwerveGroup extends SubsystemBase {
  
  SwerveModule frontLeftModule;
  SwerveModule backRightModule;

  public SwerveGroup() {
    frontLeftModule = RobotContainer.frontLeftModule;
    backRightModule = RobotContainer.backRightModule;
  }

  public void moveCrab(Vector2d translation, double rotation) {
    frontLeftModule.moveCrab(translation, rotation);
    backRightModule.moveCrab(translation, rotation);
  }

  @Override
  public void periodic() {
  }
}
