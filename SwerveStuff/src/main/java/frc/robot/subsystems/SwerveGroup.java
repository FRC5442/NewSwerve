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
import frc.robot.commands.Drive;

public class SwerveGroup extends SubsystemBase {
  
  SwerveModule backRightModule;

  public SwerveGroup() {
  
  }

  public void moveCrab(Vector2d translation, double rotation) {
    backRightModule = RobotContainer.backRightModule;
    backRightModule.moveCrab(translation, rotation);
  }

  @Override
  public void periodic() {
  }
}
