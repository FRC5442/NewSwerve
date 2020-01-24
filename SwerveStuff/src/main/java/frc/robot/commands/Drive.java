/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Drive extends CommandBase {

  public Drive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveGroup);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Joystick driveStick = RobotContainer.xboxController1;

    Vector2d translation = new Vector2d(Math.pow(driveStick.getRawAxis(0), 2), Math.pow(driveStick.getRawAxis(1), 2));
    RobotContainer.swerveGroup.moveSwerve(translation, Math.pow(driveStick.getRawAxis(4), 2));

    SmartDashboard.putNumber("Left Joystick Magnitude: ", Math.pow(translation.magnitude(), 2));
    SmartDashboard.putNumber("Left Joystick Angle: ", (Math.atan2(translation.y, -translation.x) * (180/Math.PI)) + 180);

    SmartDashboard.putNumber("Right Joystick Magnitude: ", Math.pow(driveStick.getRawAxis(4), 2));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
