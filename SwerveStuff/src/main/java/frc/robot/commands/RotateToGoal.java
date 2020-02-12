/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;

public class RotateToGoal extends CommandBase {

  double speed = 0;

  /**
   * Creates a new RotateToGoal.
   */
  public RotateToGoal(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveGroup, RobotContainer.piVisionTable);

    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Rotating to tape...");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yawOffset = RobotContainer.piVisionTable.getYawOffset();

    SmartDashboard.putBoolean("Tape Detected: ", RobotContainer.piVisionTable.isTapeDetected());
    SmartDashboard.putNumber("Yaw Offset: ", yawOffset);

    if (RobotContainer.piVisionTable.isTapeDetected() && Math.abs(yawOffset) > 5) {
      RobotContainer.swerveGroup.moveSwerve(new Vector2d(0, 0), speed * MathUtil.clamp(yawOffset / 10, -1, 1));
    }
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
