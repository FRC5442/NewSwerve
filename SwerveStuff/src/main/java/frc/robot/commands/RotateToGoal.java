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
  double convertedSpeed = 0;
  double yawOffset = 0;
  boolean tapeDetected = false;

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
    tapeDetected = RobotContainer.piVisionTable.isTapeDetected();

    yawOffset = RobotContainer.piVisionTable.getYawOffset();

    if (tapeDetected) {

      if (Math.abs(yawOffset) > 3) {
        convertedSpeed = speed * MathUtil.clamp((yawOffset / 12) * Math.abs(yawOffset / 15), -1, 1);
        RobotContainer.swerveGroup.moveSwerve(new Vector2d(0, 0), convertedSpeed);
        System.out.println("Rotating...");
      } else {
        this.isFinished();
      }
    } else if (!tapeDetected) {
      double currentAngel = RobotContainer.navX.getAngle();

      new RotateToAngle(30, (currentAngel + 30));
      System.out.println("Scanning...");
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveGroup.moveSwerve(new Vector2d(0,0), 0);
    System.out.println("RotateToGoal Stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(yawOffset) < 3) {
      return true;
    } else {
      return false;
    }
  }
}
