/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;

public class RotateToAngle extends CommandBase {

  double speed, angle;
  boolean debugging = false;

  /**
   * Creates a new RotateToAngle.
   */
  public RotateToAngle(double speed, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveGroup);

    assert angle >= 0 && angle <= 359;

    this.speed = speed;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Rotating to angle...");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turnRobot();
  }

  public void turnRobot() {
    double currentAngle = RobotContainer.swerveGroup.getConvertedGyroAngle();
    double minSpeed = 0.07;
    double convertedSpeed = 0;

    if (speed <= minSpeed) {
      convertedSpeed = minSpeed;
    }
    else {
      convertedSpeed = MathUtil.clamp(speed * Math.sqrt((Math.abs(currentAngle - angle) / 75)), minSpeed, speed);
    }

    if (angle > currentAngle) {
      double error = angle - currentAngle;
      
      if (error < 180) {
        //move D by increasing C
        RobotContainer.swerveGroup.moveSwerve(new Vector2d(0, 0), -convertedSpeed, debugging);
      }
      else if (error >= 180) {
        //move towards D by decreasing C
        RobotContainer.swerveGroup.moveSwerve(new Vector2d(0, 0), convertedSpeed, debugging);
      }
    }
    else if (angle < currentAngle) {
      double error = currentAngle - angle;
      
      if (error < 180) {
        //move towards D decreasing C
        RobotContainer.swerveGroup.moveSwerve(new Vector2d(0, 0), convertedSpeed, debugging);
      }
      else if (error >= 180) {
        //move towards D by increasing C
        RobotContainer.swerveGroup.moveSwerve(new Vector2d(0, 0), -convertedSpeed, debugging);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveGroup.moveSwerve(new Vector2d(0, 0), 0, debugging);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentAngle = RobotContainer.swerveGroup.getConvertedGyroAngle();
    boolean isAtAngle = Math.abs(currentAngle - angle) <= 1 || Math.abs(currentAngle - angle) >= 359;

    return isAtAngle;
  }
}
