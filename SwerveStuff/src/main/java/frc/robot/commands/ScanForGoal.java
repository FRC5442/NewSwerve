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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.RotateToGoal;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive;

public class ScanForGoal extends CommandBase {

  double speed = 0;
  double convertedSpeed = 0;
  double yawOffset = 0;
  double rightX = 0.25;
  double currentAngle = 0;
  boolean tapeDetected = false;

  /**
   * Creates a new RotateToGoal.
   */
  public ScanForGoal() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveGroup, RobotContainer.piVisionTable);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Rotating to tape...");
    currentAngle = RobotContainer.swerveGroup.getConvertedGyroAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    yawOffset = RobotContainer.piVisionTable.getYawOffset();
    tapeDetected = RobotContainer.piVisionTable.isTapeDetected();
    
    if (!tapeDetected) {
        if (currentAngle + 90 == RobotContainer.swerveGroup.getConvertedGyroAngle()) {
        System.out.println("Rotating...(180-270)");
        RobotContainer.swerveGroup.moveSwerve(new Vector2d(0,0), rightX);
        } else if (currentAngle + 90 == RobotContainer.swerveGroup.getConvertedGyroAngle()) {
        System.out.println("Rotating...(270-360)");
        RobotContainer.swerveGroup.moveSwerve(new Vector2d(0,0), -rightX);
        } else {
        System.out.println("Rotating...(else)");
        RobotContainer.swerveGroup.moveSwerve(new Vector2d(0,0), rightX);
        }
            
        System.out.println("Scanning...: " + currentAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveGroup.moveSwerve(new Vector2d(0,0), 0);
    System.out.println("ScanForGoal Stopped");
    Robot.scheduleCommand(new RotateToGoal(0.25));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (tapeDetected) {
        return true;
    } else {
        return false;
    }
  }
}