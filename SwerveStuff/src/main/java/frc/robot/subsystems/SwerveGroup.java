/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  public void moveSwerve(Vector2d translation, double rotation) {
    double FWD = translation.y;
    double STR = -translation.x;
    double RCW = rotation;

    double A = STR - RCW * (Constants.ROBOT_LENGTH / Constants.ROBOT_RADIUS);
    double B = STR + RCW * (Constants.ROBOT_LENGTH / Constants.ROBOT_RADIUS);
    double C = FWD - RCW * (Constants.ROBOT_WIDTH / Constants.ROBOT_RADIUS);
    double D = FWD + RCW * (Constants.ROBOT_WIDTH / Constants.ROBOT_RADIUS);

    double ws1 = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2)); 
    double wa1 = Math.atan2(B, C) * (180 / Math.PI);

    double ws2 = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2)); 
    double wa2 = Math.atan2(B, D) * (180 / Math.PI);

    double ws3 = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2)); 
    double wa3 = Math.atan2(A, D) * (180 / Math.PI);

    double ws4 = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2)); 
    double wa4 = Math.atan2(A, C) * (180 / Math.PI);
  }

  public void calibrate() {
    frontLeftModule.calibrate();
    backRightModule.calibrate();
  }

  @Override
  public void periodic() {
  }
}
