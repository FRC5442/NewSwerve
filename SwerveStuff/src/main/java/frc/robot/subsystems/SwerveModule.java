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

import com.revrobotics.*;

public class SwerveModule extends SubsystemBase {

  CANSparkMax topGear, bottomGear;

  public SwerveModule(CANSparkMax topGear, CANSparkMax bottomGear) {
    this.topGear = topGear;
    this.bottomGear = bottomGear;
  }

  public void moveCrab(Vector2d translationVector, double rotation) {
    //double currentAngle = absEncoder.get();

    double speed = translationVector.magnitude();

    if (Math.abs(speed) > 0.25) {
      topGear.set((translationVector.magnitude() * 0.5) + (rotation));
      bottomGear.set((-translationVector.magnitude() * 0.5) + (rotation));
    }
  }

  @Override
  public void periodic() {
  }
}
