/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  CANSparkMax topGear, bottomGear;

  public SwerveModule(CANSparkMax topGear, CANSparkMax bottomGear) {
    this.topGear = topGear;
    this.bottomGear = bottomGear;
  }

  public void moveCrab(Vector2d translationVector, double rotation) {
    //double currentAngle = absEncoder.get();

    double topGearSpeed = 0, bottomGearSpeed = 0;
    final double DEAD_ZONE = Constants.JOYSTICK_DEAD_ZONE;
    final double TRANSLATION_MOD = 0.5;
    final double ROTATE_MOD = 0.1;

    if (Math.abs(translationVector.magnitude()) > DEAD_ZONE) {
      topGearSpeed += translationVector.magnitude() * TRANSLATION_MOD;
      bottomGearSpeed += -translationVector.magnitude() * TRANSLATION_MOD;
    }

    if (Math.abs(rotation) > DEAD_ZONE) {
      topGearSpeed += -rotation * ROTATE_MOD;
      bottomGearSpeed += -rotation * ROTATE_MOD;
    }

    topGear.set(topGearSpeed);
    bottomGear.set(bottomGearSpeed);
  }

  @Override
  public void periodic() {
  }
}
