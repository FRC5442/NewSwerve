/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SharedMethods;

public class SwerveModule extends SubsystemBase {

  CANSparkMax topGear, bottomGear;
  AnalogPotentiometer absEncoder;

  public SwerveModule(CANSparkMax topGear, CANSparkMax bottomGear, AnalogPotentiometer absEncoder) {
    this.topGear = topGear;
    this.bottomGear = bottomGear;
    this.absEncoder = absEncoder;
  }
  public void moveCrab(Vector2d translationVector, double rotation) {
    //initialize variables and constants
    double topGearSpeed = 0, bottomGearSpeed = 0;
    final double TRANSLATE_MOD = 0.5;
    final double ROTATE_MOD = 0.1;

    //convert absolute encoder voltage to degrees and post to smartdashboard for testing
    double currentEncVoltage = SharedMethods.roundTo(absEncoder.get(), 2) - Constants.ENCODER_OFFSET;
    double currentBearing = ((currentEncVoltage / 0.93) * 359);
    double currentAngle = SharedMethods.bearingToAngle(currentBearing);
    SmartDashboard.putNumber("Absolute Encoder: ", currentBearing);

    //cartesian translation
    if (Math.abs(translationVector.magnitude()) > Constants.JOYSTICK_DEAD_ZONE) {
      //get the desired angle
      double desiredAngle = (Math.atan(translationVector.y/translationVector.x) * (180/Math.PI));
      SmartDashboard.putNumber("Desired Angle: ", desiredAngle);
      SmartDashboard.putNumber("Module Adjustment: ", ((desiredAngle - currentAngle) / 100));
      
      topGearSpeed += translationVector.magnitude() * TRANSLATE_MOD;
      bottomGearSpeed += -translationVector.magnitude() * TRANSLATE_MOD;
    }

    //rotation
    if (Math.abs(rotation) > Constants.JOYSTICK_DEAD_ZONE) {
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
