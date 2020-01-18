/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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
  double currentAngle = 0.0;
  double startTime = 0.0;
  double elapsedTime = 500;

  final double TRANSLATE_MOD = 0.5;
  final double ROTATE_MOD = 0.1;
  final double ERROR_BOUND = 5;

  public SwerveModule(CANSparkMax topGear, CANSparkMax bottomGear, AnalogPotentiometer absEncoder) {
    this.topGear = topGear;
    this.bottomGear = bottomGear;
    this.absEncoder = absEncoder;
  }
  public void moveCrab(Vector2d translationVector, double rotation) {
    //initialize variables and constants
    double topGearSpeed = 0, bottomGearSpeed = 0;

    SmartDashboard.putNumber("Joystick Magnitude: ", translationVector.magnitude());

    //cartesian translation
    if (Math.abs(translationVector.magnitude()) > Constants.JOYSTICK_DEAD_ZONE) {
      //get the desired angle
      double desiredAngle = (Math.atan2(translationVector.y, -translationVector.x) * (180/Math.PI)) + 180;
      SmartDashboard.putNumber("Desired Angle: ", desiredAngle);

      double[] angleGearSpeeds = turnToAngle(desiredAngle);

      topGearSpeed += angleGearSpeeds[0];
      bottomGearSpeed += angleGearSpeeds[1];

      topGearSpeed += -translationVector.magnitude() * TRANSLATE_MOD;
      bottomGearSpeed += translationVector.magnitude() * TRANSLATE_MOD;
    }

    //rotation
    if (Math.abs(rotation) > Constants.JOYSTICK_DEAD_ZONE) {
      topGearSpeed += -rotation * ROTATE_MOD;
      bottomGearSpeed += -rotation * ROTATE_MOD;
    }

    //set minimum gear speeds
    if (topGearSpeed > 0 && topGearSpeed < 0.02) { topGearSpeed = 0.02; }
    if (topGearSpeed < 0 && topGearSpeed > -0.02) { topGearSpeed = -0.02; }
    if (bottomGearSpeed > 0 && bottomGearSpeed < 0.02) { bottomGearSpeed = 0.02; }
    if (bottomGearSpeed < 0 && bottomGearSpeed > -0.02) { bottomGearSpeed = -0.02; }

    topGear.set(topGearSpeed);
    bottomGear.set(bottomGearSpeed);
  }

  private double[] turnToAngle(double desiredAngle) {
    double topGearSpeed = 0, bottomGearSpeed = 0;

    //get the error
    double error = 0;

    if (desiredAngle > currentAngle) {
      error = desiredAngle - currentAngle;
      if (error < 180) {
        //move D by increasing C
        topGearSpeed += Math.abs(error) / 100 * ROTATE_MOD;
        bottomGearSpeed += Math.abs(error) / 100 * ROTATE_MOD;
      }
      else if (error >= 180) {
        //move towards D by decreasing C
        topGearSpeed += -Math.abs(360 - error) / 100 * ROTATE_MOD;
        bottomGearSpeed += -Math.abs(360 - error) / 100 * ROTATE_MOD;
      }
      else {
        //idk man, shit broke
      }
    }
    else if (desiredAngle < currentAngle) {
      error = currentAngle - desiredAngle;
      if (error < 180) {
        //move towards D decreasing C
        topGearSpeed += -Math.abs(error) / 100 * ROTATE_MOD;
        bottomGearSpeed += -Math.abs(error) / 100 * ROTATE_MOD;
      }
      else if (error >= 180) {
        //move towards D by increasing C
        topGearSpeed += Math.abs(360 - error) / 100 * ROTATE_MOD;
        bottomGearSpeed += Math.abs(360 - error) / 100 * ROTATE_MOD;
      }
      else {
        //see above
      }
    }

    return new double[] {topGearSpeed, bottomGearSpeed};
  }

  @Override
  public void periodic() {
    updateCurrentAngle();
  }

  public void updateCurrentAngle() {
    elapsedTime = (System.nanoTime() / 1000000) - startTime;
    SmartDashboard.putNumber("Elapsed Time: ", elapsedTime);
    if (elapsedTime >= 50) {
      startTime = System.nanoTime() / 1000000;

      //convert absolute encoder voltage to degrees and post to smartdashboard for testing
      currentAngle = SharedMethods.roundTo(((absEncoder.get() - Constants.ENCODER_OFFSET) / 335) * 360, 0);
      SmartDashboard.putNumber("Current Angle: ", currentAngle);
    }
  }
}
