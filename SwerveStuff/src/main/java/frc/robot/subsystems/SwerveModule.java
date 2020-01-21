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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SharedMethods;

public class SwerveModule extends SubsystemBase {

  CANSparkMax topGear, bottomGear;
  AnalogPotentiometer absEncoder;
  double currentAngle = 0.0;
  double startTime = 0.0;
  double elapsedTime = 500;
  double zeroOffset = 0;

  double TRANSLATE_MOD = 0.5;
  double ROTATE_MOD = 0.1;
  double ERROR_BOUND = 3;
  String MODULE_ID = "";

  double topGearSpeed = 0, bottomGearSpeed = 0;

  public SwerveModule(String moduleID, CANSparkMax topGear, CANSparkMax bottomGear, AnalogPotentiometer absEncoder, boolean inverted) {
    assert moduleID.equals("FRONT_LEFT") || moduleID.equals("FRONT_RIGHT") || moduleID.equals("BACK_LEFT") || moduleID.equals("BACK_RIGHT");
    this.MODULE_ID = moduleID;

    this.topGear = topGear;
    this.bottomGear = bottomGear;
    this.absEncoder = absEncoder;

    if (inverted) {
      TRANSLATE_MOD *= -1;
      ROTATE_MOD *= -1;
    }
  }

  public void moveCrab(Vector2d translationVector, double rotation) {
    topGearSpeed = 0;
    bottomGearSpeed = 0;

    //cartesian translation
    if (Math.abs(translationVector.magnitude()) > Constants.JOYSTICK_DEAD_ZONE) {
      //get the desired angle
      double desiredAngle = (Math.atan2(translationVector.y, -translationVector.x) * (180/Math.PI)) + 180;

      turnToAngle(desiredAngle);

      topGearSpeed += -translationVector.magnitude() * TRANSLATE_MOD;
      bottomGearSpeed += translationVector.magnitude() * TRANSLATE_MOD;
    }
    else if (Math.abs(rotation) > Constants.JOYSTICK_DEAD_ZONE) {
      if (MODULE_ID.equals("FRONT_LEFT")) {
        turnToAngle(225);
        topGearSpeed += -rotation * TRANSLATE_MOD;
        bottomGearSpeed += rotation * TRANSLATE_MOD;
      }
      else if (MODULE_ID.equals("FRONT_RIGHT")) {
        turnToAngle(315); //TODO: may need to change if facing wrong direction
        topGearSpeed += -rotation * TRANSLATE_MOD;
        bottomGearSpeed += rotation * TRANSLATE_MOD;
      }
      else if (MODULE_ID.equals("BACK_LEFT")) {
        turnToAngle(315);
        topGearSpeed += -rotation * TRANSLATE_MOD;
        bottomGearSpeed += rotation * TRANSLATE_MOD;
      }
      else if (MODULE_ID.equals("BACK_RIGHT")) {
        turnToAngle(225);
        topGearSpeed += -rotation * TRANSLATE_MOD;
        bottomGearSpeed += rotation * TRANSLATE_MOD;
      }
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
  }

  public void turnToAngle(double desiredAngle) {
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
        //if it gets here, youre in deep shit
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
        //again, if it gets here, you're in deep shit
      }
    }
  }

  public void calibrate() {
    zeroOffset = currentAngle;
  }

  @Override
  public void periodic() {
    updateGearSpeeds();
    updateCurrentAngle();
    updateSmartDashboard();
  }

  public void updateCurrentAngle() {
    elapsedTime = (System.nanoTime() / 1000000) - startTime;
    if (elapsedTime >= 50) {
      startTime = System.nanoTime() / 1000000;

      //convert absolute encoder voltage to degrees and post to smartdashboard for testing
      currentAngle = (SharedMethods.roundTo(((absEncoder.get() - Constants.ENCODER_OFFSET) / 335) * 360, 0));

      //do if statement with 360 minus for negative numbers
      double newAngle = currentAngle - zeroOffset;

      if (newAngle < 0) {
        currentAngle = 360 + newAngle; //new angle is always negative so current angle = 360 - (a negative number)
      }
      else {
        currentAngle = newAngle;
      }
    }
  }

  public void updateSmartDashboard() {
    //override in module specific class
  }

  public void updateGearSpeeds() {
    topGear.set(topGearSpeed);
    bottomGear.set(bottomGearSpeed);
  }
}
