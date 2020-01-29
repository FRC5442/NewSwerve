/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SharedMethods;

public class SwerveModule extends SubsystemBase {

  CANSparkMax topGear, bottomGear;
  AnalogPotentiometer absEncoder;
  CANEncoder topEncoder, bottomEncoder;
  double currentAngle = 0.0;
  double rawAngle = 0.0;
  double startTime = 0.0;
  double elapsedTime = 500;
  double zeroOffset = 0;

  double TRANSLATE_MOD = 0.5;
  double ROTATE_MOD = 0.25;
  double ERROR_BOUND = 5;

  double topGearSpeed = 0;
  double bottomGearSpeed = 0;

  public SwerveModule(CANSparkMax topGear, CANSparkMax bottomGear, AnalogPotentiometer absEncoder, boolean inverted) {

    this.topGear = topGear;
    this.bottomGear = bottomGear;
    this.absEncoder = absEncoder;

    topEncoder = topGear.getEncoder();
    topEncoder.setVelocityConversionFactor(360);
    bottomEncoder = bottomGear.getEncoder();
    bottomEncoder.setVelocityConversionFactor(360);

    if (inverted) {
      TRANSLATE_MOD *= -1;
      ROTATE_MOD *= -1;
    }
  }
  
  public void move(double speed, double angle) {
    topGearSpeed = 0;
    bottomGearSpeed = 0;

    turnToAngle(angle);

    topGearSpeed += (-speed * TRANSLATE_MOD);
    bottomGearSpeed += (speed * TRANSLATE_MOD);
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
    }
  }

  public void calibrate() {
    zeroOffset = rawAngle;
  }

  public void stop() {
    topGearSpeed = 0;
    bottomGearSpeed = 0;
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
      rawAngle = (SharedMethods.roundTo(((absEncoder.get() - Constants.ENCODER_OFFSET) / 335) * 360, 0));

      //do if statement with 360 minus for negative numbers
      double newAngle = rawAngle - zeroOffset;

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
