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

  public SwerveModule(CANSparkMax topGear, CANSparkMax bottomGear, AnalogPotentiometer absEncoder) {
    this.topGear = topGear;
    this.bottomGear = bottomGear;
    this.absEncoder = absEncoder;
  }
  public void moveCrab(Vector2d translationVector, double rotation, double angle) {
    //initialize variables and constants
    double topGearSpeed = 0, bottomGearSpeed = 0;
    final double TRANSLATE_MOD = 0.5;
    final double ROTATE_MOD = 0.1;
    final double ERROR_BOUND = 3;

    //get the desired angle
    double desiredAngle = (Math.atan2(translationVector.y, -translationVector.x) * (180/Math.PI)) + 180;
    //double desiredAngle = angle;
    SmartDashboard.putNumber("Drive Stick (X): ", translationVector.x);
    SmartDashboard.putNumber("Drive Stick (Y): ", translationVector.y);
    SmartDashboard.putNumber("Desired Angle: ", desiredAngle);
    SmartDashboard.putNumber("Module Adjustment: ", ((desiredAngle - currentAngle) / 500) * ROTATE_MOD);
    SmartDashboard.putNumber("absEncoder: ", absEncoder.get());

    //cartesian translation
    /*
    if (Math.abs(translationVector.magnitude()) > Constants.JOYSTICK_DEAD_ZONE || desiredAngle != -1) {
      double error = desiredAngle - currentAngle;
      SmartDashboard.putNumber("Error: ", error);

      if (Math.abs(error) > ERROR_BOUND && Math.abs(error) < (360 - ERROR_BOUND)) {
        if (Math.abs(error) < 0) {
          topGearSpeed += Math.abs(error) / 500 * ROTATE_MOD;
          bottomGearSpeed += Math.abs(error) / 500 * ROTATE_MOD;
        }
        else {
          topGearSpeed += -Math.abs(error) / 500 * ROTATE_MOD;
          bottomGearSpeed += -Math.abs(error) / 500 * ROTATE_MOD;
        }
      }
      
      //topGearSpeed += translationVector.magnitude() * TRANSLATE_MOD;
      //bottomGearSpeed += -translationVector.magnitude() * TRANSLATE_MOD;
    }
    */
    //rotation
    //if (Math.abs(rotation) > Constants.JOYSTICK_DEAD_ZONE) {
    //  topGearSpeed += -rotation * ROTATE_MOD;
    //  bottomGearSpeed += -rotation * ROTATE_MOD;
    //}

    if (Math.abs(translationVector.magnitude()) > Constants.JOYSTICK_DEAD_ZONE)
    {
      topGearSpeed = translationVector.magnitude() * .1;
      bottomGearSpeed = translationVector.magnitude() * -.1;
    }

    //Turn to heading
    
    if (angle != -1 && Math.abs(translationVector.magnitude()) > Constants.JOYSTICK_DEAD_ZONE)
    {
      /*
        if (absEncoder.get() < desiredAngle - 5)
        {
            topGearSpeed =- .1 * ((absEncoder.get() - desiredAngle) / desiredAngle);
        }
        if (absEncoder.get() > desiredAngle + 5)
        {
            bottomGearSpeed =- .1 * ((desiredAngle - absEncoder.get()) / desiredAngle);
        }
      */
        if (currentAngle > desiredAngle + 5 )
        {
            topGearSpeed =- .001 * ((currentAngle - desiredAngle) / desiredAngle);
        }
        if (currentAngle < desiredAngle - 5 )
        {
            bottomGearSpeed =- .001 * ((desiredAngle - currentAngle) / desiredAngle);
        }
    }

    

    //Tank like turning
    /*
    if (Math.abs(translationVector.x) > Constants.JOYSTICK_DEAD_ZONE)
    {
      if (translationVector.x < 0)
      {
          topGearSpeed =- translationVector.x * .1;
      }
      else
      {
        bottomGearSpeed =- translationVector.x * .1;
      }
    }
    */

    SmartDashboard.putNumber("topGearSpeed:    ", topGearSpeed);
    SmartDashboard.putNumber("bottomGearSpeed: ", bottomGearSpeed);

    topGear.set(topGearSpeed);
    bottomGear.set(bottomGearSpeed);
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
