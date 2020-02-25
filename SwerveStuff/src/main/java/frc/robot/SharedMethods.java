/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.RobotContainer;

/**
 * Add your docs here.
 */
public class SharedMethods {
    public static double roundTo(double value, int places) {
        double scale = Math.pow(10, places);
        return Math.round(value * scale) / scale;
    }

    public static void customDelay(double time) {
        System.out.println("Starting Delay...");
        double initTimestamp = Timer.getFPGATimestamp();

        while (Timer.getFPGATimestamp() < initTimestamp + time) {
            RobotContainer.swerveGroup.moveSwerve(new Vector2d(0, 0), 0);
            //System.out.println("Delaying...: " + initTimestamp + " " + Timer.getFPGATimestamp() + " " + time);
        }
        System.out.println("Done Delaying");
    }

    public static double bearingToAngle(double bearing) {
        if (bearing >= 270) {
            return 90 + (90 - (bearing - (90 * ((int) (bearing / 90)))));
        }
        else if (bearing >= 180) {
            return 180 + (90 - (bearing - (90 * ((int) (bearing / 90)))));
        }
        else if (bearing >= 90) {
            return 270 + (90 - (bearing - (90 * ((int) (bearing / 90)))));
        }
        else {
            return (90 - (bearing - (90 * ((int) (bearing / 90)))));
        }
    }
}
