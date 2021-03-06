/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class BackRightModule extends SwerveModule {
    public BackRightModule(CANSparkMax topGear, CANSparkMax bottomGear, AnalogPotentiometer absEncoder, boolean inverted) {
        super("BACK_RIGHT", topGear, bottomGear, absEncoder, inverted);
    }

    @Override
    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Back Right Encoder: ", this.currentAngle);
    }
}
