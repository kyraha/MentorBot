// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import com.studica.frc.AHRS;

/** Add your docs here. */
public class NavX {
    private static AHRS chip = new AHRS(AHRS.NavXComType.kMXP_SPI);

    public static void reset(){
        chip.reset();
    }

    /**
     * get the angle in Rotation2D
     * @return the angle as a rotation 2d
     */
    public static Rotation2d getRotation2d() {
        return chip.getRotation2d();
    }
}
