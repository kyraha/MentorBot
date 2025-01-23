// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.ConfigReader;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;

/** Add your docs here. */
public class IMU {
    private AHRS navX;
    private Pigeon2 pigeon;

    public IMU(ConfigReader config) {
        var type = config.getAsString("type");
        if (type.equalsIgnoreCase("navx")) {
            navX = new AHRS(AHRS.NavXComType.kMXP_SPI);
        }
        else if (type.equalsIgnoreCase("pigeon2")) {
            var pigeonCAN = config.getAsInt("can/id");
            var pigeonBus = config.getAsString("can/bus");
            pigeon = new Pigeon2(pigeonCAN, pigeonBus);
        }
    }

    /**
     * Resets the IMU to heading zero
     * Should not really be used, or used very rarely as it's an expensive operation
     * Instead: Get Rotation2d and store it as your "zero heading", and then use it as an offset later
     */
    public void reset(){
        if (navX != null) navX.reset();
        if (pigeon != null) pigeon.reset();
    }

    /**
     * get the angle in Rotation2D
     * @return the angle as a rotation 2d
     */
    public Rotation2d getRotation2d() {
        if (pigeon != null) return pigeon.getRotation2d();
        if (navX != null) return navX.getRotation2d();
        return new Rotation2d();
    }
}
