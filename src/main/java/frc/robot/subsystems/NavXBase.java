/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This is a class that manages a NavX. Copy-paste it, whatever The methods are
 * corrected to the robot orientation TODO: Have the methods return the correct axis
 * axis for their name: ie, correct for NavX orientation
 */
public class NavXBase extends Subsystem {
    AHRS NavX;

    /**
     * This constructs the navx: it may throw an exception if the NavX is not found
     */
    public NavXBase() {// PUT ARGUMENTS WHEN WE NEED THEM
        try {
            NavX = new AHRS(SPI.Port.kMXP);// TODO: Check port num
        } catch (RuntimeException ex) {
            // TODO: We need to set up a drivers system output system: perhaps Robot.java
            // should catch this exception
            // Or we could output to the driver system directly, as they do in their example
            // DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(),
            // true);
        }

    }

    @Override
    public void initDefaultCommand() {
        // runs every scheduler run
    }

    /**
     * Gets the pitch of the robot (X axis rotation) (pitch is rotation around the
     * horisontal axis perpendicular to straight forward)
     * 
     * @return The pitch of the robot
     */
    public double getPitch() {
        return NavX.getPitch();
    }

    /**
     * Gets the roll of the robot (Y axis rotation) (roll is the leaning around the
     * axis that goes straight forward)
     * 
     * @return
     */
    public double getRoll() {
        return NavX.getRoll();
    }

    /**
     * Gets the yaw of the robot (Z axis rotation) (yaw is the direction that the
     * robot is facing around an axis that shoots straight up)
     * 
     * @return
     */
    public double getYaw() {
        return NavX.getYaw();
    }

    /**
     * Zeroes the yaw of the robot
     * 
     * @return The previous yaw
     */
    public double zeroYaw() {
        double temporaryDouble = NavX.getYaw();
        NavX.zeroYaw();
        return temporaryDouble;
    }
}