/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.RobotMap;

public class DriveSequentialForwardCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public DriveSequentialForwardCommand(double inches) {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand((int) inches * RobotMap.encoderTicksPerInch));
    addSequential(new DriveStopCommand());
    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }

  public DriveSequentialForwardCommand(double start, double end){
    addSequential(new WaitCommand(.1));
    addSequential(new DriveForwardCommand((int) Math.round(Math.abs(end - start) * RobotMap.encoderTicksPerInch)));
    addSequential(new DriveStopCommand());
  }
}
