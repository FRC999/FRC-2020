/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class DriveTurnAbsoluteCommand extends CommandGroup {

  static double curOrientation = Robot.navXSubsystem.getYaw();
  /**
   * Add your docs here.
   */
  public DriveTurnAbsoluteCommand(int target) {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.
    /**
     * Okay, so here is what I have figured out
     * This is a java constructor: so my technique wont work for primitive double
     * It might work for an updating object: basically, what will be happening is that
     * the CommandGroup code is tracking not the current state of curOrientation, but rather
     * the state that it was during initialization.
     * We might be able to accomplish this with the Double type (capital D), but I am not certain.
     * This code will defiantly not work, however.
     * - Calum McConnell
     */
    addSequential(new DriveTurnCommand(target-curOrientation));
    

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
  public void initialize(){
    curOrientation = Robot.navXSubsystem.getYaw();
  }
  
}
