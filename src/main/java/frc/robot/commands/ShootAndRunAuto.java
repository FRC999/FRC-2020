/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap;

public class ShootAndRunAuto extends CommandGroup {

  //TODO: Add shooting code to autonomous sequence

  public static int startingRotation = 0;
  // 1 is pointing towards the target, 2 is 90 degrees to the right of 1, 3 is 180 degrees, 4 is 270 degrees

  public static double xPositionIn = 0;
  public static double yPositionIn = 0;
  // X position is distance from the starting line (negative X pointing towards the target)
  /* Y position is distance from the far wall of the field -- the side with the other team's ball dispenser
      Field is 323.25 in tall
      228.59 when starting in the big shot zone
      276.75 - RobotMap.robotLength / 2 (259.25) when the front is aligned with the outer line of the trench
      161.75 when the center of the robot is aligned with the center of the starting line*/

  public ShootAndRunAuto(double startingYPosition, int initialRotation, boolean shootingFromCenter) {

    yPositionIn = startingYPosition;
    startingRotation = initialRotation;
    if(startingRotation != 1 && startingRotation != 2 && startingRotation != 3 && startingRotation != 4){
      System.out.println("Auto: Something's wrong with initial rotation; should be 1/2/3/4");
    }

    /* Robot shoots its pre-loaded balls */
    addSequential(new DriveSequentialZeroEncodersCommand());
    if(shootingFromCenter == true) {
      if(yPositionIn == 228.59) {
        // — — — — PUT SHOOTING CODE (for sweetspot) HERE — — — —
      } else {
        if(startingRotation == 4 && yPositionIn > 228.59 || startingRotation == 2 && yPositionIn < 228.59){
          addSequential(new DriveSequentialForwardCommand(228.59, yPositionIn));
          yPositionIn = 228.59;
        } else {System.out.println("Auto: Something's wrong with initial position");}
        // — — — — PUT SHOOTING CODE (for sweetspot) HERE — — — —
      }
    } else {
      // — — — — PUT SHOOTING CODE (for non-sweetspot / angled) HERE — — — —
    }
    
    /* Robot gets out of the way */
    switch(startingRotation) {
      case 1:
        addSequential(new DriveSequentialTurnCommand(-30));
        break;
      case 2:
        addSequential(new DriveSequentialTurnCommand(-120));
        break;
      case 3:
        addSequential(new DriveSequentialTurnCommand(120));
        break;
      case 4:
        addSequential(new DriveSequentialTurnCommand(60));
        break;
    }
    addSequential(new DriveSequentialForwardCommand(80));
    addSequential(new DriveSequentialTurnCommand(-150));
  }

  public ShootAndRunAuto(double startingYPosition, int initialRotation, boolean shootingFromCenter, double yPositionBuddy) {

    yPositionIn = startingYPosition;
    startingRotation = initialRotation;
    if(startingRotation != 1 && startingRotation != 2 && startingRotation != 3 && startingRotation != 4){
      System.out.println("Auto: Something's wrong with initial rotation; should be 1/2/3/4");
    }

    /* Robot shoots its pre-loaded balls */
    addSequential(new DriveSequentialZeroEncodersCommand());
    if(shootingFromCenter == true) {
      if(yPositionIn == 228.59) {
        // — — — — PUT SHOOTING CODE (for sweetspot) HERE — — — —
      } else {
        if(startingRotation == 4 && yPositionIn > 228.59 || startingRotation == 2 && yPositionIn < 228.59){
          addSequential(new DriveSequentialForwardCommand(228.59, yPositionIn));
          yPositionIn = 228.59;
        } else {System.out.println("Auto: Something's wrong with initial position");}
        // — — — — PUT SHOOTING CODE (for sweetspot) HERE — — — —
      }
    } else {
      // — — — — PUT SHOOTING CODE (for non-sweetspot / angled) HERE — — — —
    }
    
    /* Robot positions itself to push non-autonomous friend */
    
    switch(startingRotation) {
       case 1:
        addSequential(new DriveSequentialTurnCommand(180));
        break;
      case 2:
        addSequential(new DriveSequentialTurnCommand(90));
        break;
       case 3:
        break;
       case 4:
         addSequential(new DriveSequentialTurnCommand(-90));
         break;
    }
    
    addSequential(new DriveSequentialForwardCommand(50, xPositionIn));
    xPositionIn = 50;

    if(yPositionBuddy > yPositionIn) {
      /* Continues positioning */
      addSequential(new DriveSequentialTurnCommand(-90));
      addSequential(new DriveSequentialForwardCommand(yPositionBuddy, yPositionIn));
      yPositionIn = yPositionBuddy;
      addSequential(new DriveSequentialTurnCommand(-90));

      /* Robot pushes poor non-autonomous friend*/
      addSequential(new DriveSequentialForwardCommand(xPositionIn * 3)); // 3 is a bit of an arbitrary multiplier; might be a good idea to test it

      /* Robot positions itself for game start*/
      addSequential(new DriveSequentialForwardCommand(-10));
      addSequential(new DriveSequentialTurnCommand(-90));
      addSequential(new DriveSequentialForwardCommand(RobotMap.robotLength));
      addSequential(new DriveSequentialTurnCommand(-90));
      addSequential(new DriveSequentialForwardCommand(RobotMap.robotLength));

    } else {
      /* Continues positioning */
      addSequential(new DriveSequentialTurnCommand(90));
      addSequential(new DriveSequentialForwardCommand(yPositionBuddy, yPositionIn));
      yPositionIn = yPositionBuddy;
      addSequential(new DriveSequentialTurnCommand(90));

      /* Robot pushes poor non-autonomous friend*/
      addSequential(new DriveSequentialForwardCommand(xPositionIn * 3)); // 3 is a bit of an arbitrary multiplier; might be a good idea to test it

      /* Robot positions itself for game start*/
      addSequential(new DriveSequentialForwardCommand(-10));
      addSequential(new DriveSequentialTurnCommand(90));
      addSequential(new DriveSequentialForwardCommand(RobotMap.robotLength));
      addSequential(new DriveSequentialTurnCommand(90));
      addSequential(new DriveSequentialForwardCommand(RobotMap.robotLength));
    }
  }

  @Override 
  public void initialize() {
    System.out.println("++++++++++ AUTO INIT ++++++++++");
    super.initialize();
  }
}