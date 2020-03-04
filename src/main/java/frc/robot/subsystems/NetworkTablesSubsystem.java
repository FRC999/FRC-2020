/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class NetworkTablesSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  NetworkTableInstance ntInst = NetworkTableInstance.getDefault();

  public void startNT(Boolean server, int team) {
    if (server) {
      System.out.println("Setting up NetworkTables server");
      ntInst.startServer();
    } else {
      System.out.println("Setting up NetworkTables client for team " + team);
      ntInst.startClientTeam(team);
    }

  }

  public void useTable(String x) {
    ntInst.getTable(x);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public enum Location {
    ROOT("Root"),
    LIVEWINDOW("Livewindow"), 
    TESTTABLE("TestTable");
 
    private String table;
 
    Location(String envUrl) {
        this.table = envUrl;
    }
 
    public String getLocation() {
        return table;
    }
  }
}