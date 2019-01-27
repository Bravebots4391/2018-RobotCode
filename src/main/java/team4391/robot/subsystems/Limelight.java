/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team4391.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  static boolean _toggle = false;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void ToggleCamera()
  {
    int outVal = 0;
    int ledVal = 3;
    if(_toggle == true)
    {
        _toggle = false;
        outVal = 0;
        ledVal = 3;
    }
    else
    {
      _toggle = true;
      outVal = 1;
      ledVal = 1;
    }

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(outVal);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledVal);
  }
}
