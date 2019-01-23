/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team4391.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import team4391.robot.OI;
import team4391.robot.Robot;

public class TeleopArm extends Command {
  public TeleopArm() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.armSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    var up = OI.winchUp1.get() ? 1.0 : 0;
    var end = OI.winchDown1.get() ? -1.0 : 0;

    Robot.armSubsystem.setKnuckle(up + end);
    Robot.armSubsystem.setWrist(OI._xBoxCntrl.getRawAxis(3) - OI._xBoxCntrl.getRawAxis(2));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.armSubsystem.setWrist(0.0);
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
