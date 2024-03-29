package team4391.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import team4391.robot.Robot;

/**
 *
 */
public class McTwist180 extends Command {

	
    public McTwist180() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveSubsystem.setupMcTwist(0.4, 1.0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveSubsystem.stopMotors();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
