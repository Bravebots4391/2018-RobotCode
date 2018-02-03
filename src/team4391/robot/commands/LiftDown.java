package team4391.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import team4391.robot.Robot;

/**
 *
 */
public class LiftDown extends Command {

    public LiftDown() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	  requires(Robot.cubevatorSubsystem);
    }
    

    // Called just before this Command runs the first time
    protected void initialize() {
}

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.cubevatorSubsystem.down();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.cubevatorSubsystem.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
