package team4391.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import team4391.robot.Constants;
import team4391.robot.Robot;

/**
 *
 */
public class ArmPushOutMedTimed extends Command {

    public ArmPushOutMedTimed(double timeoutSeconds) {
        // Use requires() here to declare subsystem dependencies
        
    	super(timeoutSeconds);
    	requires(Robot.armSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {    	
    	Robot.armSubsystem.setPushOut(Constants.kArmModeratlyFastOutputSpeed);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
