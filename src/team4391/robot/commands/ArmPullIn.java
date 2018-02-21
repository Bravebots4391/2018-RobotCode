package team4391.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import team4391.robot.Robot;

/**
 *
 */
public class ArmPullIn extends Command {

    public ArmPullIn() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.armSubsystem);
        requires(Robot.cubevatorSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.armSubsystem.setPullIn();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        
    	boolean isCubeSensed = Robot.armSubsystem.isCubeSensed();

    	if(isCubeSensed)
    	{
    		CubevatorDefaultHeight cmd = new CubevatorDefaultHeight();
    		cmd.start();
    	}
        
    	return isCubeSensed;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.armSubsystem.setHolding();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
