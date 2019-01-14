package team4391.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import team4391.robot.Robot;
import team4391.robot.subsystems.Drive.DriveState;

/**
 *
 */
public class RotateDegrees extends Command {

	private double _degrees;
	
    public RotateDegrees(double degrees) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveSubsystem);
        
        _degrees = degrees;
        
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("RotateDegrees Command Init");
    	
    	Robot.driveSubsystem.rotateDegrees(_degrees);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {    	
        return Robot.driveSubsystem.getDriveState() != DriveState.Rotate;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("RotateDegrees Command end");
    	Robot.driveSubsystem.setOpenLoop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
