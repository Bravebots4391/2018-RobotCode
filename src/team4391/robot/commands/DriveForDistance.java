package team4391.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import team4391.robot.Constants;
import team4391.robot.Robot;
import team4391.robot.subsystems.Drive.DriveState;

/**
 *
 */
public class DriveForDistance extends Command {

	private double _distance;
	private double _speed;
	private double _heading;
	
    public DriveForDistance(double distanceInches, double speedFps, double heading) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveSubsystem);
        
        _distance = distanceInches;
        _speed = speedFps;
        _heading = heading;              
    }

    // Called just before this Command runs the first time
    protected void initialize() {    	
    	String spdStr = String.format("%f", _speed);
    	
    	String init = "DriveForDistance Command Init ";
    	String all = init + spdStr;
    	
    	System.out.println(all);
    	Robot.driveSubsystem.driveForDistance(_distance, _speed, _heading);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {    	
        return Robot.driveSubsystem.getDriveState() != DriveState.DriveForDistance;
    }

    // Called once after isFinished returns true
    protected void end() {    	
    	System.out.println("DriveForDistance Command end");
    	Robot.driveSubsystem.setOpenLoop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
