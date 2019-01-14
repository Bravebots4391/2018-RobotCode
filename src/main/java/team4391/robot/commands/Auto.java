package team4391.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Auto extends CommandGroup {

    public Auto() {

    	addSequential(new DriveForDistance(200, .8, 0));
    	
    	// add additional commands as needed.
    	
    	// see http://wpilib.screenstepslive.com/s/currentCS/m/java/l/599738-creating-groups-of-commands 
    	// for more information about creating command groups.
    }
}
