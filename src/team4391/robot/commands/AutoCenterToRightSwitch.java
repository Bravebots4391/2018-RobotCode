package team4391.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import team4391.robot.Constants;

/**
 *
 */
public class AutoCenterToRightSwitch extends CommandGroup {

    public AutoCenterToRightSwitch() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	
    	addSequential(new CubevatorBumpDown(0.05));
    	addSequential(new AutoDriveCenterToLeftSwitch(115.0, 0.7, 22.0));
    	addSequential(new ArmPushOutTimed(1.0));
    	addSequential(new DriveForDistance(28, -0.4, 0.0));
    	addSequential(new RotateDegrees(-Constants.kCenterAutoSecondTurn));
    	addSequential(new CubevatorToBottom(2.0));
    	addParallel(new ArmPullIn(false));
    	addSequential(new DriveForDistance(44, 0.4, 0.0, 2.5));
    	
    	// Backup and add the new cube to the switch
    	addSequential(new DriveForDistance(44, -0.4, 0.0));
    	addSequential(new CubevatorDefaultHeight());
    	addSequential(new RotateDegrees(Constants.kCenterAutoSecondTurn));
    	addSequential(new DriveForDistance(28, 0.4, 0.0));
    	addSequential(new ArmPushOutTimed(1.0));
    	
    	addSequential(new CubevatorDefaultHeight());  

    }
}
