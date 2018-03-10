package team4391.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

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
    	
    	addSequential(new AutoDriveCenterToLeftSwitch(120.0, 0.6, 30.0));
    	addSequential(new ArmPushOutTimed(1.5));
    	addSequential(new DriveForDistance(24, -0.3, 0.0));
    	addSequential(new RotateDegrees(-65.0));
    	addSequential(new CubevatorToBottom());
    	addParallel(new ArmPullIn());
    	addSequential(new DriveForDistance(40, 0.2, 0.0));
    	
    	
    }
}
