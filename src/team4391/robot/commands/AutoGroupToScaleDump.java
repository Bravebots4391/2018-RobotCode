package team4391.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoGroupToScaleDump extends CommandGroup {

    public AutoGroupToScaleDump(double heading, double rotate) {
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
        
    	
    	
    	addSequential(new DriveForDistance(268, 0.6, heading));    	
    	addSequential(new CubevatorClimbHeight());
    	addSequential(new RotateDegrees(rotate));
    	addSequential(new ArmPushOutFastTimed(1.5));
    	//addSequential(new DriveForDistance(14, 0.3, 180));
    	
    	addSequential(new RotateDegrees(-rotate));
    	addSequential(new RotateDegrees(75*-Math.signum(rotate)));
    	addSequential(new CubevatorToBottom());
    	addParallel(new ArmPullIn(false));
    	addSequential(new DriveForDistance(70, 0.4, 0, 2.0));
    	
    	addSequential(new DriveForDistance(60, -0.4, 0));
    	addSequential(new RotateDegrees(75*Math.signum(rotate)));
    	addSequential(new RotateDegrees(rotate));
    	addSequential(new CubevatorClimbHeight());
    	addSequential(new RotateDegrees(rotate));
    	addSequential(new ArmPushOutFastTimed(1.5));
    	
    }
}
