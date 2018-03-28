package team4391.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import team4391.robot.Robot;

/**
 *
 */
public class AutoStrafeDropCubeGroup extends CommandGroup {
  
	private double _distance;
	private double _speed;
	private double _heading;
	
    public AutoStrafeDropCubeGroup(double distanceInches, double speedFps, double heading) {
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
        _distance = distanceInches;
        _speed = speedFps;
        _heading = heading;
                
        addSequential(new CubevatorBumpDown(0.05));
        addSequential(new StrafeForDistanceDropCube(_distance, _speed, _heading));
    }
}
