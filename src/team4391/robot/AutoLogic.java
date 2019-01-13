package team4391.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4391.robot.commands.ArmPullIn;
import team4391.robot.commands.ArmPullInCmdGrpTest;
import team4391.robot.commands.Auto;
import team4391.robot.commands.AutoCenterToLeftSwitch;
import team4391.robot.commands.AutoCenterToRightSwitch;
import team4391.robot.commands.AutoGroupToScaleDump;
import team4391.robot.commands.AutoStrafeDropCubeGroup;
import team4391.robot.commands.AutoStrafeDropCubeGroup2;
import team4391.robot.commands.CubevatorBumpDown;
import team4391.robot.commands.DriveForDistance;
import team4391.robot.commands.ExampleCommand;
import team4391.robot.commands.StrafeForDistanceDropCube;

public class AutoLogic 
{
	private SendableChooser<String> _chooser = new SendableChooser<String>();
	private SendableChooser<String> _chooserPosition = new SendableChooser<String>();
	
	public void setupAutonomousChooser() 
	{	
		_chooser.addDefault("Default Auto", "switch");
		_chooser.addObject("Switch", "switch");
		_chooser.addObject("Scale", "scale");
		_chooser.addObject("Drive Over Line", "driveOverLine");
		_chooser.addObject("DFDTest", "DFDTest");
		_chooser.addObject("SFDTest", "SFDTest");
		_chooser.addObject("PullInTest", "PullInTest");
		_chooser.addObject("AutoGroupToScaleDump", "ScaleTest");
		
		SmartDashboard.putData("Auto mode", _chooser);
		
		_chooserPosition.addDefault("Default Position(Center)", "center");
		_chooserPosition.addObject("left", "left");
		_chooserPosition.addObject("center", "center");
		_chooserPosition.addObject("right", "right");
		_chooserPosition.addObject("Test Only", "test");
		
		SmartDashboard.putData("Auto Position", _chooserPosition);
	}
	
	public void runAuto()
	{
		String chooserVal = _chooser.getSelected();
		String chooserValPosition = _chooserPosition.getSelected();
	    String gameInfo = DriverStation.getInstance().getGameSpecificMessage();
		
	    boolean switchIsRight = gameInfo.charAt(0) == 'R';
	    boolean switchIsLeft = gameInfo.charAt(0) == 'L';
	    boolean scaleIsRight = gameInfo.charAt(1) == 'R';
	    boolean scaleIsLeft = gameInfo.charAt(1) == 'L';
	    
	    // Set the approximate starting height of the cubevator
	    Robot.cubevatorSubsystem.setPositionInches(Constants.kCubevatorStartHeightInches);
	    
	    // Send bump command to drop the arm
	    Command cm = new CubevatorBumpDown(0.05);
	    cm.start();
	    
		if(chooserValPosition == "center")
		{
			if(switchIsRight)
			{
				CommandGroup cg = new AutoCenterToRightSwitch();
				cg.start();
			}
			else
			{
				CommandGroup cg = new AutoCenterToLeftSwitch();
				cg.start();
			}

		}
		else if(chooserValPosition == "left")
		{
			if(switchIsLeft)
			{
				//CommandGroup cg = new AutoStrafeDropCubeGroup(180, 0.5, -90);
				CommandGroup cg = new AutoStrafeDropCubeGroup2(135, 0.5, -90);
				cg.start();
			}		
			else
			{
				// Drive over the line
				Command cg = new DriveForDistance(135, 0.5, -90);
				cg.start();
			}
		}
		else if(chooserValPosition == "right")
		{
			if(switchIsRight)
			{				
				//CommandGroup cg = new AutoStrafeDropCubeGroup(180, 0.5, 90);
				CommandGroup cg = new AutoStrafeDropCubeGroup2(135, 0.5, 90);
				cg.start();
			}
			else
			{
				// drive over the line?
				Command cg = new DriveForDistance(135, 0.5, 90);
				cg.start();
			}
		}
		else if(chooserValPosition == "test")
		{
			if(chooserVal == "DFDTest")
			{
		    	Preferences pref = Preferences.getInstance();
		    	double _distance = pref.getDouble("DFDDistance", Constants.DriveDistance);
		    	double _speed = pref.getDouble("DFDSpeed", Constants.Speed);
		    	double _heading = pref.getDouble("DFDHeading", Constants.Heading);
				Command cg = new DriveForDistance(_distance,_speed,_heading);
				cg.start();
			}
			else if(chooserVal == "SFDTest")
			{
		    	Preferences pref = Preferences.getInstance();
		    	double _distance = pref.getDouble("DFDDistance", Constants.DriveDistance);
		    	double _speed = pref.getDouble("DFDSpeed", Constants.Speed);
		    	double _heading = pref.getDouble("DFDHeading", Constants.Heading);
				Command cg = new StrafeForDistanceDropCube(_distance,_speed,_heading);
				cg.start();
			}
			else if(chooserVal == "ScaleTest")
			{
				Preferences pref = Preferences.getInstance();
		    	double _distance = pref.getDouble("DFDDistance", Constants.DriveDistance);
		    	double _speed = pref.getDouble("DFDSpeed", Constants.Speed);
		    	double _heading = pref.getDouble("DFDHeading", Constants.Heading);
				CommandGroup cg = new AutoGroupToScaleDump(_heading, _speed);
				cg.start();
			}
			else if(chooserVal == "PullInTest")
			{
				CommandGroup cmd = new ArmPullInCmdGrpTest();
				cmd.start();
			}
		}
	}
}
