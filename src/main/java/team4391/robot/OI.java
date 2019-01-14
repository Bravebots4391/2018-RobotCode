/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team4391.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4391.robot.commands.ArmPullIn;
import team4391.robot.commands.ArmPullIn2;
import team4391.robot.commands.ArmPushOut;
import team4391.robot.commands.CubevatorBumpDown;
import team4391.robot.commands.CubevatorDefaultHeight;
import team4391.robot.commands.DriveForDistance;
import team4391.robot.commands.FastArmPushOut;
import team4391.robot.commands.LiftDown;
import team4391.robot.commands.LiftUp;
import team4391.robot.commands.McTwist180;
import team4391.robot.commands.RotateDegrees;
import team4391.robot.commands.WinchUp;
import team4391.util.XboxControllerPOVButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
			
	public static XboxController xb = new XboxController(0);
	public static XboxController xb2 = new XboxController(1);
	
	public static Joystick _xBoxCntrl = new Joystick(0);
    public static Joystick _xBoxCntrl2 = new Joystick(1);
    	
	Button inny = new JoystickButton(_xBoxCntrl, 2);
	Button outty = new JoystickButton(_xBoxCntrl, 4);
    //Button liftUp = new JoystickButton(_xBoxCntrl, 5);
    //Button liftDown= new JoystickButton(_xBoxCntrl, 6);
    Button fastoutty = new JoystickButton(_xBoxCntrl, 3);
    Button IntakeCube = new JoystickButton(_xBoxCntrl, 1);
    Button defaultheight = new JoystickButton(_xBoxCntrl, 7);
    
	Button inny2 = new JoystickButton(_xBoxCntrl2, 2);
	Button outty2 = new JoystickButton(_xBoxCntrl2, 4);
    Button liftUp2 = new JoystickButton(_xBoxCntrl2, 5);
    Button liftDown2= new JoystickButton(_xBoxCntrl2,6);
    Button fastoutty2 = new JoystickButton(_xBoxCntrl2,3);
    Button IntakeCube2 = new JoystickButton(_xBoxCntrl2, 1);
    
    Button winchUp1 = new XboxControllerPOVButton(xb, 0);
    Button winchDown1 = new XboxControllerPOVButton(xb, 180);
//    Button winchUp2 = new XboxControllerPOVButton(xb2, 0);
//    Button winchDown2 = new XboxControllerPOVButton(xb2, 180);
    
    //Button mcTwist = new XboxControllerPOVButton(xb, 90);
    
	public void init() {
		
		//IntakeCube.whenPressed(new team4391.robot.commands.IntakeCube());
		defaultheight.whenPressed(new CubevatorDefaultHeight());
		IntakeCube.whenPressed(new ArmPullIn());
		inny.whileHeld(new ArmPullIn2());
		outty.whileHeld(new ArmPushOut());
		fastoutty.whileHeld(new FastArmPushOut());
		//liftUp.whileHeld(new LiftUp());
		//liftDown.whileHeld(new LiftDown());
		
		// Toggle the cube sucker in
		IntakeCube2.whenPressed(new ArmPullIn());
		//inny2.toggleWhenPressed(new ArmPullIn());
		inny2.whileHeld(new ArmPullIn2());		
		outty2.whileHeld(new ArmPushOut());
		fastoutty2.whileHeld(new FastArmPushOut());
		liftUp2.whileHeld(new LiftUp());
		liftDown2.whileHeld(new LiftDown());
		
		winchUp1.whileHeld(new WinchUp());
		//winchDown1.whileHeld(new WinchDown());
		//winchUp2.whileHeld(new WinchUp());
		//winchDown2.whileHeld(new WinchDown());
		
		//mcTwist.whenPressed(new McTwist180());
		
    	Preferences pref = Preferences.getInstance();
    	double _distance = pref.getDouble("DFDDistance", Constants.DriveDistance);
    	double _speed = pref.getDouble("DFDSpeed", Constants.Speed);
    	double _heading = pref.getDouble("DFDHeading", Constants.Heading);
		
		SmartDashboard.putData("DriveForDistance", new DriveForDistance(_distance, _speed, _heading));		
		SmartDashboard.putData("CubevatorDefault", new CubevatorDefaultHeight());
		SmartDashboard.putData("Rotate90", new RotateDegrees(85.0));	
		SmartDashboard.putData("mcTwist", new McTwist180());
		SmartDashboard.putData("Bump", new CubevatorBumpDown(.02));
	}
}
