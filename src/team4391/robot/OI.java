/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team4391.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4391.robot.commands.ArmPullIn;
import team4391.robot.commands.ArmPushOut;
import team4391.robot.commands.CubevatorClimbHeight;
import team4391.robot.commands.CubevatorDefaultHeight;
import team4391.robot.commands.DriveForDistance;
import team4391.robot.commands.FastArmPushOut;
import team4391.robot.commands.LiftDown;
import team4391.robot.commands.LiftUp;
import team4391.robot.commands.McTwist180;
import team4391.robot.commands.WinchDown;
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
	
	public static Joystick _xBoxCntrl = new Joystick(0);
    public static Joystick _xBoxCntrl2 = new Joystick(1);
    	
	Button inny = new JoystickButton(_xBoxCntrl, 4);
	Button outty = new JoystickButton(_xBoxCntrl, 2);
    Button liftUp = new JoystickButton(_xBoxCntrl, 7);
    Button liftDown= new JoystickButton(_xBoxCntrl,8);
    Button fastoutty = new JoystickButton(_xBoxCntrl,3);
    
	Button inny2 = new JoystickButton(_xBoxCntrl2, 4);
	Button outty2 = new JoystickButton(_xBoxCntrl2, 2);
    Button liftUp2 = new JoystickButton(_xBoxCntrl2, 5);
    Button liftDown2= new JoystickButton(_xBoxCntrl2,6);
    Button fastoutty2 = new JoystickButton(_xBoxCntrl2,3);
    Button climbheight = new JoystickButton(_xBoxCntrl2, 7);
    
    Button winchUp1 = new XboxControllerPOVButton(xb, 0);
    Button winchDown1 = new XboxControllerPOVButton(xb, 180);
    
    //Button mcTwist = new XboxControllerPOVButton(xb, 90);
    
	public void init() {
		
		climbheight.whileHeld(new CubevatorClimbHeight());	
		inny.whileHeld(new ArmPullIn());		
		outty.whileHeld(new ArmPushOut());
		fastoutty.whileHeld(new FastArmPushOut());
		liftUp.whileHeld(new LiftUp());;
		liftDown.whileHeld(new LiftDown());
		
		
		inny2.whileHeld(new ArmPullIn());		
		outty2.whileHeld(new ArmPushOut());
		fastoutty2.whileHeld(new FastArmPushOut());
		liftUp2.whileHeld(new LiftUp());;
		liftDown2.whileHeld(new LiftDown());
		
		winchUp1.whileHeld(new WinchUp());
		winchDown1.whileHeld(new WinchDown());
		
		//mcTwist.whenPressed(new McTwist180());
		
		SmartDashboard.putData("DriveForDistance", new DriveForDistance(290.0, 0.5, 0.0));		
		SmartDashboard.putData("CubevatorDefault", new CubevatorDefaultHeight());
	}
}
