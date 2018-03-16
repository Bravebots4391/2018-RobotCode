package team4391.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class LED extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	DigitalOutput do1 = new DigitalOutput(2);
	DigitalOutput do2 = new DigitalOutput(3);
	DigitalOutput do3 = new DigitalOutput(4);
	DigitalOutput do4 = new DigitalOutput(5);

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());    	
    }
    
    public void init()
    {
    	disable();
    }
    
    public void enable()
    {
    	do1.set(true);
    	do2.set(true);
    	do3.set(true);
    	do4.set(true);

    }
    
    public void disable()
    {
    	do1.set(false);
    	do2.set(false);
    	do3.set(false);
    	do4.set(false);
    }
}

