package team4391.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import team4391.robot.Constants;

/**
 *
 */
public class Climb extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	// Setup Talons here
	public TalonSRX _climbMotor = new TalonSRX(Constants.kClimberId);
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void init()
    {
    	_climbMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    public void Up()
    {
    	_climbMotor.set(ControlMode.PercentOutput, 1.0);
    }
    
    public void Down()
    {
    	_climbMotor.set(ControlMode.PercentOutput, -1.0);
    }
    
    public void Stop()
    {
    	_climbMotor.set(ControlMode.PercentOutput, 0.0);
    }

	public void updateDashboard() {
		// TODO Auto-generated method stub
		
	}
}

