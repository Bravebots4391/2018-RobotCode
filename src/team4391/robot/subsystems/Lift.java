package team4391.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4391.robot.Constants;
import team4391.robot.commands.LiftStop;

/**
 *
 */
public class Lift extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	TalonSRX _cubevatorTalon = new TalonSRX(Constants.kCubevatorId);
	

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new LiftStop());
    }

	public void up() {
		_cubevatorTalon.set(ControlMode.PercentOutput, -1);
		
	}
	
	public void down() {
		_cubevatorTalon.set(ControlMode.PercentOutput, .75);
		
	}

	public void stop() {		
		_cubevatorTalon.set(ControlMode.PercentOutput, 0.0);
	}
}

//public void updateDashboard() {
//	 SmartDashboard.getNumber(key, defaultValue);
//}

