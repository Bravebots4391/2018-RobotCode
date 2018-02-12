package team4391.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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
	
	public Lift()
	{
		init();
	}

	public void init()
	{
		_cubevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		
		_cubevatorTalon.setSensorPhase(true); // Change this to false if not counting positive numbers when going up
			
		// May want to enable a "soft" limit on distance
//		_cubevatorTalon.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
//		int softLimitPosition = getEncoderPositionFromInches(Constants.kCubevatorTopLimitInches);
//		_cubevatorTalon.configForwardSoftLimitThreshold(softLimitPosition, Constants.kTimeoutMs);			
	}	
	
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
	
	public void resetPosition()
	{		
		_cubevatorTalon.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
	}
	
	public int getEncoderPositionFromInches(double inches)
	{
		double revolutions = inches / (Constants.kCubevatorDrumDiameterInches * Math.PI);
		double position = revolutions * (double)Constants.kCubevatorEncoderCountsPerRev;
		
		return (int)position;
	}
	
	public double getHeightInches()
	{
		int position = _cubevatorTalon.getSelectedSensorPosition(0);
		
		double revolutions = ((double)position / (double)Constants.kCubevatorEncoderCountsPerRev);
		double inches = (revolutions * (Constants.kCubevatorDrumDiameterInches * Math.PI));
		
		return inches;
	}
	
	public void updateDashboard() {
		 SmartDashboard.putNumber("CubevatorHeightInches", getHeightInches());
	}
}



