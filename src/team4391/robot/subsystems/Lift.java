package team4391.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4391.robot.Constants;
import team4391.robot.Robot;
import team4391.robot.commands.LiftStop;

/**
 *
 */
public class Lift extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	TalonSRX _cubevatorTalon = new TalonSRX(Constants.kCubevatorId);
	TalonSRX _cubevatorSlave = new TalonSRX(Constants.kCubevatorSlaveId);
	
	DigitalInput _limitSwitch = new DigitalInput(0);
	private double _targetHeight;
	
	public Lift()
	{
		init();
	}

	public void init()
	{
		_cubevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		_cubevatorTalon.setInverted(true);			
		
		_cubevatorTalon.setSensorPhase(true); // Change this to false if not counting positive numbers when going up
			
		// May want to enable a "soft" limit on distance
		_cubevatorTalon.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
		int softLimitPosition = getEncoderPositionFromInches(Constants.kCubevatorTopLimitInches);
		_cubevatorTalon.configForwardSoftLimitThreshold(softLimitPosition, Constants.kTimeoutMs);			
		
		_cubevatorSlave.setInverted(true);
		_cubevatorSlave.follow(_cubevatorTalon);
		Robot._gyroTalon = _cubevatorSlave;
		
		// Enable brake mode on both talons.
		_cubevatorSlave.setNeutralMode(NeutralMode.Brake);
		_cubevatorTalon.setNeutralMode(NeutralMode.Brake);
	}	
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new LiftStop());
    }

	public void up() {
		_cubevatorTalon.set(ControlMode.PercentOutput, 1);
		
	}
	
	public void down() {
		
		if(!IsAtBottomLimit())
		{
			_cubevatorTalon.set(ControlMode.PercentOutput, -.75);
		}
		else
		{
			resetPosition();	
			stop();
		}
		
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
		SmartDashboard.putNumber("cubevatorPossition", position);
		
		double revolutions = ((double)position / (double)Constants.kCubevatorEncoderCountsPerRev);
		double inches = (revolutions * (Constants.kCubevatorDrumDiameterInches * Math.PI));
		
		return inches;
	}
	
	public boolean IsAtTopLimit()
	{
		return _cubevatorTalon.getSensorCollection().isFwdLimitSwitchClosed();
	}
	
	public boolean IsAtBottomLimit()
	{		
		return _limitSwitch.get();
	}
	
	public void goToPosition(double heightInches)
	{
		_targetHeight = heightInches;
		
		// assume the talon will do  this for us
	}
	
	public boolean isAtPosition()
	{
		boolean weAreThereMan = Math.abs(_targetHeight - getHeightInches()) < 2.0;
		return weAreThereMan;
	}

	public void updateDashboard() {
		 SmartDashboard.putNumber("CubevatorHeightInches", getHeightInches());
		 SmartDashboard.putBoolean("IsAtBottom", IsAtBottomLimit());
		 SmartDashboard.putBoolean("IsAtTop", IsAtTopLimit());
	}
}



