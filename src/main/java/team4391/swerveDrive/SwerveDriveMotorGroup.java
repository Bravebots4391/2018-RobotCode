package team4391.swerveDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class SwerveDriveMotorGroup
{
	private WPI_TalonSRX _motorFR;
	private WPI_TalonSRX _motorRR;
	private WPI_TalonSRX _motorFL;
	private WPI_TalonSRX _motorRL;
	
	private WPI_TalonSRX _turnFl;
	private WPI_TalonSRX _turnFR;
	private WPI_TalonSRX _turnBl;
	private WPI_TalonSRX _turnBR;	
	
	public SwerveDriveMotorGroup(WPI_TalonSRX motorFR, WPI_TalonSRX motorFL, WPI_TalonSRX motorRR, WPI_TalonSRX motorRL, WPI_TalonSRX turnFl, WPI_TalonSRX turnFR, WPI_TalonSRX turnBl, WPI_TalonSRX turnBR) 
	{
		_motorFR = motorFR;
		_motorRR = motorRR;
		_motorFL = motorFL;
		_motorRL = motorRL;
		
		_turnFl = turnFl;
		_turnFR = turnFR;
		_turnBl = turnBl;
		_turnBR = turnBR;
	}
	
	public WPI_TalonSRX get_motorFR() {
		return _motorFR;
	}

	public WPI_TalonSRX get_motorRR() {
		return _motorRR;
	}

	public WPI_TalonSRX get_motorFL() {
		return _motorFL;
	}

	public WPI_TalonSRX get_motorRL() {
		return _motorRL;
	}

	public WPI_TalonSRX get_turnFl() {
		return _turnFl;
	}

	public WPI_TalonSRX get_turnFR() {
		return _turnFR;
	}

	public WPI_TalonSRX get_turnBl() {
		return _turnBl;
	}

	public WPI_TalonSRX get_turnBR() {
		return _turnBR;
	}

}
