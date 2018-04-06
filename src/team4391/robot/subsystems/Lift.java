package team4391.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4391.loops.Loop;
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
	TalonSRX _cubevatorSlave;
	Timer _bottomTimer = new Timer();
	
	DigitalInput _limitSwitch = new DigitalInput(0);
	private double _targetHeight;
	private int _bottomCount;
	private boolean _isAtBottomLimit;
	private double _holdHeight;
	
	private final static int CountMax = 3;
	
	public Lift()
	{
		init();
	}

	public void init()
	{
		_bottomCount = 0;
		
		_cubevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		_cubevatorTalon.setInverted(true);			
		
		_cubevatorTalon.setSensorPhase(true); // Change this to false if not counting positive numbers when going up
			
		// May want to enable a "soft" limit on distance
		_cubevatorTalon.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
		int softLimitPosition = getEncoderPositionFromInches(Constants.kCubevatorTopLimitInches);
		_cubevatorTalon.configForwardSoftLimitThreshold(softLimitPosition, Constants.kTimeoutMs);			
		
		if(Constants.useSlaveMotors)
		{
			_cubevatorSlave = new TalonSRX(Constants.kCubevatorSlaveId);
			_cubevatorSlave.setInverted(true);
			_cubevatorSlave.follow(_cubevatorTalon);
			Robot._gyroTalon = _cubevatorSlave;
			_cubevatorSlave.setNeutralMode(NeutralMode.Brake);
		}
		
		// Enable brake mode on both talons.		
		_cubevatorTalon.setNeutralMode(NeutralMode.Brake);
		
        /* set the peak and nominal outputs, 12V means full */
		_cubevatorTalon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_cubevatorTalon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_cubevatorTalon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_cubevatorTalon.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		
		_cubevatorTalon.configAllowableClosedloopError(3, Constants.kPIDLoopIdx, Constants.kTimeoutMs); /* always servo */
        /* set closed loop gains in slot0 */
		_cubevatorTalon.config_kF(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		_cubevatorTalon.config_kP(Constants.kPIDLoopIdx, 1.0, Constants.kTimeoutMs);
		_cubevatorTalon.config_kI(Constants.kPIDLoopIdx, 0.01, Constants.kTimeoutMs);
		_cubevatorTalon.config_kD(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);			
	}	
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new LiftStop());
    }

    private final Loop mLoop = new Loop() {
		 
		

		@Override
		public void onStart() {
			setHolding();			
		}
	
		@Override
		public void onLoop() {		
			synchronized (Lift.this) {							
				
				if(_limitSwitch.get())
				{
					_bottomCount++;
				}				
				else
				{
					_bottomCount--;
				}
				
				if(_bottomCount >= CountMax)
				{
					_bottomCount = CountMax;
					_isAtBottomLimit = true;
				}				
				else if(_bottomCount <= 0)
				{
					_bottomCount = 0;
					_isAtBottomLimit = false;
				}				
			}													
		}	
	
		@Override
		public void onStop() {
		// TODO Auto-generated method stub		
			setHolding();
		}
		 
	 };
	 
	 public Loop getLoop() {
	        return mLoop;
	    }    
    
	 public void setHolding()
	 {
		 //_cubevatorTalon.set(ControlMode.PercentOutput, 0);
		 
		 double currentPosition = getHeightInches();
		 
		 int holdCounts = getEncoderPositionFromInches(currentPosition);
		 _cubevatorTalon.set(ControlMode.Position, holdCounts);		 
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
		//_cubevatorTalon.set(ControlMode.PercentOutput, 0.0);
		setHolding();
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
		return _isAtBottomLimit;
	}
	
	public void goToPosition(double heightInches)
	{
		_targetHeight = heightInches;			
		
		// assume the talon will do  this for us
		_cubevatorTalon.set(ControlMode.Position, getEncoderPositionFromInches(heightInches));
	}
	
	public boolean isAtPosition()
	{
		boolean weAreThereMan = Math.abs(_targetHeight - getHeightInches()) < 2.0;		
		
		if(weAreThereMan)
		{
			double _holdHeight = getHeightInches();
			setHolding();
		}
		
		return weAreThereMan;
	}

	public void updateDashboard() {
		 SmartDashboard.putNumber("CubevatorHeightInches", getHeightInches());
		 SmartDashboard.putBoolean("IsAtBottom", IsAtBottomLimit());
		 SmartDashboard.putBoolean("IsAtTop", IsAtTopLimit());		 
		 
		 SmartDashboard.putNumber("CubevatorTarget", _targetHeight);		 
		 SmartDashboard.putNumber("CubevatorError", _cubevatorTalon.getClosedLoopError(0));		 
		 SmartDashboard.putString("CubeTalonMode",_cubevatorTalon.getControlMode().toString()); 
	}

	public void setPositionInches(double heightInches) {		
				
		_cubevatorTalon.setSelectedSensorPosition(getEncoderPositionFromInches(heightInches), 0, Constants.kTimeoutMs);
	}


}



