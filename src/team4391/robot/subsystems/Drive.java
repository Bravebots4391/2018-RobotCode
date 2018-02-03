package team4391.robot.subsystems;

import team4391.swerveDrive.SwerveDrive;
import team4391.swerveDrive.SwerveDrive.SwerveMode;
import team4391.util.InterpolatingDouble;
import team4391.util.SyncronousRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4391.loops.Loop;
import team4391.robot.Constants;
import team4391.robot.commands.TeleopDrive;


public class Drive extends Subsystem implements PIDOutput {
	
	private SwerveDrive _swerveDrive = new SwerveDrive();
	private double _pidOutput;
	private double _myTargetAngle;
	private double _myTurnRate;
	private double _myHeadingError;
	private double _myTargetSpeed;
	private Preferences prefs;
	
	public final PIDController _myHeadingPid = new PIDController(0.010, 0, 0, _swerveDrive.getGyro(), this);
    public SyncronousRateLimiter _srl = new SyncronousRateLimiter(Constants.kLooperDt, 1.0 , 0);
    public SyncronousRateLimiter _accelRateLimiter = new SyncronousRateLimiter(Constants.kLooperDt, 1.0, 0);
    
	
	public enum DriveState {
        OpenLoop, DirectionSetpoint, CameraHeadingControl
    }
	
	private DriveState _myDriveState;
	
	private final Loop mLoop = new Loop() {
		 
		@Override
		public void onStart() {
			setOpenLoop();			
		}
	
		@Override
		public void onLoop() {		
			synchronized (Drive.this) {							
				
			switch(_myDriveState)
			{
				case OpenLoop:
					break;
				
				case DirectionSetpoint:					
					//updateDegTurnHeadingControl();
					break;
					
				case CameraHeadingControl:					
					//updateCameraHeadingControl();
					break;
					
				default:
					System.out.println("Unexpected drive control state: " + _myDriveState);
	           break;
		}									
					
		//updateDistanceCheck();
		
		//putTargetHeadingOnDashboard();
		
		}
	}
	
	@Override
	public void onStop() {
		// TODO Auto-generated method stub		
			setOpenLoop();
		}
		 
	 };	 
	private double _myTargetSetpoint;
	
	public Drive()
	{
		prefs = Preferences.getInstance();
	}
	
	 public Loop getLoop() {
	        return mLoop;
	    }
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	setDefaultCommand(new TeleopDrive());
    }
    
    public void stopMotors(){
    	_swerveDrive.setDrive(SwerveMode.crab, 0, 0);
    	_myHeadingPid.disable();
    }
    
    public void updateDashboard() {
    	_swerveDrive.UpdateDashboard();
    	
    	SmartDashboard.putString("DriveMode", _myDriveState.toString());
    	SmartDashboard.putNumber("RotateSetpoint", _myTargetAngle);
    	SmartDashboard.putNumber("RotateError", _myHeadingError);
    }

	public void resetSensors() {
		// TODO Auto-generated method stub
		
	}
	
	public void teleopDrive(Joystick cntrl)
	{
		double angle = ConvertJoystickXYtoAngle(cntrl.getX(), -cntrl.getY());
		double pctSpeed = Math.pow(Math.sqrt(cntrl.getX() * cntrl.getX() + cntrl.getY() * cntrl.getY()), 2);		
	
		double X = cntrl.getX();
		double Y = -cntrl.getY();
		double rX= cntrl.getRawAxis(4);
		boolean isPivot = cntrl.getRawButton(6);			
		
		if(DB(X) || DB(Y)) // Left stick is out of deadband
		{		
			if(DB(rX) && Y > 0 && !isPivot) 
			{
				_swerveDrive.setDrive(SwerveMode.carTurn, pctSpeed, rX);
			}
			else if(DB(rX) && Y < 0 && !isPivot)
			{
				_swerveDrive.setDrive(SwerveMode.carTurnReverse, pctSpeed, rX);
			}
			else if(Y > 0 && DB(rX) && isPivot)
			{
				_swerveDrive.setDrive(SwerveMode.frontPivot, rX, 0);
			}	
			else if(Y < 0 && DB(X) && isPivot)
			{
				_swerveDrive.setDrive(SwerveMode.rearPivot, rX, 0);
			}
			else 
			{			
				_swerveDrive.setDrive(SwerveMode.crab, pctSpeed, angle);
			}
		}		
		else if(!DB(X) && !DB(Y) && DB(rX)) // only the right stick is out of deadband
		{
			_swerveDrive.setDrive(SwerveMode.rotate, rX, 0);
		}
		else
		{
			_swerveDrive.setDrive(SwerveMode.crab, 0, 0);
		}
	}
	
	private double ConvertJoystickXYtoAngle(double x, double y)
	{	
		double angle = (Math.atan2(y, x) * Constants.toDegrees);		
		
		if(x == 0 && y == 0)
		{
			angle = 90;			
		}

		// convert the polar coordinate to a heading
		double coordinate = ((450 - angle) % 360);
		
		return coordinate;
	}
	
	double Db(double axisVal) {	
		if(Math.abs(axisVal) > Constants.kJoystickDeadband)
			return axisVal;
		else
			return 0;
	}
	
	private boolean DB(double x) {
		return Math.abs(x) > Constants.kJoystickDeadband;
	}
	
	public synchronized void setOpenLoop(){
    	if (_myDriveState != DriveState.OpenLoop) {            
            _myDriveState = DriveState.OpenLoop;
            //myHeadingPid.reset();
            //myHeadingPid.disable();
            
            _swerveDrive.setDrive(SwerveMode.crab, 0, 0);
        }    	    	
    }
	
    public synchronized void targetDiscreteAngle(double degreesTarget, double speed){
    	if (_myDriveState != DriveState.DirectionSetpoint) {            
            _myDriveState = DriveState.DirectionSetpoint;
            _myHeadingPid.reset();
        }    
    	
    	_myTargetSpeed = speed;
    	_myTargetAngle = degreesTarget;
    	
    	//SetBrakeEnable(true);
    	setupPID(1.0);
    	_myHeadingPid.enable();
    	updateDegTurnHeadingControl();
    }
	
    private void setupPID(double tolerance){
    	Double myKp = prefs.getDouble("targetKp", Constants.kDriveTurnKp);
    	Double myKi = prefs.getDouble("targetKi", Constants.kDriveTurnKi);
    	Double myKd = prefs.getDouble("targetKd", Constants.kDriveTurnKd);
    	Double myFF = prefs.getDouble("targetKf", Constants.kDriveTurnKf);
    	
    	_myHeadingPid.setPID(myKp, myKi, myKd, myFF);
    	_myHeadingPid.setAbsoluteTolerance(tolerance);
    	_myHeadingPid.setOutputRange(-0.6, 0.6);
    	_myHeadingPid.reset();
    	
    	_swerveDrive.resetDistance();
    	_srl.Reset();
    	_swerveDrive.getGyro().reset();
    }
    
    private synchronized void updateDegTurnHeadingControl(){
    	// Calculate rotate error    	
		_myHeadingError = _myTargetAngle - _swerveDrive.getGyro().gyroGetFusedHeading();
				
		updateRotationControl(_myHeadingError);
    }
	
    private synchronized void updateRotationControl(double error){
    	
    	// Run Through Lookup
    	InterpolatingDouble rate = Constants.kRateLimitMap.getInterpolated(new InterpolatingDouble(error));
    	if(_myDriveState == DriveState.CameraHeadingControl)
    	{
    		rate = Constants.kRateLimitMapAuto.getInterpolated(new InterpolatingDouble(error));
    	}
    	
		_myTurnRate = rate.value;
		
		// set accel rate limit
		//accelRateLimiter.update();
		//accelRateLimiter.getOutput();
		
		// Set Rate Limit 
		_srl.SetOutputRate(rate.value);
		
		// Update rotation rate limiter
		_srl.update();
		
		// Update the PID setpoint from our current accumulated RATE limiter output
		_myTargetSetpoint = _srl.getOutput();
		_myHeadingPid.setSetpoint(_srl.getOutput());	
		
		_swerveDrive.setDrive(SwerveMode.rotate, _pidOutput, 0);			
    }

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		_pidOutput = output;
	}
	
}

