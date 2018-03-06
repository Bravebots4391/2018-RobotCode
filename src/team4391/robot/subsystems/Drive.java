package team4391.robot.subsystems;

import team4391.swerveDrive.Gyro;
import team4391.swerveDrive.SwerveDrive;
import team4391.swerveDrive.SwerveDrive.SwerveMode;
import team4391.swerveDrive.SwerveDriveMotorGroup;
import team4391.util.InterpolatingDouble;
import team4391.util.InterpolatingTreeMap;
import team4391.util.MaxSonar_MB1033;
import team4391.util.MaxSonar_MB1200;
import team4391.util.SyncronousRateLimiter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4391.loops.Loop;
import team4391.robot.Constants;
import team4391.robot.Robot;
import team4391.robot.commands.TeleopDrive;


public class Drive extends Subsystem implements PIDOutput {
	
	private SwerveDrive _swerveDrive;
	private double _pidOutput;
	private double _myTargetHeading;
	private double _myTurnRate;
	private double _myHeadingError;
	private double _myTargetSpeed;
	private double _myTargetDistanceIn;
	private double _myTargetSetpoint;
	private Preferences prefs;
	
	private static WPI_TalonSRX _motorFR = new WPI_TalonSRX(Constants.kFrontRightDriveMotorId);
	private static WPI_TalonSRX _motorRR = new WPI_TalonSRX(Constants.kBackRightDriveMotorId);
	private static WPI_TalonSRX _motorFL = new WPI_TalonSRX(Constants.kFrontLeftDriveMotorId);
	private static WPI_TalonSRX _motorRL = new WPI_TalonSRX(Constants.kBackLeftDriveMotorId);	
	private static WPI_TalonSRX _turnFl = new WPI_TalonSRX(Constants.kFrontLeftTurnMotorId);
	private static WPI_TalonSRX _turnFR = new WPI_TalonSRX(Constants.kFrontRightTurnMotorId);
	private static WPI_TalonSRX _turnBl = new WPI_TalonSRX(Constants.kBackLeftTurnMotorId);
	private static WPI_TalonSRX _turnBR = new WPI_TalonSRX(Constants.kBackRightTurnMotorId);	
	
	private static Gyro _gyro = new Gyro(Robot._gyroTalon );
	
	MaxSonar_MB1200 _sonarLeft = new MaxSonar_MB1200(1);
	MaxSonar_MB1033 _sonarRight = new MaxSonar_MB1033(0);
	
	public final PIDController _myHeadingPid = new PIDController(0.010, 0, 0, _gyro, this);
    public SyncronousRateLimiter _srl = new SyncronousRateLimiter(Constants.kLooperDt, 1.0 , 0);
    public SyncronousRateLimiter _accelRateLimiter = new SyncronousRateLimiter(Constants.kLooperDt, 1.0, 0);
    
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> _distanceSpeedProfile = new InterpolatingTreeMap<>();
    	
	public enum DriveState {
        OpenLoop, DirectionSetpoint, CameraHeadingControl, DriveForDistance, McTwist, Rotate
    }
	
	private DriveState _myDriveState;
	
	public Drive()
	{
		prefs = Preferences.getInstance();	
		
		SwerveDriveMotorGroup sdmg = new SwerveDriveMotorGroup(_motorFR, _motorFL, _motorRR, _motorRL, _turnFl, _turnFR, _turnBl, _turnBR);		
		_swerveDrive = new SwerveDrive(sdmg, _gyro);
	}
	
	private final Loop mLoop = new Loop() {
		 
		@Override
		public void onStart() {
			setOpenLoop();			
		}
	
		@Override
		public void onLoop() 
		{		
			synchronized (Drive.this) 
			{											
				// make sure the Gyro updates
				_gyro.updateFusedHeading();				
				
				switch(_myDriveState)
				{
					case OpenLoop:
						break;
					
					case DriveForDistance:
						updateDriveForDistance();
						break;
						
					case McTwist:
						updateMcTwist();
						break;
						
					case Rotate:
						updateRotateDegrees();
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
		public void onStop() 
		{
			// TODO Auto-generated method stub		
				setOpenLoop();
		}			
	};	 		
	
	public Loop getLoop() 
	{
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
    	_gyro.updateDashboard();
    	
    	SmartDashboard.putString("DriveMode", _myDriveState.toString());
    	SmartDashboard.putNumber("RotateSetpoint", _myTargetHeading);
    	SmartDashboard.putNumber("RotateError", _myHeadingError);
    	
    	SmartDashboard.putData(this);    
    	
    	SmartDashboard.putNumber("LeftSonarInches", _sonarLeft.getDistanceInInches());
    	SmartDashboard.putNumber("RightSonarInches", _sonarRight.getDistanceInInches());
    }
    

    public DriveState getDriveState()
    {
    	return _myDriveState;
    }
    
	public void resetSensors() {
		_swerveDrive.resetDistance();
		
	}
	
	public void teleopDrive(Joystick cntrl)
	{
		_swerveDrive.setUseSpeedCntrl(false);
		
		double angle = ConvertJoystickXYtoAngle(cntrl.getX(), -cntrl.getY());
		double pctSpeed = Math.pow(Math.sqrt(cntrl.getX() * cntrl.getX() + cntrl.getY() * cntrl.getY()), 3);		
	
		double X = cntrl.getX();
		double Y = -cntrl.getY();
		double rX= cntrl.getRawAxis(4);
		boolean isPivot = cntrl.getRawButton(6);			
		
		if(Robot.cubevatorSubsystem.getHeightInches() > Constants.kCubevatorFirstStageHeightInches)
		{
			pctSpeed = pctSpeed * Constants.kDriveWhileHighLimit2;
			rX = rX*Constants.kDriveWhileHighRotateLimit;
		}
		
		SmartDashboard.putNumber("X", X);
		SmartDashboard.putNumber("Y", Y);
		SmartDashboard.putNumber("rX", rX);
		
		if(DB(X) || DB(Y)) // Left stick is out of deadband
		{
			if(DB(rX) && !isPivot) 
			{
				_swerveDrive.setDrive(SwerveMode.swerveAndTurn, pctSpeed, angle, rX);
			}
			else if(Y > 0 && DB(rX) && isPivot)
			{
				_swerveDrive.setDrive(SwerveMode.frontPivot, rX, 0);
			}	
			else if(Y < 0 && DB(rX) && isPivot)	
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
			_swerveDrive.setDrive(SwerveMode.coast, 0, 0);
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
	
	private boolean DB(double x) {
		return Math.abs(x) > Constants.kJoystickDeadband;
	}
	
	public synchronized void setOpenLoop(boolean updateNeutralDriveMode){
		if (_myDriveState != DriveState.OpenLoop) {                            	
    		
			if(updateNeutralDriveMode)
			{
				_swerveDrive.SetNeutralModeForDrive(NeutralMode.Coast);
			}
			
            _swerveDrive.setDrive(SwerveMode.crab, 0, 0);
            
            _myDriveState = DriveState.OpenLoop;
        } 
	}
	
	public synchronized void setOpenLoop(){
    	setOpenLoop(true);   	    	
    }
	
    public synchronized void targetDiscreteAngle(double degreesTarget, double speed){
    	if (_myDriveState != DriveState.DirectionSetpoint) {            
            _myDriveState = DriveState.DirectionSetpoint;
            _myHeadingPid.reset();
        }    
    	
    	_myTargetSpeed = speed;
    	_myTargetHeading = degreesTarget;
    	
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
		_myHeadingError = _myTargetHeading - _swerveDrive.getGyro().gyroGetFusedHeading();
				
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
    
    public void rotateDegrees(double degrees)
    {
    	// (calculate arc length = 2*PI*R*theta)/360    	
    	double arcLenInches = (2*Math.PI*Constants.kRobotRadius*degrees)/360;
    	
    	if(_myDriveState != DriveState.Rotate)
    	{
    		_swerveDrive.SetNeutralModeForDrive(NeutralMode.Brake);
    		_myTargetSpeed = 0.5;
    		_myTargetDistanceIn = arcLenInches;
    		
    		// Setup lookup table
        	setupDistanceProfile(_myTargetDistanceIn, _myTargetSpeed);
    		
    		_swerveDrive.resetDistance();
    		
    		_swerveDrive.setDrive(SwerveMode.rotate, 0, 0);
    		
    		_myDriveState = DriveState.Rotate;
    		
    		
    		
    		
    	}    	
    }
    
    public void updateRotateDegrees()
    {
    	if(_myDriveState == DriveState.Rotate)
    	{
    		double distance = _swerveDrive.getDistanceInches();
    		
    		if(Math.abs(distance - Math.abs(_myTargetDistanceIn)) < 1.0)
    		{
    			setOpenLoop(false);
    			return;
    		}
    		    		    	
    		// Get drive info from the lookup
    		double speed = _distanceSpeedProfile.getInterpolated(new InterpolatingDouble(Math.abs(distance))).value;  		
    		
    		_swerveDrive.setDrive(SwerveMode.rotate, speed, 0);
    	}
    }
    
    public void driveForDistance(double distanceInches, double speedFps, double heading)
    {    
    	// This gets called only once
    	if(_myDriveState != DriveState.DriveForDistance)
    	{
    		_swerveDrive.SetNeutralModeForDrive(NeutralMode.Brake);
    		_myTargetSpeed = speedFps;
        	_myTargetHeading = heading;
        	_myTargetDistanceIn = distanceInches;
        	
        	_swerveDrive.resetDistance();
        	
        	// Setup lookup table
        	setupDistanceProfile(distanceInches, speedFps);
        	
        	// Setup the Drive
        	_gyro.setupDriveCorrection(0.0);
        	_swerveDrive.setDrive(SwerveMode.crab, 0, _myTargetHeading);        	                	
    		_myDriveState = DriveState.DriveForDistance;
    	}
    }   
    
    private void setupDistanceProfile(double distanceInches, double speedFps)
    {
        // Rotation PID Rate Limit Constants.  Limits for normal turning commands.
        _distanceSpeedProfile = new InterpolatingTreeMap<>();

        _distanceSpeedProfile.put(new InterpolatingDouble(0.0), new InterpolatingDouble(.2));
        _distanceSpeedProfile.put(new InterpolatingDouble(12.0), new InterpolatingDouble(speedFps));
        _distanceSpeedProfile.put(new InterpolatingDouble(distanceInches - 36), new InterpolatingDouble(speedFps));
        _distanceSpeedProfile.put(new InterpolatingDouble(distanceInches), new InterpolatingDouble(.2));
        
    }
    
    public synchronized void updateDriveForDistance()
    {
    	if(_myDriveState == DriveState.DriveForDistance)
    	{
    		double distance = _swerveDrive.getDistanceInches();
    		
    		SmartDashboard.putNumber("xsetDistance", distance);
    		
    		if(distance >= _myTargetDistanceIn)
    		{
    			setOpenLoop(false);
    			return;
    		}
    		
    		// Get drive info from the lookup
    		double speed = _distanceSpeedProfile.getInterpolated(new InterpolatingDouble(distance)).value;  		
    		
    		
    		SmartDashboard.putNumber("xsetFPS", speed);
    		
    		_swerveDrive.setDrive(SwerveMode.crab, speed, _myTargetHeading);
    	}
    }
    
    public void setupMcTwist(double forwardSpeedFps, double rotateRateFps)
    {
    	if(_myDriveState != DriveState.McTwist)
    	{
    		_myTargetSpeed = forwardSpeedFps;
    		_myTurnRate = rotateRateFps;
    		
    		_swerveDrive.resetDistance();
    		_swerveDrive.getGyro().reset();
    		
    		_myDriveState = DriveState.McTwist;
    	}    	
    }
    
    private void updateMcTwist()
    {    	
    	_swerveDrive.setDrive(SwerveMode.mcTwist, _myTargetSpeed, _myTurnRate);
    }

	@Override
	public void pidWrite(double output) {
		
		// TODO Auto-generated method stub
		_pidOutput = output;
	}
	
}

