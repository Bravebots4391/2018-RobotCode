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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4391.loops.Loop;
import team4391.robot.Constants;
import team4391.robot.Robot;
import team4391.robot.commands.TeleopDrive;


public class Drive extends Subsystem implements PIDOutput, PIDSource {
	
	private SwerveDrive _swerveDrive;
	private double _pidOutput;
	private double _myTargetHeading;
	private double _myTurnRate;
	private double _myHeadingError;
	private double _myTargetSpeed;
	private double _myTargetDistanceIn;
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
	
	public final PIDController _myHeadingPid = new PIDController(0.010, 0, 0, this, this);
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
						
					// case McTwist:
					// 	updateMcTwist();
					// 	break;
						
					case Rotate:
						updateRotateDegrees();
						break;
						
					case DirectionSetpoint:					
						//updateDegTurnHeadingControl();
						break;
						
					case CameraHeadingControl:					
						updateCameraHeadingControl();
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
		setOpenLoop();
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
		_swerveDrive.getGyro().reset();
	}
	
	public void teleopDrive(Joystick cntrl)
	{
		_swerveDrive.setUseSpeedCntrl(false);		
		
		double power = 3.0;
		boolean lowGear =cntrl.getRawButton(5) && cntrl.getRawButton(6); 
		if(lowGear)
		{
			power = 1;
		}
		
		double angle = ConvertJoystickXYtoAngle(cntrl.getX(), -cntrl.getY());
		double pctSpeed = Math.pow(Math.sqrt(cntrl.getX() * cntrl.getX() + cntrl.getY() * cntrl.getY()), power);			
		
		if(lowGear)
		{
			pctSpeed = pctSpeed*0.35;
		}
		
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
		else if(!DB(X) && !DB(Y) && !DB(rX) && cntrl.getRawAxis(2)>0.75 && cntrl.getRawAxis(3) > 0.75)
		{
			// put on the brakes if both triggers are pulled
			_swerveDrive.setDrive(SwerveMode.airBrakes, 0, 0);
		}
		else if(!DB(X) && !DB(Y) && DB(rX) && cntrl.getRawAxis(2)>0.75 && !DB(cntrl.getRawAxis(3))) // only the right stick is out of deadband
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
			System.out.println("setOpenLoop");
    		
			if(updateNeutralDriveMode)
			{
				_swerveDrive.SetNeutralModeForDrive(NeutralMode.Coast);
			}
			
			_swerveDrive.set_isFieldOriented(true);
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
    	Double myKp = Constants.kDriveTurnKp;
    	Double myKi = Constants.kDriveTurnKi; //prefs.getDouble("targetKi", Constants.kDriveTurnKi);
    	Double myKd = 0.0; //prefs.getDouble("targetKd", Constants.kDriveTurnKd);
    	Double myFF = 0.02; //prefs.getDouble("targetKf", Constants.kDriveTurnKf);
    	
    	_myHeadingPid.setPID(myKp, myKi, myKd, myFF);
    	_myHeadingPid.setAbsoluteTolerance(tolerance);
    	_myHeadingPid.setOutputRange(-0.6, 0.6);
    	_myHeadingPid.reset();
		
		SmartDashboard.putData("myPid", _myHeadingPid);

    	_swerveDrive.resetDistance();
    	_srl.Reset();
    	//_swerveDrive.getGyro().reset();
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
		_myHeadingPid.setSetpoint(_srl.getOutput());	
		
		_swerveDrive.setDrive(SwerveMode.rotate, _pidOutput, 0);			
    }
    
    public synchronized void rotateDegrees(double degrees)
    {
    	// (calculate arc length = 2*PI*R*theta)/360    	
    	double arcLenInches = (2*Math.PI*Constants.kRobotRadius*degrees)/360;
    	
    	if(_myDriveState != DriveState.Rotate)
    	{
    		System.out.println("setup rotateDegrees");
    		
    		_gyro.reset();
    		
    		_swerveDrive.SetNeutralModeForDrive(NeutralMode.Brake);
    		_myTargetSpeed = 0.5 * Math.signum(degrees);
    		_myTargetDistanceIn = arcLenInches;
    		
    		// Setup lookup table
        	setupDistanceProfile(_myTargetDistanceIn, _myTargetSpeed);
    		
    		_swerveDrive.resetDistance();
    		_swerveDrive.resetDistance();
        	_swerveDrive.resetDistance();
        	_swerveDrive.resetDistance();
    		
    		_swerveDrive.setDrive(SwerveMode.rotate, 0, 0);
    		
    		_myDriveState = DriveState.Rotate;
    		    		    	    		
    	}    	
    }
	
	public synchronized void driveToTarget()
	{
		if(_myDriveState != DriveState.CameraHeadingControl)
		{
			_myDriveState = DriveState.CameraHeadingControl;
			setupPID(0.5);
			_myHeadingPid.setSetpoint(0.0);
			_myHeadingPid.enable();
			_myHeadingPid.setSetpoint(0.0);

			_swerveDrive.set_isFieldOriented(false);
			updateCameraHeadingControl();
		}
	}

	public synchronized void updateCameraHeadingControl()
	{
		if(_myDriveState == DriveState.CameraHeadingControl)
		{
			SmartDashboard.putData("myPid", _myHeadingPid);

			double error = Math.abs(_myHeadingPid.getError());
			if(error<1.0)
			{
				_myHeadingPid.disable();
				stopMotors();
			}

			// drive left or right
			double sign = Math.signum(_pidOutput);
			double heading = 0.0;
			if(sign >= 0)
			{
				heading = 270.0;
			}
			else
			{
				heading = 90.0;
			}
			
			_swerveDrive.setDrive(SwerveMode.crab, Math.abs(_pidOutput), heading);
		}
	}

    public synchronized void updateRotateDegrees()
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
    		
    		_swerveDrive.setDrive(SwerveMode.rotate, _myTargetSpeed, 0);
    	}
    }
    
    public synchronized void driveForDistance(double distanceInches, double speedFps, double heading)
    {    
    	// This gets called only once
    	if(_myDriveState != DriveState.DriveForDistance)
    	{
    		System.out.println("setup driveForDistance");
    		
    		_gyro.reset();
    		_gyro.reset();
    		
    		_swerveDrive.SetNeutralModeForDrive(NeutralMode.Brake);
    		_myTargetSpeed = speedFps;
        	_myTargetHeading = heading;
        	_myTargetDistanceIn = distanceInches;
        	
        	_swerveDrive.resetDistance();
        	_swerveDrive.resetDistance();
        	_swerveDrive.resetDistance();
        	_swerveDrive.resetDistance();
        	
        	// Setup lookup table
        	setupDistanceProfile(distanceInches, speedFps);
        	
        	// Setup the Drive
        	_gyro.setupDriveCorrection(0.0);
        	_swerveDrive.setDrive(SwerveMode.crab, 0, _myTargetHeading);        	                	
    		_myDriveState = DriveState.DriveForDistance;
    	}
    }   
    
    private synchronized void setupDistanceProfile(double distanceInches, double speedFps)
    {
        // Rotation PID Rate Limit Constants.  Limits for normal turning commands.
        _distanceSpeedProfile = new InterpolatingTreeMap<>();

        if(distanceInches > 36.0)
        {        
        _distanceSpeedProfile.put(new InterpolatingDouble(0.0), new InterpolatingDouble(.3 * Math.signum(speedFps)));
        _distanceSpeedProfile.put(new InterpolatingDouble(12.0), new InterpolatingDouble(speedFps));
        _distanceSpeedProfile.put(new InterpolatingDouble(distanceInches - 36), new InterpolatingDouble(speedFps));
        _distanceSpeedProfile.put(new InterpolatingDouble(distanceInches), new InterpolatingDouble(.2 * Math.signum(speedFps)));
        }
        else
        {
        	_distanceSpeedProfile.put(new InterpolatingDouble(0.0), new InterpolatingDouble(speedFps));
        }
        
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
    
    // public void setupMcTwist(double forwardSpeedFps, double rotateRateFps)
    // {
	// 	if(_myDriveState != DriveState.McTwist)
	
    // 	{
    // 		_myTargetSpeed = forwardSpeedFps;
    // 		_myTurnRate = rotateRateFps;
    		
    // 		_swerveDrive.resetDistance();
    // 		_swerveDrive.getGyro().reset();
    		
    // 		_myDriveState = DriveState.McTwist;
    // 	}    	
    // }
    
    // private void updateMcTwist()
    // {    	
    // 	_swerveDrive.setDrive(SwerveMode.mcTwist, _myTargetSpeed, _myTurnRate);
    // }

	@Override
	public void pidWrite(double output) {
		
		_pidOutput = output;
		SmartDashboard.putNumber("PIDOutput", output);
	}
	
	public double getDistanceInches()
	{
		return _swerveDrive.getDistanceInches();
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {

	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
}

	@Override
	public double pidGet() {
		double pidSource = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

		SmartDashboard.putNumber("PIDSource", pidSource);
		return pidSource;
	}
	
}

