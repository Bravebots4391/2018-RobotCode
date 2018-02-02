package team4391.robot.subsystems;

import team4391.swerveDrive.SwerveDrive;
import team4391.util.InterpolatingDouble;
import team4391.util.SyncronousRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import team4391.loops.Loop;
import team4391.robot.Constants;
import team4391.robot.commands.TeleopDrive;


public class Drive extends Subsystem implements PIDOutput {
	
	private SwerveDrive _swerveDrive = new SwerveDrive();
	private double _pidOutput;
	
	public final PIDController _myHeadingPid = new PIDController(0.010, 0, 0, _swerveDrive.getGyro(), this);
    public SyncronousRateLimiter _srl = new SyncronousRateLimiter(Constants.kLooperDt, 1.0 , 0);
    public SyncronousRateLimiter _accelRateLimiter = new SyncronousRateLimiter(Constants.kLooperDt, 1.0, 0);
    
	
	public enum DriveState {
        OpenLoop, DirectionSetpoint, CameraHeadingControl
    }
	
	private DriveState myDriveState;
	
	private final Loop mLoop = new Loop() {
		 
		@Override
		public void onStart() {
			setOpenLoop();			
		}
	
		@Override
		public void onLoop() {		
			synchronized (Drive.this) {							
				
			switch(myDriveState)
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
					System.out.println("Unexpected drive control state: " + myDriveState);
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
	private Double _myTurnRate;
	 
	 public Loop getLoop() {
	        return mLoop;
	    }
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	setDefaultCommand(new TeleopDrive());
    }
    
    public void stopMotors(){
    	_swerveDrive.setDrive(0.0, 0.0,0.0, false);
    	//myHeadingPid.disable();
    }
    
    public void updateDashboard() {
    	_swerveDrive.UpdateDashboard();
    }

	public void resetSensors() {
		// TODO Auto-generated method stub
		
	}
	
	public void teleopDrive(Joystick cntrl) {
		_swerveDrive.setDrive(Db(cntrl.getX()), Db(-cntrl.getY()), Db(cntrl.getRawAxis(4)), cntrl.getRawButton(6));
	}
	
	double Db(double axisVal) {	
		if(Math.abs(axisVal) > Constants.kJoystickDeadband)
			return axisVal;
		else
			return 0;
	}
	
	public synchronized void setOpenLoop(){
    	if (myDriveState != DriveState.OpenLoop) {            
            myDriveState = DriveState.OpenLoop;
            //myHeadingPid.reset();
            //myHeadingPid.disable();
            
            _swerveDrive.setDrive(0, 0, 0, false);
        }    	    	
    }
	
    private synchronized void updateRotationControl(double error){
    	
    	// Run Through Lookup
    	InterpolatingDouble rate = Constants.kRateLimitMap.getInterpolated(new InterpolatingDouble(error));
    	if(myDriveState == DriveState.CameraHeadingControl)
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
		
		//robotDrive.arcadeDrive(-1* m_targetSpeed, m_rotateValue);				
    }

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		_pidOutput = output;
	}
}

