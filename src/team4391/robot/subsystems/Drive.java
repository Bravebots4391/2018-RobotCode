package team4391.robot.subsystems;

import team4391.swerveDrive.SwerveDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import team4391.loops.Loop;
import team4391.robot.commands.TeleopDrive;


public class Drive extends Subsystem {
	
	private SwerveDrive _swerveDrive = new SwerveDrive();
	
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
		if (axisVal < -0.10)
			return axisVal;
		if (axisVal > +0.10)
			return axisVal;
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
}

