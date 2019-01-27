/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team4391.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4391.util.CrashTracker;
import team4391.loops.Looper;
import team4391.robot.subsystems.Arm;
import team4391.robot.subsystems.Climb;
import team4391.robot.subsystems.Drive;
import team4391.robot.subsystems.ExampleSubsystem;
import team4391.robot.subsystems.LED;
import team4391.robot.subsystems.Lift;
import team4391.robot.subsystems.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static final ExampleSubsystem kExampleSubsystem
			= new ExampleSubsystem();
	public static OI m_oi;
	
	public static LED ledSubsystem;
	public static Drive driveSubsystem;
	public static Arm armSubsystem;
	public static Lift cubevatorSubsystem;
	public static Climb climbSubsystem;
	public static Limelight limelightSubsystem;
	public static PowerDistributionPanel _pdpModule;
	public static AutoLogic _autoLogic;
	
	public static TalonSRX _gyroTalon;
	
	public int _counter = 0;


    // Enabled looper is called at 100Hz whenever the robot is enabled
    Looper mEnabledLooper = new Looper();
    // Disabled looper is called at 100Hz whenever the robot is disabled
    Looper mDisabledLooper = new Looper();	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_oi = new OI();
		
		Constants.putValuesInNetworkTables();
		
		// Initialize subsystems
		ledSubsystem = new LED();
		ledSubsystem.init();
		
		armSubsystem = new Arm();
		cubevatorSubsystem = new Lift();	
		
		limelightSubsystem = new Limelight();
		
		//_pdpModule = new PowerDistributionPanel(Constants.kPDP);
		//SmartDashboard.putData("PDP", _pdpModule);
		
		climbSubsystem = new Climb();
		climbSubsystem.init();
		
		driveSubsystem = new Drive();
		
		m_oi.init();
		
		_autoLogic = new AutoLogic();
		_autoLogic.setupAutonomousChooser();
		
		
		
		try
		{ 
        	CrashTracker.logRobotInit();           
        	 
            // Reset all sensors
            zeroAllSensors();
            
        	// Enable Loops
        	mEnabledLooper.register(driveSubsystem.getLoop());
        	mEnabledLooper.register(armSubsystem.getLoop());
        	mEnabledLooper.register(cubevatorSubsystem.getLoop());
//        	mEnabledLooper.register(shooter.getLoop());
//        	mEnabledLooper.register(led.getLoop());        
        	
        }
		catch (Throwable t) 
		{
            CrashTracker.logThrowableCrash(t);
            throw t;
        }        
        
        SmartDashboard.putData(Scheduler.getInstance());
		
		
    //     CameraServer.getInstance().startAutomaticCapture();
        
	//       new Thread(() -> {
	//       UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
	//       camera.setResolution(320, 240);
	//       //camera.setExposureAuto();
	//       camera.setExposureManual(3);	      
	      
	//       CvSink cvSink = CameraServer.getInstance().getVideo();
	//       CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 320, 240);
	      
	//       Mat source = new Mat();
	//       Mat output = new Mat();
	      
	//       while(true) {            	
	//           cvSink.grabFrameNoTimeout(source);
	//           Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
	//           outputStream.putFrame(output);
	//       }
	//   }, "camera").start();
      
	}

	   public void outputAllToSmartDashboard() {	    	
	    	
	    	driveSubsystem.updateDashboard();
	    	armSubsystem.updateDashboard();
	    	cubevatorSubsystem.updateDashboard();
	    	climbSubsystem.updateDashboard();	    	

//	    	SmartDashboard.putBoolean("cameraConnected", mVisionServer.isConnected());
	    	
	    	SmartDashboard.putNumber("Counter", _counter++);     	    	
	        
//	        SmartDashboard.putBoolean("CompressorEnabled", RobotMap.compressor.enabled());	            	    	
	    }

	    public void stopAll() {
	        driveSubsystem.stopMotors();
	    }
	    
	    public void zeroAllSensors() {
			driveSubsystem.resetSensors();
			
	    }	
	
	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		try{			
			driveSubsystem.setOpenLoop();
			
			ledSubsystem.disable();
			
		    disableLoops();
		    zeroAllSensors();		    
		    stopAll();		    		   
		    
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
            throw t;
		}
	}
	
	private void disableLoops() {
		mEnabledLooper.stop();
		mDisabledLooper.start();
	}
    
	private void enableLoops() {
		mDisabledLooper.stop();
		mEnabledLooper.start();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		
		try {        	                	
            stopAll();                        
            System.gc();            
        } 
		catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
		
		outputAllToSmartDashboard();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {		
		ledSubsystem.enable();
		_autoLogic.runAuto();	
		
		zeroAllSensors();		
		enableLoops();
		
//		// schedule the autonomous command (example)
//		if (m_autonomousCommand != null) {
//			m_autonomousCommand.start();
//			
//			// Reset all sensors
//            zeroAllSensors();        
//	        enableLoops();
//		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		outputAllToSmartDashboard();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
//		if (m_autonomousCommand != null) {
//			m_autonomousCommand.cancel();
//		}
		
		try {
	        ledSubsystem.enable();
			driveSubsystem.setOpenLoop();
        	
	        // Configure loopers
	        enableLoops();
        
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		outputAllToSmartDashboard();
		
		
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
