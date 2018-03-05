package team4391.swerveDrive;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4391.loops.Loop;
import team4391.robot.Constants;
import team4391.robot.Robot;
import team4391.robot.subsystems.Drive;


public class Gyro extends GyroBase implements PIDOutput
{
	PigeonIMU _pidgey;
	Preferences prefs;
	private final PIDController _myHeadingPid = new PIDController(0.010, 0, 0, this, this);
	
	double kMaxCorrectionRatio = 0.05; /* cap corrective turning throttle to 30 percent of forward throttle */
	
	/** holds the current angle to servo to */
	double _targetAngle = 0;
	private double _fusedHeading;
		

	private boolean _isInitOk;
	private double _pidWriteOutput;
	
	public Gyro(TalonSRX talon)
	{
		prefs = Preferences.getInstance();
		_pidgey = new PigeonIMU(talon);
		_isInitOk = true;
	}
	
	public Gyro(int id){
		_pidgey = new PigeonIMU(id);
		_isInitOk = true;
	}
	
	public void gyroInit(int talonId){			
		//_pidgey = new PigeonIMU(talonId);
		_pidgey = new PigeonIMU(Robot.armSubsystem._suckerInnerOuter);			
	}
	
	public void gyroReset(){
		if(_isInitOk) {
			_pidgey.setFusedHeading(0.0, 10);
		}
	}
	
	public double gyroGetFusedHeading()
	{
		if(_isInitOk){
			PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
			_pidgey.getFusedHeading(fusionStatus);
			double currentAngle = -fusionStatus.heading;
			boolean angleIsGood = (_pidgey.getState() == PigeonIMU.PigeonState.Ready) ? true : false;
		
			return currentAngle;
		}
		else{
			return 0.0;
		}
	}
	
	public void setupDriveCorrection(double heading)
	{
		double kPgain = prefs.getDouble("GyroKp", Constants.GyroKp); 
		double kDgain = prefs.getDouble("GyroKd", Constants.GyroKd); 
		
		_myHeadingPid.setP(kPgain);
		_myHeadingPid.setD(kDgain);
		_myHeadingPid.setI(0.0);
		_myHeadingPid.setF(0.0);
		_myHeadingPid.setAbsoluteTolerance(0.01);
		_myHeadingPid.setOutputRange(-0.05, 0.05);
		
		_myHeadingPid.enable();
		_myHeadingPid.setSetpoint(heading);
	}
	
	public void stopDriveCorrection()
	{
		_myHeadingPid.disable();
		_myHeadingPid.reset();
	}
	
	public double getDriveCorrection()
	{	
		return _pidWriteOutput;
	}
	
//	public double getDriveCorrection(double throttle, double heading) 
//	{
//		double kPgain = prefs.getDouble("GyroKp", Constants.GyroKp) ; /* percent throttle per degree of error */
//		double kDgain = prefs.getDouble("GyroKd", Constants.GyroKd); /* percent throttle per angular velocity dps */
//		
//		if(!_isInitOk)
//		{
//			return 0.0;
//		}
//		
//		/* some temps for Pigeon API */
//		PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
//		double [] xyz_dps = new double [3];
//		
//		/* grab some input data from Pigeon and gamepad*/		
//		_pidgey.getGeneralStatus(genStatus);
//		_pidgey.getRawGyro(xyz_dps);		
//		double currentAngle = _fusedHeading;		
//		boolean angleIsGood = (_pidgey.getState() == PigeonIMU.PigeonState.Ready) ? true : false;
//		double currentAngularRate = xyz_dps[2];
//		
//		/* get input from gamepad */		
//		double forwardThrottle = throttle;// = _driveStick.getAxis(AxisType.kY) * -1.0; /* sign so that positive is forward */
//		double turnThrottle = 0.0;// = _driveStick.getAxis(AxisType.kTwist) * -1.0; /* sign so that positive means turn left */
//		
//		/* deadbands so centering joysticks always results in zero output */
//		forwardThrottle = Db(forwardThrottle);
//		turnThrottle = Db(turnThrottle);		
//
//		/* very simple Proportional and Derivative (PD) loop with a cap,
//		 * replace with favorite close loop strategy or leverage future Talon <=> Pigeon features. */
//		double error = _targetAngle - currentAngle;
//		SmartDashboard.putNumber("turnError", error);
//		
//		turnThrottle = (error) * kPgain - (currentAngularRate) * kDgain;		
//		SmartDashboard.putNumber("turnThrottle", turnThrottle);
//		
//		/* the max correction is the forward throttle times a scalar,
//		 * This can be done a number of ways but basically only apply small turning correction when we are moving slow
//		 * and larger correction the faster we move.  Otherwise you may need stiffer pgain at higher velocities. */
//		double maxThrot = MaxCorrection(forwardThrottle, kMaxCorrectionRatio);
//		turnThrottle = Cap(turnThrottle, maxThrot);
//
//		return turnThrottle;
//		
//
//	}
//
//	
//	/** @return 10% deadband */
//	double Db(double axisVal) {
//		if (axisVal < -0.10)
//			return axisVal;
//		if (axisVal > +0.10)
//			return axisVal;
//		return 0;
//	}
//	
//	/** @param value to cap.
//	 * @param peak positive double representing the maximum (peak) value.
//	 * @return a capped value.
//	 */
//	double Cap(double value, double peak) {
//		if (value < -peak)
//			return -peak;
//		if (value > +peak)
//			return +peak;
//		return value;
//	}
//
//	/**
//	 * Given the robot forward throttle and ratio, return the max
//	 * corrective turning throttle to adjust for heading.  This is
//	 * a simple method of avoiding using different gains for
//	 * low speed, high speed, and no-speed (zero turns).
//	 */
//	double MaxCorrection(double forwardThrot, double scalor) {
//		/* make it positive */
//		if(forwardThrot < 0) {forwardThrot = -forwardThrot;}
//		/* max correction is the current forward throttle scaled down */
//		forwardThrot *= scalor;
//		/* ensure caller is allowed at least 10% throttle,
//		 * regardless of forward throttle */
//		if(forwardThrot < 0.10)
//			return 0.10;
//		return forwardThrot;
//	}
	
	public void updateDashboard() 
	{
		SmartDashboard.putNumber("gyroCorrectError", _myHeadingPid.getError());
		SmartDashboard.putNumber("gyroCorrectOutput", _pidWriteOutput);
	}


	@Override
	public void calibrate() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void reset() {
		gyroReset();
	}

	@Override
	public double getAngle() {
		return _fusedHeading;
	}

	@Override
	public double getRate() {
		// TODO Auto-generated method stub
		return 0;
	}

	public void updateFusedHeading() {
		_fusedHeading = gyroGetFusedHeading();		
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		_pidWriteOutput = output;
	}
}
