package team4391.swerveDrive;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Gyro extends GyroBase
{
	PigeonIMU _pidgey;
	
	/*
	 * Some gains for heading servo, these were tweaked by using the web-based
	 * config (CAN Talon) and pressing gamepad button 6 to load them.
	 */
	double kPgain = 0.04; /* percent throttle per degree of error */
	double kDgain = 0.0004; /* percent throttle per angular velocity dps */
	double kMaxCorrectionRatio = 0.30; /* cap corrective turning throttle to 30 percent of forward throttle */
	/** holds the current angle to servo to */
	double _targetAngle = 0;
		

	private boolean _isInitOk;
	
	public Gyro(int id){
		if(id != -1)
		{
			gyroInit(id);
			_isInitOk = true;
		}
		else
		{
			_isInitOk = false;
		}
	}
	
	public void gyroInit(int talonId){
		_pidgey = new PigeonIMU(talonId);		
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
		
			return currentAngle;
		}
		else{
			return 0.0;
		}
	}
	
	public GyroOutput getDriveCorrection(double throttle, double joystickY) 
	{
		if(!_isInitOk)
		{
			return new GyroOutput(throttle, throttle);
		}
		
		/* some temps for Pigeon API */
		PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		double [] xyz_dps = new double [3];
		
		/* grab some input data from Pigeon and gamepad*/		
		_pidgey.getGeneralStatus(genStatus);
		_pidgey.getRawGyro(xyz_dps);
		_pidgey.getFusedHeading(fusionStatus);
		double currentAngle = fusionStatus.heading;
		boolean angleIsGood = (_pidgey.getState() == PigeonIMU.PigeonState.Ready) ? true : false;
		double currentAngularRate = xyz_dps[2];
		
		/* get input from gamepad */		
		double forwardThrottle = throttle;// = _driveStick.getAxis(AxisType.kY) * -1.0; /* sign so that positive is forward */
		double turnThrottle = 0.0;// = _driveStick.getAxis(AxisType.kTwist) * -1.0; /* sign so that positive means turn left */
		
		/* deadbands so centering joysticks always results in zero output */
		forwardThrottle = Db(forwardThrottle);
		turnThrottle = Db(turnThrottle);		

		/* very simple Proportional and Derivative (PD) loop with a cap,
		 * replace with favorite close loop strategy or leverage future Talon <=> Pigeon features. */
		double error = _targetAngle - currentAngle;
		SmartDashboard.putNumber("turnError", error);
		
		turnThrottle = (error) * kPgain - (currentAngularRate) * kDgain;		
		SmartDashboard.putNumber("turnThrottle", turnThrottle);
		
		/* the max correction is the forward throttle times a scalar,
		 * This can be done a number of ways but basically only apply small turning correction when we are moving slow
		 * and larger correction the faster we move.  Otherwise you may need stiffer pgain at higher velocities. */
		double maxThrot = MaxCorrection(forwardThrottle, kMaxCorrectionRatio);
		turnThrottle = Cap(turnThrottle, maxThrot);


		/* positive turnThrottle means turn to the left, this can be replaced with ArcadeDrive object, or teams drivetrain object */
	
		double left = forwardThrottle - turnThrottle;
		double right = forwardThrottle + turnThrottle;
		
		if(joystickY < 0)
		{
			left = forwardThrottle + turnThrottle;
			right = forwardThrottle - turnThrottle;
		}
		
		left = Cap(left, 1.0);
		right = Cap(right, 1.0);
	
		return new GyroOutput(left, right);					  
	}

	
	/** @return 10% deadband */
	double Db(double axisVal) {
		if (axisVal < -0.10)
			return axisVal;
		if (axisVal > +0.10)
			return axisVal;
		return 0;
	}
	
	/** @param value to cap.
	 * @param peak positive double representing the maximum (peak) value.
	 * @return a capped value.
	 */
	double Cap(double value, double peak) {
		if (value < -peak)
			return -peak;
		if (value > +peak)
			return +peak;
		return value;
	}

	/**
	 * Given the robot forward throttle and ratio, return the max
	 * corrective turning throttle to adjust for heading.  This is
	 * a simple method of avoiding using different gains for
	 * low speed, high speed, and no-speed (zero turns).
	 */
	double MaxCorrection(double forwardThrot, double scalor) {
		/* make it positive */
		if(forwardThrot < 0) {forwardThrot = -forwardThrot;}
		/* max correction is the current forward throttle scaled down */
		forwardThrot *= scalor;
		/* ensure caller is allowed at least 10% throttle,
		 * regardless of forward throttle */
		if(forwardThrot < 0.10)
			return 0.10;
		return forwardThrot;
	}
	
	void updateDashboard() {
		//SmartDashboard.putData(this);
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
		return gyroGetFusedHeading();
	}

	@Override
	public double getRate() {
		// TODO Auto-generated method stub
		return 0;
	}
}
