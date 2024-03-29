package team4391.swerveDrive;

import team4391.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {

	public enum SwerveMode{
		crab, frontPivot, rearPivot, rotate, stop, coast, mcTwist, swerveAndTurn, airBrakes
	}
	
	private static WPI_TalonSRX _motorFR;
	private static WPI_TalonSRX _motorRR;
	private static WPI_TalonSRX _motorFL;
	private static WPI_TalonSRX _motorRL;	
	private static WPI_TalonSRX _turnFl;
	private static WPI_TalonSRX _turnFR;
	private static WPI_TalonSRX _turnBl;
	private static WPI_TalonSRX _turnBR;			
	private static Gyro _gyro;
	
	double _targetPositionDegrees;
	private static boolean _isTurning = false;
	private SwerveMode _swerveMode;
	private boolean _useSpeedControl = true;
	private SwerveAndRotate _swerveAndRotate;
	private boolean _isOkToRotate = false;
	
	public SwerveDrive(SwerveDriveMotorGroup motors, Gyro gyro)
	{
		_gyro = gyro;
		
		_motorFR = motors.get_motorFR();
		_motorRR = motors.get_motorRR();
		_motorFL = motors.get_motorFL();
		_motorRL = motors.get_motorRL();
		
		_turnFl = motors.get_turnFl();
		_turnFR = motors.get_turnFR();
		_turnBl = motors.get_turnBl();
		_turnBR = motors.get_turnBR();
		
		init();
		
		SmartDashboard.putData("pigeon", _gyro);
	}
	
	public void setUseSpeedCntrl(boolean isSpeedCntrl)
	{
		_useSpeedControl = isSpeedCntrl;
	}
	
	public void setDrive(SwerveMode mode, double pctSpeed, double angle)
	{
		setDrive(mode, pctSpeed, angle, 0.0);
	}
	
	public void setDrive(SwerveMode mode, double pctSpeed, double angle, double rotate)
	{			
		_swerveMode = mode;	
		
		switch(mode)
		{
		case crab:
			if(_isTurning)
			{
				_isTurning = false;
				//_gyro.stopDriveCorrection();
				//_gyro.reset();
				//_gyro.setupDriveCorrection(0.0);
			}
			_isOkToRotate = false;
			swerveAndTurn(pctSpeed, angle, 0.0);
			break;
			
		case frontPivot:
			_isTurning = true;
			pivotTurn(pctSpeed);
			break;
			
		case rearPivot:
			_isTurning = true;
			pivotTurnReverse(pctSpeed);
			break;
			
		case rotate:
			if(!_isOkToRotate && getVelocity(_motorFR) < 0.5)
			{
				_isTurning = true;
				rotateInPlace(pctSpeed);
				_isOkToRotate = true;
			}
			else if(_isOkToRotate)
			{
				rotateInPlace(pctSpeed);
			}
			break;
			
		case airBrakes:
		{
			if(!_isOkToRotate && getVelocity(_motorFR) < 0.5)
			{
				_isTurning = true;
				brakes();
				_isOkToRotate = true;
			}
			else if(_isOkToRotate)
			{
				brakes();
			}
			break;			
		}
			
//		case carTurn:
//			_isTurning = true;
//			carTurn(pctSpeed, angle);
//			break;
//			
//		case carTurnReverse:
//			_isTurning = true;
//			carTurnReverse(pctSpeed, angle);
//			break;
		
		case swerveAndTurn:
			_isTurning = true;
			_isOkToRotate = false;
			swerveAndTurn(pctSpeed, angle, rotate);
			break;
			
		case stop:
			swerveAndTurn(0, 0, 0);
			//_gyro.reset();
			break;
			
		case coast:
			coast();
			//_gyro.reset();
			break;
			
		case mcTwist:
			mcTwist(pctSpeed, angle);
			break;
			
		}
		
		
	}
		
	private void swerveAndTurn(double pctSpeed, double angle, double rotate) 
	{
		double myRotate = rotate;
		
		if(DriverStation.getInstance().isAutonomous())// pctSpeed > 0 && rotate == 0.0)
		{
			myRotate = _gyro.getDriveCorrection();
		}
		
		WheelPositionInfo wi = _swerveAndRotate.swerveAndTurn(angle, myRotate);
		
		SmartDashboard.putNumber("srFL", wi.getFlAngle());
		
		// set each wheelSpeed
		_motorFL.set(ControlMode.PercentOutput, pctSpeed * wi.getFlSpeed());
		_motorFR.set(ControlMode.PercentOutput, pctSpeed * wi.getfRSpeed());
		_motorRL.set(ControlMode.PercentOutput, pctSpeed * wi.getrLSpeed());
		_motorRR.set(ControlMode.PercentOutput, pctSpeed * wi.getrRSpeed());			
		
		//set all the wheels angles
		setWheelAngle(wi.getFlAngle(), _turnFl);
		setWheelAngle(wi.getFrAngle(), _turnFR);
		setWheelAngle(wi.getrLAngle(), _turnBl);
		setWheelAngle(wi.getrRAngle(), _turnBR);
	}

	public void rotateInPlace (double rotatePct) {
		rotateBot(rotatePct);
	}
	
	
	private void init() {
		// Set all turning wheels to brake mode
		_turnFl.setNeutralMode(NeutralMode.Brake);
		_turnFR.setNeutralMode(NeutralMode.Brake);
		_turnBl.setNeutralMode(NeutralMode.Brake);
		_turnBR.setNeutralMode(NeutralMode.Brake);
		
		_turnFl.setInverted(true);
		_turnFR.setInverted(true);
		_turnBl.setInverted(true);
		_turnBR.setInverted(true);
		
		SetupPIDAnalog(_turnFl, Constants.kFrontLeftCal);
		SetupPIDAnalog(_turnFR, Constants.kFrontRightCal);
		SetupPIDAnalog(_turnBl, Constants.kRearLeftCal);
		SetupPIDAnalog(_turnBR, Constants.kRearRightCal);		
		
		SetupDriveWheelPID(_motorFR);
		SetupDriveWheelPID(_motorRR);
		SetupDriveWheelPID(_motorFL);
		SetupDriveWheelPID(_motorRL);
		
		SetNeutralModeForDrive(NeutralMode.Coast);
		
		_motorFR.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		_motorFR.setSensorPhase(true);
		
		_swerveAndRotate = new SwerveAndRotate(Constants.Width, Constants.Length, Constants.kMaxRotateRadius);
	}
	
//	private void crabDrive(double pctSpeed, double heading)
//	{
//		SmartDashboard.putNumber("JoystickAngle", heading);
//		setAllWheelPositions(heading);
//		
//		boolean isForward = (heading >= 0 && heading <= 90 || heading >= 270 && heading <=360);
//		
//		setSpeedsForGyro(pctSpeed, heading);
//	}
	
	private void coast()
	{
		// set each wheelSpeed
		_motorFL.set(ControlMode.PercentOutput, 0);
		_motorFR.set(ControlMode.PercentOutput, 0);
		_motorRL.set(ControlMode.PercentOutput, 0);
		_motorRR.set(ControlMode.PercentOutput, 0);
	}			
	
	private void setAllMotorsDistance(int encoderPosition)
	{
		_motorFR.set(ControlMode.Position, encoderPosition);
		_motorRR.set(ControlMode.Position, encoderPosition);
		_motorFL.set(ControlMode.Position, encoderPosition);
		_motorRL.set(ControlMode.Position, encoderPosition);
	}

	private double speedToTargetVelocity(double pctSpeed) {
		// Convert Fps to RPM
		
		if(!_useSpeedControl)
		{
			return pctSpeed;
		}
		
		double speedFps = pctSpeed * Constants.kMaxDriveFPS;		
		double rpm = speedFps * ((720)/(Math.PI * Constants.kWheelDiameterInches))*Constants.kSwerveDriveRatio;
		
		// Convert RPM to units / 100ms.
		// 4096 Units/Rev * RPM / 600 100ms/min in either direction:
		// velocity setpoint is in units/100ms		 

		double targetVelocity_UnitsPer100ms = rpm * Constants.kCimcoderPulsesPerRev / 600;
		
		return targetVelocity_UnitsPer100ms;
	}
	
	private void rotateBot(double speed) {
				
		_isTurning = true; // let the rest of the module know we are in a turn mode
		
		//set all the wheels on the tangent
		setWheelAngle(45.0, _turnFl);
		setWheelAngle(315.0, _turnFR);
		setWheelAngle(315.0, _turnBl);
		setWheelAngle(45.0, _turnBR);
		
		// command forward or backward on the wheels	
		double rotateSpeed = speed * Constants.kRotateMaxPctSpeed;
		_motorFR.set(ControlMode.PercentOutput, -rotateSpeed);
		_motorRR.set(ControlMode.PercentOutput, -rotateSpeed);
		_motorFL.set(ControlMode.PercentOutput, rotateSpeed);
		_motorRL.set(ControlMode.PercentOutput, rotateSpeed);
	}	
	
	private void brakes()
	{
		_isTurning = true; // let the rest of the module know we are in a turn mode
		
		//set all the wheels on the tangent
		setWheelAngle(-45.0, _turnFl);
		setWheelAngle(45.0, _turnFR);
		setWheelAngle(45.0, _turnBl);
		setWheelAngle(-45.0, _turnBR);
		
		_motorFR.set(ControlMode.PercentOutput, 0);
		_motorRR.set(ControlMode.PercentOutput, 0);
		_motorFL.set(ControlMode.PercentOutput, 0);
		_motorRL.set(ControlMode.PercentOutput, 0);
	}
	
	public void pivotTurn(double speed)
	{
		setWheelAngle(0.0, _turnFl);
		setWheelAngle(0.0, _turnFR);
		
		// command forward or backward on the wheels	
		double rotateSpeed = Math.abs(speed);	
		double hypot = Math.sqrt(Math.pow(Constants.Width, 2) + Math.pow(Constants.Length, 2));
		double rotateShortWheel = (Constants.Width / hypot) * rotateSpeed;			
		
		_isTurning = true; // let the rest of the module know we are in a turn mode
		
		//set all the wheels on the tangent
		if(speed < 0)
		{
			setWheelAngle(90.0, _turnBl);
			setWheelAngle(45.0, _turnBR);
			
			_motorFL.set(ControlMode.PercentOutput, 0);
			_motorFR.set(ControlMode.PercentOutput, rotateShortWheel);
			_motorRL.set(ControlMode.PercentOutput, rotateShortWheel);
			_motorRR.set(ControlMode.PercentOutput, rotateSpeed);
		}
		if(speed > 0)
		{
			setWheelAngle(315.0, _turnBl);
			setWheelAngle(270.0, _turnBR);
			
			_motorFL.set(ControlMode.PercentOutput, rotateShortWheel);
			_motorFR.set(ControlMode.PercentOutput, 0);
			_motorRL.set(ControlMode.PercentOutput, rotateSpeed);
			_motorRR.set(ControlMode.PercentOutput, rotateShortWheel);
		}					
	}
	
	public void pivotTurnReverse(double speed)
	{
		setWheelAngle(180.0, _turnBl);
		setWheelAngle(180.0, _turnBR);
		
		// command forward or backward on the wheels	
		double rotateSpeed = Math.abs(speed);
		double hypot = Math.sqrt(Math.pow(Constants.Width, 2) + Math.pow(Constants.Length, 2));
		double rotateShortWheel = (Constants.Width / hypot) * rotateSpeed;			
		
		_isTurning = true; // let the rest of the module know we are in a turn mode
		
		//set all the wheels on the tangent
		if(speed < 0)
		{
			setWheelAngle(235, _turnFl);
			setWheelAngle(270.0, _turnFR);
			
			_motorFL.set(ControlMode.PercentOutput, rotateShortWheel);
			_motorFR.set(ControlMode.PercentOutput, rotateSpeed);
			_motorRL.set(ControlMode.PercentOutput, 0);
			_motorRR.set(ControlMode.PercentOutput, rotateShortWheel);
		}
		if(speed > 0)
		{
			setWheelAngle(90.0, _turnFl);
			setWheelAngle(135.0, _turnFR);
			
			_motorFL.set(ControlMode.PercentOutput, rotateSpeed);
			_motorFR.set(ControlMode.PercentOutput, rotateShortWheel);
			_motorRL.set(ControlMode.PercentOutput, rotateShortWheel);
			_motorRR.set(ControlMode.PercentOutput, 0);
		}					
	}	

	public void mcTwist(double forwardSpeedFps, double rotateRateFps) 
	{			
		// set all wheel positions to the negative of Heading		
		double gyroHeading = _gyro.getAngle();
		double wheelPos = 0.0 - gyroHeading;
		
		swerveAndTurn(forwardSpeedFps, wheelPos, rotateRateFps);				
	}	
	
//	private void setAllWheelPositions(double targetAngle)
//	{
//		setWheelAngle(targetAngle, _turnFl);
//		setWheelAngle(targetAngle, _turnFR);
//		setWheelAngle(targetAngle, _turnBl);
//		setWheelAngle(targetAngle, _turnBR);
//	}
	
	private void setWheelAngle(double targetAngle, TalonSRX srx) {
		
		SmartDashboard.putNumber("setWheelPos_targetAngle", targetAngle);
		
		// get current encoder position
		double position = (double)srx.getSelectedSensorPosition(Constants.kPIDLoopIdx);			
		SmartDashboard.putNumber("setWheelPos_position", position);
		
		// convert position to heading angle.
		double heading = convertEncoderPositionToAngle(position);
		SmartDashboard.putNumber("setWheelPos_heading", heading);
		
		// calculate error with target
		double error = targetAngle - heading;
		SmartDashboard.putNumber("setWheelPos_error", error);
		
		// find shortest path
		if(error > 180) error -=360;
		else if(error < -180) error += 360;			
		
		_targetPositionDegrees = targetAngle;
		double targetPosition = convertAngleToEncoderPosition(error) + position;
		
		SmartDashboard.putNumber("setWheelPos_error", error);
		SmartDashboard.putNumber("setWheelPos_targetPos", targetPosition);
		
		srx.set(ControlMode.Position, targetPosition);
	}
	
	public void UpdateDashboard()
	{	
		SmartDashboard.putNumber("positionRawFL", _turnFl.getSelectedSensorPosition(0));
		//SmartDashboard.putNumber("positionDegreesFL", convertEncoderPositionToAngle(_turnFl.getSelectedSensorPosition(0)));
		SmartDashboard.putNumber("positionRawFR", _turnFR.getSelectedSensorPosition(0));
		//SmartDashboard.putNumber("positionDegreesFR", convertEncoderPositionToAngle(_turnFR.getSelectedSensorPosition(0)));
		SmartDashboard.putNumber("positionRawBL", _turnBl.getSelectedSensorPosition(0));
		//SmartDashboard.putNumber("positionDegreesBL", convertEncoderPositionToAngle(_turnBl.getSelectedSensorPosition(0)));
		SmartDashboard.putNumber("positionRawBR", _turnBR.getSelectedSensorPosition(0));
		//SmartDashboard.putNumber("positionDegreesBR", convertEncoderPositionToAngle(_turnBR.getSelectedSensorPosition(0)));
		
		SmartDashboard.putNumber("FL Output", _motorFL.getMotorOutputPercent());
		
		SmartDashboard.putNumber("motorCurrent", _motorFR.getOutputCurrent());		
		SmartDashboard.putNumber("targetPosition", _targetPositionDegrees);		
		SmartDashboard.putNumber("closedLoopError", _turnFl.getClosedLoopError(0));		
		SmartDashboard.putNumber("Distance(inches)", getDistanceInches());		
				
		SmartDashboard.putNumber("SpeedFR",getVelocity(_motorFR));
		SmartDashboard.putNumber("SpeedFL",getVelocity(_motorFL));
		SmartDashboard.putNumber("SpeedRR",getVelocity(_motorRR));
		SmartDashboard.putNumber("SpeedRL",getVelocity(_motorRL));					
		
		SmartDashboard.putString("SwerveMode", _swerveMode.toString());
	}	
	
	private double convertAngleToEncoderPosition(double angle) 
	{	
		double position = (angle / 360.0) * Constants.kEncoderCountsPerRev;		
		return position;
	}
	
	private double convertEncoderPositionToAngle(double position)
	{
		// 	Positions are always in sensor units. 
		//  Quadrature units are 4X measurements 
		//  where X units represents X quadrature edges.		
		
		double countsPerRev = Constants.kEncoderCountsPerRev;		
		double rotations = position / countsPerRev;
		
		double angle = (rotations * 360) % 360;
		
		// always express as a positive angle
		if(angle < 0) {
			angle+=360;
		}
		
		return angle;
	}
	
	private double getVelocity(TalonSRX talon)
	{
		double x = 1/((Constants.kCimcoderPulsesPerRev * Constants.kSwerveDriveRatio)/(Constants.kWheelDiameterInches * Math.PI));		
		//double position = talon.getSelectedSensorPosition(0);
		
		double velocity = talon.getSelectedSensorVelocity(0);
		
		return velocity*x*(10.0/12.0);
	}
	
	public double getDistanceInches()
	{
		double d1 = Math.abs(getDistanceInches(_motorFR));
		double d2 = Math.abs(getDistanceInches(_motorRR));
		double d3 = Math.abs(getDistanceInches(_motorFL));
		double d4 = Math.abs(getDistanceInches(_motorRL));
		
		double distanceAvg = (d1 + d2 + d3 + d4)/4.0;
		
		return distanceAvg;
	}
	
	private double getDistanceInches(TalonSRX talon)
	{
		double x = 1/((Constants.kCimcoderPulsesPerRev * Constants.kSwerveDriveRatio)/(Constants.kWheelDiameterInches * Math.PI));
		
		double position = talon.getSelectedSensorPosition(0);
		
		return position*x;
	}
	
	private void resetDistance(TalonSRX talon){
		talon.setSelectedSensorPosition(0,  Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}
		
	public void SetNeutralModeForDrive(NeutralMode mode)
	{
		_motorFR.setNeutralMode(mode);
		_motorRR.setNeutralMode(mode);
		_motorFL.setNeutralMode(mode);
		_motorRL.setNeutralMode(mode);
	}
	
	private void SetupDriveWheelPID(TalonSRX talon)
	{	
		talon.configClosedloopRamp(0.5, Constants.kTimeoutMs);
		
		talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kTimeoutMs);
		talon.setSensorPhase(false);

		/* set the peak, nominal outputs */
		talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* set closed loop gains in slot0 */
		talon.config_kF(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		talon.config_kP(Constants.kPIDLoopIdx, 5.0, Constants.kTimeoutMs);
		talon.config_kI(Constants.kPIDLoopIdx, 0.0001, Constants.kTimeoutMs);
		talon.config_kD(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);
	}
	
	private void SetupPIDAnalog(TalonSRX talon, int positionCal)
	{
		int raw = talon.getSensorCollection().getAnalogInRaw();
		
		int absolutePosition = talon.getSelectedSensorPosition(Constants.kTimeoutMs); 
		
		if(!Constants.kCalibrateSwerves)
		{
			talon.getSensorCollection().setAnalogPosition(raw - positionCal, Constants.kTimeoutMs);
		}
		
		/* choose the sensor and sensor direction */
        talon.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		
		/* lets grab the 360 degree position of the encoders absolute position */
		absolutePosition = talon.getSelectedSensorPosition(Constants.kTimeoutMs); /* mask out the bottom12 bits, we don't care about the wrap arounds */
		
        /* choose the sensor and sensor direction */
        //talon.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        talon.setSensorPhase(false);
                       
        /* set the peak and nominal outputs, 12V means full */
        talon.configNominalOutputForward(0, Constants.kTimeoutMs);
        talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
        talon.configPeakOutputForward(1, Constants.kTimeoutMs);
        talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        /* set the allowable closed-loop error,
         * Closed-Loop output will be neutral within this range.
         * See Table in Section 17.2.1 for native units per rotation. 
         */
        talon.configAllowableClosedloopError(3, Constants.kPIDLoopIdx, Constants.kTimeoutMs); /* always servo */
        /* set closed loop gains in slot0 */
        talon.config_kF(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
        talon.config_kP(Constants.kPIDLoopIdx, 10.0, Constants.kTimeoutMs);
        talon.config_kI(Constants.kPIDLoopIdx, 0.000, Constants.kTimeoutMs);
        talon.config_kD(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
	}

	public Gyro getGyro() {
		// TODO Auto-generated method stub
		return _gyro;
	}

	public void resetDistance() {
		resetDistance(_motorFR);
		resetDistance(_motorRR);
		resetDistance(_motorFL);
		resetDistance(_motorRL);
	}

}
