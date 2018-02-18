package team4391.swerveDrive;

import team4391.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {

	public enum SwerveMode{
		crab, frontPivot, rearPivot, rotate, carTurn, carTurnReverse, stop, coast, mcTwist
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
	private boolean _useSpeedControl = false;
	
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
	}
	
	public void setDrive(SwerveMode mode, double pctSpeed, double angle)
	{
		_swerveMode = mode;						
		
		switch(mode)
		{
		case crab:
			if(_isTurning)
			{
				_isTurning = false;
				_gyro.reset();
			}
			
			crabDrive(pctSpeed, angle);
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
			_isTurning = true;
			rotateInPlace(pctSpeed);
			break;
			
		case carTurn:
			_isTurning = true;
			carTurn(pctSpeed, angle);
			break;
			
		case carTurnReverse:
			_isTurning = true;
			carTurnReverse(pctSpeed, angle);
			break;
		
		case stop:
			crabDrive(0, 0);
			_gyro.reset();
			break;
			
		case coast:
			coast();
			_gyro.reset();
			break;
			
		case mcTwist:
			mcTwist(pctSpeed, angle);
			break;
			
		}
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
	}
	
	private void crabDrive(double pctSpeed, double heading)
	{
		SmartDashboard.putNumber("JoystickAngle", heading);
		setAllWheelPositions(heading);
		
		boolean isForward = (heading > 0 && heading <= 90 || heading >= 270 && heading <=360);
		
		setAllSpeeds(pctSpeed, isForward);
	}
	
	private void coast()
	{
		setAllSpeeds(0, true);
	}
			
	private void setAllSpeeds(double pctSpeed, boolean isForward)
	{
		if(_useSpeedControl)
		{		
			if(Math.abs(pctSpeed) == 0)
			{
				_motorFR.set(ControlMode.Disabled, 0);
				_motorRR.set(ControlMode.Disabled, 0);
				_motorFL.set(ControlMode.Disabled, 0);
				_motorRL.set(ControlMode.Disabled, 0);
			}
			else
			{
				setAllSpeedsFps(pctSpeed, isForward);			
			}
			
			return;
		}
		
		GyroOutput data = _gyro.getDriveCorrection(pctSpeed, isForward);
		
		_motorFR.set(ControlMode.PercentOutput, data.get_right());
		_motorRR.set(ControlMode.PercentOutput, data.get_right());
		_motorFL.set(ControlMode.PercentOutput, data.get_left());
		_motorRL.set(ControlMode.PercentOutput, data.get_left());
	}
		
	private void setAllSpeedsFps(double pctSpeed, boolean isForward)
	{	
		GyroOutput data = _gyro.getDriveCorrection(pctSpeed, isForward);
		
		//double speed = speedToTargetVelocity(pctSpeed);
		
		_motorFR.set(ControlMode.Velocity, speedToTargetVelocity(data.get_right()));
		_motorRR.set(ControlMode.Velocity, speedToTargetVelocity(data.get_right()));
		_motorFL.set(ControlMode.Velocity, speedToTargetVelocity(data.get_left()));
		_motorRL.set(ControlMode.Velocity, speedToTargetVelocity(data.get_left()));
	}

	private double speedToTargetVelocity(double pctSpeed) {
		// Convert Fps to RPM
		
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
	
	public void carTurnReverse(double ReverseSpeed, double rotate)
	{
		double BackWheelAngle = rotate * 135;
		_isTurning = true; // let the rest of the module know we are in a turn mode
		
		setWheelAngle(180.0, _turnFl);
		setWheelAngle(180.0, _turnFR);
		setWheelAngle(BackWheelAngle, _turnBl);
		setWheelAngle(BackWheelAngle, _turnBR);
		
		_motorFR.set(ControlMode.PercentOutput, ReverseSpeed);
		_motorRR.set(ControlMode.PercentOutput, ReverseSpeed);
		_motorFL.set(ControlMode.PercentOutput, ReverseSpeed);
		_motorRL.set(ControlMode.PercentOutput, ReverseSpeed);
	}
	
	public void carTurn(double forwardSpeed, double rotate)
	{
		double frontWheelAngle = rotate * 45.0;
		_isTurning = true; // let the rest of the module know we are in a turn mode
		
		setWheelAngle(frontWheelAngle, _turnFl);
		setWheelAngle(frontWheelAngle, _turnFR);
		setWheelAngle(0.0, _turnBl);
		setWheelAngle(0.0, _turnBR);
		
		_motorFR.set(ControlMode.PercentOutput, forwardSpeed);
		_motorRR.set(ControlMode.PercentOutput, forwardSpeed);
		_motorFL.set(ControlMode.PercentOutput, forwardSpeed);
		_motorRL.set(ControlMode.PercentOutput, forwardSpeed);
	}

	public void mcTwist(double forwardSpeedFps, double rotateRateFps) 
	{			
		// set all wheel positions to the negative of Heading		
		double heading = _gyro.getAngle();
		double wheelPos = -_gyro.getAngle();
		
		setAllWheelPositions(wheelPos);
		
		// get Theta for each wheel
		double FLt = heading + 315;
		double FRt = heading + 45;
		double RLt = heading + 225;
		double RRt = heading + 135;
		
		// Calculate each wheel speed offset
		double FLs = forwardSpeedFps + (-rotateRateFps)*Math.sin(FLt)*0.5;
		double FRs = forwardSpeedFps + (-rotateRateFps)*Math.sin(FRt)*0.5;
		double RLs = forwardSpeedFps + (-rotateRateFps)*Math.sin(RLt)*0.5;
		double RRs = forwardSpeedFps + (-rotateRateFps)*Math.sin(RRt)*0.5;
		
		SmartDashboard.putNumber("mcTwistSpeed", FLs);
		SmartDashboard.putNumber("mcTwistSpeed1", FRs);
		SmartDashboard.putNumber("mcTwistSpeed2", RLs);
		SmartDashboard.putNumber("mcTwistSpeed3", RRs);
		
		//11*speedToTargetVelocity(speedFps)
		
		_motorFL.set(ControlMode.Velocity, 11*speedToTargetVelocity(FLs));
		_motorFR.set(ControlMode.Velocity, 11*speedToTargetVelocity(FRs));
		_motorRL.set(ControlMode.Velocity, 11*speedToTargetVelocity(RLs));
		_motorRR.set(ControlMode.Velocity, 11*speedToTargetVelocity(RRs));			
	}	
	
	private void setAllWheelPositions(double targetAngle)
	{
		setWheelAngle(targetAngle, _turnFl);
		setWheelAngle(targetAngle, _turnFR);
		setWheelAngle(targetAngle, _turnBl);
		setWheelAngle(targetAngle, _turnBR);
	}
	
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
		SmartDashboard.putNumber("positionDegreesFL", convertEncoderPositionToAngle(_turnFl.getSelectedSensorPosition(0)));
		SmartDashboard.putNumber("positionRawFR", _turnFR.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("positionDegreesFR", convertEncoderPositionToAngle(_turnFR.getSelectedSensorPosition(0)));
		SmartDashboard.putNumber("positionRawBL", _turnBl.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("positionDegreesBL", convertEncoderPositionToAngle(_turnBl.getSelectedSensorPosition(0)));
		SmartDashboard.putNumber("positionRawBR", _turnBR.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("positionDegreesBR", convertEncoderPositionToAngle(_turnBR.getSelectedSensorPosition(0)));
		
		SmartDashboard.putNumber("motorCurrent", _motorFR.getOutputCurrent());		
		SmartDashboard.putNumber("targetPosition", _targetPositionDegrees);		
		SmartDashboard.putNumber("closedLoopError", _turnFl.getClosedLoopError(0));		
		SmartDashboard.putNumber("Distance(inches)", getDistanceInches(_motorFR));		
				
		SmartDashboard.putNumber("SpeedFR",getVelocity(_motorFR));
		SmartDashboard.putNumber("SpeedFL",getVelocity(_motorFL));
		SmartDashboard.putNumber("SpeedRR",getVelocity(_motorRR));
		SmartDashboard.putNumber("SpeedRL",getVelocity(_motorRL));
				
		SmartDashboard.putData("pigeon", _gyro);	
		
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
		double d1 = getDistanceInches(_motorFR);
		double d2 = getDistanceInches(_motorRR);
		double d3 = getDistanceInches(_motorFL);
		double d4 = getDistanceInches(_motorRL);
		
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
		//talon.configClosedloopRamp(0.5, Constants.kTimeoutMs);
		
		talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kTimeoutMs);
		talon.setSensorPhase(false);

		/* set the peak, nominal outputs */
		talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* set closed loop gains in slot0 */
		talon.config_kF(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		talon.config_kP(Constants.kPIDLoopIdx, 7.0, Constants.kTimeoutMs);
		talon.config_kI(Constants.kPIDLoopIdx, 0.0001, Constants.kTimeoutMs);
		talon.config_kD(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);
	}
	
	private void SetupPIDAnalog(TalonSRX talon, int positionCal)
	{
		int raw = talon.getSensorCollection().getAnalogInRaw();
		
		int absolutePosition = talon.getSelectedSensorPosition(Constants.kTimeoutMs); 
		talon.getSensorCollection().setAnalogPosition(raw - positionCal, Constants.kTimeoutMs);
		
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
