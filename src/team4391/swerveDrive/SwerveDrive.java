package team4391.swerveDrive;


import team4391.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {

	public enum SwerveMode{
		crab, frontPivot, rearPivot, rotate, carTurn, carTurnReverse
	}
	
	private double toDegrees = 180 / Math.PI;
	
	private static WPI_TalonSRX _motorFR = new WPI_TalonSRX(Constants.kFrontRightDriveMotorId);
	private static WPI_TalonSRX _motorRR = new WPI_TalonSRX(Constants.kBackRightDriveMotorId);
	private static WPI_TalonSRX _motorFL = new WPI_TalonSRX(Constants.kFrontLeftDriveMotorId);
	private static WPI_TalonSRX _motorRL = new WPI_TalonSRX(Constants.kBackLeftDriveMotorId);
	
	private static WPI_TalonSRX _turnFl = new WPI_TalonSRX(Constants.kFrontLeftTurnMotorId);
	private static WPI_TalonSRX _turnFR = new WPI_TalonSRX(Constants.kFrontRightTurnMotorId);
	private static WPI_TalonSRX _turnBl = new WPI_TalonSRX(Constants.kBackLeftTurnMotorId);
	private static WPI_TalonSRX _turnBR = new WPI_TalonSRX(Constants.kBackRightTurnMotorId);
	
	private static Gyro _gyro = new Gyro(Constants.kPigeonGyroId);
	
	double _targetPositionDegrees;
	private static boolean _isTurning = false;
	private SwerveMode _swerveMode;
	
	public SwerveDrive(){
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
			
		}
	}
	
//	public void setDrive(double leftStickX, double leftStickY, double rightStickX, boolean isPivot) {
//    	
//		if(DB(leftStickX) || DB(leftStickY)) {
//			
//			if(DB(rightStickX) && !isPivot) {
//				crabDrive(leftStickX, leftStickY, rightStickX);
//			}
//			else if(leftStickY > 0 && DB(rightStickX) && isPivot){
//				pivotTurn(rightStickX);
//			}	
//			else if(leftStickY < 0 && DB(rightStickX) && isPivot){
//				pivotTurnReverse(rightStickX);
//			}
//			else {
//				crabDrive(leftStickX, leftStickY, 0);
//			}			
//		}		
//		else if(!DB(leftStickX) && !DB(leftStickY) && DB(rightStickX)) {
//			rotateInPlace(rightStickX);
//		}		
//		else {
//			_gyro.gyroReset();
//			setAllSpeeds(0, true);
//		}
//	}
	
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
		
		_motorFR.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		_motorFR.setSensorPhase(true);
	}
	
	private boolean DB(double x) {
		return Math.abs(x) > 0;
	}
	
	private void crabDrive(double pctSpeed, double angle)
	{
		SmartDashboard.putNumber("JoystickAngle", angle);
		setAllWheelPositions(angle);
		
		boolean isForward = (angle > 0 && angle <= 90 || angle >= 270 && angle <=360);
		
		setAllSpeeds(pctSpeed, isForward);
	}
	
	private void crabDrive(double leftStickX, double leftStickY, double rotate)
	{
		if (rotate == 0) {
			
			if(_isTurning){
				_isTurning = false;
				_gyro.reset();
			}
			
			double angle = ConvertJoystickXYtoAngle(leftStickX, leftStickY);
			SmartDashboard.putNumber("JoystickAngle", angle);
			setAllWheelPositions(angle);
			// calculate speed
			double speed = Math.pow(Math.sqrt(leftStickX * leftStickX + leftStickY * leftStickY), 2);
			setAllSpeeds(speed, leftStickY>0);
		}
		else if(Math.abs(rotate)>0 && leftStickY > 0){
			carTurn(leftStickY, rotate);
		}
		else if (Math.abs(rotate)>0 && leftStickY < 0){
			carTurnReverse(leftStickY, rotate);
		}
	}
	
	private void setAllSpeeds(double speed, boolean isForward)
	{
		GyroOutput data = _gyro.getDriveCorrection(speed, isForward);
		
		_motorFR.set(ControlMode.PercentOutput, data.get_right());
		_motorRR.set(ControlMode.PercentOutput, data.get_right());
		_motorFL.set(ControlMode.PercentOutput, data.get_left());
		_motorRL.set(ControlMode.PercentOutput, data.get_left());
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
		double BackWheelAngle = rotate * 135.0;
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
		double position = srx.getSelectedSensorPosition(Constants.kPIDLoopIdx);			
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
		
		SmartDashboard.putNumber("Speed",getVelocity(_motorFR));
				
		SmartDashboard.putData("pigeon", _gyro);	
		
		SmartDashboard.putString("SwerveMode", _swerveMode.toString());
	}	
	
	private double ConvertJoystickXYtoAngle(double x, double y)
	{	
		double angle = (Math.atan2(y, x) * toDegrees);		
		
		if(x == 0 && y == 0)
		{
			angle = 90;
			_gyro.gyroReset();
		}

		// convert the polar coordinate to a heading
		double coordinate = ((450 - angle) % 360);
		
		return coordinate;
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
		return angle;
	}
	
	private double getVelocity(TalonSRX talon)
	{
		double x = 1/((Constants.kCimcoderPulsesPerRev * Constants.kSwerveDriveRatio)/(Constants.kWheelDiameterInches * Math.PI));		
		//double position = talon.getSelectedSensorPosition(0);
		
		double velocity = talon.getSelectedSensorVelocity(0);
		
		return velocity*x*(10.0/12.0);
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
	
//	private void resetPosition()
//	{
//		_turnFl.getSensorCollection().setQuadraturePosition(0, 10);
//		
//		_turnFl.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);		
//		_turnFR.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
//		_turnBl.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
//		_turnBR.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
//	}
	
//	private void updatePositionForWrapAround()
//	{
//		int pos = _turnFl.getSelectedSensorPosition(Constants.kPIDLoopIdx);
//		_turnFl.setSelectedSensorPosition(pos%Constants.kEncoderCountsPerRev, Constants.kPIDLoopIdx, Constants.kTimeoutMs);				
//		
//		int pos2 = _turnFR.getSelectedSensorPosition(Constants.kPIDLoopIdx);
//		_turnFR.setSelectedSensorPosition(pos2%Constants.kEncoderCountsPerRev, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
//		
//		int pos3 = _turnBl.getSelectedSensorPosition(Constants.kPIDLoopIdx);
//		_turnBl.setSelectedSensorPosition(pos3%Constants.kEncoderCountsPerRev, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
//		
//		int pos4 = _turnBR.getSelectedSensorPosition(Constants.kPIDLoopIdx);
//		_turnBR.setSelectedSensorPosition(pos4%Constants.kEncoderCountsPerRev, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
//	}
		
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

	public PIDSource getGyro() {
		// TODO Auto-generated method stub
		return _gyro;
	}

}
