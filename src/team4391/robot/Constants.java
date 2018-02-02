package team4391.robot;

public class Constants {
	
	/**
	 * Which PID slot to pull gains from.  Starting 2018, you can choose 
	 * from 0,1,2 or 3.  Only the first two (0,1) are visible in web-based configuration.
	 */
	public static final int kSlotIdx = 0;
	
	/* Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops.  
	 * For now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/*
	 * set to zero to skip waiting for confirmation, set to nonzero to wait
	 * and report to DS if action fails.
	 */
	public static final int kTimeoutMs = 10;
	
	public static final int kEncoderCountsPerRev = 1024; //1024 when using the analog input
	
	public static final double kJoystickDeadband = 0.1;
	
	// Robot wheel dimentions
	public static final double Width = 26;
	public static final double Length = 26;
	
	
	public static double kLooperDt = 0.01;

	public static double kRotateMaxPctSpeed = 0.5;

	public static int kPCMId = 9;

	
	
	
	// Wheel position calibration
	public static final int kFrontLeftCal = 172;
	public static final int kFrontRightCal = 843;
	public static final int kRearLeftCal = 428;
	public static final int kRearRightCal = 284;
	
	// Cimcoder constants
	public static final int kCimcoderPulsesPerRev = 80;
	public static final double kSwerveDriveRatio = 6.67;
	public static final double kWheelDiameterInches = 4.0;
	
	// Talon CAN Address Number
	public static final int kFrontLeftDriveMotorId = 9;
	public static final int kFrontRightDriveMotorId = 1;
	public static final int kBackLeftDriveMotorId = 11;
	public static final int kBackRightDriveMotorId = 7;
	
	public static final int kFrontLeftTurnMotorId = 4;
	public static final int kFrontRightTurnMotorId = 3;
	public static final int kBackLeftTurnMotorId = 2;
	public static final int kBackRightTurnMotorId = 6;
	
	public static final int kSuckerInnerOuterId = 5;
	public static final int kCubevatorId = 10;
	public static final int kArmOpenyCloseyId = 53;
	
	//
	// ARM Constants
	//
	public static final double kArmInputPctSpeed = 1.0;
	public static final double kArmOutputPctSpeed = -1.0;
}
