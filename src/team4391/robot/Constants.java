package team4391.robot;


import edu.wpi.first.wpilibj.Preferences;
import team4391.util.InterpolatingDouble;
import team4391.util.InterpolatingTreeMap;

public class Constants {
	
	public static final boolean useSlaveMotors = false;
	
	public static final double toDegrees = 180 / Math.PI;
	
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
	
	public static double kMaxDriveFPS = 11;
	
	// Robot wheel dimensions
	public static final double Width = 24.5;
	public static final double Length = 23.0;
	public static final int kRobotRadius = 17;
	public static final double kMaxRotateRadius = 100.0;  // Inches for swerveAndRotate
 		
	public static double kLooperDt = 0.01;
	public static double kRotateMaxPctSpeed = 0.5;		
	
	
	// Wheel position calibration
	
	//Set to true
	//Shut off bot
	//Line up wheels
	//turn on bot
	//set true
	//push code/restart
	
	public static final boolean kCalibrateSwerves = false;
	public static final int kFrontLeftCal = /*415;*/ 851;
	public static final int kFrontRightCal = /*846;*/ 40;
	public static final int kRearLeftCal = /*375;*/ 502;
	public static final int kRearRightCal = /*846;*/ 719;
	
	// Cimcoder constants
	public static final int kCimcoderPulsesPerRev = 80;
	public static final double kSwerveDriveRatio = 6.67;
	public static final double kWheelDiameterInches = 4.0;
	
	// CAN Address Number
	public static final int kFrontLeftDriveMotorId = 11;
	public static final int kFrontRightDriveMotorId = 9;
	public static final int kBackLeftDriveMotorId = 7;
	public static final int kBackRightDriveMotorId = 1;
	
	public static final int kFrontLeftTurnMotorId = 2;
	public static final int kFrontRightTurnMotorId = 4;
	public static final int kBackLeftTurnMotorId = 6;
	public static final int kBackRightTurnMotorId = 3;
	
	public static final int kArmRightId = 5;
	public static final int kArmLeftId = 8;
	public static final int kCubevatorId = 10;
	public static final int kCubevatorSlaveId = 12;
	
	public static final int kClimberId = 13;
	
	public static final int kPCMId = 9;	
	public static final int kPDP = 0;
	public static final int kPigeonGyroId = kCubevatorSlaveId;  // The pigeon is plugged into the suckerInnerOuter talon.	
	
	//
	// ARM Constants
	//
	public static final double kArmInputPctSpeed = 1.0;
	public static final double kArmOutputPctSpeed = -0.2;
	public static final double kArmFastOutputPctSpeed = -1.0;
	
	//
	// Cubevator Constants
	//
	public static final int kCubevatorEncoderCountsPerRev = 4096; // 1:1 with the output shaft
	public static final double kCubevatorDrumDiameterInches = 3.5;
	public static final double kCubevatorTopLimitInches = 90;
	public static final double kCubevatorScaleHeight = 78; // height to throw into scale roughly
	
	public static final double kCubevatorClimbHeight = 80.5;  // height to get above to the rung
	public static final double kCubevatorDefaultHeight = 30.5;
	public static final double kCubevatorFeederHeight = 26.5;
	
    // PID gains for rotating in place
    public static double kDriveTurnKp = 0.05;
    public static double kDriveTurnKi = 0.0;
    public static double kDriveTurnKd = 0.05;
    public static double kDriveTurnKf = 0.0;
    public static double kDriveTurnAbsTollerance = 0.8;
    
    
	// Gyro Tuning
    public static final double GyroKp = 0.05;
	public static final double GyroKd = 0.0004;
	public static final double GyroKi = 0;
	
    // Autonomous Constants
	public static final double DriveDistance = 24;
	public static final double Speed = 0.8;
	public static final double Heading = 0.0;

	
    // Rotation PID Rate Limit Constants. In Auto Mode  Limits rotation rate based on angle from the target.
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kRateLimitMapAuto = new InterpolatingTreeMap<>();

    static {
    	kRateLimitMapAuto.put(new InterpolatingDouble(-10.0), new InterpolatingDouble(-8.0));
    	kRateLimitMapAuto.put(new InterpolatingDouble(-5.0), new InterpolatingDouble(-5.0));
    	kRateLimitMapAuto.put(new InterpolatingDouble(-1.0), new InterpolatingDouble(-2.5));
    	kRateLimitMapAuto.put(new InterpolatingDouble(1.0), new InterpolatingDouble(2.5));
    	kRateLimitMapAuto.put(new InterpolatingDouble(5.0), new InterpolatingDouble(5.0));
    	kRateLimitMapAuto.put(new InterpolatingDouble(10.0), new InterpolatingDouble(8.0));
    }    
    
    // Rotation PID Rate Limit Constants.  Limits for normal turning commands.
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kRateLimitMap = new InterpolatingTreeMap<>();

	public static double kCubevatorFirstStageHeightInches = 40;

	public static double kDriveWhileHighRotateLimit = 0.55;
	public static double kDriveWhileHighLimit2 = 0.55;

	



    static {
    	kRateLimitMap.put(new InterpolatingDouble(-40.0), new InterpolatingDouble(-60.0));
    	kRateLimitMap.put(new InterpolatingDouble(-10.0), new InterpolatingDouble(-15.0));
    	kRateLimitMap.put(new InterpolatingDouble(-2.0), new InterpolatingDouble(-7.0));
    	kRateLimitMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(7.0));
    	kRateLimitMap.put(new InterpolatingDouble(10.0), new InterpolatingDouble(15.0));
    	kRateLimitMap.put(new InterpolatingDouble(40.0), new InterpolatingDouble(60.0));
    }
    
    public static void putValuesInNetworkTables()
    {
    	Preferences prefs = Preferences.getInstance();
    	
    	prefs.putDouble("targetKp", Constants.kDriveTurnKp);
    	prefs.putDouble("targetKi", Constants.kDriveTurnKi);
    	prefs.putDouble("targetKd", Constants.kDriveTurnKd);
    	prefs.putDouble("targetKf", Constants.kDriveTurnKf);
    	
    	prefs.putDouble("GyroKp", Constants.GyroKp);
    	prefs.putDouble("GyroKd", Constants.GyroKd);
    	prefs.putDouble("GyroKi", Constants.GyroKi);
    	prefs.putDouble("height", Constants.kCubevatorDefaultHeight);
    	prefs.putDouble("DFDDistance", Constants.DriveDistance);
    	prefs.putDouble("DFDSpeed", Constants.Speed);
    	prefs.putDouble("DFDHeading", Constants.Heading);
    	prefs.putDouble("FeederHeight", kCubevatorFeederHeight);
    }
}
