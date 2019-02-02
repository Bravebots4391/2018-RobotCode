/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team4391.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4391.robot.Constants;
import team4391.robot.subsystems.Drive;
import team4391.util.InterpolatingDouble;
import team4391.util.InterpolatingTreeMap;
import team4391.util.SyncronousRateLimiter;

/**
 * Add your docs here.
 */
public class VisionPID implements PIDOutput, PIDSource {

    private PIDController _pid;

    public SyncronousRateLimiter _speedRate;
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> _speedProfile;
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> _areaToDistance;
    private VisionPIDData _output;

    private String _xOffsetVar = "tx";
    private String _areaVar = "ta";

    private double _alignmentBias = 1.0;
    private double _pidOutput;
    private Drive _sd;

    private Double _distance;

    static int _counter = 0;

    public VisionPID(Drive sd)
    {
        Init();
        _sd = sd;
    }

    private void Init()
    {
        _pid = new PIDController(0.01, 0.001, 0.0, this, this);

        _speedRate = new SyncronousRateLimiter(Constants.kLooperDt, 1.0 , 0);
        
        _speedProfile = new InterpolatingTreeMap<>();
        _areaToDistance = new InterpolatingTreeMap<>();

        // Setup PID parameters
        _pid.setOutputRange(-0.5, 0.5);

        _pid.setPID(0.1, 0.01, 0.0, 0.0);
    	_pid.setAbsoluteTolerance(0.1);
    	_pid.setOutputRange(-0.6, 0.6);
    	_pid.reset();

        //
        // Setup Mapping Profiles
        //
        // degrees per second update based on degrees from target.
        _speedProfile.put(new InterpolatingDouble(10.0), new InterpolatingDouble(1.0));
        _speedProfile.put(new InterpolatingDouble(5.0), new InterpolatingDouble(0.6));
        _speedProfile.put(new InterpolatingDouble(2.0), new InterpolatingDouble(0.1));

        _areaToDistance.put(new InterpolatingDouble(6.67), new InterpolatingDouble(1.0));
        _areaToDistance.put(new InterpolatingDouble(3.78), new InterpolatingDouble(2.0));
        _areaToDistance.put(new InterpolatingDouble(2.37), new InterpolatingDouble(3.0));
        _areaToDistance.put(new InterpolatingDouble(1.63), new InterpolatingDouble(4.0));
        _areaToDistance.put(new InterpolatingDouble(1.19), new InterpolatingDouble(5.0));
        _areaToDistance.put(new InterpolatingDouble(1.0), new InterpolatingDouble(6.0));
        _areaToDistance.put(new InterpolatingDouble(0.76), new InterpolatingDouble(7.0));
        _areaToDistance.put(new InterpolatingDouble(0.67), new InterpolatingDouble(8.0));

    }

    public void Update()
    {
        _counter++;

        UpdateDashboard();

        // get xOffset (degrees xOffset from Target)
        var xOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry(_xOffsetVar).getDouble(0);
        
        // get area (area of target)
        var area = NetworkTableInstance.getDefault().getTable("limelight").getEntry(_areaVar).getDouble(0);
        // convert area data from camera to a distance
        _distance = _areaToDistance.getInterpolated(new InterpolatingDouble(area)).value;

        // Get our heading based on x and distance vector 
        var heading = Math.atan((xOffset * _alignmentBias)/ _distance ) * (180/Math.PI);

        // Determine magnitude of summed vector
        var distanceVector = Math.sqrt(Math.pow(xOffset, 2) + Math.pow(_distance, 2));

        // Determine our rate of speed change based on the magnitude of the distance vector.  
        // Unless in speed control, this is really a percent speed value. 
        double speedRate = _speedProfile.getInterpolated(new InterpolatingDouble(_distance)).value;

        // Rate Limit Speed
        _speedRate.SetOutputRate(speedRate);
        _speedRate.update();
        var speedR = _speedRate.getOutput();

        // Update PID with rate limited input
        _pid.setSetpoint(speedR);

        // get the pid output....this will be our speed
        _output = new VisionPIDData(heading, _pidOutput); 

        SmartDashboard.putNumber("xxCount", _counter);
        SmartDashboard.putNumber("speedR", speedR);
        SmartDashboard.putNumber("xxDistance", _distance);
        SmartDashboard.putNumber("xxHeading", heading);
        SmartDashboard.putNumber("xxDistanceVector", distanceVector);
        SmartDashboard.putNumber("xxSpeedRate", speedRate);
        SmartDashboard.putNumber("xxOutput", _pidOutput);
    }

    public double GetDistance()
    {
        return _distance;
    }

    public VisionPIDData GetOutput()
    {
        return _output; 
    }

    public void Enable()
    {
        // update the initial values for the rate limiters
        // with the current target position. 
        _speedRate.Reset();
        _pid.reset();
        // Enable the PID
        _pid.enable();
    }

    public void Disable()
    {
        _pid.disable();

        _speedRate.Reset();
    }

	public void Reset() {
        Disable();

    }
    
    private void UpdateDashboard()
    {
        SmartDashboard.putData("visionPID", _pid);
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {

    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return _sd._avgSpeed;
    }

    @Override
    public void pidWrite(double output) {
        _pidOutput = output;
	}

}
