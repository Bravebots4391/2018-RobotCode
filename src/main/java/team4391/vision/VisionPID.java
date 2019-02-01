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
    private VisionPIDData _output;

    private String _xOffsetVar = "tx";
    private String _areaVar = "ta";

    private double _alignmentBias = 2.0;
    private double _pidOutput;
    private Drive _sd;

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

        // Setup PID parameters
        _pid.setOutputRange(-0.5, 0.5);

        _pid.setPID(0.01, 0.001, 0.0, 0.0);
    	_pid.setAbsoluteTolerance(0.2);
    	_pid.setOutputRange(-0.6, 0.6);
    	_pid.reset();

        //
        // Setup Mapping Profiles
        //
        // degrees per second update based on degrees from target.
        _speedProfile.put(new InterpolatingDouble(10.0), new InterpolatingDouble(1.0));
        _speedProfile.put(new InterpolatingDouble(5.0), new InterpolatingDouble(0.5));
        _speedProfile.put(new InterpolatingDouble(2.0), new InterpolatingDouble(0.2));
    }

    public void Update()
    {
        UpdateDashboard();

        // get xOffset (degrees xOffset from Target)
        var xOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry(_xOffsetVar).getDouble(0);
        
        // get area (area of target)
        var area = NetworkTableInstance.getDefault().getTable("limelight").getEntry(_areaVar).getDouble(0);
        // convert area data from camera to a distance
        var distance = area;

        // Get our heading based on x and distance vector 
        var heading = Math.atan((distance * _alignmentBias)/ xOffset);

        // Determine magnitude of summed vector
        var distanceVector = Math.sqrt(Math.pow(xOffset, 2) + Math.pow(distance, 2));

        // Determine our rate of speed change based on the magnitude of the distance vector.  
        // Unless in speed control, this is really a percent speed value. 
        double speedRate = _speedProfile.getInterpolated(new InterpolatingDouble(distanceVector)).value;

        // Rate Limit Speed
        _speedRate.SetOutputRate(speedRate);
        _speedRate.update();

        // Update PID with rate limited input
        _pid.setSetpoint(_speedRate.getOutput());

        // get the pid output....this will be our speed
        _output = new VisionPIDData(heading, _pidOutput); 
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
