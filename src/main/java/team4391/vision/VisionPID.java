/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team4391.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4391.robot.Constants;
import team4391.util.InterpolatingDouble;
import team4391.util.InterpolatingTreeMap;
import team4391.util.SyncronousRateLimiter;

/**
 * Add your docs here.
 */
public class VisionPID {

    private VisionPIDBase _alignmentPID;
    private VisionPIDBase _distancePID;

    public SyncronousRateLimiter _alignmentRate;
    public SyncronousRateLimiter _distanceRate;

    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> _alignmentProfile;
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> _distanceProfile;

    public VisionPID()
    {
        Init();
    }

    private void Init()
    {
        _alignmentPID = new VisionPIDBase("tx");
        _distancePID = new VisionPIDBase("ta");

        _alignmentRate = new SyncronousRateLimiter(Constants.kLooperDt, 1.0 , 0);
        _distanceRate = new SyncronousRateLimiter(Constants.kLooperDt, 1.0, 0);
        
        _alignmentProfile = new InterpolatingTreeMap<>();
        _distanceProfile = new InterpolatingTreeMap<>();

        // Setup PID parameters
        _alignmentPID.getPid().setOutputRange(-0.5, 0.5);
        _distancePID.getPid().setOutputRange(-0.5, 0.5);

        _alignmentPID.getPid().setPID(0.01, 0.001, 0.0, 0.0);
    	_alignmentPID.getPid().setAbsoluteTolerance(0.2);
    	_alignmentPID.getPid().setOutputRange(-0.6, 0.6);
    	_alignmentPID.getPid().reset();

        //
        // Setup Mapping Profiles
        //
        // degrees per second update based on degrees from target.
        _alignmentProfile.put(new InterpolatingDouble(10.0), new InterpolatingDouble(-1.0));
        _alignmentProfile.put(new InterpolatingDouble(5.0), new InterpolatingDouble(-0.5));
        _alignmentProfile.put(new InterpolatingDouble(2.0), new InterpolatingDouble(-0.3));
        _alignmentProfile.put(new InterpolatingDouble(-2.0), new InterpolatingDouble(0.3));
        _alignmentProfile.put(new InterpolatingDouble(-5.0), new InterpolatingDouble(0.5));
        _alignmentProfile.put(new InterpolatingDouble(-10.0), new InterpolatingDouble(1.0));

        // % area per second update based on distance
        _distanceProfile.put(new InterpolatingDouble(0.2), new InterpolatingDouble(1.0));
        _distanceProfile.put(new InterpolatingDouble(2.0), new InterpolatingDouble(0.5));
        _distanceProfile.put(new InterpolatingDouble(4.0), new InterpolatingDouble(0.1));
    }

    public void Update()
    {
        UpdateDashboard();

        // Get our current distance from the target        
        double x = _alignmentPID.pidGet();
        double y = _distancePID.pidGet();

        // Map Speed based on how far away we are            
        double xInterp = _alignmentProfile.getInterpolated(new InterpolatingDouble(x)).value;
        double yInterp = _alignmentProfile.getInterpolated(new InterpolatingDouble(y)).value;

        _alignmentRate.SetOutputRate(xInterp);
        _distanceRate.SetOutputRate(yInterp);

        _alignmentRate.update();
        _distanceRate.update();

        // Update PID with rate limited input
        _alignmentPID.getPid().setSetpoint(_alignmentRate.getOutput());
        _distancePID.getPid().setSetpoint(_distanceRate.getOutput());
    }

    public VisionPIDData GetOutput()
    {
            // Get the outputs from each PID
            double alignmentOutput = _alignmentPID.GetOutput();
            double distanceOutput = _distancePID.GetOutput();
                        
            // Sum up the PID data to get a vector for the robot.
            var magnitude = Math.sqrt(Math.pow(alignmentOutput, 2) + Math.pow(distanceOutput, 2));

            // Get heading
            var heading = Math.atan(distanceOutput / alignmentOutput);

        	// drive left or right
			double sign = Math.signum(alignmentOutput);
            double speed = magnitude;

            if(sign >= 0)
			{
				heading = 90.0;
			}
			else
			{
				heading = 270.0;
			}


        return new VisionPIDData(heading, speed);
    }

    public void Enable()
    {
        // update the initial values for the rate limiters
        // with the current target position. 
        _alignmentRate.Reset(_alignmentPID.pidGet());
        _distanceRate.Reset(_distancePID.pidGet());

        // Enable the PIDs
        _alignmentPID.enable();
        _distancePID.enable();
    }

    public void Disable()
    {
        _alignmentPID.disable();
        _distancePID.disable();

        _alignmentRate.Reset();
        _distanceRate.Reset();
    }

	public void Reset() {
        Disable();

    }
    
    private void UpdateDashboard()
    {
        SmartDashboard.putData("alignmentPID", _alignmentPID.getPid());	
        SmartDashboard.putData("distancePID", _distancePID.getPid());
    }
}
