/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team4391.vision;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4391.robot.Constants;
import team4391.util.InterpolatingDouble;
import team4391.util.InterpolatingTreeMap;
import team4391.util.SyncronousRateLimiter;

/**
 * Add your docs here.
 */
public class VisionPID implements PIDOutput, PIDSource {

    private VisionPIDBase _positionPID;

    public SyncronousRateLimiter _alignmentRate;
    public SyncronousRateLimiter _distanceRate;

    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> _positionProfile;

    public VisionPID()
    {
        Init();
    }

    private void Init()
    {
        _positionPID = new VisionPIDBase("tx");
        //_distancePID = new VisionPIDBase("ta");

        _alignmentRate = new SyncronousRateLimiter(Constants.kLooperDt, 1.0 , 0);
        _distanceRate = new SyncronousRateLimiter(Constants.kLooperDt, 1.0, 0);
        
        _positionProfile = new InterpolatingTreeMap<>();

        // Setup PID parameters
        _positionPID.getPid().setOutputRange(-0.5, 0.5);

        _positionPID.getPid().setPID(0.01, 0.001, 0.0, 0.0);
    	_positionPID.getPid().setAbsoluteTolerance(0.2);
    	_positionPID.getPid().setOutputRange(-0.6, 0.6);
    	_positionPID.getPid().reset();

        //
        // Setup Mapping Profiles
        //
        // degrees per second update based on degrees from target.
        _positionProfile.put(new InterpolatingDouble(10.0), new InterpolatingDouble(-1.0));
        _positionProfile.put(new InterpolatingDouble(5.0), new InterpolatingDouble(-0.5));
        _positionProfile.put(new InterpolatingDouble(2.0), new InterpolatingDouble(-0.3));
        _positionProfile.put(new InterpolatingDouble(-2.0), new InterpolatingDouble(0.3));
        _positionProfile.put(new InterpolatingDouble(-5.0), new InterpolatingDouble(0.5));
        _positionProfile.put(new InterpolatingDouble(-10.0), new InterpolatingDouble(1.0));
    }

    public void Update()
    {
        UpdateDashboard();

        // Get our current distance from the target        
        double x = _positionPID.pidGet();

        // Map Speed based on how far away we are            
        double xInterp = _positionProfile.getInterpolated(new InterpolatingDouble(x)).value;
        double yInterp = _positionProfile.getInterpolated(new InterpolatingDouble(y)).value;

        _alignmentRate.SetOutputRate(xInterp);
        _distanceRate.SetOutputRate(yInterp);

        _alignmentRate.update();
        _distanceRate.update();

        // Update PID with rate limited input
        _positionPID.getPid().setSetpoint(_alignmentRate.getOutput());
    }

    public VisionPIDData GetOutput()
    {
            // Get the outputs from each PID
            double alignmentOutput = _positionPID.GetOutput();
                        
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
        _alignmentRate.Reset(_positionPID.pidGet());

        // Enable the PIDs
        _positionPID.enable();
    }

    public void Disable()
    {
        _positionPID.disable();

        _alignmentRate.Reset();
        _distanceRate.Reset();
    }

	public void Reset() {
        Disable();

    }
    
    private void UpdateDashboard()
    {
        SmartDashboard.putData("alignmentPID", _positionPID.getPid());	
    }
}
