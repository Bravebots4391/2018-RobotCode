/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team4391.vision;

import team4391.util.SyncronousRateLimiter;

/**
 * Add your docs here.
 */
public class VisionPID {

    private VisionPIDBase _alignmentPID;
    private VisionPIDBase _distancePID;

    public SyncronousRateLimiter _alignmentRate;
    public SyncronousRateLimiter _distanceRate;

    public VisionPID()
    {
        Init();
    }

    private void Init()
    {
        _alignmentPID = new VisionPIDBase();
        _distancePID = new VisionPIDBase();

        //_alignmentRate = new SyncronousRateLimiter(Constants.kLooperDt, 1.0 , 0);
        //_distanceRate = new SyncronousRateLimiter(Constants.kLooperDt, 1.0, 0);

        // setup PID parameters
        
    }

    public VisionPIDData GetOutput()
    {
        	// drive left or right
			double sign = Math.signum(_alignmentPID.GetOutput());
			double heading = 0.0;
			if(sign >= 0)
			{
				heading = 270.0;
			}
			else
			{
				heading = 90.0;
			}


        return new VisionPIDData(0,0);
    }

    public void Enable()
    {
        _alignmentPID.enable();
        _distancePID.enable();
    }

    public void Disable()
    {
        _alignmentPID.disable();
        _distancePID.disable();
    }
}
