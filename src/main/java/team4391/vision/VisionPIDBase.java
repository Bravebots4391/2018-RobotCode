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
import team4391.robot.Constants;
import team4391.util.SyncronousRateLimiter;

/**
 * Add your docs here.
 */
public class VisionPIDBase implements PIDSource, PIDOutput {

    PIDController _pid;
    SyncronousRateLimiter _srl;

    double _pidOutput = 0.0;
    String _tableVariable = "";

    public VisionPIDBase(String tableVariable)
    {
        _tableVariable = tableVariable;
        Init();        
    }

    private void Init()
    {
        _pid = new PIDController(0.010, 0, 0, this, this);
        _srl = new SyncronousRateLimiter(Constants.kLooperDt, 1.0 , 0);
    }

    public double GetOutput()
    {
        return _pidOutput;
    }

    public PIDController getPid()
    {
        return _pid;
    }

    @Override
    public void pidWrite(double output) {
        _pidOutput = output;
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
        double pidSource = NetworkTableInstance.getDefault().getTable("limelight").getEntry(_tableVariable).getDouble(0);
		return pidSource;
	}

	public void disable() {
        _pid.disable();
	}

	public void enable() {
        _pid.enable();
	}
}
