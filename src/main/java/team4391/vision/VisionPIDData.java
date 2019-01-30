/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team4391.vision;

/**
 * Add your docs here.
 */
public class VisionPIDData {


    private double _heading;
    private double _speed;

    public VisionPIDData(double heading, double speed)
    {
        _heading = heading;
        _speed = speed;
    }

    public double GetHeading()
    {
        return _heading;
    }

    public double GetSpeed()
    {
        return _speed;
    }
}
