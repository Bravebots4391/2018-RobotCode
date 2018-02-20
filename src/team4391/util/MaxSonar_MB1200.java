package team4391.util;

import edu.wpi.first.wpilibj.AnalogInput;

public class MaxSonar_MB1200 {
	
    public static AnalogInput _in_Analog_LeftSonar;
    public static AnalogInput _in_Analog_RightSonar;
	
	private boolean _isUsingLeftAndRight = false;

	public MaxSonar_MB1200(int analogChannel){
		_isUsingLeftAndRight = false;
		_in_Analog_LeftSonar = new AnalogInput(analogChannel);
	}
	
	public MaxSonar_MB1200(int analogChannelLeft, int analogChannelRight){
		_isUsingLeftAndRight = true;
		_in_Analog_LeftSonar = new AnalogInput(analogChannelLeft);
		_in_Analog_RightSonar = new AnalogInput(analogChannelRight);
	}
	
	
	private int getAverageCounts(){
		int averageCounts = 0;
		
		if(_isUsingLeftAndRight)
		{
			int leftCounts = _in_Analog_LeftSonar.getAverageValue();
			int rightCounts = _in_Analog_RightSonar.getAverageValue();
			
			averageCounts = (int) ((leftCounts + rightCounts) / 2);
		}
		else
		{
			averageCounts = _in_Analog_LeftSonar.getAverageValue();
		}
		
		return averageCounts;
	}
	
	public int getDistanceInMm(){
		
    	//Get counts from the analog inputs
    	int counts = getAverageCounts();    	    
    	
    	// Sensor uses a 10-bit DtoA with scaling of 5mm per bit
    	// We use a 12-bit AtoD, we can shift off the top bits and 
    	// multiply by 5 to get distance in mm.
    	
    	int distanceInMm = (counts >> 2) * 5;
    	
    	return distanceInMm;
	}
	
	public double getDistanceInInches(){
		return (0.0394 * getDistanceInMm());
	}
}
