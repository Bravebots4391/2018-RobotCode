package team4391.util;

public class SyncronousRateLimiter 
{	
	private double _output;
	private double _outputPerSec;
	private double _initialValue;
	private double _incrementDivisor;
	
	public SyncronousRateLimiter(double loopPeriodSeconds, double outputPerSec, double initialValue){
		_outputPerSec = outputPerSec;
		_initialValue = initialValue;
		
		_incrementDivisor = loopPeriodSeconds;
		_output = _initialValue;	
	}
		
	public void SetOutputRate(double outputPerSec)
	{
		_outputPerSec = outputPerSec;
	}
	
	public void update()
	{
		_output += _outputPerSec*_incrementDivisor;
	}
	
	public double getOutput(){		
		return _output;
	}
	
	public void Reset(){
		_output=0.0;
	}

	public void Reset(double initialValue)
	{
		_output = initialValue;
	}
	
	// comment
	
//	public void doWork(double requestedRate){
//		if(_timer.hasPeriodPassed(period)){
//			_output = _output + (requestedRate / _incrementDivisor);
//		}
//	}
}
