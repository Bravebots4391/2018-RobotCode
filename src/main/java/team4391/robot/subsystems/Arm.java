package team4391.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4391.loops.Loop;
import team4391.robot.Constants;
import team4391.robot.Robot;
import team4391.robot.commands.ArmPushOutAnalog;
import team4391.robot.commands.TeleopArm;

/**
 *
 */
public class Arm extends Subsystem {

	public TalonSRX _suckerInnerOuter = new TalonSRX(Constants.kArmRightId);	
	public TalonSRX _suckerInnerOuterSlave;

	
	private CANSparkMax _wristMotor;
	private CANPIDController m_pidController;
	private CANEncoder m_encoder;

	DigitalInput _cubeSensor = new DigitalInput(1);
	
	public enum ArmState {
        Holding, PullIn, PushOut, CubePresent
    }
	
	private ArmState myArmState;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	public Arm()
	{
		_wristMotor = new CANSparkMax(21, MotorType.kBrushless);

		if(Constants.useSlaveMotors)
		{
			Robot._gyroTalon = _suckerInnerOuter;

			_suckerInnerOuterSlave = new TalonSRX(Constants.kArmLeftId);
			_suckerInnerOuterSlave.follow(_suckerInnerOuter);
			_suckerInnerOuter.setInverted(true);
			_suckerInnerOuterSlave.setInverted(true);
			
			_suckerInnerOuter.configContinuousCurrentLimit(10, Constants.kTimeoutMs);
			_suckerInnerOuter.configPeakCurrentLimit(10, Constants.kTimeoutMs);
			_suckerInnerOuter.enableCurrentLimit(true);
			
			_suckerInnerOuterSlave.configContinuousCurrentLimit(10, Constants.kTimeoutMs);
			_suckerInnerOuterSlave.configPeakCurrentLimit(10, Constants.kTimeoutMs);
			_suckerInnerOuterSlave.enableCurrentLimit(true);
								
			System.out.println("Arm Constructor has been run.");
		}
		else
		{
			Robot._gyroTalon = _suckerInnerOuter;
		}
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new TeleopArm());
        
        myArmState = ArmState.Holding;
        
    }
    
    public ArmState getArmState()
    {
    	return myArmState;
    }
    
	private final Loop mLoop = new Loop() {
		 
		@Override
		public void onStart() {
			setHolding();			
		}
	
		@Override
		public void onLoop() {		
			synchronized (Arm.this) {							
				
			switch(myArmState)
			{
				case Holding:
					break;
				
				case PullIn:					
					checkForCubePresent();
					break;
					
				case PushOut:					
					//updateCameraHeadingControl();
					break;
					
				default:
					System.out.println("Unexpected drive control state: " + myArmState);
	           break;
			}													
		}
	}
	
	@Override
	public void onStop() {
			setHolding();
		}
		 
	 };
	 
	 public Loop getLoop() {
	        return mLoop;
	    }
	 
	 public void setHolding() {
		 if(myArmState != ArmState.Holding)
			 myArmState = ArmState.Holding;
		 
		 _suckerInnerOuter.set(ControlMode.PercentOutput, 0);
	 }
	 
	 public void setPullIn() {
		 if(myArmState != ArmState.PullIn)
			 myArmState = ArmState.PullIn;		 		 		
		 
		 	System.out.println("Arm pull in " + String.format("%f", Constants.kArmInputPctSpeed));
		 
		 _suckerInnerOuter.set(ControlMode.PercentOutput, Constants.kArmInputPctSpeed);
	 }
	 
	 public void setPushOut(double speed) {
		 if(myArmState != ArmState.PushOut)
			 myArmState = ArmState.PushOut;
		 
		 //System.out.println("Arm push out " + String.format("%f", speed));
		 
		 _suckerInnerOuter.set(ControlMode.PercentOutput, speed);
	 }
	 	 
	 private void checkForCubePresent() 
	 {		 
		 if(isCubeSensed())
		 {
		  	setHolding();
		 }		 
	 }
	
	 public boolean isCubeSensed()
	 {
		 return !_cubeSensor.get();
	 }
	 
	 public void updateDashboard() {
		 SmartDashboard.putString("ArmState", myArmState.toString());
		 SmartDashboard.putBoolean("CubePresent", isCubeSensed());
	 }

	 public void setKnuckle(double d){
		var cmd = d * 1.0;
		SmartDashboard.putNumber("Knuckle", cmd);
		_suckerInnerOuter.set(ControlMode.PercentOutput, cmd);
	 }

	public void setWrist(double d) {
		var cmd = d * 0.3;
		SmartDashboard.putNumber("Wrist", cmd);
		_wristMotor.set(cmd);
	}
	
}
