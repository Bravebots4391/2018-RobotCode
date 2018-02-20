package team4391.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4391.loops.Loop;
import team4391.robot.Constants;
import team4391.robot.commands.ArmHold;

/**
 *
 */
public class Arm extends Subsystem {

	public TalonSRX _suckerInnerOuter = new TalonSRX(Constants.kArmRightId);
	public TalonSRX _suckerInnerOuterSlave = new TalonSRX(Constants.kArmLeftId);
	
	private TalonSRX _armPulley = new TalonSRX(Constants.kArmOpenyCloseyId);
	DigitalInput _cubeSensor = new DigitalInput(1);
	
	public enum ArmState {
        Holding, PullIn, PushOut, CubePresent
    }
	
	private ArmState myArmState;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	public Arm()
	{
		_suckerInnerOuterSlave.follow(_suckerInnerOuter);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new ArmHold());
        
        myArmState = ArmState.Holding;
        
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
		// TODO Auto-generated method stub		
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
		 
		 _suckerInnerOuter.set(ControlMode.PercentOutput, Constants.kArmInputPctSpeed);
	 }
	 
	 public void setPushOut() {
		 if(myArmState != ArmState.PushOut)
			 myArmState = ArmState.PushOut;
		 
		 _suckerInnerOuter.set(ControlMode.PercentOutput, Constants.kArmOutputPctSpeed);
	 }
	 
	 public void setFastPushOut() {
		 if(myArmState != ArmState.PushOut)
			 myArmState = ArmState.PushOut;
		 
		 _suckerInnerOuter.set(ControlMode.PercentOutput, Constants.kArmFastOutputPctSpeed);
	 }
	 
	 private void checkForCubePresent() {
		 // TODO Add a sensor to determine if the cube is present
		 
		 if(isCubeSensed()){
		  	setHolding();
		 }
		 
	 }
	 
	 public boolean isCubeSensed()
	 {
		 return !_cubeSensor.get();
	 }
	 
	 public void openArm(double speed){
		 _armPulley.set(ControlMode.PercentOutput, 1);
	 }
	 
	 public void closeArm(){
		 _armPulley.set(ControlMode.PercentOutput, 0);
	 }	 
	 
	 public void updateDashboard() {
		 SmartDashboard.putString("ArmState", myArmState.toString());
		 SmartDashboard.putBoolean("CubePresent", isCubeSensed());
	 }
}
	
