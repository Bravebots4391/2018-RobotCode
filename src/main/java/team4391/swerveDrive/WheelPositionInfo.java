package team4391.swerveDrive;

public class WheelPositionInfo 
{
	private double flAngle;
	private double frAngle;
	private double rLAngle;
	private double rRAngle;
	
	private double flSpeed;
	private double fRSpeed;
	private double rLSpeed;
	private double rRSpeed;
		
	public WheelPositionInfo(double flAngle, double frAngle, double rLAngle, double rRAngle, double flSpeed,
			double fRSpeed, double rLSpeed, double rRSpeed) {
		super();
		this.flAngle = flAngle;
		this.frAngle = frAngle;
		this.rLAngle = rLAngle;
		this.rRAngle = rRAngle;
		this.flSpeed = flSpeed;
		this.fRSpeed = fRSpeed;
		this.rLSpeed = rLSpeed;
		this.rRSpeed = rRSpeed;
	}
	
	public double getFlAngle() {
		return flAngle;
	}
	public void setFlAngle(double flAngle) {
		this.flAngle = flAngle;
	}
	public double getFrAngle() {
		return frAngle;
	}
	public void setFrAngle(double frAngle) {
		this.frAngle = frAngle;
	}
	public double getrLAngle() {
		return rLAngle;
	}
	public void setrLAngle(double rLAngle) {
		this.rLAngle = rLAngle;
	}
	public double getrRAngle() {
		return rRAngle;
	}
	public void setrRAngle(double rRAngle) {
		this.rRAngle = rRAngle;
	}
	public double getFlSpeed() {
		return flSpeed;
	}
	public void setFlSpeed(double flSpeed) {
		this.flSpeed = flSpeed;
	}
	public double getfRSpeed() {
		return fRSpeed;
	}
	public void setfRSpeed(double fRSpeed) {
		this.fRSpeed = fRSpeed;
	}
	public double getrLSpeed() {
		return rLSpeed;
	}
	public void setrLSpeed(double rLSpeed) {
		this.rLSpeed = rLSpeed;
	}
	public double getrRSpeed() {
		return rRSpeed;
	}
	public void setrRSpeed(double rRSpeed) {
		this.rRSpeed = rRSpeed;
	}
	
	
}
