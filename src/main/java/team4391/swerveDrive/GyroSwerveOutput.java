package team4391.swerveDrive;

public class GyroSwerveOutput 
{
	double fl;
	double fr;
	double rl;
	double rr;
	
	public GyroSwerveOutput(double fl, double fr, double rl, double rr) {
		super();
		this.fl = fl;
		this.fr = fr;
		this.rl = rl;
		this.rr = rr;
	}

	public double getFl() {
		return fl;
	}

	public double getFr() {
		return fr;
	}

	public double getRl() {
		return rl;
	}

	public double getRr() {
		return rr;
	}
	
	
}
