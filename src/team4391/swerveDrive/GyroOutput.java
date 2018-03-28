package team4391.swerveDrive;

public class GyroOutput
{
	private double _left;
	private double _right;
	
	public GyroOutput(double left, double right)
	{
		set_left(left);
		set_right(right);
	}

	public double get_left() {
		return _left;
	}

	public void set_left(double _left) {
		this._left = _left;
	}

	public double get_right() {
		return _right;
	}

	public void set_right(double _right) {
		this._right = _right;
	}
}