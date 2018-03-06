package team4391.swerveDrive;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveAndRotate 
{
	double Lx;
	double Ly;
	double xTurnRadiusMax;
	double xTurnRadiusMin;
			
	
	public SwerveAndRotate(double lengthX, double lengthY, double maximumTurnRadius) {
		super();
		
		Lx = lengthX;
		Ly = lengthY;
		this.xTurnRadiusMax = maximumTurnRadius;
		xTurnRadiusMin = Math.sqrt(Math.pow(Lx / 2.0, 2) + Math.pow(Ly / 2.0, 2));
	}

	WheelPositionInfo swerveAndTurn(double heading, double pctTurn)
	{
		double Xt = pctTurn;
		double scaledXt = Math.signum(Xt) * Math.abs(xTurnRadiusMax - (xTurnRadiusMax * Math.abs(Xt)) + xTurnRadiusMin+8);
		
		SmartDashboard.putNumber("scaledXt", scaledXt);
		
		XYCoord center = GetCenter(heading, scaledXt);			
		double X = center.getX();
		double Y = center.getY();
		
		SmartDashboard.putNumber("XX", X);
		SmartDashboard.putNumber("YY", Y);

		double x1 = X - (Lx / 2);
		double y1 = Y - (Ly / 2);
		double x2 = X - (Lx / 2);
		double y2 = Y + (Ly / 2);
		double x3 = X + (Lx / 2);
		double y3 = Y + (Ly / 2);
		double x4 = X + (Lx / 2);
		double y4 = Y - (Ly / 2);

		double d1 = Dist(x1, y1);
		double d2 = Dist(x2, y2);
		double d3 = Dist(x3, y3);
		double d4 = Dist(x4, y4);
		
		double w1 = (WheelAngle(x1, y1, scaledXt));
		double w2 = (WheelAngle(x2, y2, scaledXt));
		double w3 = (WheelAngle(x3, y3, scaledXt));
		double w4 = (WheelAngle(x4, y4, scaledXt));
		
		if(pctTurn == 0.0)
		{
			w1 = heading;
			w2 = heading;
			w3 = heading;
			w4 = heading;
		}

		// Find maximum speed
		ArrayList<Double> list = new ArrayList<Double>();
		list.add(d1);
		list.add(d2);
		list.add(d3);
		list.add(d4);		
		double dMax = Collections.max(list);
		
		// Create object for use by swerve
		WheelPositionInfo wi = new WheelPositionInfo(w4, w1, w3, w2, (d4 / dMax), (d1 / dMax), (d3 / dMax), (d2 / dMax));
		
		return wi;
	}
	
	double Dist(double x, double y)
	{
		return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
	}

	double WheelAngle(double x, double y, double xT)
	{
		double polar = Math.atan2(y, x) * 180 / Math.PI;

		double coord = toHeading(polar) - 90;

		// invert the angle if we are asking for "negative" rotation
		if (Math.signum(xT) == -1)
		{
			coord -= 180;
		}

		// Make it a positive angle
		if (coord < 0)
			coord += 360;

		return coord;
	}
	
	double toHeading(double polar)
	{
		return ((450 - polar) % 360);
	}
	
	double toPolar(double heading)
	{
		return (-(heading - 90) + 360) % 360;
	}

	XYCoord GetCenter(double heading, double xT)
	{
		double polar = toPolar(heading + 90);
		double rad = polar * Math.PI / 180.0;

		if (Math.abs(Math.cos(rad)) <= 0.00000001)
		{			
			return new XYCoord(0.0, xT * Math.sin(rad));
		}
		else if (Math.abs(Math.sin(rad)) <= 0.00000001)
		{	
			return new XYCoord(xT * Math.cos(rad), 0);
		}
		else
		{
			return new XYCoord(xT * Math.cos(rad), xT * Math.sin(rad));
		}
	}
	
}
