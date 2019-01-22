package team4391.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;

public class XboxControllerPOVButton extends Button {

	private final XboxController _cntrl;
	private final int _povLocation;

	/**
	   * Create a joystick button for triggering commands.
	   *
	   * @param joystick     The XboxController object that has the button (e.g. Joystick, KinectStick,
	   *                     etc)
	   * @param povLocation The povLocation number 
	   */
	public XboxControllerPOVButton(XboxController cntrl, int povLocation)
	{
		_cntrl = cntrl;
		_povLocation = povLocation;
	}
	
	@Override
	public boolean get() {
		return _cntrl.getPOV()==_povLocation;
	}

}
