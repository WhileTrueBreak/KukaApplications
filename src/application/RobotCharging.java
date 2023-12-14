package application;


import javax.inject.Inject;

import com.kuka.nav.line.VirtualLineMotion;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.capabilities.floorMountedCharge.ChargingType;
import com.kuka.roboticsAPI.capabilities.floorMountedCharge.IFloorMountedChargeCapability;
import com.kuka.roboticsAPI.capabilities.interfaces.ICapability;
import com.kuka.roboticsAPI.commandModel.IAction;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformRelativeMotion;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class RobotCharging extends RoboticsAPIApplication {

	@Inject
	private KmpOmniMove base;
	
	@Inject
	private SunriseOmniMoveMobilePlatform kmp;

	@Override
	public void initialize() {
		// initialize your application here
	}

	@Override
	public void run() {
		// Charges with floor contacts for 10 minutes.
		int timeoutInSeconds = 3600; // value in seconds
		
		IFloorMountedChargeCapability chargeCapability = kmp.getCapability(IFloorMountedChargeCapability.class);
		chargeCapability.enableCharge(ChargingType.FOR_GIVEN_TIME,timeoutInSeconds);
		// kmp.enableCharge(ChargingType.FOR_GIVEN_TIME, timeoutInSeconds);
	}
}

