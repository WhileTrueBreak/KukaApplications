package application;


import java.util.Map;

import javax.inject.Inject;

import com.kuka.basictoolbox.container.IContainer;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.Gripper2FIOGroup;
import com.kuka.generated.ioAccess.RemoteControlIOGroup;
import com.kuka.nav.Pose;
import com.kuka.nav.data.LocationData;
import com.kuka.nav.robot.MobileRobot;
import com.kuka.nav.fleet.FleetManager;
import com.kuka.resource.IResourceManager;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;

import com.kuka.roboticsAPI.capabilities.floorMountedCharge.ChargingType;
import com.kuka.roboticsAPI.capabilities.floorMountedCharge.IFloorMountedChargeCapability;
import com.kuka.roboticsAPI.capabilities.interfaces.ILEDStripControlCapability;
import com.kuka.roboticsAPI.capabilities.ledStrip.LEDStripUserStatePriority;
import com.kuka.roboticsAPI.capabilities.ledStrip.SegmentColor;
import com.kuka.roboticsAPI.controllerModel.sunrise.ISafetyState;
import com.kuka.roboticsAPI.controllerModel.sunrise.state.kmp.IMobilePlatformBatteryState;
import com.kuka.roboticsAPI.deviceModel.OperationMode;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.task.ITaskLogger;
import com.kuka.task.ITaskManager;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformPosition;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformParameterSet;

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
	private KmpOmniMove kmp1;

	@Inject
	private MobileRobot MR;
	
	@Inject
	private SunriseOmniMoveMobilePlatform kmp;
	
	@Inject
	private MobilePlatformPosition MPM;
	
	@Inject
	private ILEDStripControlCapability ledStrip;
	
	@Inject
	private ITaskLogger logger;
	
	@Inject
	private FleetManager FM;
	
	@Inject
    private LocationData locData;
	
	@Inject
	private RemoteControlIOGroup RCIO;
	
	private final static String informationText=
	         "Robot going to charge!"+ "\n" +
				"\n" +
				"press cancel to cancel.";
	
	@Override
	public void initialize() {
		// initialize your application here
	}

	@Override
	public void run() {
		
		//IMobilePlatformBatteryState battStatus = kmp.getMobilePlatformBatteryState();
		long batteryLevel = kmp.getMobilePlatformBatteryState().getStateOfCharge();
		Boolean chargeEnableState = kmp.getMobilePlatformBatteryState().isChargingEnabled();
		
		boolean motion = kmp.isMotionEnabled();
		Map<String, Object> frame = kmp.getAllUserParameters();
		double[] Jpos = kmp.getCurrentJointPosition();
		LoadData Load = kmp.getLoadData();
		
		OperationMode opMode = kmp.getOperationMode();
		ISafetyState safe = kmp.getSafetyState();
		
		logger.info("batteryLevel is" + batteryLevel);
		logger.info("charge enabled" + chargeEnableState);
		logger.info("motion enable" + motion);
		logger.info("frame" + frame);
		logger.info("J pos" + Jpos);
		logger.info("Load" + Load);
		logger.info("opMode is" + opMode);
		logger.info("safety" + safe);
		
		//////////
		int mod = RCIO.getMode();
		logger.info("mod is" + mod);
		
		int graph = FM.getGraph().getMapId();
		logger.info("graph" + graph);
		
		//////////
		double posX = MPM.getX();
		double posY = MPM.getY();
		double posT = MPM.getTheta();
		logger.info("X" + posX);
		logger.info("Y" + posY);
		logger.info("T" + posT);
		
		Pose position = MR.getPose();
		
		logger.info("Position" + position);
		///// LED STRIP //////
		ledStrip.setDefaultColor(SegmentColor.BLUE, LEDStripUserStatePriority.USER);
		ledStrip.showUserEffect();
		ThreadUtil.milliSleep(2000);
		ledStrip.stopUserEffect();
		int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "OK", "Cancel");
		if (isCancel == 1)
        {
			logger.info("cancelled");
        } else {
        	logger.info("continuing");
        }
		
//		//Charges with floor contacts for 10 minutes.
//		int timeoutInMinutes = 60; 
//		IFloorMountedChargeCapability chargeCapability = kmp.getCapability(IFloorMountedChargeCapability.class);
//		chargeCapability.enableCharge(ChargingType.FOR_GIVEN_TIME,timeoutInMinutes);
	}
}

