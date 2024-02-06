package application;
 
 
import java.util.Map;
 
import javax.inject.Inject;
 
import com.kuka.generated.ioAccess.RemoteControlIOGroup;
import com.kuka.nav.Pose;
import com.kuka.nav.data.LocationData;
import com.kuka.nav.robot.MobileRobot;
import com.kuka.resource.IResourceManager;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
 
import com.kuka.roboticsAPI.controllerModel.sunrise.ISafetyState;
import com.kuka.roboticsAPI.deviceModel.OperationMode;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.task.ITaskLogger;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformPosition;
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
 
//	@Inject
//	private KmpOmniMove kmp1;
 
	@Inject
	private MobileRobot MR;
	@Inject
	private SunriseOmniMoveMobilePlatform kmp;
	@Inject
	private MobilePlatformPosition MPM;

	@Inject
	private ITaskLogger logger;

//	@Inject
//   private LocationData locData;
//	
//	@Inject
//	private RemoteControlIOGroup RCIO;
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
//      testing	
//		IMobilePlatformBatteryState battStatus = kmp.getMobilePlatformBatteryState();
//		Map<String, Object> frame = kmp.getAllUserParameters();
//		double[] Jpos = kmp.getCurrentJointPosition();
//		LoadData Load = kmp.getLoadData();
//		ISafetyState safety = kmp.getSafetyState();
		long batteryLevel = kmp.getMobilePlatformBatteryState().getStateOfCharge();
		Boolean chargeEnableState = kmp.getMobilePlatformBatteryState().isChargingEnabled();
		boolean motion = kmp.isMotionEnabled();
		OperationMode opMode = kmp.getOperationMode();
		logger.info("batteryLevel is :" + batteryLevel);
		logger.info("charge enabled : " + chargeEnableState);
		logger.info("motion enable :" + motion);
		logger.info("opMode is: " + opMode);
		///////// Base position from NAV ////////
		Pose position = MR.getPose();
		logger.info("Position on map" + position);
		///////// moving Base ///////
		double x = -20;
		double y = -30;
		double tita = 0;
		MobilePlatformRelativeMotion motion1 = new MobilePlatformRelativeMotion(x, y, tita);
		kmp.move(motion1.setVelocity(10, 10));

//		//Charges with floor contacts for 10 minutes.
//		int timeoutInMinutes = 60; 
//		IFloorMountedChargeCapability chargeCapability = kmp.getCapability(IFloorMountedChargeCapability.class);
//		chargeCapability.enableCharge(ChargingType.FOR_GIVEN_TIME,timeoutInMinutes);
	}
}
 