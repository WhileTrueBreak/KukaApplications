package application;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.nav.robot.MobileRobot;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.capabilities.driveBrakeTest.IDriveBrakeTestCapability;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformPosition;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformRelativeMotion;

public class move1 extends RoboticsAPIApplication {
	
	@Inject
	private KmpOmniMove kmp;
	private IDriveBrakeTestCapability driveBrakeCapability;
	
	@Inject
	@Named("Nav_KMR_200_iiwa_14_R820_2")
	private MobileRobot mobileRobot;

	public void initialize() {
		driveBrakeCapability = kmp.getCapability(IDriveBrakeTestCapability.class);
	}

	@Override
	public void run() throws Exception {
		try {
			mobileRobot.lock();
			ICommandContainer container = kmp.move(new MobilePlatformRelativeMotion(-100,0,0)
					 .setVelocity(100, 0.8)
					 .setAcceleration(300, 0.5)
					 .setTimeout(60));
			 MobilePlatformPosition coveredDistance =
					  MobilePlatformRelativeMotion
					  .getCoveredDistance(container);
			 getLogger().info("Covered distance " +
					  "X=" + coveredDistance.getX() +
					  ", Y=" + coveredDistance.getY() +
					  ", Theta=" + coveredDistance.getTheta());
		} catch (Exception e) {
			getLogger().error("Error: ", e);
		} finally {
			 mobileRobot.unlock();
		}
	}

}
