package application;

import javax.inject.Inject;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformPosition;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformRelativeMotion;
import com.kuka.task.ITaskLogger;

public class move1 extends RoboticsAPIApplication {
	
	@Inject
	private SunriseOmniMoveMobilePlatform kmp;
	@Inject
	private ITaskLogger logger;

	@Override
	public void run() throws Exception {
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
	}

}
