package application;

import java.util.Collection;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.nav.Location;
import com.kuka.nav.Pose;
import com.kuka.nav.data.LocationData;
import com.kuka.nav.rel.RelativeMotion;
import com.kuka.nav.robot.MobileRobot;
import com.kuka.nav.robot.MobileRobotManager;
import com.kuka.nav.task.NavTaskCategory;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPITask;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.capabilities.honk.IHonkCapability;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformRelativeMotion;
import com.kuka.task.ITaskLogger;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;

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

public class test_move extends RoboticsAPIApplication {
	@Inject
	private LBR robot;
	
	@Inject 
	private SunriseOmniMoveMobilePlatform kmp;
	
	@Inject
	private KmpOmniMove base;
	
	@Inject
	private MobileRobotManager robotManager;
	
	@Inject
	private LocationData locationData;
	
	@Inject 
	private Gripper2F gripper2F1;

	@Inject
	private MediaFlangeIOGroup mF;

	@Inject
	@Named("RobotiqGripper")
	private Tool gripper;


	@Inject
	private ITaskLogger logger;
	
	@Override
	public void initialize() {
		gripper.attachTo(robot.getFlange());
		
		gripper2F1.initalise();
		gripper2F1.setSpeed(189);
		gripper2F1.close();
		ThreadUtil.milliSleep(200);
		gripper2F1.open();
	}

	@Override
	public void run() {
		// Request current position of platform
		// “[Name in StationSetup]”
		//Pose currentPose = robot.getPose();
		
		//Collections<MobileRobot> robots = robotManager.getRobots(MobileRobot.class);

		//Collection<Location> locations = locationData.getAll();
		double x = 20;
		double y = 30;
		double tita = 0;
		MobilePlatformRelativeMotion motion = new MobilePlatformRelativeMotion(x, y, tita);
		base.move(motion.setVelocity(10, 10));
	}
}