package application;

import javax.inject.Inject;
import javax.inject.Named;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
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
public class RobotPickandPlace extends RoboticsAPIApplication {
	@Inject
	private LBR robot;

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
		gripper2F1.open();
		mF.setLEDBlue(true);
		ThreadUtil.milliSleep(200);
		mF.setLEDBlue(false);
		gripper2F1.close();
		ThreadUtil.milliSleep(200);
		
		//FORCE CONDITIONS EXAMPLE
		
		//gripper.move(linRel(0, 0, -30, World.Current.getRootFrame()).setCartVelocity(50).breakWhen(touch10)); 
	}

	@Override
	public void run() {

		gripper.move(ptp(getApplicationData().getFrame("/P1")).setJointVelocityRel(0.8));//frame1
	    gripper.move(linRel(0, 0, -30, World.Current.getRootFrame()).setCartVelocity(50));//going down
		gripper2F1.open();
		gripper.move(linRel(-50, 0, 5, World.Current.getRootFrame()).setCartVelocity(490));
		gripper.move(linRel(0, 0, 60, World.Current.getRootFrame()).setCartVelocity(490));
		mF.setLEDBlue(true);
		gripper.move(ptp(getApplicationData().getFrame("/P2")).setJointVelocityRel(1));// go to frame2
	    gripper.move(linRel(0, 0, -5, World.Current.getRootFrame()).setCartVelocity(50));// going down
		gripper2F1.close();
		mF.setLEDBlue(false);
		
		gripper.move(ptp(getApplicationData().getFrame("/P1")).setJointVelocityRel(0.4));//frame1
				
	}
}