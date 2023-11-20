package application;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
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
public class print extends RoboticsAPIApplication {
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
		gripper2F1.close();
		ThreadUtil.milliSleep(100);
		gripper2F1.setForce(150);
		mF.setLEDBlue(true);
		
	}

	@Override
	public void run() {
		
		gripper.move(ptp(getApplicationData().getFrame("/P6")).setJointVelocityRel(0.2));//frame1
		ThreadUtil.milliSleep(200);
		mF.setLEDBlue(true);
	    //JointTorqueCondition cond_1 = new JointTorqueCondition(JointEnum.J3, -12.0, 0.0);
		ForceCondition cond_2 = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"), 10.0);
		IMotionContainer motion0 = gripper.move(linRel(0, 0, -500, World.Current.getRootFrame()).setCartVelocity(100).breakWhen(cond_2));//going down
		mF.setLEDBlue(false);
		ThreadUtil.milliSleep(200);
		mF.setLEDBlue(true);
		if (motion0.getFiredBreakConditionInfo() == null){
			logger.info("No Collision Detected");
			mF.setLEDBlue(false);
		}
		else{
			logger.info("Collision Detected");
			mF.setLEDBlue(true);
			IMotionContainer motion1 = gripper.move(linRel(0, 500, 0, World.Current.getRootFrame()).setCartVelocity(100).breakWhen(cond_2));//going left
			if (motion1.getFiredBreakConditionInfo() == null){
				logger.info("No Collision Detected");
				mF.setLEDBlue(false);
			}
			else{
				logger.info("Collision Detected");
				mF.setLEDBlue(true);
			}
		}
		//gripper2F1.open();

	}
}