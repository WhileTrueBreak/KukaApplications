package application;
 
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
 
import com.kuka.roboticsAPI.capabilities.honk.IHonkCapability;
import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICallbackAction;
import com.kuka.roboticsAPI.conditionModel.ITriggerAction;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;
import com.kuka.roboticsAPI.executionModel.IFiredTriggerInfo;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.task.ITaskLogger;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.*;

 
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
public class hand_over extends RoboticsAPIApplication {
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
	
	@Inject 
	private SunriseOmniMoveMobilePlatform kmp;
	
	CartesianImpedanceControlMode springRobot;
	
	@Override
	public void initialize() {
		
		gripper.attachTo(robot.getFlange());
		gripper2F1.initalise();
		gripper2F1.setSpeed(189);
		gripper2F1.open();
		mF.setLEDBlue(true);
		ThreadUtil.milliSleep(200);
		mF.setLEDBlue(false);
		ThreadUtil.milliSleep(200);
		//Spring motion initialisation
		
		springRobot = new CartesianImpedanceControlMode(); 
		
		
		springRobot.parametrize(CartDOF.X).setStiffness(1000);
		springRobot.parametrize(CartDOF.Y).setStiffness(1000);
		springRobot.parametrize(CartDOF.Z).setStiffness(1000);
		springRobot.parametrize(CartDOF.C).setStiffness(250);
		springRobot.parametrize(CartDOF.B).setStiffness(250);
		springRobot.parametrize(CartDOF.A).setStiffness(250);
		springRobot.setReferenceSystem(World.Current.getRootFrame());
		springRobot.parametrize(CartDOF.ALL).setDamping(0.7);
		//USAGE, will move to next line when triggered
		//LOOK at pipecutting.java for examples on analysing the break condition. 
		//gripper.move(linRel(0, 0, -30, World.Current.getRootFrame()).setCartVelocity(50).breakWhen(touch10)); 
	}
 
	@Override
	public void run() {
		IHonkCapability honkCapability = kmp.getCapability(IHonkCapability.class);
		honkCapability.honk();
		gripper2F1.open();
 
		mF.setLEDBlue(false);
		ThreadUtil.milliSleep(200);
		
		robot.move(ptp(getApplicationData().getFrame("/PART_1/p1_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/PART_1")).setJointVelocityRel(0.3));//frame1
		gripper2F1.close();
		robot.move(ptp(getApplicationData().getFrame("/PART_1/p1_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(lin(getApplicationData().getFrame("/HAND_OVER")).setJointVelocityRel(0.3));
		
		mF.setLEDBlue(true);
		ThreadUtil.milliSleep(200);
		
////////////
		
		//TCP
		ForceCondition condition = ForceCondition.createSpatialForceCondition(gripper.getFrame("/HAND_OVER"), 15.0);

		ICallbackAction Action = new ICallbackAction() {
			@Override
			public void onTriggerFired(IFiredTriggerInfo triggerInformation) {
			 //toggle output state when trigger fired
				gripper2F1.open();
				logger.info("yaaaaayyyyyyyyyyyy");
			}
		};
		
//		triggerWhen(condition, action)
		gripper.move(positionHold(springRobot, -1, TimeUnit.SECONDS).triggerWhen(condition, Action));
		
		
	}
}