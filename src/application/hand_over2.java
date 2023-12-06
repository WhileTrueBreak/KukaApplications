package application;
 
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;

//import com.kuka.math.geometry.Vector3D;
import com.kuka.nav.geometry.Vector2D;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
 
import com.kuka.roboticsAPI.capabilities.honk.IHonkCapability;
import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICallbackAction;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.ITriggerAction;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;
import com.kuka.roboticsAPI.executionModel.IFiredTriggerInfo;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.task.ITaskLogger;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.vividsolutions.jts.math.Vector3D;

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
public class hand_over2 extends RoboticsAPIApplication {
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
	
	private double PosX, PosY, PosZ;
	
	@Override
	public void initialize() {
		
		gripper.attachTo(robot.getFlange());
		gripper2F1.initalise();
		gripper2F1.setSpeed(200);
		gripper2F1.open();
		mF.setLEDBlue(true);
		ThreadUtil.milliSleep(200);
		mF.setLEDBlue(false);
		ThreadUtil.milliSleep(200);
		//Spring motion initialisation
		
		springRobot = new CartesianImpedanceControlMode(); 
		
		
		springRobot.parametrize(CartDOF.X).setStiffness(180);
		springRobot.parametrize(CartDOF.Y).setStiffness(180);
		springRobot.parametrize(CartDOF.Z).setStiffness(1000);
		springRobot.parametrize(CartDOF.C).setStiffness(300);
		springRobot.parametrize(CartDOF.B).setStiffness(300);
		springRobot.parametrize(CartDOF.A).setStiffness(300);
		springRobot.setReferenceSystem(World.Current.getRootFrame());
		springRobot.parametrize(CartDOF.ALL).setDamping(0.5);
		//USAGE, will move to next line when triggered
		//LOOK at pipecutting.java for examples on analysing the break condition. 
		//gripper.move(linRel(0, 0, -30, World.Current.getRootFrame()).setCartVelocity(50).breakWhen(touch10)); 
	}
	
	public void update(){
		Frame Position = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		
		PosX = Position.getX();
		PosY = Position.getY();
		PosZ = Position.getZ();
		double PosA = Position.getAlphaRad();
		double PosB = Position.getBetaRad();
		double PosC = Position.getGammaRad();
		
	}
	
	@Override
	public void run() {
		IHonkCapability honkCapability = kmp.getCapability(IHonkCapability.class);
		//honkCapability.honk();
		gripper2F1.open();
 
		mF.setLEDBlue(false);
		ThreadUtil.milliSleep(180);
		
		robot.move(ptp(getApplicationData().getFrame("/PART_1/p1_transition")).setJointVelocityRel(0.4));//frame1
		robot.move(ptp(getApplicationData().getFrame("/PART_1")).setJointVelocityRel(0.4));//frame1
		gripper2F1.close();
		
		ThreadUtil.milliSleep(1000);
		
		if (gripper2F1.readObjectDetection() == 2){
			logger.info("Object detected");
			mF.setLEDBlue(true);
			ThreadUtil.milliSleep(200);
		} else if (gripper2F1.readObjectDetection() == 3) {
			logger.info("No objects detected");
			honkCapability.honk();
			mF.setLEDBlue(false);
			ThreadUtil.milliSleep(200);
			ThreadUtil.milliSleep(200);
			mF.setLEDBlue(false);
			ThreadUtil.milliSleep(200);
			ThreadUtil.milliSleep(200);
			ThreadUtil.milliSleep(200);
			mF.setLEDBlue(false);
			ThreadUtil.milliSleep(200);
			ThreadUtil.milliSleep(200);
		}
		
		robot.move(ptp(getApplicationData().getFrame("/PART_1/p1_transition")).setJointVelocityRel(0.4));//frame1
		robot.move(lin(getApplicationData().getFrame("/HAND_OVER")).setJointVelocityRel(0.4));
		
		mF.setLEDBlue(true);
		ThreadUtil.milliSleep(200);
		
////////////
		
		
		
		//ForceCondition condition = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"), 16.5);

//		ICallbackAction Action = new ICallbackAction() {
//			@Override
//			public void onTriggerFired(IFiredTriggerInfo triggerInformation) {
//			  //toggle output state when trigger fired
//				mF.setLEDBlue(true);
//				ThreadUtil.milliSleep(200);
//				gripper2F1.open();
//				logger.info("yaaaaayyyyyyyyyyyy :)");
//				mF.setLEDBlue(false);
//				ThreadUtil.milliSleep(1000);
//				mF.setLEDBlue(true);
//				gripper2F1.close();
//			}
//		};
		
//		while (gripper2F1.readPos() != 0) {
//			logger.info("Please, Take the thing :)");
//			mF.setLEDBlue(true);
//			robot.move(positionHold(springRobot, -1, TimeUnit.SECONDS).triggerWhen(condition, Action).breakWhen(condition));
//			ThreadUtil.milliSleep(2000);
//		}
		update();
		logger.info("Please, Take the thing :)");
		mF.setLEDBlue(true);
		double PosX_pre = PosX;
		double PosY_pre = PosY;
		double PosZ_pre = PosZ;
		
		robot.moveAsync(positionHold(springRobot, 2, TimeUnit.SECONDS));
		
		boolean condition = false;
		
		while (condition != true) {
			update();
			Vector3D v1 = new Vector3D((PosX_pre-PosX), (PosY_pre-PosY), (PosZ_pre-PosZ));
			if (v1.length() > 20 && v1.getX() > 0) {
				mF.setLEDBlue(true);
				gripper2F1.open();
				logger.info("yaaaaayyyyyyyyyyyy :)");
				mF.setLEDBlue(false);
				condition = true;
			} 
			
		}
		
		robot.move(lin(getApplicationData().getFrame("/HAND_OVER")).setJointVelocityRel(0.4));
		robot.moveAsync(positionHold(springRobot, 2, TimeUnit.SECONDS));
		logger.info("smak me to grab :)");
		condition = false;
		while (condition != true) {
			update();
			Vector3D v1 = new Vector3D((PosX_pre-PosX), (PosY_pre-PosY), (PosZ_pre-PosZ));
			if (v1.length() > 10) {
				mF.setLEDBlue(true);
				gripper2F1.close();
				logger.info("yaaaaayyyyyyyyyyyy :)");
				mF.setLEDBlue(false);
				condition = true;
			}
		} 
	
		
//////// ???? ///////
		
		
		ThreadUtil.milliSleep(5000);
		honkCapability.honk();
		
//		//		
		robot.move(ptp(getApplicationData().getFrame("/PART_1/p1_transition")).setJointVelocityRel(0.4));//frame1
		robot.move(ptp(getApplicationData().getFrame("/PART_1")).setJointVelocityRel(0.4));//frame1
		gripper2F1.open();
//		//moving back
//		robot.move(ptp(getApplicationData().getFrame("/PART_1/p1_transition")).setJointVelocityRel(0.4));
	}
}
