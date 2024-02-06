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
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
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
	IMotionContainer m1;
	private double PosX, PosY, PosZ;
	@Override
	public void initialize() {
		gripper.attachTo(robot.getFlange());
		gripper2F1.initalise();
		gripper2F1.setForce(10);
		gripper2F1.setSpeed(200);
		gripper2F1.open();
		mF.setLEDBlue(true);
		ThreadUtil.milliSleep(200);
		mF.setLEDBlue(false);
		ThreadUtil.milliSleep(200);
		//Spring motion initialisation
		springRobot = new CartesianImpedanceControlMode(); 
		springRobot.parametrize(CartDOF.X).setStiffness(120); 
		springRobot.parametrize(CartDOF.Y).setStiffness(120);
		springRobot.parametrize(CartDOF.Z).setStiffness(1500);
		springRobot.parametrize(CartDOF.C).setStiffness(100);
		springRobot.parametrize(CartDOF.B).setStiffness(100);
		springRobot.parametrize(CartDOF.A).setStiffness(100);
		springRobot.setReferenceSystem(World.Current.getRootFrame());
		springRobot.parametrize(CartDOF.ALL).setDamping(0.8);
		//USAGE, will move to next line when triggered
		//LOOK at pipecutting.java for examples on analysing the break condition. 
		//gripper.move(linRel(0, 0, -30, World.Current.getRootFrame()).setCartVelocity(50).breakWhen(touch10)); 
	}
	private Vector3D dist(Frame pose){
		Frame newPosition = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		Vector3D distance = new Vector3D((pose.getX()-newPosition.getX()), (pose.getY()-newPosition.getY()), (pose.getZ()-newPosition.getZ()));
		return distance;
	}
	@Override
	public void run() {
		IHonkCapability honkCapability = kmp.getCapability(IHonkCapability.class);
		honkCapability.honk();
		for (int i = 0; i < 5; i++) {
			gripper2F1.open();
			mF.setLEDBlue(true);
			ThreadUtil.milliSleep(180);
			robot.move(ptp(getApplicationData().getFrame("/P2")).setJointVelocityRel(0.5).setMode(springRobot));//frame1
			//robot.move(ptp(getApplicationData().getFrame("/P2/P1")).setJointVelocityRel(0.4));//frame1
			double offset = i*25;
			robot.move(linRel(offset,0,0,0,0,0).setJointVelocityRel(0.3).setMode(springRobot));
			robot.move(linRel(0,0,85,0,0,0).setJointVelocityRel(0.3).setMode(springRobot));
			ThreadUtil.milliSleep(1000);
			gripper2F1.close();
			ThreadUtil.milliSleep(200);
			while (gripper2F1.readObjectDetection() == 3){
				logger.info("No objects detected");
				honkCapability.honk();
				gripper2F1.open();
				mF.setLEDBlue(true);
				ThreadUtil.milliSleep(200);
				mF.setLEDBlue(false);
				ThreadUtil.milliSleep(200);
				//robot.move(ptp(getApplicationData().getFrame("/P2/P1")).setJointVelocityRel(0.4));//frame1
				robot.move(linRel(offset,0,0,0,0,0).setJointVelocityRel(0.3).setMode(springRobot));
				robot.move(linRel(0,0,85,0,0,0).setJointVelocityRel(0.3).setMode(springRobot));
				gripper2F1.close();
			}
			
			logger.info("Object detected");
			mF.setLEDBlue(true);
			ThreadUtil.milliSleep(200);
			mF.setLEDBlue(false);
			robot.move(ptp(getApplicationData().getFrame("/P2/P1")).setJointVelocityRel(0.4));//frame1
			robot.move(lin(getApplicationData().getFrame("/P3")).setJointVelocityRel(0.4));

			IMotionContainer m1 = robot.moveAsync(positionHold(springRobot, 20, TimeUnit.SECONDS));
			Frame pose = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
			logger.info("Please take the object!");
			mF.setLEDBlue(true);
			ThreadUtil.milliSleep(200);
			mF.setLEDBlue(false);
			while (true) {
				Vector3D v1 = dist(pose);
				if (v1.length() > 30 && v1.getX() < 0) {
					mF.setLEDBlue(true);
					gripper2F1.open();
					logger.info("yaaaaayyyyyyyyyyyy :)");
					mF.setLEDBlue(false);
					m1.cancel();
					break;
				} else if (m1.isFinished()) {
					logger.info("Sorry, Time out!");
					break;
				}
			}
			robot.move(lin(getApplicationData().getFrame("/P3")).setJointVelocityRel(0.4).setMode(springRobot));
			IMotionContainer m2 = robot.moveAsync(positionHold(springRobot, 20, TimeUnit.SECONDS));
			logger.info("hit me to grab or go back");
			mF.setLEDBlue(true);
			ThreadUtil.milliSleep(200);
			mF.setLEDBlue(false);
			/////
			robot.move(linRel(Transformation.ofDeg(0,0,0,0,0,90)).setJointVelocityRel(0.6).setMode(springRobot));
			/////
			pose = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
			while (true) {
				Vector3D v2 = dist(pose);
				if (v2.length() > 2) {
					mF.setLEDBlue(true);
					gripper2F1.setSpeed(50);
					gripper2F1.close();
					logger.info("yaaaaayyyyyyyyyyyy :)");
					mF.setLEDBlue(false);
					m2.cancel();
					break;
				} else if (m2.isFinished()) {
					logger.info("Sorry, Time out!");
					break;
				}
			}
			robot.move(ptp(getApplicationData().getFrame("/P2")).setJointVelocityRel(0.4).setMode(springRobot));//frame1
			//robot.move(ptp(getApplicationData().getFrame("/P2/P1")).setJointVelocityRel(0.4));//frame1
		}
	}
}