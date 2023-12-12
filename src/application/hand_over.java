package application;
 
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.math.geometry.Vector3D;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import java.util.Arrays;

import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICallbackAction;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.ITriggerAction;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.IFiredTriggerInfo;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.task.ITaskLogger;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;

import java.util.Timer;
import java.util.TimerTask;

import static com.kuka.roboticsAPI.motionModel.HRCMotions.*;

 
/**
* By: Sri :)
* last edited: 9/12/23
* FROM ROBOTICS TEAM
* 
* Implementation of a robot application to hand over and take bak an object.
* 
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
	
	CartesianImpedanceControlMode springRobot;
	
	public double PosX, PosY, PosZ;
	private static boolean keepRunning = true;
	
	@Override
	public void initialize() {
		
		gripper.attachTo(robot.getFlange());
		gripper2F1.initalise();
		gripper2F1.setSpeed(180);
		gripper2F1.open();
		mF.setLEDBlue(true);
		ThreadUtil.milliSleep(200);
		mF.setLEDBlue(false);
		ThreadUtil.milliSleep(200);
		gripper2F1.close();
		
		//Spring motion initialisation
		
		springRobot = new CartesianImpedanceControlMode(); 
		
		springRobot.parametrize(CartDOF.X).setStiffness(180);
		springRobot.parametrize(CartDOF.Y).setStiffness(180);
		springRobot.parametrize(CartDOF.Z).setStiffness(1000);
		springRobot.parametrize(CartDOF.C).setStiffness(300);
		springRobot.parametrize(CartDOF.B).setStiffness(300);
		springRobot.parametrize(CartDOF.A).setStiffness(300);
		springRobot.setReferenceSystem(World.Current.getRootFrame());
		springRobot.parametrize(CartDOF.ALL).setDamping(0.4);
		
	}
	
	public Vector3D frameToVector(){
		Frame frame = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		return Vector3D.of(frame.getX(), frame.getY(), frame.getZ());		
	}
	
	
	
	@Override
	public void run(){
 
		mF.setLEDBlue(false);
		ThreadUtil.milliSleep(180);
		
		Frame object = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		ForceCondition detectObject = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"),10 );
		
		gripper2F1.close();
		robot.move(ptp(getApplicationData().getFrame("/pickUp")).setJointVelocityRel(0.4));
		robot.move(linRel(0, 0, -110, World.Current.getRootFrame()).setCartVelocity(50).breakWhen(detectObject));
		robot.move(linRel(0, 0, 10, World.Current.getRootFrame()).setCartVelocity(50));
		object = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		
		gripper2F1.open();
		robot.move(linRel(0, 0, -20, World.Current.getRootFrame()).setCartVelocity(50).breakWhen(detectObject));
		gripper2F1.close();
		
		ThreadUtil.milliSleep(2000);
		
		if (gripper2F1.readObjectDetection() == 2){
			logger.info("Object detected");
			mF.setLEDBlue(true);
			ThreadUtil.milliSleep(200);
		} else if (gripper2F1.readObjectDetection() == 3) {
			logger.info("No objects detected");
			mF.setLEDBlue(true);
			ThreadUtil.milliSleep(200);
			mF.setLEDBlue(false);
			ThreadUtil.milliSleep(200);
			mF.setLEDBlue(true);
			ThreadUtil.milliSleep(200);
		}
		
		robot.move(ptp(getApplicationData().getFrame("/pickUp")).setJointVelocityRel(0.4));
		robot.move(lin(getApplicationData().getFrame("/handOver")).setJointVelocityRel(0.4));
		
		mF.setLEDBlue(true);
		ThreadUtil.milliSleep(200);
		
		robot.moveAsync(positionHold(springRobot, -1, TimeUnit.SECONDS));
		Vector3D fixPos = frameToVector();
		
		logger.info("Please, Take the thing :)");
		mF.setLEDBlue(true);
		
		
		boolean condition = false;
		while (condition != true) {
			Vector3D currentPos = frameToVector();
			Vector3D v1 = currentPos.subtract(fixPos);
			if (v1.length() > 30 && v1.getX() > 0) {
				mF.setLEDBlue(true);
				ThreadUtil.milliSleep(200);
				gripper2F1.open();
				mF.setLEDBlue(false);
				ThreadUtil.milliSleep(200);
				condition = true;
			} 
		}
		
		
		logger.info("slap me to take back the object");
		
		Timer timer = new Timer();
		timer.schedule(new TimerTask() {
            @Override
            public void run() {
                System.out.println("Time's up! Going back.");
                keepRunning = false;
            }
        }, 10*1000); // ten seconds timeout
		
		
		Vector3D currentPos = frameToVector();
		Vector3D v2 = currentPos.subtract(fixPos);
		
		while (v2.length() <= 10 || keepRunning) {
			v2 = currentPos.subtract(fixPos);
			if (v2.length() > 10) {
				mF.setLEDBlue(true);
				ThreadUtil.milliSleep(400);
				gripper2F1.setSpeed(100);
				gripper2F1.close();
				mF.setLEDBlue(false);
				
				ThreadUtil.milliSleep(3000);
				
				robot.move(ptp(getApplicationData().getFrame("/pickUp")).setJointVelocityRel(0.4));//frame1
				robot.move(ptp(getApplicationData().getFrame("/object")).setJointVelocityRel(0.4));//frame1
				gripper2F1.open();
			}
		} 
		
		//moving back
		robot.move(ptp(getApplicationData().getFrame("/DrivePos")).setJointVelocityRel(0.4));
	}
}
