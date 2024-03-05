package application;
 
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
		//FORCE CONDITIONS EXAMPLE
		ForceCondition touch10 = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"),10 );
		ForceCondition touch15 = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"),15 );
		//USAGE, will move to next line when triggered
		//LOOK at pipecutting.java for examples on analysing the break condition. 
		//gripper.move(linRel(0, 0, -30, World.Current.getRootFrame()).setCartVelocity(50).breakWhen(touch10)); 
	}
 
	@Override
	public void run() {
		IHonkCapability honkCapability = kmp.getCapability(IHonkCapability.class);
		honkCapability.honk();
		gripper2F1.close();
 
		mF.setLEDBlue(false);
		ThreadUtil.milliSleep(200);
 
		gripper.move(ptp(getApplicationData().getFrame("/P9")).setJointVelocityRel(0.3));//frame1-collabtable
//	    gripper.move(linRel(0, 0, -30, World.Current.getRootFrame()).setCartVelocity(50));//going down
		gripper2F1.close();
		mF.setLEDBlue(true);
//		gripper.move(lin(getApplicationData().getFrame("/P1")).setCartVelocity(100));//get back to frame1
//		gripper.move(lin(getApplicationData().getFrame("/P2")).setCartVelocity(200));// go to frame2
	    gripper.move(linRel(0, 0, 90, World.Current.getRootFrame()).setCartVelocity(50));// going up
//		gripper2F1.open();
//		mF.setLEDBlue(false);
	    gripper.move(ptp(getApplicationData().getFrame("/P9/P1")).setJointVelocityRel(0.2));
// 
//		
	    gripper.move(ptp(getApplicationData().getFrame("/P7")).setJointVelocityRel(0.3));
//	    gripper.move(linRel(0, 0, -60, World.Current.getRootFrame()).setCartVelocity(50));//going down
//		gripper2F1.close();
//		gripper.move(lin(getApplicationData().getFrame("/P1")).setCartVelocity(100));//get back to frame1
//		gripper.move(lin(getApplicationData().getFrame("/P2")).setCartVelocity(200));// go to frame2
//	    gripper.move(linRel(0, 0, -60, World.Current.getRootFrame()).setCartVelocity(50));// going down
		gripper2F1.open();
//	    gripper.move(lin(getApplicationData().getFrame("/P2")).setCartVelocity(100));
//		gripper.move(lin(getApplicationData().getFrame("/P1")).setCartVelocity(200));//frame1
//	    gripper.move(linRel(0, 0, -90, World.Current.getRootFrame()).setCartVelocity(50));//going down
//		gripper2F1.close();
//		mF.setLEDBlue(true);
//		gripper.move(lin(getApplicationData().getFrame("/P1")).setCartVelocity(100));//get back to frame1
//		gripper.move(lin(getApplicationData().getFrame("/P2")).setCartVelocity(200));// go to frame2
//	    gripper.move(linRel(0, 0, -30, World.Current.getRootFrame()).setCartVelocity(50));// going down
//		gripper2F1.open();
//		mF.setLEDBlue(false);
//	    gripper.move(lin(getApplicationData().getFrame("/P2")).setCartVelocity(100));
//	    robot.move(lin(getApplicationData().getFrame("/DrivePos")).setCartVelocity(100));
		
		/// try this hand guiding mode ///
//		double x = 0.3;
//		double y = 0.3;
//		robot.move(ptp(getApplicationData().getFrame("/P1")).setJointVelocityRel(x));
//		robot.move(handGuiding().setJointVelocityLimit(x));
//		//robot.setESMState("1");
//		robot.move(ptp(getApplicationData().getFrame("/P2")));
//		
		/// try circ and spline motions///
		
		// // ///// force conditions / torquea ////////
//		double forceInX = 0;
//		double forceInY = 0;
//		double forceInZ = 0;
//		
//		while (forceInX < 10) {
//			logger.info("No force in X ditected"+forceInX);
//			mF.setLEDBlue(false);
//			ForceSensorData data = robot.getExternalForceTorque(robot.getFlange(),World.Current.getRootFrame());
//			Vector force = data.getForce();
//			forceInX = force.getX();
//			ThreadUtil.milliSleep(200);
//		}
//		gripper2F1.open();
//		logger.info("Force in X ditected"+forceInX);
		
		//
////		
//		ForceCondition condition = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"), 10.0);
//
//		
//		ICallbackAction Action = new ICallbackAction() {
//			@Override
//			public void onTriggerFired(IFiredTriggerInfo triggerInformation) {
//			 //toggle output state when trigger fired
//				gripper2F1.open();
//				logger.info("yay");
//			}
//		};
//		
//
//		robot.move(linRel(0, 0, -60, World.Current.getRootFrame()).setCartVelocity(10).triggerWhen(condition, Action));
//		robot.move(linRel(0, 0, 60, World.Current.getRootFrame()).setCartVelocity(10));
//		
//		ForceCondition cond_2 = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"), 10.0);
//		gripper.move(linRel(0, 400, 0, World.Current.getRootFrame()).setCartVelocity(20).breakWhen(cond_2));//going straight
//		if (cond_2 == null){
//			logger.info("No Collision Detected");
//			mF.setLEDBlue(false);
//			ThreadUtil.milliSleep(200);
//		}
//		else{
//			logger.info("Collision Detected");
//			mF.setLEDBlue(true);
//			ThreadUtil.milliSleep(200);
//		}

	}
}