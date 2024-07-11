package application;
 
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;
import javax.inject.Inject;
import javax.inject.Named;
//import com.kuka.math.geometry.Vector3D;
import com.kuka.math.geometry.Vector3D;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICallbackAction;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.ITriggerAction;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.executionModel.IFiredTriggerInfo;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.CartPlane;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.RelativeLIN;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.SplineMotionCP;
import com.kuka.roboticsAPI.motionModel.SplineOrientationType;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
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
public class quick extends RoboticsAPIApplication {
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
	private window wind;
	CartesianSineImpedanceControlMode lissajousMode;
	IMotionContainer m1;
	
	public double PosX, PosY, PosZ;
	private static boolean keepRunning = true;
	
	@Override
	public void initialize() {
		gripper.attachTo(robot.getFlange());
		gripper2F1.initalise();
		gripper2F1.setForce(200);
		gripper2F1.setSpeed(100);
		gripper2F1.setPos(150);
		mF.setLEDBlue(true);
		ThreadUtil.milliSleep(200);
		gripper2F1.close();
		mF.setLEDBlue(false);
		ThreadUtil.milliSleep(200);
		//Spring motion initialisation
		
		//USAGE, will move to next line when triggered
		//LOOK at pipecutting.java for examples on analysing the break condition. 
		//gripper.move(linRel(0, 0, -30, World.Current.getRootFrame()).setCartVelocity(50).breakWhen(touch10));
 

	}
	
	@Override
	public void run() {
//		wind.initialize();
//		wind.run();
//		
		
		gripper2F1.close();
		
		mF.setLEDBlue(true);
		gripper.move(ptp(getApplicationData().getFrame("/P10")).setJointVelocityRel(0.3));
		CartesianSineImpedanceControlMode lissajousMode;
		lissajousMode = CartesianSineImpedanceControlMode.createLissajousPattern(CartPlane.YZ, 1, 6.0, 300.0);
		lissajousMode.parametrize(CartDOF.A).setStiffness(80);
		lissajousMode.parametrize(CartDOF.B).setStiffness(80);
		lissajousMode.parametrize(CartDOF.C).setStiffness(80);
		lissajousMode.parametrize(CartDOF.X).setStiffness(300);
		mF.setLEDBlue(true);
		IMotionContainer m2 = gripper.move(positionHold(lissajousMode, 10, TimeUnit.MINUTES));
	}
}