package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.circ;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.spl;

import java.util.Arrays;
import java.util.List;
import java.util.ListIterator;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;
import com.kuka.common.Pair;
import com.kuka.common.ThreadUtil;
import com.kuka.core.geometry.Vector;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.math.geometry.Vector3D;
import com.kuka.nav.geometry.Vector2D;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ITransformationProvider;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.ITransformation;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.SPL;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.persistenceModel.IPersistenceEngine;
import com.kuka.roboticsAPI.persistenceModel.XmlApplicationDataSource;
import com.kuka.task.ITaskLogger;

public class window extends RoboticsAPIApplication{
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
	
	private CartesianImpedanceControlMode springRobot;
	
	@Override
	public void initialize() {
		// Initialize
		springRobot = new CartesianImpedanceControlMode(); 
		
		// Set stiffness

		// TODO: Stiff in every direction except plane perpendicular to flange
		springRobot.parametrize(CartDOF.X).setStiffness(1750);
		springRobot.parametrize(CartDOF.Y).setStiffness(5000);
		springRobot.parametrize(CartDOF.Z).setStiffness(5000);

		// Stiff rotation
		springRobot.parametrize(CartDOF.C).setStiffness(300);
		springRobot.parametrize(CartDOF.B).setStiffness(300);
		springRobot.parametrize(CartDOF.A).setStiffness(300);
		springRobot.setReferenceSystem(World.Current.getRootFrame());
		springRobot.parametrize(CartDOF.ALL).setDamping(0.4);
		
		// Inits the Robot
		gripper.attachTo(robot.getFlange());
		gripper2F1.initalise();
		gripper2F1.setSpeed(150);
		gripper2F1.setForce(50);
		mF.setLEDBlue(false);
		gripper2F1.close();
		ThreadUtil.milliSleep(200);
	}
	
	private Frame calibrateFrame(Tool grip, double force){
		ForceCondition touch = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"), force);
		IMotionContainer motion1 = gripper.move(linRel(0, 0, 150, gripper.getFrame("/TCP")).setCartVelocity(30).breakWhen(touch));
		if (motion1.getFiredBreakConditionInfo() == null){
			logger.info("No Collision Detected");
			return null;
		}
		else{
			logger.info("Collision Detected");
			return robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		}

	}
	
	private Vector3D getCanvasPlane(Vector3D origin, Vector3D right){
		Vector3D hor = right.subtract(origin).normalize();
		return hor;
	}
	
	private Vector3D frameToVector(Frame frame){
		return Vector3D.of(frame.getX(), frame.getY(), frame.getZ());
	}
	
	public void run() {
		// Calibration sequence
		mF.setLEDBlue(true);

		

//		
//		//opening the lock
//		gripper.move(linRel(0, 0, -50).setJointVelocityRel(0.2));
//		gripper.move(lin(getApplicationData().getFrame("/Window_Main/lockUp")).setJointVelocityRel(0.2));
//		gripper.move(linRel(0, 0, 100).setJointVelocityRel(0.2).breakWhen(touch));
//		gripper.move(spl(getApplicationData().getFrame("/Window_Main/lockDown")).setJointVelocityRel(0.2));
//		
//		
//		
//		gripper.move(linRel(0, 0, -50).setJointVelocityRel(0.2));
//		gripper.move(ptp(getApplicationData().getFrame("/windowMainFrame/handle")).setJointVelocityRel(0.2));
//		gripper2F1.setPos(100);
//		gripper.move(linRel(0, 0, 100).setJointVelocityRel(0.2).breakWhen(touch));
//		
//		//linear movement
//		Vector3D openLine = openvector.multiply(500);
////		ForceCondition force = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"), 60);
//		gripper.move(linRel(0, 0, -30).setJointVelocityRel(0.2));
//		logger.info("moving on a line");
//		gripper.move(linRel(openLine.getZ(), openLine.getX(), openLine.getY()).setCartVelocity(30).setCartAcceleration(10).breakWhen(force));
//		
		robot.move(ptp(getApplicationData().getFrame("/P1")).setJointVelocityRel(0.5));
		
		//getting the vector
		robot.move(ptp(getApplicationData().getFrame("/windowHandle/P1")).setJointVelocityRel(0.5));
		logger.info("Calibrating vector point 1");
		Vector3D origin = frameToVector(calibrateFrame(gripper,10));
		logger.info(String.format("Origin: %s", origin.toString()));

		logger.info("Moving to left");
		robot.move(ptp(getApplicationData().getFrame("/windowHandle/P2")).setJointVelocityRel(0.5));
		logger.info("Calibrating vector point 2");
		ThreadUtil.milliSleep(1000);
		Vector3D right = frameToVector(calibrateFrame(gripper,10));
		logger.info(String.format("Right: %s", right.toString()));
			
		robot.move(linRel(0, 0, -20).setJointVelocityRel(0.2));
		// get world unit vectors
		Vector3D openvector = getCanvasPlane(origin, right);
		logger.info(String.format("Canvas X: (%s)", openvector.toString()));
		
		
		robot.move(ptp(getApplicationData().getFrame("/P1")).setJointVelocityRel(0.5));
		robot.move(ptp(getApplicationData().getFrame("/windowHandle")).setJointVelocityRel(0.5));
		robot.move(linRel(0, 0, 30).setJointVelocityRel(0.3));
		
		//ForceCondition force = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"), 500);
		
		Vector3D openLine = openvector.multiply(100);
		logger.info("moving on a line");
		robot.move(linRel(openLine.getX(), openLine.getY(), openLine.getZ()).setCartVelocity(20).setCartAcceleration(10));
		robot.move(linRel(-20, 0, -40).setJointVelocityRel(0.2));
		robot.move(linRel(openLine.getX(), openLine.getY(), openLine.getZ()).setCartVelocity(20).setCartAcceleration(10));
		robot.move(linRel(openLine.getX(), openLine.getY(), openLine.getZ()).setCartVelocity(20).setCartAcceleration(10));
	}
}

