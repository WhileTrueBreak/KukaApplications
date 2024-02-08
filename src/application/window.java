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
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.math.geometry.Vector3D;
import com.kuka.nav.geometry.Vector2D;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ForceComponentCondition;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ITransformationProvider;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.geometricModel.math.ITransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.SPL;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.persistenceModel.IPersistenceEngine;
import com.kuka.roboticsAPI.persistenceModel.XmlApplicationDataSource;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
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
		springRobot.parametrize(CartDOF.X).setStiffness(1000);
		springRobot.parametrize(CartDOF.Y).setStiffness(1000);
		springRobot.parametrize(CartDOF.Z).setStiffness(1000);
 
		// Stiff rotation
		springRobot.parametrize(CartDOF.C).setStiffness(100);
		springRobot.parametrize(CartDOF.B).setStiffness(100);
		springRobot.parametrize(CartDOF.A).setStiffness(100);
		springRobot.setReferenceSystem(World.Current.getRootFrame());
		springRobot.parametrize(CartDOF.ALL).setDamping(0.8);
		// Inits the Robot
		gripper.attachTo(robot.getFlange());
		gripper2F1.initalise();
		gripper2F1.setSpeed(100);
		gripper2F1.setForce(10);
		mF.setLEDBlue(false);
		gripper2F1.close();
		ThreadUtil.milliSleep(200);
	}
	private Frame calibrateFrame(Tool grip, double force){
		ForceCondition touch = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"), force);
		IMotionContainer motion1 = gripper.move(linRel(0, 0, 15, gripper.getFrame("/TCP")).setCartVelocity(30).breakWhen(touch));
		if (motion1.getFiredBreakConditionInfo() == null){
			logger.info("No Collision Detected");
			return null;
		}
		else{
			logger.info("Collision Detected");
			return robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		}
 
	}
	private Pair<Vector3D, Vector3D> getCanvasPlane(Vector3D origin, Vector3D up, Vector3D right){
		Vector3D ver = up.subtract(origin).normalize();
		Vector3D hor = right.subtract(origin).normalize();
 
		return new Pair<Vector3D, Vector3D>(hor, ver);
	}
	private Vector3D frameToVector(Frame frame){
		return Vector3D.of(frame.getX(), frame.getY(), frame.getZ());
	}
	public void run() {
		// Calibration sequence
		Frame window = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		Frame away = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		Frame handle = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		Frame lock1 = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		Frame lock2 = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		Frame lock3 = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		Frame lock4 = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		
		mF.setLEDBlue(true);
		//getting the vector
 
		logger.info("Moving to left");
		robot.move(linRel(0, 0, -10).setJointVelocityRel(0.2));
		gripper.move(ptp(getApplicationData().getFrame("/window/v2")).setJointVelocityRel(0.5));
		logger.info("Calibrating vector point 2");
		ThreadUtil.milliSleep(500);
		Vector3D right = frameToVector(calibrateFrame(gripper,30));
		logger.info(String.format("Right: %s", right.toString()));
		robot.move(linRel(0, 0, -10).setJointVelocityRel(0.2));
		logger.info("Moving to up");
		gripper.move(ptp(getApplicationData().getFrame("/window/v3")).setJointVelocityRel(0.5));
		logger.info("Calibrating vector point 3");
		ThreadUtil.milliSleep(500);
		Vector3D up = frameToVector(calibrateFrame(gripper,200));
		logger.info(String.format("Right: %s", up.toString()));
		
		gripper.move(ptp(getApplicationData().getFrame("/window/v1")).setJointVelocityRel(0.5));
		logger.info("Calibrating vector point 1");
		Vector3D origin = frameToVector(calibrateFrame(gripper,30));
		logger.info(String.format("Origin: %s", origin.toString()));
		
		// get world unit vectors
		Pair<Vector3D,Vector3D> openLine = getCanvasPlane(origin, up, right);
		logger.info(String.format("Canvas X, Y: (%s), (%s)", openLine.getA().toString(), openLine.getB().toString()));
		
		//calibrating Main frame
		
		
		gripper.move(linRel(0, 0, -10).setJointVelocityRel(0.3));
		//ForceCondition touch = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"), 200);
		
		ForceComponentCondition FORCE = new ForceComponentCondition(gripper.getFrame("/TCP"), CoordinateAxis.Y, -250.0,-25);
		ICondition FORCE1 = FORCE.invert();

		IMotionContainer motion = gripper.move(linRel(0,100, 0, gripper.getFrame("/TCP")).setCartVelocity(30).breakWhen(FORCE1));
		gripper.move(linRel(0,-10,0).setJointVelocityRel(0.3));
		if (FORCE1 != null){
			logger.error("No Collision Detected");
		}
	
		window = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		
		// defining other frames
		away.setX(window.getX() -16);
		away.setY(window.getY()-36);
		away.setZ(window.getZ()-156);
		away.setAlphaRad(window.getAlphaRad() + 1.5);
		away.setBetaRad(window.getBetaRad());
		away.setGammaRad(window.getGammaRad());
		
		handle.setX(window.getX());
		handle.setY(window.getY()+48);
		handle.setZ(window.getZ()-39);
		handle.setAlphaRad(window.getAlphaRad());
		handle.setBetaRad(window.getBetaRad());
		handle.setGammaRad(window.getGammaRad());
		
		lock1.setX(window.getX() -118);
		lock1.setY(window.getY()+ 14);
		lock1.setZ(window.getZ()-29);
		lock1.setAlphaRad(window.getAlphaRad());
		lock1.setBetaRad(window.getBetaRad());
		lock1.setGammaRad(window.getGammaRad());
		
		lock2.setX(window.getX()-154);
		lock2.setY(window.getY()+19);
		lock2.setZ(window.getZ()-19);
		lock2.setAlphaRad(window.getAlphaRad());
		lock2.setBetaRad(window.getBetaRad());
		lock2.setGammaRad(window.getGammaRad());
		
		lock3.setX(window.getX() -180);
		lock3.setY(window.getY()+27);
		lock3.setZ(window.getZ()-29);
		lock3.setAlphaRad(window.getAlphaRad());
		lock3.setBetaRad(window.getBetaRad());
		lock3.setGammaRad(window.getGammaRad());
		
		lock4.setX(window.getX() -211);
		lock4.setY(window.getY()+44);
		lock4.setZ(window.getZ()-55);
		lock4.setAlphaRad(window.getAlphaRad());
		lock4.setBetaRad(window.getBetaRad());
		lock4.setGammaRad(window.getGammaRad());
		
		Spline mySpline = new Spline(
				spl(away),
				spl(lock1),
				spl(lock2),
				spl(lock3),
				spl(lock4),
				spl(away)
		);
		logger.info("test 1");
		gripper.move(ptp(away).setJointVelocityRel(0.3));
		logger.info("test2");
		gripper.move(circ(lock1,lock2).setJointVelocityRel(0.2));
		logger.info("test3");
		gripper.move(mySpline.setJointVelocityRel(0.4));			
		
		gripper.move(ptp(handle).setJointVelocityRel(0.4).setMode(springRobot));
		robot.move(linRel(0, 0, -10).setJointVelocityRel(0.3).setMode(springRobot));
		gripper2F1.setPos(20);
		robot.move(linRel(0, 0, 20).setJointVelocityRel(0.3).setMode(springRobot));
		ThreadUtil.milliSleep(100);
		gripper2F1.close();
		
		Vector3D diag = openLine.getA().multiply(600);
		logger.info("moving on a line");
		double acc = 20;
		robot.move(linRel(diag.getZ(), diag.getX(), diag.getY()).setCartVelocity(10).setCartAcceleration(acc).setMode(springRobot));
//		int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "OK", "Cancel");
//		if (isCancel == 1)
//        {
//			logger.info("cancelled");
//        } else {
//        	logger.info("continuing");
//        }
	}
}