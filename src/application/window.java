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
import com.kuka.roboticsAPI.RoboticsAPIContext;
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
import com.kuka.roboticsAPI.motionModel.OrientationReferenceSystem;
import com.kuka.roboticsAPI.motionModel.SPL;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.SplineOrientationType;
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
		springRobot.parametrize(CartDOF.X).setStiffness(400);
		springRobot.parametrize(CartDOF.Y).setStiffness(400);
		springRobot.parametrize(CartDOF.Z).setStiffness(2000);
		// Stiff rotation
		springRobot.parametrize(CartDOF.C).setStiffness(100);
		springRobot.parametrize(CartDOF.B).setStiffness(100);
		springRobot.parametrize(CartDOF.A).setStiffness(100);
		springRobot.setReferenceSystem(World.Current.getRootFrame());
		springRobot.parametrize(CartDOF.ALL).setDamping(1);
		// Inits the Robot
		gripper.attachTo(robot.getFlange());
		gripper2F1.initalise();
		gripper2F1.setPos(180);
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
			return robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"),World.Current.getRootFrame());
		}
	}
	private Pair<Vector3D, Vector3D> getCanvasPlane(Vector3D origin, Vector3D up, Vector3D left){
		Vector3D ver = up.subtract(origin).normalize();
		Vector3D hor = origin.subtract(left).normalize();
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
		gripper.move(ptp(getApplicationData().getFrame("/window/v2")).setJointVelocityRel(0.5));
		logger.info("Calibrating vector point 2");
		ThreadUtil.milliSleep(500);
		Vector3D left = frameToVector(calibrateFrame(gripper,30));
		logger.info(String.format("left: %s", left.toString()));
		//try this///
		logger.info("testing relative movement");
		gripper.move(linRel(0, -10, 0, World.Current.getRootFrame()).setJointVelocityRel(0.2));
		logger.info("Moving to up");
		gripper.move(ptp(getApplicationData().getFrame("/window/v3")).setJointVelocityRel(0.3));
		logger.info("Calibrating vector point 3");
		ThreadUtil.milliSleep(500);
		Vector3D up = frameToVector(calibrateFrame(gripper,60));
		logger.info(String.format("Right: %s", up.toString()));
		robot.move(linRel(0, 0, -10).setJointVelocityRel(0.2));
		logger.info("Moving to origin");
		gripper.move(ptp(getApplicationData().getFrame("/window/v1")).setJointVelocityRel(0.5));
		logger.info("Calibrating vector point 1");
		Vector3D origin = frameToVector(calibrateFrame(gripper,35));
		logger.info(String.format("Origin: %s", origin.toString()));
		// get world unit vectors
		Pair<Vector3D,Vector3D> openLine = getCanvasPlane(origin, up, left);
		logger.info(String.format("Canvas X, Y: (%s), (%s)", openLine.getA().toString(), openLine.getB().toString()));
//		//calibrating Main frame
// 
//		gripper.move(linRel(0, 0, -10).setJointVelocityRel(0.3));
//		//ForceCondition touch = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"), 200);
//		ForceComponentCondition FORCE = new ForceComponentCondition(gripper.getFrame("/TCP"), CoordinateAxis.Y, -250.0,-25);
//		ICondition FORCE1 = FORCE.invert();
//		IMotionContainer motion = gripper.move(linRel(0,100, 0, gripper.getFrame("/TCP")).setCartVelocity(30).breakWhen(FORCE1));
//		gripper.move(linRel(0,-10,0).setJointVelocityRel(0.3));
//		if (motion.getFiredBreakConditionInfo() == null){
//			logger.info("No Collision Detected");
//		}
//		else{
//			logger.info("Collision Detected");
//			window = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"),World.Current.getRootFrame());
//		}
//		// defining other frames
//		// YZX
//		away.setX(window.getX()-35.8);
//		away.setY(window.getY()-156);
//		away.setZ(window.getZ()-16);
//		away.setAlphaRad(window.getAlphaRad() + 1.5);
//		away.setBetaRad(window.getBetaRad());
//		away.setGammaRad(window.getGammaRad());
//		handle.setX(window.getX()+48);   
//		handle.setY(window.getY()-45); 
//		handle.setZ(window.getZ());
//		handle.setAlphaRad(window.getAlphaRad());
//		handle.setBetaRad(window.getBetaRad());
//		handle.setGammaRad(window.getGammaRad());
//		lock1.setX(window.getX()+10);
//		lock1.setY(window.getY()-30);
//		lock1.setZ(window.getZ()-105); 
//		lock1.setAlphaRad(window.getAlphaRad());
//		lock1.setBetaRad(window.getBetaRad());
//		lock1.setGammaRad(window.getGammaRad());
//		lock2.setX(window.getX()+15);
//		lock2.setY(window.getY()-15);
//		lock2.setZ(window.getZ()-154);
//		lock2.setAlphaRad(window.getAlphaRad());
//		lock2.setBetaRad(window.getBetaRad());
//		lock2.setGammaRad(window.getGammaRad());
//		lock3.setX(window.getX()+30);
//		lock3.setY(window.getY()-20);
//		lock3.setZ(window.getZ()-180);
//		lock3.setAlphaRad(window.getAlphaRad());
//		lock3.setBetaRad(window.getBetaRad());
//		lock3.setGammaRad(window.getGammaRad());
//		lock4.setX(window.getX()+45);
//		lock4.setY(window.getY()-45);
//		lock4.setZ(window.getZ()-210);
//		lock4.setAlphaRad(window.getAlphaRad());
//		lock4.setBetaRad(window.getBetaRad());
//		lock4.setGammaRad(window.getGammaRad());
//
//		Spline mySpline = new Spline(
//				spl(lock1),
//				spl(lock2),
//				spl(lock3),
//				spl(lock4),
//				spl(away)
//		);
//		logger.info("spline");
//		gripper.move(ptp(getApplicationData().getFrame("/window/away")).setJointVelocityRel(0.2).setMode(springRobot));
//		gripper.move(mySpline.setJointVelocityRel(0.4).setMode(springRobot).setOrientationType(SplineOrientationType.Constant));	
//		//gripper.move(ptp(getApplicationData().getFrame("/window/away")).setJointVelocityRel(0.2).setMode(springRobot));
//		gripper.move(ptp(handle).setJointVelocityRel(0.4).setMode(springRobot));
		//gripper.move(linRel(0, -10, 0).setJointVelocityRel(0.3).setMode(springRobot));
		gripper2F1.setPos(150);
		gripper.move(linRel(0, -20, 0, World.Current.getRootFrame()).setJointVelocityRel(0.3).setMode(springRobot));
		ThreadUtil.milliSleep(500);
		gripper2F1.close();
		Vector3D diag = openLine.getA().multiply(-600);
		logger.info("moving on a line");
		double acc = 20;
		robot.move(linRel(diag.getX(), diag.getY(), diag.getZ()).setCartVelocity(10).setCartAcceleration(acc).setMode(springRobot));
 
	}
}
 
//private void updateFrame(ObjectFrame baseName, AbstractFrame newBase)
//{
//    RoboticsAPIContext context = _lbr.getContext();
//    final IPersistenceEngine engine = context.getEngine(IPersistenceEngine.class);
//    final XmlApplicationDataSource defaultDataSource = (XmlApplicationDataSource) engine.getDefaultDataSource();
//    defaultDataSource.changeFrameTransformation(baseName, baseName.getParent().transformationTo(newBase));
//    defaultDataSource.saveFile(false);
//}
 
//int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "OK", "Cancel");
//if (isCancel == 1)
//{
//	logger.info("cancelled");
//} else {
//	logger.info("continuing");
//}