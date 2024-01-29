package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.ArrayList;
import java.util.List;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.Pair;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.math.geometry.Vector3D;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.RobotMotion;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.task.ITaskLogger;

import application.parser.FileReader;
import application.path.PathPlan;
import application.path.PointPath;
import application.robotControl.Canvas;
import application.robotControl.RobotController;
import application.text.TextManager;
import application.utils.Handler;

/*
 * Writes monash\n innovation\n labs mirrored
 */

public class Drawerer1 extends RoboticsAPIApplication{
	@Inject
	private SunriseOmniMoveMobilePlatform kmp;
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

	public static final double PEN_UP_DIST = 5;
	public static final double PEN_DOWN_DIST = 5;
	
	@Override
	public void initialize() {
		
		Handler.setRobot(robot);
		Handler.setGripper(gripper2F1);
		Handler.setMediaFlangeIO(mF);
		Handler.setTool(gripper);
		Handler.setLogger(logger);

		// Initializes the boing boing
		springRobot = new CartesianImpedanceControlMode(); 
		
		// Set stiffness

		// TODO: Stiff in every direction except plane perpendicular to flange
		springRobot.parametrize(CartDOF.X).setStiffness(500);
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
		gripper2F1.setSpeed(189);
//		gripper2F1.open();
//		mF.setLEDBlue(true);
//		ThreadUtil.milliSleep(10000);
		mF.setLEDBlue(false);
		gripper2F1.close();
		ThreadUtil.milliSleep(200);
	}

	private void penUp(){
		gripper.move(linRel(0,0, -Drawerer1.PEN_UP_DIST).setJointVelocityRel(0.2));
	}
	
	private void penDown(){
		gripper.move(linRel(0, 0, Drawerer1.PEN_DOWN_DIST+Drawerer1.PEN_UP_DIST).setMode(springRobot).setCartVelocity(20));
	}
	
	private void springyMove(RobotMotion<?> motion){
		gripper.move(motion.setMode(springRobot));
	}

	private void drawPathPlan(PathPlan plan, Frame originFrame, Canvas canvas) {
		logger.info("Paths: " + plan.getMotions().size());
		logger.info("Start Drawing");
		Vector3D v = Vector3D.of(-Drawerer1.PEN_UP_DIST,0,0);
		for(int i = 0;i < plan.getStartLocs().size();i++) {
			logger.info("Start path "+i);
			Vector3D first = canvas.toWorld(plan.getStartLocs().get(i)).add(RobotController.frameToVector(originFrame)).add(v);
			logger.info("Moving to first frame");
			gripper.move(lin(RobotController.vectorToFrame(first, originFrame)).setCartVelocity(200));
			penDown();
			logger.info("Start path");
			springyMove(plan.getMotions().get(i));
			logger.info("Finished Path");
			penUp();
		}
	}
	
	@Override
	public void run() throws Exception {
		// Calibration sequence
		mF.setLEDBlue(true);
		logger.info("Moving to bottom left");
		try {
			gripper.move(lin(getApplicationData().getFrame("/bottom_left")).setJointVelocityRel(0.2));
		} catch (Exception e) {
			gripper.move(ptp(getApplicationData().getFrame("/bottom_left")).setJointVelocityRel(0.2));
		}
		
		logger.info("Calibrating point 1");
		Frame originFrame = RobotController.calibrateFrame(robot, gripper, 150);
		gripper.move(linRel(0,0, -Drawerer1.PEN_UP_DIST*2).setJointVelocityRel(0.2));
		Frame originUpFrame = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		Vector3D origin = RobotController.frameToVector(originFrame);
		logger.info(String.format("Origin: %s", origin.toString()));

		logger.info("Moving to Origin up");
		RobotController.safeMove(gripper, lin(originUpFrame).setJointVelocityRel(0.2));
		gripper.move(linRel(0, 50, 0).setJointVelocityRel(0.2));
		logger.info("Calibrating point 2");
		Vector3D up = RobotController.frameToVector(RobotController.calibrateFrame(robot, gripper, 150));
		penUp();
		logger.info(String.format("Up: %s", up.toString()));

		logger.info("Moving to Origin up");
		RobotController.safeMove(gripper, lin(originUpFrame).setJointVelocityRel(0.2));
		gripper.move(linRel(-50, 0,0).setJointVelocityRel(0.2));
		logger.info("Calibrating point 3");
		Vector3D right = RobotController.frameToVector(RobotController.calibrateFrame(robot, gripper, 150));
		penUp();
		logger.info(String.format("Right: %s", right.toString()));
		

		// get world unit vectors
		Pair<Vector3D,Vector3D> canvasPlane = Canvas.getCanvasPlane(origin, up, right);
		logger.info(String.format("Canvas X, Y: (%s), (%s)", canvasPlane.getA().toString(), canvasPlane.getB().toString()));

		// check upper right bound
		gripper.move(lin(originUpFrame).setJointVelocityRel(0.2));
		Vector3D diag = canvasPlane.getA().add(canvasPlane.getB());
		logger.info("Diagonal vector: " + diag.toString());
		logger.info("Moving to top right");
		double dist = RobotController.maxMove(gripper, diag);
		logger.info(String.format("Found max at top right: %s", diag.toString()));
		
		// gets top right frame
		Vector3D top_right = RobotController.frameToVector(robot.getCurrentCartesianPosition(gripper.getFrame("/TCP")));
		double diag_mag = top_right.subtract(origin).length();
		double size = Math.min(diag_mag/Math.sqrt(2), Math.sqrt(dist*dist/2))*0.9;
		Canvas canvas = new Canvas(origin, canvasPlane, size);
		logger.info(String.format("Canvas size: %f", size));
		logger.info("Calibration completed.");

		logger.info("Reading file");
		String resPath = FileReader.findUniqueFolder("res", "..");
		
		double buffer = 0.05;
		double scale = 0.15;
		double charHeight = scale;
		double spacing = scale/10;
		double currentY = 0.8-charHeight-buffer;
		double currentX;
		
		TextManager.setFontPath(resPath+"/font/arialnarrow");
		TextManager.setBaseScale(scale);
		
		String chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
		for(int i = 0;i < chars.length();i++) {
			logger.info("Loading char: " + chars.charAt(i));
			TextManager.loadChar(chars.charAt(i), canvas);
		}
		
		List<String> text = new ArrayList<String>();
		text.add("Monash");
		text.add("Innovation");
		text.add("Labs");
		for(String line:text) {
			currentX = 1-buffer;
			List<PointPath> PointPaths = new ArrayList<PointPath>();
			for(int i = 0;i < line.length();i++) {
				if(line.charAt(i) == ' ') {
					currentX -= spacing*3;
					continue;
				}
				PointPath pointPath = TextManager.getCharPath(line.charAt(i));
				pointPath.mirrorPaths();
				pointPath.scalePaths(scale);
				pointPath.offsetPaths(-pointPath.getBounds().getX(), 0);
				pointPath.offsetPaths(currentX-pointPath.getBounds().getWidth(), currentY);
				currentX -= pointPath.getBounds().getWidth() + spacing;
				PointPaths.add(pointPath);
			}
			for(PointPath pointPath:PointPaths) {
				drawPathPlan(pointPath.toPathPlan(robot, originFrame, canvas, 100), originFrame, canvas);
			}
			currentY -= charHeight + buffer; 
		}
		
		logger.info("Moving to base");
		gripper.move(lin(originUpFrame).setJointVelocityRel(0.2));
		penUp();
	}
}
