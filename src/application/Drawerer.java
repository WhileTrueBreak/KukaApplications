package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.ListIterator;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.Pair;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.math.geometry.Vector3D;
import com.kuka.nav.geometry.Vector2D;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.task.ITaskLogger;

import application.parser.FileReader;
import application.parser.PathParser;
import application.path.Path;
import application.robotControl.Canvas;
import application.robotControl.RobotController;
import application.utils.Handler;

public class Drawerer extends RoboticsAPIApplication{
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
		
		Handler.setRobot(robot);
		Handler.setGripper(gripper2F1);
		Handler.setMediaFlangeIO(mF);
		Handler.setTool(gripper);
		Handler.setLogger(logger);


		logger.info(logger.toString());
		logger.info(robot.toString());
		logger.info(gripper2F1.toString());
		logger.info(mF.toString());
		logger.info(gripper.toString());
		logger.info(Handler.getLogger().toString());
		Handler.getLogger().info(Handler.getRobot().toString());
		Handler.getLogger().info(Handler.getGripper().toString());
		Handler.getLogger().info(Handler.getMediaFlangeIO().toString());
		Handler.getLogger().info(Handler.getTool().toString());
		
		// Initializes the boing boing
		springRobot = new CartesianImpedanceControlMode(); 
		
		// Set stiffness

		// TODO: Stiff in every direction except plane perpendicular to flange
		springRobot.parametrize(CartDOF.X).setStiffness(750);
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
		gripper.move(linRel(0,0, -20).setJointVelocityRel(0.2));
	}
	
	private void penDown(){
		gripper.move(linRel(0, 0, 30).setMode(springRobot).setCartVelocity(20));
	}
	
	private void springyMove(Spline path){
		int vel = 80;
		gripper.move(path.setMode(springRobot).setCartVelocity(vel));
	}

	private void drawSplines(List<Spline> splines, List<Vector2D> startLocs, Canvas canvas, Frame originFrame) {
		logger.info("Start Drawing");
		ListIterator<Spline> splineIterator = splines.listIterator();
		while(splineIterator.hasNext()){
			int index = splineIterator.nextIndex();
			logger.info("Start path "+index);
			Vector3D first = canvas.toWorld(startLocs.get(index)).add(RobotController.frameToVector(originFrame));
			logger.info("Moving to first frame");
			gripper.move(lin(RobotController.vectorToFrame(first, originFrame)).setCartVelocity(300));
			penDown();
			logger.info("Start spline path");
			springyMove(splineIterator.next());
			logger.info("Finished path");
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
		Frame originFrame = RobotController.calibrateFrame(gripper, 150);
		penUp();
		Frame originUpFrame = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		Vector3D origin = RobotController.frameToVector(originFrame);
		logger.info(String.format("Origin: %s", origin.toString()));

		logger.info("Moving to Origin up");
		RobotController.safeMove(lin(originUpFrame).setJointVelocityRel(0.2));
		gripper.move(linRel(0, 40, 0).setJointVelocityRel(0.2));
		logger.info("Calibrating point 2");
		Vector3D up = RobotController.frameToVector(RobotController.calibrateFrame(gripper, 150));
		penUp();
		logger.info(String.format("Up: %s", up.toString()));

		logger.info("Moving to Origin up");
		RobotController.safeMove(lin(originUpFrame).setJointVelocityRel(0.2));
		gripper.move(linRel(-40, 0,0).setJointVelocityRel(0.2));
		logger.info("Calibrating point 3");
		Vector3D right = RobotController.frameToVector(RobotController.calibrateFrame(gripper, 150));
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
		double dist = RobotController.maxMove(diag);
		logger.info(String.format("Found max at top right: %s", diag.toString()));
		
		// gets top right frame
		Vector3D top_right = RobotController.frameToVector(robot.getCurrentCartesianPosition(gripper.getFrame("/TCP")));
		double diag_mag = top_right.subtract(origin).length();
		double size = Math.min(diag_mag/Math.sqrt(2), Math.sqrt(dist*dist/2))*0.9;
		Canvas canvas = new Canvas(origin, canvasPlane, size);
		logger.info(String.format("Canvas size: %f", size));
		logger.info("Calibration completed.");
		mF.setLEDBlue(false);
		
		logger.info("Reading Path File");
		String resPath = FileReader.findUniqueFolder("res", "..");
//		List<String> file = FileReader.readFile(resPath+"/linie_c.txt");
//		if(file == null || file.size() != 1) {
//			logger.info("File is invalid");
//			return;
//		}
//		List<List<Vector2D>> paths = PathParser.parsePathV1(file.get(0), size);
//		logger.info(String.format("Paths: %d", paths.size()));
//		Spline[] splines = new Spline[paths.size()];
//		
//		logger.info("Creating Spline");
//		Vector3D v = Vector3D.of(10,0,0);
//		for (int i=0;i<paths.size();i++){
//			Frame[] tempFrames = new Frame[paths.get(i).size()];
//			for (int j=0;j<paths.get(i).size();j++) {
//				Vector3D path3D = canvas.toWorld(paths.get(i).get(j)).add(origin).add(v);
//				tempFrames[j] = RobotController.vectorToFrame(path3D, originFrame);
//			}
//
//			splines[i] = RobotController.framesToSpline(tempFrames);
//		}
//
//		gripper.move(lin(originUpFrame).setCartVelocity(300));
//		drawSplines(splines, paths, canvas, originFrame);
		
		List<Path> paths = PathParser.parsePathV2(resPath+"/font.txt");
		ArrayList<Spline> splines = new ArrayList<Spline>();
		List<Vector2D> startLocs = new ArrayList<Vector2D>();
		for(Path path:paths) {
			splines.add(RobotController.pathToSpline(path, canvas, originFrame));
			startLocs.add(path.getPath().get(0).getPos());
		}
		
		drawSplines(splines, startLocs, canvas, originFrame);
		
		logger.info("Moving to base");
		gripper.move(lin(originUpFrame).setJointVelocityRel(0.2));
		penUp();
		mF.setLEDBlue(true);
	}
}
