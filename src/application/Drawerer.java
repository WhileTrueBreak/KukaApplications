package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
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
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.RobotMotion;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.task.ITaskLogger;

import application.parser.FileReader;
import application.parser.PathParser;
import application.path.Path;
import application.robotControl.Canvas;
import application.robotControl.RobotController;
import application.utils.Handler;
import application.utils.MathHelper;

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
	
	private void springyMove(RobotMotion<?> motion){
		gripper.move(motion.setMode(springRobot));
	}

	@SuppressWarnings("unused")
	private void drawSplines(List<Spline> splines, List<Vector2D> startLocs, Canvas canvas, Frame originFrame) {
		logger.info("Start Drawing");
		ListIterator<Spline> splineIterator = splines.listIterator();
		while(splineIterator.hasNext()){
			int index = splineIterator.nextIndex();
			logger.info("Start path "+index);
			Vector3D first = canvas.toWorld(startLocs.get(index)).add(RobotController.frameToVector(originFrame)).add(Vector3D.of(-20, 0, 0));
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

		List<MotionBatch> motions = new ArrayList<MotionBatch>();
		List<Vector2D> startLocs = new ArrayList<Vector2D>();
		
		List<String> file = FileReader.readFile(resPath+"/font.txt");
		List<Path> paths = PathParser.parsePathV2(file);

		Vector3D v = Vector3D.of(10,0,0);
		for(int n=0;n<paths.size();n++) {
			Path path = paths.get(n);
			Rectangle2D bounds = path.getBounds();
			List<RobotMotion<?>> pathMotions = new ArrayList<RobotMotion<?>>();
			List<Vector3D> points = new ArrayList<Vector3D>();
			List<Vector3D> controlPoints = new ArrayList<Vector3D>();
			for(int i = 0;i < path.getPath().size();i++) {
				Vector3D currPos = canvas.toWorld(path.getPath().get(i).getPos()).add(canvas.getOrigin()).add(v);
				
				if(path.getPath().get(i).isBlend() || controlPoints.isEmpty()) {
					controlPoints.add(currPos);
					continue;
				}
				if(controlPoints.size() == 1) {
					points.add(currPos);
					controlPoints.clear();
					controlPoints.add(currPos);
					continue;
				}
				controlPoints.add(currPos);
				points.addAll(RobotController.bezierToVectors(controlPoints, 100));
				controlPoints.clear();
				controlPoints.add(currPos);
			}
			Vector3D prevDir = null;
			Vector3D prevPos = null;
			for(Vector3D currPos:points) {
				if(prevPos != null) {
					Vector3D currDir = currPos.subtract(prevPos);
					if(prevDir != null) {
						double angle = currDir.angleRad(prevDir);
						double blend = MathHelper.qerp(0,0,1,MathHelper.clamp(angle/(Math.PI),0,1))*10;
						pathMotions.get(pathMotions.size()-1).setBlendingCart(blend);
					}
					prevDir = currDir;
				}
				prevPos = currPos;
				
				Frame frame = RobotController.vectorToFrame(currPos, originFrame);
				LBRE1Redundancy e1val = new LBRE1Redundancy();
				e1val.setE1(0);
				frame.setRedundancyInformation(robot, e1val);
				
				pathMotions.add(new LIN(frame).setCartVelocity(100).setBlendingRel(0).setCartAcceleration(100));
			}
			MotionBatch motionBatch = new MotionBatch(pathMotions.toArray(new RobotMotion<?>[pathMotions.size()]));
			motions.add(motionBatch);
			startLocs.add(paths.get(n).getPath().get(0).getPos());
		}
		
//		List<String> file = FileReader.readFile(resPath+"/newyears_mirror.txt");
//		if(file == null || file.size() != 1) {
//			logger.info("File is invalid");
//			return;
//		}
//		List<List<Vector2D>> paths = PathParser.parsePathV1(file.get(0), size);
//		logger.info(String.format("Paths: %d", paths.size()));
//
//		logger.info("Calculating paths");
//		Vector3D v = Vector3D.of(10,0,0);
//		for (int i=0;i<paths.size();i++){
//			RobotMotion<?>[] pathMotions = new RobotMotion<?>[paths.get(i).size()];
//			Vector3D prevDir = null;
//			Vector3D prevPos = null;
//			for (int j=0;j<paths.get(i).size();j++) {
//				Vector3D path3D = canvas.toWorld(paths.get(i).get(j)).add(origin).add(v);
//				Frame frame = RobotController.vectorToFrame(path3D, originFrame);
//				Vector3D currPos = RobotController.frameToVector(frame);
//				if(prevPos != null) {
//					Vector3D currDir = currPos.subtract(prevPos);
//					if(prevDir != null) {
//						double angle = currDir.angleRad(prevDir);
//						double blend = MathHelper.qerp(0,0,1,MathHelper.clamp(angle/(Math.PI),0,1))*30;
//						pathMotions[j-1].setBlendingCart(blend);
//					}
//					prevDir = currDir;
//				}
//				prevPos = currPos;
//				LBRE1Redundancy e1val = new LBRE1Redundancy();
//				e1val.setE1(0);
//				frame.setRedundancyInformation(robot, e1val);
//				
//				
//				pathMotions[j] = new LIN(frame).setCartVelocity(100).setBlendingRel(0).setCartAcceleration(100);
//			}
//			MotionBatch motionBatch = new MotionBatch(pathMotions);
//			motions.add(motionBatch);
//			startLocs.add(paths.get(i).get(0));
//		}

		gripper.move(lin(originUpFrame).setCartVelocity(100));

		logger.info("Paths: " + motions.size());
		logger.info("Start Drawing");
		for(int i = 0;i < startLocs.size();i++) {
			logger.info("Start path "+i);
			Vector3D first = canvas.toWorld(startLocs.get(i)).add(RobotController.frameToVector(originUpFrame));
			logger.info("Moving to first frame");
			gripper.move(lin(RobotController.vectorToFrame(first, originFrame)).setCartVelocity(300));
			penDown();
			logger.info("Start path");
			springyMove(motions.get(i));
			logger.info("Finished Path");
			penUp();
		}
		
//		List<Path> paths = PathParser.parsePathV2(resPath+"/font.txt");
//		ArrayList<Spline> splines = new ArrayList<Spline>();
//		List<Vector2D> startLocs = new ArrayList<Vector2D>();
//		for(Path path:paths) {
//			splines.add(RobotController.pathToSpline(path, canvas, originFrame));
//			startLocs.add(path.getPath().get(0).getPos());
//		}
//		
//		drawSplines(splines, startLocs, canvas, originFrame);
		
//		//hard coded spline?
//		Vector3D p1 = canvas.toWorld(new Vector2D(0, 0)).add(RobotController.frameToVector(originUpFrame));
//		Vector3D p2 = canvas.toWorld(new Vector2D(0, 1)).add(RobotController.frameToVector(originUpFrame));
//		Vector3D p3 = canvas.toWorld(new Vector2D(1, 1)).add(RobotController.frameToVector(originUpFrame));
//
//		gripper.move(new LIN(RobotController.vectorToFrame(p1, originUpFrame)).setCartVelocity(100));
//		
//		List<RobotMotion<?>> motions = new ArrayList<RobotMotion<?>>();
//		for(double t = 0;t < 1;t+=0.01) {
//			Vector3D tmp = Vector3D.of(
//					MathHelper.qerp(p1.getX(), p2.getX(), p3.getX(), t), 
//					MathHelper.qerp(p1.getY(), p2.getY(), p3.getY(), t), 
//					MathHelper.qerp(p1.getZ(), p2.getZ(), p3.getZ(), t));
//			AbstractFrame frame = RobotController.vectorToFrame(tmp, originUpFrame);
//			LBRE1Redundancy e1val = new LBRE1Redundancy();
//			e1val.setE1(0);
//			frame.setRedundancyInformation(robot, e1val);
//			motions.add(new LIN(frame).setCartVelocity(100).setBlendingRel(1).setCartAcceleration(100));
//		}
//		MotionBatch motionBatch = new MotionBatch(motions.toArray(new RobotMotion<?>[motions.size()]));
//		IMotionContainer motionContainer = gripper.moveAsync(motionBatch);
//		motionContainer.await();
//		gripper.move(new LIN(RobotController.vectorToFrame(p3, originUpFrame)).setCartVelocity(100));
		
		
		logger.info("Moving to base");
		gripper.move(lin(originUpFrame).setJointVelocityRel(0.2));
		penUp();
		mF.setLEDBlue(true);
	}
}
