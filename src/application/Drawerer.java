package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.spl;

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
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.SPL;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.task.ITaskLogger;

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
		
		// Initializes the boing boing
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
		gripper2F1.setSpeed(189);
//		gripper2F1.open();
//		mF.setLEDBlue(true);
//		ThreadUtil.milliSleep(10000);
		mF.setLEDBlue(false);
		gripper2F1.close();
		ThreadUtil.milliSleep(200);
	}

	public static List<List<Vector2D>> getPathsFromString(String pathString){
		String[] pathStrings = pathString.split("\\|");
		List<String[]> coordStrings = new ArrayList<String[]>();
		for(String string:pathStrings) {
			coordStrings.add(string.split("(?<=\\d)-"));
		}
		
		List<List<Vector2D>> paths = new ArrayList<List<Vector2D>>();
		for(String[] e:coordStrings) {
			List<Vector2D> path = new ArrayList<Vector2D>();
			for(String coordString:e) {
				if(coordString.length() == 0) continue;
				String[] c = coordString.split(",");
				Vector2D coord = new Vector2D(Float.parseFloat(c[0]), Float.parseFloat(c[1]));
				path.add(coord);
			}
			paths.add(path);
		}
		return paths;
	}

	private void penUp(){
		gripper.move(linRel(0,0, -20).setJointVelocityRel(0.2));
		logger.info("Moving Pen Up");
	}
	
	@SuppressWarnings("unused")
	private void penDown(){
		gripper.move(linRel(0,0, 20).setMode(springRobot).setJointVelocityRel(0.2));
		logger.info("Moving Pen Down");
	}
	
	private Frame calibrateFrame(Tool grip){
		ForceCondition touch = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"), 10);
		IMotionContainer motion1 = gripper.move(linRel(0, 0, 150, gripper.getFrame("/TCP")).setCartVelocity(20).breakWhen(touch));
		if (motion1.getFiredBreakConditionInfo() == null){
			logger.info("No Collision Detected");
			return null;
		}
		else{
			logger.info("Collision Detected");
			return robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		}

	}

	private Vector3D frameToVector(Frame frame){
		return Vector3D.of(frame.getX(), frame.getY(), frame.getZ());
	}

	private Pair<Vector3D, Vector3D> getCanvasPlane(Vector3D origin, Vector3D up, Vector3D right){
		Vector3D ver = up.subtract(origin).normalize();
		Vector3D hor = right.subtract(origin).normalize();

		return new Pair<Vector3D, Vector3D>(hor, ver);
	}

	private Vector3D canvasToWorld(Vector2D point, Pair<Vector3D, Vector3D> canvas, double size){
		return canvas.getA().multiply(point.getX()*size).add(canvas.getB().multiply(point.getY()*size));
	}
	
	private Frame vectorToFrame(Vector3D vector, Frame baseFrame){
		return new Frame(vector.getX(), vector.getY(), vector.getZ(), baseFrame.getAlphaRad(), baseFrame.getBetaRad(), baseFrame.getGammaRad());
	}

	private Spline framesToSpline(Frame[] frames){
		SPL[] splines = new SPL[frames.length];
		for (int i=0;i<frames.length;i++){
			splines[i] = spl(frames[i]);
		}

		return new Spline(splines);
		// return new Spline((SPL[])Arrays.asList(frames).stream().map(x->spl(x)).collect(Collectors.toList()).toArray());
	}

	private void springyMove(Spline path){
		int vel = 40;
		gripper.move(path.setMode(springRobot).setCartVelocity(vel));
	}
	
	private double maxMove(Vector3D dir) {
		Vector3D normDir = dir.normalize();
		double moveDist = 1000;
		double totalDist = 0;
		Vector3D moveVector = normDir.multiply(moveDist);
		while(true) {
			if(moveDist <= 0) break;
			try {
				moveVector = normDir.multiply(moveDist);
				logger.info(moveVector.toString());
				gripper.move(linRel(moveVector.getY(), moveVector.getZ(), moveVector.getX()).setCartVelocity(100));
				totalDist += moveDist;
			} catch (Exception e) {
				logger.error("Reducing move dist");
				moveDist /= 2;
			}
		}
		return totalDist;
	}
	
	IMotionContainer m1;
	@Override
	public void run() throws Exception {
		// Calibration sequence
		mF.setLEDBlue(true);
		logger.info("Moving to bottom left");
		gripper.move(ptp(getApplicationData().getFrame("/bottom_left")).setJointVelocityRel(0.2));
		logger.info("Calibrating point 1");
		Frame originFrame = calibrateFrame(gripper);
		penUp();
		Frame originUpFrame = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		Vector3D origin = frameToVector(originFrame);
		logger.info(String.format("Origin: %s", origin.toString()));

		logger.info("Moving to bottom left");
		gripper.move(ptp(getApplicationData().getFrame("/bottom_left")).setJointVelocityRel(0.2));
		gripper.move(linRel(0, 40, 0).setJointVelocityRel(0.2));
		logger.info("Calibrating point 2");
		Vector3D up = frameToVector(calibrateFrame(gripper));
		logger.info(String.format("Up: %s", up.toString()));

		logger.info("Moving to bottom left");
		gripper.move(ptp(getApplicationData().getFrame("/bottom_left")).setJointVelocityRel(0.2));
		gripper.move(linRel(-40, 0,0).setJointVelocityRel(0.2));
		logger.info("Calibrating point 3");
		Vector3D right = frameToVector(calibrateFrame(gripper));
		logger.info(String.format("Right: %s", right.toString()));
		

		// get world unit vectors
		Pair<Vector3D,Vector3D> canvas = getCanvasPlane(origin, up, right);
		logger.info(String.format("Canvas X, Y: (%s), (%s)", canvas.getA().toString(), canvas.getB().toString()));

		// check upper right bound
		gripper.move(ptp(getApplicationData().getFrame("/bottom_left")).setJointVelocityRel(0.2));
		Vector3D diag = canvas.getA().add(canvas.getB());
		logger.info("Diagonal vector: " + diag.toString());
		logger.info("Moving to top right");
		double dist = maxMove(diag);
		logger.info(String.format("Found max at top right: %s", diag.toString()));

		// gets top right frame
		Vector3D top_right = frameToVector(robot.getCurrentCartesianPosition(gripper.getFrame("/TCP")));
		double diag_mag = top_right.subtract(origin).length();
		double size = diag_mag/Math.sqrt(2);
		logger.info(String.format("Canvas size from frame: %f", size));
		size = Math.sqrt(dist*dist/2);
		logger.info(String.format("Canvas size from move: %f", size));
		mF.setLEDBlue(false);
		logger.info("Calibration completed.");
		
		logger.info("Reading Path File");
		String resPath = FileReader.findUniqueFolder("res", "..");
		List<String> file = FileReader.readFile(resPath+"/malogo_c.txt");
		if(file == null || file.size() != 1) {
			logger.info("File is invalid");
			return;
		}
		List<List<Vector2D>> paths = getPathsFromString(file.get(0));
		Spline[] splines = new Spline[paths.size()];
		
		logger.info("Creating Spline");
		Vector3D v = Vector3D.of(0,0,20);
		for (int i=0;i<paths.size();i++){
			Frame[] tempFrames = new Frame[paths.get(i).size()];
			for (int j=0;j<paths.get(i).size();j++) {
				logger.info(i + "-" + j + " : " + paths.get(i).get(j).toString());
				Vector3D path3D = canvasToWorld(paths.get(i).get(j), canvas, size).add(origin).add(v);
				tempFrames[j] = vectorToFrame(path3D, originFrame);
				logger.info(i + "-" + j + " : " + path3D.toString());
			}

			splines[i] = framesToSpline(tempFrames);
			// Frame[] frames = (Frame[])Arrays.asList(path).stream().map(x->vectorToFrame(canvasToWorld(x, canvas, origin), originFrame)).collect(Collectors.toList()).toArray();
			// Spline spline = framesToSpline(frames);
		}

		logger.info("Start Drawing");
		// Spline[] splines = (Spline[])Arrays.asList(paths).stream().map(y-> framesToSpline((Frame[])Arrays.asList(y).stream().map(x->vectorToFrame(canvasToWorld(x, canvas, origin), originFrame)).collect(Collectors.toList()).toArray())).collect(Collectors.toList()).toArray();
		ListIterator<Spline> splineIterator = Arrays.asList(splines).listIterator();
		while(splineIterator.hasNext()){
			int index = splineIterator.nextIndex();
			logger.info("Start path "+index);
			gripper.move(lin(originUpFrame).setCartVelocity(100));
			Vector3D first = canvasToWorld(paths.get(index).get(0), canvas, size);
			logger.info("Moving to first frame");
			gripper.move(linRel(first.getY(), first.getZ(), first.getX()).setCartVelocity(100));
			logger.info("Start spline path");
			springyMove(splineIterator.next());
			logger.info("Finished path");
			penUp();
		}

		mF.setLEDBlue(true);
		//		ThreadUtil.milliSleep(120000);
	}
}
