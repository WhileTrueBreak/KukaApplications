package application.robotControl;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.spl;

import com.kuka.math.geometry.Vector3D;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.RobotMotion;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.SplineMotionCP;
import com.kuka.roboticsAPI.motionModel.SplineOrientationType;

import application.path.Node;
import application.path.Path;
import application.utils.Handler;

public class RobotController {
	
	public static Frame calibrateFrame(Tool tool, int distance){
		ForceCondition touch10 = ForceCondition.createSpatialForceCondition(tool.getFrame("/TCP"), 10);
		IMotionContainer motion1 = tool.move(linRel(0, 0, distance, tool.getFrame("/TCP")).setCartVelocity(10).breakWhen(touch10));
		if (motion1.getFiredBreakConditionInfo() == null){
			Handler.getLogger().info("No Collision Detected");
			return null;
		}
		else{
			Handler.getLogger().info("Collision Detected");
			return Handler.getRobot().getCurrentCartesianPosition(tool.getFrame("/TCP"));
		}

	}

	public static Vector3D frameToVector(Frame frame){
		return Vector3D.of(frame.getX(), frame.getY(), frame.getZ());
	}

	public static Frame vectorToFrame(Vector3D vector, Frame rotFrame){
		return new Frame(vector.getX(), vector.getY(), vector.getZ(), rotFrame.getAlphaRad(), rotFrame.getBetaRad(), rotFrame.getGammaRad());
	}

	public static Spline framesToSpline(Frame[] frames){
		SplineMotionCP<?>[] motions = new SplineMotionCP[frames.length];
		Vector3D lastPos = null;
		for (int i=0;i<frames.length;i++){
			if(lastPos == null) {
				motions[i] = lin(frames[i]);
				continue;
			}
			Vector3D pos = frameToVector(frames[i]);
			double dist = pos.subtract(lastPos).length();
			motions[i] = dist < 10 ? spl(frames[i]) : lin(frames[i]);
			if(i != 0 || i != frames.length-1) motions[i].setOrientationType(SplineOrientationType.Ignore);
			lastPos = pos;
		}

		return new Spline(motions);
		// return new Spline((SPL[])Arrays.asList(frames).stream().map(x->spl(x)).collect(Collectors.toList()).toArray());
	}
	
	public static double maxMove(Vector3D dir) {
		Vector3D normDir = dir.normalize();
		double moveThresh = 1;
		double moveDist = 1000;
		double totalDist = 0;
		Vector3D moveVector = normDir.multiply(moveDist);
		while(true) {
			if(moveDist <= moveThresh) break;
			try {
				moveVector = normDir.multiply(moveDist);
				Handler.getTool().move(linRel(moveVector.getY(), moveVector.getZ(), moveVector.getX()).setCartVelocity(100));
				totalDist += moveDist;
			} catch (Exception e) {
				moveDist /= 2;
			}
		}
		Handler.getLogger().info("Moved: " + totalDist + "mm");
		return totalDist;
	}
	
	public static void safeMove(RobotMotion<?> motion) throws Exception {
		ForceCondition touch15 = ForceCondition.createSpatialForceCondition(Handler.getTool().getFrame("/TCP"), 15);
		IMotionContainer motionContainer = Handler.getTool().move(motion.breakWhen(touch15));
		if(motionContainer.getFiredBreakConditionInfo() != null) {
			Handler.getLogger().error("Touched something on safe move");
			Handler.getLogger().error(motionContainer.getFiredBreakConditionInfo().toString());
			Handler.getLogger().error(motionContainer.getErrorMessage());

			throw new Exception("Safe move tiggered");
		}
	}
	
	public static Spline pathToSpline(Path path, Canvas canvas, Frame rotFrame) {
		Handler.getLogger().info("Creating spline");
		SplineMotionCP<?>[] motions = new SplineMotionCP[path.getPath().size()];
		for(int i = 0;i < path.getPath().size();i++) {
			Node node = path.getPath().get(i);
			Frame nFrame = vectorToFrame(canvas.toWorld(node.getPos()), rotFrame);
			motions[i] = lin(nFrame);
			if(node.isBlend()) motions[i].setBlendingRel(1);
			if(i != 0 || i != path.getPath().size()-1) motions[i].setOrientationType(SplineOrientationType.Ignore);
		}
		return new Spline(motions);
	}
	
}
