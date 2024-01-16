package application.path;

import java.util.ArrayList;
import java.util.List;

import com.kuka.math.geometry.Vector3D;
import com.kuka.nav.geometry.Vector2D;
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.RobotMotion;

import application.Drawerer;
import application.robotControl.Canvas;
import application.robotControl.RobotController;
import application.utils.Handler;
import application.utils.MathHelper;

public class PointPath {

	private List<List<Vector2D>> pointPaths;
	
	public PointPath(List<List<Vector2D>> pointPaths) {
		this.pointPaths = pointPaths;
	}

	public void offsetPaths(double xoff, double yoff) {
		List<List<Vector2D>> newPointPaths = new ArrayList<List<Vector2D>>();
		for(List<Vector2D> path: this.pointPaths) {
			List<Vector2D> newPath = new ArrayList<Vector2D>();
			for(Vector2D point: path) {
				newPath.add(point.add(Vector2D.of(xoff, yoff)));
			}
			newPointPaths.add(newPath);
		}
		this.pointPaths = newPointPaths;
	}
	
	public void scalePaths(double scale) {
		List<List<Vector2D>> newPointPaths = new ArrayList<List<Vector2D>>();
		for(List<Vector2D> path: this.pointPaths) {
			List<Vector2D> newPath = new ArrayList<Vector2D>();
			for(Vector2D point: path) {
				newPath.add(point.multiply(scale));
			}
			newPointPaths.add(newPath);
		}
		this.pointPaths = newPointPaths;
	}

	public List<List<Vector2D>> getPointPaths() {
		return pointPaths;
	}
	
	public PathPlan toPathPlan(Frame originFrame, Canvas canvas) {
		List<MotionBatch> motions = new ArrayList<MotionBatch>();
		List<Vector2D> startLocs = new ArrayList<Vector2D>();
		
		Vector3D v = Vector3D.of(Drawerer.PEN_DOWN_DIST,0,0);
		for(List<Vector2D> points:this.pointPaths) {
			List<RobotMotion<?>> pathMotions = new ArrayList<RobotMotion<?>>();
			Vector3D prevDir = null;
			Vector3D prevPos = null;
			for(Vector2D point: points) {
				Vector3D currPos = canvas.toWorld(point).add(canvas.getOrigin()).add(v);
				if(prevPos != null) {
					Vector3D currDir = currPos.subtract(prevPos);
					if(prevDir != null) {
						double angle = currDir.angleRad(prevDir);
						double blend = MathHelper.qerp(1,0.8,0,MathHelper.clamp(angle/(Math.PI/4),0,1))*20;
						pathMotions.get(pathMotions.size()-1).setBlendingCart(blend);
					}
					prevDir = currDir;
				}
				prevPos = currPos;
				
				Frame frame = RobotController.vectorToFrame(currPos, originFrame);
				LBRE1Redundancy e1val = new LBRE1Redundancy();
				e1val.setE1(0);
				frame.setRedundancyInformation(Handler.getRobot(), e1val);
				
				pathMotions.add(new LIN(frame).setCartVelocity(100).setCartAcceleration(100));
			}
			MotionBatch motionBatch = new MotionBatch(pathMotions.toArray(new RobotMotion<?>[pathMotions.size()]));
			motions.add(motionBatch);
			startLocs.add(points.get(0));
		}
		return new PathPlan(motions, startLocs);
	}
	
}
