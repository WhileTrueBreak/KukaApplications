package application.path;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

import com.kuka.math.geometry.Vector3D;
import com.kuka.nav.geometry.Vector2D;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.RobotMotion;

import application.Drawerer;
import application.parser.PathParser;
import application.robotControl.Canvas;
import application.robotControl.RobotController;
import application.utils.Bezier;
import application.utils.MathHelper;

public class PointPath {

	private List<List<Vector2D>> pointPaths;
	private Rectangle2D bounds;
	
	public PointPath(List<List<Vector2D>> pointPaths) {
		this.pointPaths = pointPaths;
		this.updateBounds();
	}

	private void updateBounds() {
		double minx = this.pointPaths.get(0).get(0).getX();
		double miny = this.pointPaths.get(0).get(0).getY();
		double maxx = this.pointPaths.get(0).get(0).getX();
		double maxy = this.pointPaths.get(0).get(0).getY();
		for(List<Vector2D> path: this.pointPaths) {
			for(Vector2D point: path) {
				if(point.getX() > maxx) maxx = point.getX();
				if(point.getY() > maxy) maxy = point.getY();
				if(point.getX() < minx) minx = point.getX();
				if(point.getY() < miny) miny = point.getY();
			}
		}
		this.bounds = new Rectangle2D.Double(minx, miny, maxx-minx, maxy-miny);
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
		this.updateBounds();
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
		this.updateBounds();
	}

	public PathPlan toPathPlan(LBR robot, Frame originFrame, Canvas canvas) {
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
						double blend = MathHelper.qerp(1,0.8,0,MathHelper.clamp(angle/(4*Math.PI/5),0,1))*20;
						pathMotions.get(pathMotions.size()-1).setBlendingCart(blend);
					}
					prevDir = currDir;
				}
				prevPos = currPos;
				
				Frame frame = RobotController.vectorToFrame(currPos, originFrame);
				LBRE1Redundancy e1val = new LBRE1Redundancy();
				e1val.setE1(0);
				frame.setRedundancyInformation(robot, e1val);
				
				pathMotions.add(new LIN(frame).setCartVelocity(100).setCartAcceleration(100));
			}
			MotionBatch motionBatch = new MotionBatch(pathMotions.toArray(new RobotMotion<?>[pathMotions.size()]));
			motions.add(motionBatch);
			startLocs.add(points.get(0));
		}
		return new PathPlan(motions, startLocs);
	}

	
	public Rectangle2D getBounds() {
		return bounds;
	}

	public static PointPath createPointPathsV2(List<String> file, Canvas canvas, double scale) {
		List<Path> paths = PathParser.parsePathV2(file);
		List<List<Vector2D>> pointPaths = new ArrayList<List<Vector2D>>();
		
		for(int n=0;n<paths.size();n++) {
			Path path = paths.get(n);
			List<Vector2D> points = new ArrayList<Vector2D>();
			List<Vector2D> controlPoints = new ArrayList<Vector2D>();
			for(int i = 0;i < path.getPath().size();i++) {
				Vector2D currPos = path.getPath().get(i).getPos();
				if(path.getPath().get(i).isBlend() || controlPoints.isEmpty()) {
					controlPoints.add(currPos);
					continue;
				}
				if(controlPoints.size() == 1) {
					points.add(controlPoints.get(0));
					controlPoints.clear();
					controlPoints.add(currPos);
					continue;
				}
				controlPoints.add(currPos);	
				points.addAll(Bezier.bezierToVector2Ds(controlPoints, (int) Math.ceil(Bezier.approxBezierLength2D(controlPoints, 100)*canvas.getSize()*scale/3)));
				controlPoints.clear();
				controlPoints.add(currPos);
			}
			points.add(path.getPath().get(path.getPath().size()-1).getPos());
			pointPaths.add(points);
		}
		return new PointPath(pointPaths);
	}
	
}
