package application.drawerer.path;

import java.util.ArrayList;
import java.util.List;

import com.kuka.nav.geometry.Vector2D;
import com.kuka.roboticsAPI.motionModel.MotionBatch;

public class PathPlan {
	
	private List<MotionBatch> motions = new ArrayList<MotionBatch>();
	private List<Vector2D> startLocs = new ArrayList<Vector2D>();
	
	public PathPlan(List<MotionBatch> motions, List<Vector2D> startLocs) {
		this.motions = motions;
		this.startLocs = startLocs;
	}
	
	public List<MotionBatch> getMotions() {
		return motions;
	}
	public List<Vector2D> getStartLocs() {
		return startLocs;
	}
	
	
	
}
