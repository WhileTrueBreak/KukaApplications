package application.path;

import java.awt.geom.Rectangle2D;
import java.util.List;

public class Path {
	
	private List<Node> path;
	private Rectangle2D bounds;
	
	public Path(List<Node> path, Rectangle2D bounds) {
		this.path = path;
		this.bounds = bounds;
	}

	public List<Node> getPath() {
		return path;
	}

	public Rectangle2D getBounds() {
		return bounds;
	}
	
	public void updateBounds() {
		double minx = this.path.get(0).getPos().getX();
		double miny = this.path.get(0).getPos().getY();
		double maxx = this.path.get(0).getPos().getX();
		double maxy = this.path.get(0).getPos().getY();
		for(Node node:path) {
			if(node.getPos().getX() > maxx) maxx = node.getPos().getX();
			if(node.getPos().getY() > maxy) maxy = node.getPos().getY();
			if(node.getPos().getX() < minx) minx = node.getPos().getX();
			if(node.getPos().getY() < miny) miny = node.getPos().getY();
		}
		this.bounds = new Rectangle2D.Double(minx, miny, maxx-minx, maxy-miny);
	}
	
}
