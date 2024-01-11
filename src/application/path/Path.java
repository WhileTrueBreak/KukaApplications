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
	
}
