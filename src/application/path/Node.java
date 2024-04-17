package application.path;

import com.kuka.nav.geometry.Vector2D;

public class Node {
	private Vector2D pos;
	private boolean isBlend;

	public Node(Vector2D pos) {
		this.setPos(pos);
		this.setBlend(false);
	}
	
	public Node(Vector2D pos, boolean isBlend) {
		this.setPos(pos);
		this.setBlend(isBlend);
	}

	public Vector2D getPos() {
		return pos;
	}

	public void setPos(Vector2D pos) {
		this.pos = pos;
	}

	public boolean isBlend() {
		return isBlend;
	}

	public void setBlend(boolean isBlend) {
		this.isBlend = isBlend;
	}
}
