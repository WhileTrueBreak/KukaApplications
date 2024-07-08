package application.drawerer.robotControl;

import com.kuka.common.Pair;
import com.kuka.math.geometry.Vector3D;
import com.kuka.nav.geometry.Vector2D;

public class Canvas {
	
	public static Pair<Vector3D, Vector3D> getCanvasPlane(Vector3D origin, Vector3D up, Vector3D right){
		Vector3D ver = up.subtract(origin).normalize();
		Vector3D hor = right.subtract(origin).normalize();
		hor = ver.crossProduct(hor.crossProduct(ver)).normalize();
		
		return new Pair<Vector3D, Vector3D>(hor, ver);
	}
	
	public static Vector3D canvasToWorld(Vector2D point, Canvas canvas){
		return canvas.getPlane().getA().multiply(point.getX()*canvas.getSize()).add(canvas.getPlane().getB().multiply(point.getY()*canvas.getSize()));
	}
	
	private Vector3D origin;
	private Pair<Vector3D, Vector3D> plane;
	private double size;
	
	public Canvas(Vector3D origin, Pair<Vector3D, Vector3D> plane, double size) {
		this.origin = origin;
		this.plane = plane;
		this.size = size;
	}
	
	public Vector3D toWorld(Vector2D point){
		return this.plane.getA().multiply(point.getX()*this.size).add(this.plane.getB().multiply(point.getY()*this.size));
	}

	public Vector3D getOrigin() {
		return origin;
	}

	public Pair<Vector3D, Vector3D> getPlane() {
		return plane;
	}

	public double getSize() {
		return size;
	}
	
}
