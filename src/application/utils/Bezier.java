package application.utils;

import java.util.ArrayList;
import java.util.List;

import com.kuka.math.geometry.Vector3D;
import com.kuka.nav.geometry.Vector2D;

public class Bezier {

	public static List<Vector2D> bezierToVector2Ds(List<Vector2D> controlPoints, int resolution){
		List<Vector2D> points = new ArrayList<Vector2D>();
		List<Double> xs = new ArrayList<Double>();
		List<Double> ys = new ArrayList<Double>();
		for(Vector2D p:controlPoints) {
			xs.add(p.getX());
			ys.add(p.getY());
		}
		for(int i = 0;i < resolution;i++) {
			Double t = (double)i/(double)resolution;
			Vector2D tmp = Vector2D.of(
					MathHelper.bezier(xs, t), 
					MathHelper.bezier(ys, t));
			points.add(tmp);
		}
		return points;
	}

	public static List<Vector3D> bezierToVector3Ds(List<Vector3D> controlPoints, int resolution){
		List<Vector3D> points = new ArrayList<Vector3D>();
		List<Double> xs = new ArrayList<Double>();
		List<Double> ys = new ArrayList<Double>();
		List<Double> zs = new ArrayList<Double>();
		for(Vector3D p:controlPoints) {
			xs.add(p.getX());
			ys.add(p.getY());
			zs.add(p.getZ());
		}
		for(int i = 0;i < resolution;i++) {
			Double t = (double)i/(double)resolution;
			Vector3D tmp = Vector3D.of(
					MathHelper.bezier(xs, t), 
					MathHelper.bezier(ys, t), 
					MathHelper.bezier(zs, t));
			points.add(tmp);
		}
		return points;
	}
	
	public static double approxBezierLength2D(List<Vector2D> controlPoints, int resolution) {
		List<Vector2D> points = Bezier.bezierToVector2Ds(controlPoints, resolution);
		points.add(controlPoints.get(controlPoints.size()-1));
		double length = 0;
		Vector2D prev = points.get(0);
		for(int i = 1;i < points.size();i++) {
			Vector2D curr = points.get(i);
			length += curr.subtract(prev).length();
			prev = curr;
		}
		return length;
	}
	
	public static double approxBezierLength3D(List<Vector3D> controlPoints, int resolution) {
		List<Vector3D> points = Bezier.bezierToVector3Ds(controlPoints, resolution);
		points.add(controlPoints.get(controlPoints.size()-1));
		double length = 0;
		Vector3D prev = points.get(0);
		for(int i = 1;i < points.size();i++) {
			Vector3D curr = points.get(i);
			length += curr.subtract(prev).length();
			prev = curr;
		}
		return length;
	}
	
}
