package application.utils;

import java.util.List;

public class MathHelper {
	
	public static double clamp(double x, double min, double max) {
		return Math.max(Math.min(x, max), min);
	}
	
	public static double lerp(double a, double b, double t) {
		return t*(b-a)+a;
	}
	
	public static double qerp(double a, double b, double c, double t) {
		double fir = lerp(a, b, t);
		double sec = lerp(b, c, t);
		return t*(sec-fir)+fir;
	}
	
	public static double bezier(List<Double> points, double t) {
		if(points.size() < 1) return 0;
		if(points.size() == 1) return points.get(0);
		if(points.size() == 2) return lerp(points.get(0), points.get(1), t);
		double fir = bezier(points.subList(0, points.size()-1), t);
		double sec = bezier(points.subList(1, points.size()), t);
		return t*(sec-fir)+fir;
	}
	
}
