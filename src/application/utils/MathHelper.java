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
	
	public static double bezier(double t, List<Double> points) {
		if(points.size() < 2) return 0;
		if(points.size() == 2) return lerp(points.get(0), points.get(1), t);
		double fir = bezier(t, points.subList(0, points.size()-1));
		double sec = bezier(t, points.subList(1, points.size()));
		return t*(sec-fir)+fir;
	}
	
}
