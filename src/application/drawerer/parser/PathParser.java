package application.drawerer.parser;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

import com.kuka.nav.geometry.Vector2D;

import application.drawerer.path.Node;
import application.drawerer.path.Path;
import application.utils.MathHelper;

public class PathParser {
	
	public static List<List<Vector2D>> parsePathV1(String pathString, double minDist) {
		String[] pathStrings = pathString.split("\\|");
		List<String[]> coordStrings = new ArrayList<String[]>();
		for(String string:pathStrings) {
			coordStrings.add(string.split("(?<=\\d)-"));
		}
		
		List<List<Vector2D>> paths = new ArrayList<List<Vector2D>>();
		for(String[] e:coordStrings) {
			List<Vector2D> path = new ArrayList<Vector2D>();
			Vector2D lastCoord = null;
			for(String coordString:e) {
				if(coordString.length() == 0) continue;
				String[] c = coordString.split(",");
				Vector2D coord = new Vector2D(MathHelper.clamp(Double.parseDouble(c[0]),0,1), MathHelper.clamp(Double.parseDouble(c[1]),0,1));
				if(lastCoord == null) {
					path.add(coord);
					lastCoord = coord;
				}
				double dist = coord.subtract(lastCoord).length();
				if(dist == 0) continue;

				path.add(coord);
				lastCoord = coord;
			}
			if(path.get(0).subtract(path.get(path.size()-1)).length() >= minDist) path.add(path.get(0));
			paths.add(path);
		}
		return paths;
	}
	
	public static List<Path> parsePathV2(List<String> file) {
		if(file == null) return null;
		Rectangle2D bounds = parseBounds(file.get(0));
		List<Path>paths = new ArrayList<Path>();
		for(int i = 1;i < file.size();i++) {
			List<Node> nodes = new ArrayList<Node>();
			String[] coordStrings = file.get(i).split("=");
			for(String coordString:coordStrings) {
				boolean isBlend = coordString.startsWith("B");
				if(isBlend) coordString = coordString.substring(1);
				String[] c = coordString.split(",");
				Vector2D coord = new Vector2D(Double.parseDouble(c[0]), Double.parseDouble(c[1]));
				nodes.add(new Node(coord, isBlend));
			}
			paths.add(new Path(nodes, bounds));
		}
		return paths;
		
	}
	
	public static Rectangle2D parseBounds(String boundString) {
		String[] v = boundString.split(",");
		double minx = Double.parseDouble(v[0]);
		double miny = Double.parseDouble(v[1]);
		double maxx = Double.parseDouble(v[2]);
		double maxy = Double.parseDouble(v[3]);
		return new Rectangle2D.Double(minx, miny, maxx-minx, maxy-miny);
	}
	
}
