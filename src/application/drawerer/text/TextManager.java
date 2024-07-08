package application.drawerer.text;

import java.io.FileNotFoundException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import application.drawerer.parser.FileReader;
import application.drawerer.path.PointPath;
import application.drawerer.robotControl.Canvas;
import application.utils.Handler;

public class TextManager {

	private static Map<Integer, PointPath> charTable = new HashMap<Integer, PointPath>();
	private static String fontPath = null;
	private static double baseScale = 1;
	
	public static void setFontPath(String path) {
		TextManager.fontPath = path;
	}
	
	public static void loadChar(char c, Canvas canvas) {
		if(fontPath == null) {
			Handler.getLogger().info("Font path not loaded");
			System.out.println("Font path not loaded");
			return;
		}
		List<String> file = null;
		try {
			file = FileReader.readFile(TextManager.fontPath+"/"+((int) c)+".txt");
		} catch (FileNotFoundException e) {
			Handler.getLogger().info("Failed to load char: \"" + c + "\"");
			System.out.println("Failed to load char: \"" + c + "\"");
		}
		if(file == null) {
			Handler.getLogger().info("Failed to load char: \"" + c + "\"");
			System.out.println("Failed to load char: \"" + c + "\"");
			return;
		}
		charTable.put((int) c, PointPath.createPointPathsV2(file, canvas, baseScale));
	}
	
	public static PointPath getCharPath(char c) {
		if(!charTable.containsKey((int) c)) return null;
		return charTable.get((int) c).clone();
	}
	
	public static void setBaseScale(double scale) {
		baseScale = scale;
	}
	
}
