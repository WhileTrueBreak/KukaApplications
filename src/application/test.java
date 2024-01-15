package application;

import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.util.List;

import com.kuka.nav.geometry.Vector2D;

import application.parser.FileReader;
import application.parser.PathParser;
import application.path.Node;
import application.path.Path;

public class test {

	public static void main(String[] args) throws InterruptedException, IOException {
		List<String> file = FileReader.readFile("res/font/B.txt");
		List<Path> paths = PathParser.parsePathV2(file);
		
		System.out.println(paths.get(0).getBounds());
	}

}
