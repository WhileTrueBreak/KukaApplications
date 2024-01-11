package application;

import java.io.IOException;
import java.util.List;

import application.parser.PathParser;
import application.path.Node;
import application.path.Path;

public class test {

	public static void main(String[] args) throws InterruptedException, IOException {
		List<Path> paths = PathParser.parsePathV2("res/font.txt");
		for(Path path:paths) {
			System.out.println(path.getBounds().toString());
			for(Node node:path.getPath()) {
				System.out.println(node.getPos().toString() + " | blend: " + node.isBlend());
			}
		}
	}

}
