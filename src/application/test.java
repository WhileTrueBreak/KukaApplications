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
		List<String> file = FileReader.readFile("res/font/0.txt");
		List<Path> paths = PathParser.parsePathV2(file);
		
		for(int n=0;n<paths.size();n++) {
			Path path = paths.get(n);
			path.updateBounds();
		}
		
		Rectangle2D bound = paths.get(0).getBounds();
		
		double maxDim = Math.max(bound.getWidth(), bound.getHeight());
		Vector2D offset = Vector2D.of(-bound.getX(), -bound.getY());

		System.out.println(bound);
		System.out.println(offset);
		
		for(int n=0;n<paths.size();n++) {
			Path path = paths.get(n);
			for(int i = 0;i < path.getPath().size();i++) {
				Node node = path.getPath().get(i);
				node.setPos(node.getPos().add(offset));
				node.setPos(node.getPos().multiply(1/maxDim));
			}
			path.updateBounds();
		}
		
		System.out.println(paths.get(0).getBounds());
	}

}
