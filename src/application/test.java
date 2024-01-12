package application;

import java.io.IOException;
import java.util.List;

import com.kuka.math.geometry.Vector3D;
import com.kuka.nav.geometry.Vector2D;

import application.parser.FileReader;
import application.parser.PathParser;
import application.path.PathPlan;
import application.utils.MathHelper;

public class test {

	public static void main(String[] args) throws InterruptedException, IOException {
		int count = 0;
		List<String> file = FileReader.readFile("res/frieren_c.txt");
		List<List<Vector2D>> paths = PathParser.parsePathV1(file.get(0));
		for(List<Vector2D> path:paths) {
			Vector2D prevDir = null;
			Vector2D prevPos = null;
			for(Vector2D currPos:path) {
				if(prevPos != null) {
					Vector2D currDir = currPos.subtract(prevPos);
					if(currDir.length() == 0) {
						System.out.println(++count);
						continue;
					}
					if(prevDir != null) {
						double angle = currDir.angleRad(prevDir);
					}
					prevDir = currDir;
				}
				prevPos = currPos;
			}
		}
	}

}
