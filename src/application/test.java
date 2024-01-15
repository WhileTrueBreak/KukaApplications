package application;

import java.io.IOException;
import java.util.List;

import application.parser.FileReader;
import application.parser.PathParser;
import application.path.Path;

public class test {

	public static void main(String[] args) throws InterruptedException, IOException {
		List<String> file = FileReader.readFile("res/font/0.txt");
		List<Path> paths = PathParser.parsePathV2(file);
		System.out.println(paths.get(0).getBounds());
	}

}
