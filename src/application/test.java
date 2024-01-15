package application;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.kuka.math.geometry.Vector3D;

import application.parser.FileReader;
import application.utils.Bezier;

public class test {

	public static void main(String[] args) throws InterruptedException, IOException {
		List<String> file = FileReader.readFile("res/font/B.txt");
		for(String s:file) {
			System.out.println(s);
		}
		
	}

}
