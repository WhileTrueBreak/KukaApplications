package application;

import java.io.File;
import java.io.FileNotFoundException;

public class test {

	public static void main(String[] args) throws InterruptedException, FileNotFoundException {
		File dir = new File(".");
		File[] filesList = dir.listFiles();
		for (File file : filesList) {
			System.out.println(file.getName());
		}
	}

}
