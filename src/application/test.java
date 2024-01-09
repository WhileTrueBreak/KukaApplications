package application;

import java.io.File;
import java.io.IOException;

public class test {

	public static void main(String[] args) throws InterruptedException, IOException {
		String path = FileReader.findUniqueFolder("res");
		System.out.println(path);
	}

}
