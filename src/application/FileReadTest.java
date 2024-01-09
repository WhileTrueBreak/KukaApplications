package application;

import java.io.File;
import java.util.List;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.task.ITaskLogger;

public class FileReadTest extends RoboticsAPIApplication {
	
	@Inject
	private ITaskLogger logger;
	
	@Override
	public void run() throws Exception {
		File dir = new File(".");
		File[] filesList = dir.listFiles();
		for (File file : filesList) {
			logger.info(file.getName());
		}
//		List<String> file = FileReader.readFile("res/words.txt");
//		for(String line:file) {
//			logger.info(line);
//			TimeUnit.MILLISECONDS.sleep(100);
//		}
	}

}
