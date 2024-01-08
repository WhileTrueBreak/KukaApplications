package application;

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
		List<String> file = FileReader.readFile("res/words.txt");
		for(String line:file) {
			logger.info(line);
			TimeUnit.MILLISECONDS.sleep(100);
		}
	}

}
