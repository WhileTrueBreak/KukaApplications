package application;

import java.util.Set;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.generated.ioAccess.Gripper2FIOGroup;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.task.ITaskLogger;

import application.opcua.Opcua;

public class test extends RoboticsAPIApplication{

	@Inject
	private LBR robot;
	@Inject
	private Gripper2FIOGroup gripperIO;
	@Inject
	private MediaFlangeIOGroup mF;
	@Inject
	@Named("RobotiqGripper")
	private Tool tool;
	@Inject
	private ITaskLogger logger;
	
	@Override
	public void initialize() {
		
	}
	
	@Override
	public void run() throws Exception {
		
		String[] nodesToRead = {
				"Objects/RobotControl/cposx",
				"Objects/RobotControl/cposy",
				"Objects/RobotControl/cposz",
		};
		Opcua opcua;
		opcua = new Opcua("opc.tcp://172.32.1.191:4840/server");
		for(String path:nodesToRead) {
			opcua.addReadableNode(path);
		}
		for(int i = 0;i < 100;i++) {
			for(String path:nodesToRead) {
				if(!opcua.hasNodeUpdated(path)) continue;
				double value = opcua.readNode(path, Double.TYPE);
				logger.info(path+": "+value);
			}
		}
		
		opcua.disconnect();
		logger.info("Disconnected");
		
		
	}

}
