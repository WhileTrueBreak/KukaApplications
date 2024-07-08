package application;

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
		String statusNodePath = "Objects/RobotControl/cstatus";

		Opcua opcua;
		opcua = new Opcua("opc.tcp://172.32.1.191:4840/server");
		for(String path:nodesToRead) {
			opcua.addReadableNode(path);
		}
		opcua.addReadableNode(statusNodePath);
		opcua.addWritableNode(statusNodePath);
		
		boolean stop = false;
		
		while(!stop) {
			for(String path:nodesToRead) {
				if(!opcua.hasNodeUpdated(path)) continue;
				double value = opcua.readNode(path, Double.TYPE);
				logger.info(path+": "+value);
			}
			if(!opcua.hasNodeUpdated(statusNodePath)) continue;
			int value = opcua.readNode(statusNodePath, Integer.TYPE);
			if(value == -1) stop = true;
		}

		opcua.writeNode(statusNodePath, 0);
		opcua.shutdown();
		logger.info("Disconnected");
		
	}

}
