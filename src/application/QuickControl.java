package application;

import java.util.ArrayList;
import java.util.List;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.generated.ioAccess.Gripper2FIOGroup;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.task.ITaskLogger;

import application.opcua.Opcua;

public class QuickControl extends RoboticsAPIApplication{

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

	private List<IMotionContainer> queuedMotions;
	
	@Override
	public void initialize() {
		tool.attachTo(robot.getFlange());
		queuedMotions = new ArrayList<IMotionContainer>();
	}
	
	@Override
	public void run() throws Exception {
		
		double[] dest = {0,0,0,0,0,0};
		
		String[] nodesToRead = {
				"Objects/RobotControl/cposx",
				"Objects/RobotControl/cposy",
				"Objects/RobotControl/cposz",
				"Objects/RobotControl/crota",
				"Objects/RobotControl/crotb",
				"Objects/RobotControl/crotc",
		};
		String statusNodePath = "Objects/RobotControl/cstatus";
		String disconnectNodePath = "Objects/RobotControl/cdisconnect";

		logger.info("Connecting");
		Opcua opcua;
		opcua = new Opcua("opc.tcp://172.32.1.191:4840/server");
		for(String path:nodesToRead) {
			opcua.addReadableNode(path);
		}
		opcua.addReadableNode(disconnectNodePath);
		opcua.addWritableNode(disconnectNodePath);
		opcua.addWritableNode(statusNodePath);
		
		boolean stop = false;
		boolean destUpdate = false;

		logger.info("Looping");
		while(!stop) {
			for(int i = 0;i < nodesToRead.length;i++) {
				String path = nodesToRead[i];
				if(!opcua.hasNodeUpdated(path)) continue;
				destUpdate = true;
				double value = opcua.readNode(path, Double.TYPE);
				dest[i] = value;
			}
			if(destUpdate) {
				boolean success = moveToPos(dest);
				if(success) {
					opcua.writeNode(statusNodePath, 1);
				}else {
					opcua.writeNode(statusNodePath, -1);
				}
				destUpdate = false;
			}
			
			if(!opcua.hasNodeUpdated(disconnectNodePath)) continue;
			int value = opcua.readNode(disconnectNodePath, Integer.TYPE);
			if(value == -1) stop = true;
		}

		opcua.writeNode(disconnectNodePath, 0);
		opcua.shutdown();
		logger.info("Disconnected");
		
	}
	
	private boolean moveToPos(double[] pos) {
		Frame frame = new Frame(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
		try {
			queuedMotions.add(tool.moveAsync(BasicMotions.ptp(frame).setBlendingRel(1)));
			if(queuedMotions.size() < 4) {
				queuedMotions.get(0).cancel();
				queuedMotions.remove(0);
			}
		} catch (Exception e) {
			return false;
		}
		return true;
	}

}
