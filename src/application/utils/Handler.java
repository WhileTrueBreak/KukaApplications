package application.utils;

import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.task.ITaskLogger;

import application.Gripper2F;

public class Handler {

	private static LBR robot = null;
	private static Gripper2F gripper = null;
	private static MediaFlangeIOGroup mediaFlangeIO = null;
	private static Tool tool = null;
	private static ITaskLogger logger = null;
	private static Controller controller = null;
	
	public static void setLogger(ITaskLogger logger) {
		if(Handler.logger == null) Handler.logger = logger;
	}
	
	public static ITaskLogger getLogger() {
		return Handler.logger;
	}

//	public static Tool getTool() {
//		return tool;
//	}

	public static void setTool(Tool tool) {
		if(Handler.tool == null) Handler.tool = tool;
	}

//	public static LBR getRobot() {
//		return robot;
//	}

	public static void setRobot(LBR robot) {
		if(Handler.robot == null) Handler.robot = robot;
	}

//	public static Gripper2F getGripper() {
//		return gripper;
//	}

	public static void setGripper(Gripper2F gripper) {
		if(Handler.gripper == null) Handler.gripper = gripper;
	}

//	public static MediaFlangeIOGroup getMediaFlangeIO() {
//		return mediaFlangeIO;
//	}

	public static void setMediaFlangeIO(MediaFlangeIOGroup mediaFlange) {
		if(Handler.mediaFlangeIO == null) Handler.mediaFlangeIO = mediaFlange;
	}

//	public static Controller getController() {
//		return controller;
//	}

	public static void setController(Controller controller) {
		if(Handler.controller == null) Handler.controller = controller;
	}
}
