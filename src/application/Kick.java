package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.ArrayList;
import java.util.List;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.Pair;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.math.geometry.Matrix;
import com.kuka.math.geometry.Vector3D;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.RobotMotion;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.task.ITaskLogger;

import application.parser.FileReader;
import application.path.PathPlan;
import application.path.PointPath;
import application.robotControl.Canvas;
import application.robotControl.RobotController;
import application.text.TextManager;
import application.utils.Handler;

/*
 * Drawing random stuff mirrored
 */

public class Kick extends RoboticsAPIApplication{
	@Inject
	private LBR robot;
	@Inject 
	private Gripper2F gripper2F1;
	@Inject
	private MediaFlangeIOGroup mF;
	@Inject
	@Named("RobotiqGripper")
	private Tool gripper;
	@Inject
	private ITaskLogger logger;
	
	private CartesianImpedanceControlMode springRobot;
	
	@Override
	public void initialize() {
		
		Handler.setRobot(robot);
		Handler.setGripper(gripper2F1);
		Handler.setMediaFlangeIO(mF);
		Handler.setTool(gripper);
		Handler.setLogger(logger);
		
		// Inits the Robot
		gripper.attachTo(robot.getFlange());
		gripper2F1.initalise();
		gripper2F1.setSpeed(189);
//		gripper2F1.open();
//		mF.setLEDBlue(true);
//		ThreadUtil.milliSleep(10000);
		mF.setLEDBlue(false);
		gripper2F1.close();
		ThreadUtil.milliSleep(200);
	}
	
	@Override
	public void run() throws Exception {
		// Calibration sequence
		while(true){
			mF.setLEDBlue(true);
			gripper.move(lin(getApplicationData().getFrame("/P10")));
			gripper.move(lin(getApplicationData().getFrame("/P8")));
			mF.setLEDBlue(false);
		}
	}
}
