package application;


import javax.inject.Inject;

import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.geometricModel.Tool;
import javax.inject.Inject;
import javax.inject.Named;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.task.ITaskLogger;
import com.kuka.common.ThreadUtil;
//import com.kuka.generated.flexfellow.FlexFellow;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class RobotApplication extends RoboticsAPIApplication {
	@Inject
	private LBR lBR_iiwa_14_R820_1;

	@Inject
	private KmpOmniMove kMR_omniMove_200_1;

	//@Inject
	//private LBR robot;

	@Inject 
	private Gripper2F gripper2F1;

	@Inject
	private MediaFlangeIOGroup mF;

	@Inject
	@Named("RobotiqGripper")
	private Tool gripper;

	@Inject
	private ITaskLogger logger;
	
	@Override
	public void initialize() {
		gripper.attachTo(lBR_iiwa_14_R820_1.getFlange());
		gripper2F1.initalise();
		gripper2F1.setSpeed(189);
		gripper2F1.open();
		//mF.setLEDBlue(true);
		ThreadUtil.milliSleep(200);
		//mF.setLEDBlue(false);
		//mF.setLEDRed(true);
		ThreadUtil.milliSleep(200);
		//mF.setLEDRed(false);
		//mF.setLEDGreen(true);
		ThreadUtil.milliSleep(200);
		//mF.setLEDGreen(false);
	}

	@Override
	public void run() {
		lBR_iiwa_14_R820_1.move(ptpHome());
		// your application execution starts here
		gripper.move(ptp(getApplicationData().getFrame("/P1")).setJointVelocityRel(0.3));
		gripper2F1.close();
		//mF.setLEDGreen(true);
		gripper.move(ptp(getApplicationData().getFrame("/P2")).setJointVelocityRel(0.3));
		gripper2F1.open();
		//mF.setLEDGreen(false);
		gripper.move(ptp(getApplicationData().getFrame("/P1")).setJointVelocityRel(0.3));
		gripper2F1.close();
		//mF.setLEDBlue(true);
		gripper.move(lin(getApplicationData().getFrame("/P2")).setCartVelocity(200));
		gripper.move(linRel(0, 0, 20, World.Current.getRootFrame()).setCartVelocity(5));
		gripper2F1.open();
		//mF.setLEDBlue(false);
		gripper.move(lin(getApplicationData().getFrame("/P1")).setCartVelocity(100));
		gripper2F1.close();
		//mF.setLEDBlue(true);
		ThreadUtil.milliSleep(200);
		//mF.setLEDBlue(false);
		//mF.setLEDRed(true);
		ThreadUtil.milliSleep(200);
		//mF.setLEDRed(false);
		//mF.setLEDGreen(true);
		ThreadUtil.milliSleep(200);
		//mF.setLEDGreen(false);
	}
}