package application;

import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
 
import com.kuka.roboticsAPI.capabilities.honk.IHonkCapability;
import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICallbackAction;
import com.kuka.roboticsAPI.conditionModel.ITriggerAction;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;
import com.kuka.roboticsAPI.executionModel.IFiredTriggerInfo;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.task.ITaskLogger;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.*;

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
public class DirectHandoverFinlay extends RoboticsAPIApplication {
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
	
	@Inject
	private SunriseOmniMoveMobilePlatform kmp;

	CartesianImpedanceControlMode springRobot;
	
	@Override
	public void initialize() {
		gripper.attachTo(robot.getFlange());
		gripper2F1.initalise();
		gripper2F1.setSpeed(189);
		gripper2F1.open();
		mF.setLEDBlue(true);
		ThreadUtil.milliSleep(200);
		mF.setLEDBlue(false);
		ThreadUtil.milliSleep(200);
		
		//FORCE CONDITIONS EXAMPLE
		ForceCondition touch10 = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"),10 );
		ForceCondition touch15 = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"),15 );
		//USAGE, will move to next line when triggered
		//LOOK at pipecutting.java for examples on analysing the break condition. 
		//gripper.move(linRel(0, 0, -30, World.Current.getRootFrame()).setCartVelocity(50).breakWhen(touch10)); 
		springRobot = new CartesianImpedanceControlMode();
		
		springRobot.parametrize(CartDOF.X).setStiffness(1000);
		springRobot.parametrize(CartDOF.Y).setStiffness(180);
		springRobot.parametrize(CartDOF.Z).setStiffness(1000);
		springRobot.parametrize(CartDOF.C).setStiffness(300);
		springRobot.parametrize(CartDOF.B).setStiffness(300);
		springRobot.parametrize(CartDOF.A).setStiffness(300);
		springRobot.setReferenceSystem(World.Current.getRootFrame());
		springRobot.parametrize(CartDOF.ALL).setDamping(0.5);
		//USAGE, will move to next line when triggered
		//LOOK at pipecutting.java for examples on analysing the break condition.
		//gripper.move(linRel(0, 0, -30, World.Current.getRootFrame()).setCartVelocity(50).breakWhen(touch10));
	}

	
	@Override
	public void run() {

		
		
		gripper2F1.close();		
		
		get_part(1, 1);
		
		go_to_frame("/handover");	
		
		robot.moveAsync(positionHold(springRobot, -1, TimeUnit.SECONDS));
		
		detect_handover();		
		
		gripper2F1.open();
		
		get_part(2, 1);


		mF.setLEDBlue(true);
		
	}
	
	public static double calc_dist(double x1, double y1, double z1, double x2, double y2, double z2){
		
		double dist = Math.sqrt(Math.pow(x1-x2,2) +Math.pow(y1-y2,2)  + Math.pow(z1-z2,2) );
		
		return dist;
	}
	
	public void detect_handover(){
		Frame Position = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		double x1 = Position.getX();
		double y1 = Position.getY();
		double z1 = Position.getZ();
		double dist = 0;
		

		while (dist < 100) {
			  ThreadUtil.milliSleep(700);
			  Position = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
			  dist = calc_dist(x1, y1, z1, Position.getX(), Position.getY(), Position.getZ());
			}
		
				
	}
	public void get_part(int part, int grip_offset){
		// part: integer 1-4
		// grip offset: how wide the gripper should be picking up parts - important for certain parts, 0 is open
		
		robot.move(ptp(getApplicationData().getFrame("/PART_"+ Integer.toString(part) +"/p" + Integer.toString(part) + "_transition")).setJointVelocityRel(0.3));
		gripper2F1.setPos(grip_offset);
		robot.move(ptp(getApplicationData().getFrame("/PART_" + Integer.toString(part))).setJointVelocityRel(0.3));//frame1
		gripper2F1.close();
		robot.move(ptp(getApplicationData().getFrame("/PART_"+ Integer.toString(part) +"/p" + Integer.toString(part) + "_transition")).setJointVelocityRel(0.3));
		
	}
	
	public void drop_part(){
		// part: integer 1-4
		// grip offset: how wide the gripper should be picking up parts - important for certain parts, 0 is open
		robot.move(ptp(getApplicationData().getFrame("/drop_point/drop_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/drop_point/drop_DROP")).setJointVelocityRel(0.3));//frame1
		gripper2F1.open();
		robot.move(ptp(getApplicationData().getFrame("/drop_point/drop_transition")).setJointVelocityRel(0.3));//frame1
	}
	
	public void get_from_drop(int part, int grip_offset){
		// part: integer 1-4
		// grip offset: how wide the gripper should be picking up parts - important for certain parts, 0 is open
		robot.move(ptp(getApplicationData().getFrame("/drop_point/drop_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/drop_point")).setJointVelocityRel(0.3));//frame1
		gripper2F1.close();
		robot.move(ptp(getApplicationData().getFrame("/drop_point/drop_transition")).setJointVelocityRel(0.3));
		robot.move(ptp(getApplicationData().getFrame("/PART_"+ Integer.toString(part) +"/p" + Integer.toString(part) + "_transition")).setJointVelocityRel(0.3));
		robot.move(ptp(getApplicationData().getFrame("/PART_" + Integer.toString(part))).setJointVelocityRel(0.3));
		gripper2F1.setPos(grip_offset);
		robot.move(ptp(getApplicationData().getFrame("/PART_"+ Integer.toString(part) +"/p" + Integer.toString(part) + "_transition")).setJointVelocityRel(0.3));
	}
	
	public void go_to_frame(String frame){
		// part: integer 1-4
		// grip offset: how wide the gripper should be picking up parts - important for certain parts, 0 is open
		robot.move(ptp(getApplicationData().getFrame(frame)).setJointVelocityRel(0.3));//frame1

	}
	
	
}