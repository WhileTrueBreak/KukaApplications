package application;

import javax.inject.Inject;
import javax.inject.Named;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.task.ITaskLogger;
import com.kuka.common.ThreadUtil;
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
public class IndirectHandoverFinlay extends RoboticsAPIApplication {
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
	}

	
	@Override
	public void run() {
		
		// CELL COVERS

		robot.move(ptp(getApplicationData().getFrame("/PART_3/p3_transition")).setJointVelocityRel(0.3));
		gripper2F1.setPos(156);
		robot.move(ptp(getApplicationData().getFrame("/PART_3")).setJointVelocityRel(0.3));//frame1
		gripper2F1.close();
		robot.move(ptp(getApplicationData().getFrame("/PART_3/p3_transition")).setJointVelocityRel(0.3));
		robot.move(ptp(getApplicationData().getFrame("/drop_point/drop_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/drop_point")).setJointVelocityRel(0.3));//frame1
		gripper2F1.open();
		

		robot.move(ptp(getApplicationData().getFrame("/PART_4/p4_transition")).setJointVelocityRel(0.3));
		gripper2F1.setPos(156);
		robot.move(ptp(getApplicationData().getFrame("/PART_4")).setJointVelocityRel(0.3));//frame1
		gripper2F1.close();
		robot.move(ptp(getApplicationData().getFrame("/PART_4/p4_transition")).setJointVelocityRel(0.3));
		robot.move(ptp(getApplicationData().getFrame("/drop_point/drop_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/drop_point")).setJointVelocityRel(0.3));//frame1
		gripper2F1.open();
		
		
		
		/*
		// part 1
		robot.move(ptp(getApplicationData().getFrame("/PART_1/p1_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/PART_1")).setJointVelocityRel(0.3));//frame1
		gripper2F1.close();
		robot.move(ptp(getApplicationData().getFrame("/PART_1/p1_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/drop_point/drop_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/drop_point")).setJointVelocityRel(0.3));//frame1
		gripper2F1.open();
		
		// part 2
		robot.move(ptp(getApplicationData().getFrame("/PART_2/p2_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/PART_2")).setJointVelocityRel(0.3));//frame1
		gripper2F1.close();
		robot.move(ptp(getApplicationData().getFrame("/PART_2/p2_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/drop_point/drop_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/drop_point")).setJointVelocityRel(0.3));//frame1
		gripper2F1.open();
		
		robot.move(ptp(getApplicationData().getFrame("/PART_3/p3_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/PART_3")).setJointVelocityRel(0.3));//frame1
		gripper2F1.close();
		robot.move(ptp(getApplicationData().getFrame("/PART_3/p3_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/drop_point/drop_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/drop_point")).setJointVelocityRel(0.3));//frame1
		gripper2F1.open();
		
		robot.move(ptp(getApplicationData().getFrame("/PART_4/p4_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/PART_4")).setJointVelocityRel(0.3));//frame1
		gripper2F1.close();
		robot.move(ptp(getApplicationData().getFrame("/PART_4/p4_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/drop_point/drop_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/drop_point")).setJointVelocityRel(0.3));//frame1
		gripper2F1.open();
		
		robot.move(ptp(getApplicationData().getFrame("/PART_5/p5_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/PART_5")).setJointVelocityRel(0.3));//frame1
		gripper2F1.close();
		robot.move(ptp(getApplicationData().getFrame("/PART_5/p5_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/drop_point/drop_transition")).setJointVelocityRel(0.3));//frame1
		robot.move(ptp(getApplicationData().getFrame("/drop_point")).setJointVelocityRel(0.3));//frame1
		gripper2F1.open();
		*/
		mF.setLEDBlue(true);
		
	}
}