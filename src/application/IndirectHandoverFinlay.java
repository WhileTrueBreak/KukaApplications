package application;

import javax.inject.Inject;
import javax.inject.Named;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
 
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
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

		
		gripper2F1.close();
		Frame Position = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		double x1 = Position.getX();
		double y1 = Position.getY();
		double z1 = Position.getZ();
		double dist = 0;
		

		while (dist < 100) {
			  dist = calc_dist(x1, y1, z1, Position.getX(), Position.getY(), Position.getZ() );
			}
		gripper2F1.open();


		mF.setLEDBlue(true);
		
	}
	
	public static double calc_dist(double x1, double y1, double z1, double x2, double y2, double z2){
		
		double dist = Math.sqrt(Math.pow(x1-x2,2) +Math.pow(y1-y2,2)  + Math.pow(z1-z2,2) );
		
		return dist;
	}
	
	
}