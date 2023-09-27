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
import com.kuka.roboticsAPI.motionModel.IMotionContainer;

 

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
public class RobotPickandPlace extends RoboticsAPIApplication {
	int breakSpeed = 20;
	int speed = 50;
	int upSpeed = 100;
	int moveSpeed = 200;


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

		ForceCondition touch15 = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"),15 );

		//USAGE, will move to next line when triggered
		//LOOK at pipecutting.java for examples on analysing the break condition. 
		//gripper.move(linRel(0, 0, -30, World.Current.getRootFrame()).setCartVelocity(50).breakWhen(touch10)); 
	}

	public void pickup(int index,int z){
		ForceCondition touch10 = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"),15 );
		
		gripper.move(lin(getApplicationData().getFrame("/P1")).setCartVelocity(moveSpeed));//frame1

		
		
		gripper2F1.close();
		gripper.move(linRel(0, z, 0, World.Current.getRootFrame()).setCartVelocity(speed));
		IMotionContainer motion_return = gripper.move(linRel(0, 0, -90, World.Current.getRootFrame()).setCartVelocity(breakSpeed).breakWhen(touch10));//going down
		if (motion_return.getFiredBreakConditionInfo() == null){
			logger.info("No Collision Detected");
		}
		else{
			logger.info("Collision Detected");
			logger.info(motion_return.getFiredBreakConditionInfo().toString());
		}
		
		gripper.move(linRel(0, 0, 5, World.Current.getRootFrame()).setCartVelocity(speed));
		gripper2F1.open();
		gripper.move(linRel(0, 0, -25, World.Current.getRootFrame()).setCartVelocity(speed));
		gripper2F1.close();
		mF.setLEDBlue(true);
		gripper.move(lin(getApplicationData().getFrame("/P1")).setCartVelocity(upSpeed));//get back to frame1
		gripper.move(lin(getApplicationData().getFrame("/P2")).setCartVelocity(moveSpeed));// go to frame2
	    gripper.move(linRel(0, 0, -90, World.Current.getRootFrame()).setCartVelocity(breakSpeed).breakWhen(touch10));// going down
		gripper2F1.open();
		mF.setLEDBlue(false);
	    gripper.move(lin(getApplicationData().getFrame("/P2")).setCartVelocity(upSpeed));
	}

 

	@Override
	public void run() {
		int z;
		z = 0;
		for (int i = 0; i < 3; i++) {
			  pickup(i,z);
			  z = z - 60;
		}

	}
}