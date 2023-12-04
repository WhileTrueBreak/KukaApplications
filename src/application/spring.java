package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import java.util.concurrent.TimeUnit;
import javax.inject.Inject;
import javax.inject.Named;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.task.ITaskLogger;

public class spring extends RoboticsAPIApplication {
	@Inject
	private LBR robot; 

	@Inject
	private MediaFlangeIOGroup mF;
	
//	@Inject 
//	private Gripper2F gripper2F1;
	
	@Inject
	@Named("RobotiqGripper")
	private Tool gripper;
	
	@Inject
	private ITaskLogger logger;		//Printing in non background tasks
	
	
	@Inject
	private motions Motions;
	
	CartesianImpedanceControlMode springRobot;
	IMotionContainer m1;
	
	@Override
	public void initialize() {
		springRobot = new CartesianImpedanceControlMode(); 
		
		
		springRobot.parametrize(CartDOF.X).setStiffness(50);
		springRobot.parametrize(CartDOF.Y).setStiffness(50);
		springRobot.parametrize(CartDOF.Z).setStiffness(50);
		springRobot.parametrize(CartDOF.C).setStiffness(50);
		springRobot.parametrize(CartDOF.B).setStiffness(50);
		springRobot.parametrize(CartDOF.A).setStiffness(50);
		springRobot.setReferenceSystem(World.Current.getRootFrame());
		springRobot.parametrize(CartDOF.ALL).setDamping(0.1);
		
		
//		try {
//			OPCUA.setGripperControl(false);
//		} catch (Exception e1) {
//			// TODO Auto-generated catch block
//			e1.printStackTrace();
//		}
		gripper.attachTo(robot.getFlange()); //Gripper attached to flange, move robot by moving gripper!
//		mF.setLEDRed(true);
//		mF.setLEDGreen(true);
		mF.setLEDBlue(true);
		ThreadUtil.milliSleep(1000);
//		mF.setLEDRed(false);
//		mF.setLEDGreen(false);
		mF.setLEDBlue(false);
		logger.info("Initalising Gripper...");
//		gripper2F1.initalise();
//		gripper2F1.setSpeed(255);
		ThreadUtil.milliSleep(100);
		mF.setLEDBlue(true);
		//gripper.move(ptp(getApplicationData().getFrame("/DrivePos")).setJointVelocityRel(0.4));
		
	}

	@Override
	public void run(){
//					gripper2F1.close();
				//gripper.move(ptp(0.0,-0.785398,0.0,1.13446,0.0,0.436332,-1.5708).setJointVelocityRel(0.2).setMode(springRobot));

				gripper.moveAsync(positionHold(springRobot, -1, TimeUnit.SECONDS));
				logger.info("Communication Signals Reset");// End If (Start)

				ThreadUtil.milliSleep(10000);
	}//END RUN
}


