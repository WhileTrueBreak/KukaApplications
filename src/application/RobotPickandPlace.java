//package application;
//
//
//import javax.inject.Inject;
//import javax.inject.Named;
//
//import com.kuka.nav.task.remote.RemoteTaskId;
//import com.kuka.nav.task.remote.TaskRequest;
//import com.kuka.nav.task.remote.TaskRequestContainer;
//import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
//import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
//
//import com.kuka.roboticsAPI.conditionModel.ForceCondition;
//import com.kuka.roboticsAPI.deviceModel.LBR;
//import com.kuka.roboticsAPI.geometricModel.Tool;
//import com.kuka.roboticsAPI.geometricModel.World;
//import com.kuka.task.ITaskLogger;
//import com.kuka.task.ITaskManager;
//import com.kuka.common.ThreadUtil;
//import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
//
///**
// * Implementation of a robot application.
// * <p>
// * The application provides a {@link RoboticsAPITask#initialize()} and a 
// * {@link RoboticsAPITask#run()} method, which will be called successively in 
// * the application lifecycle. The application will terminate automatically after 
// * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
// * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
// * exception is thrown during initialization or run. 
// * <p>
// * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
// * {@link RoboticsAPITask#dispose()} method.</b> 
// * 
// * @see UseRoboticsAPIContext
// * @see #initialize()
// * @see #run()
// * @see #dispose()
// */
//public class RobotPickandPlace extends RoboticsAPIApplication {
//	@Inject
//	private LBR robot;
//
//	@Inject 
//	private Gripper2F gripper2F1;
//
//	@Inject
//	private MediaFlangeIOGroup mF;
//
//	@Inject
//	@Named("RobotiqGripper")
//	private Tool gripper;
//
//	@Inject
//	private ITaskLogger logger;
//	
//	@Inject
//	private ITaskManager taskManager;
//	
//	@Override
//	public void initialize() {
//		gripper.attachTo(robot.getFlange());
//		gripper2F1.initalise();
//		gripper2F1.setSpeed(189);
//		gripper2F1.open();
//		mF.setLEDBlue(true);
//		ThreadUtil.milliSleep(200);
//		mF.setLEDBlue(false);
//		ThreadUtil.milliSleep(200);
//		
//		
//	}
//
//	@Override
//	public void run() {
//		//gripper.move(lin(getApplicationData().getFrame("/P1")).setCartVelocity(499));//frame1
//		gripper.move(linRel(0, 0, 15, World.Current.getRootFrame()).setCartVelocity(50));//going down
//		gripper.move(lin(getApplicationData().getFrame("/P2")).setCartVelocity(499));//frame1
//		gripper.move(lin(getApplicationData().getFrame("/P3")).setCartVelocity(499));//frame1
//		gripper.move(lin(getApplicationData().getFrame("/P2")).setCartVelocity(499));//frame1
//		gripper.move(lin(getApplicationData().getFrame("/P4")).setCartVelocity(499));//frame1
//		gripper.move(lin(getApplicationData().getFrame("/P5")).setCartVelocity(499));//frame1
//		mF.setLEDBlue(true);
//		ThreadUtil.milliSleep(5000);
//		mF.setLEDBlue(false);
//		gripper.move(lin(getApplicationData().getFrame("/P4")).setCartVelocity(499));//frame1
//		gripper.move(lin(getApplicationData().getFrame("/P1")).setCartVelocity(499));//frame1
//		gripper2F1.open();
//
//	}
//	int robot1 =1;
//	//page 95
//	RemoteTaskId rtid = new RemoteTaskId("src.application.RobotPickandPlace");
//	TaskRequest taskRequest = new TaskRequest(rtid);
//	TaskRequestContainer cont = robot1.executeAsync(taskRequest);
//	
//}