package application;

import java.util.concurrent.*; 
import java.util.*; 

import javax.inject.Inject;
import javax.inject.Named;

//import com.kuka.math.geometry.Vector3D;
import com.kuka.nav.geometry.Vector2D;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
 
import com.kuka.roboticsAPI.capabilities.honk.IHonkCapability;
import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICallbackAction;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.ITriggerAction;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;
import com.kuka.roboticsAPI.executionModel.IFiredTriggerInfo;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.task.ITaskLogger;
import com.kuka.common.Pair;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.vividsolutions.jts.math.Vector3D;

import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
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
public class timed {
	@Inject
	private LBR robot;
 
	@Inject
	@Named("RobotiqGripper")
	private Tool gripper;
 
	@Inject
	private ITaskLogger logger;
	
	public ArrayList<Double> velocity = new ArrayList<Double>();
	public ArrayList<Double> acceleration = new ArrayList<Double>();
	public ArrayList<Frame> pose = new ArrayList<Frame>();
	public double vel ;
	public double acc;
	public ScheduledFuture<?> beeperHandle;
	
	public void initialize() {
		gripper.attachTo(robot.getFlange());
	}
	
	private Pair<ArrayList<Double>  , ArrayList<Double>> distance(Frame pose, ArrayList<Double> velocity, ArrayList<Double> acceleration) {
	    Frame newPosition = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
	    Vector3D distance = new Vector3D((pose.getX() - newPosition.getX()), (pose.getY() - newPosition.getY()), (pose.getZ() - newPosition.getZ()));
	    double currentVel = distance.length() * 100;  
	    velocity.add(currentVel);
	    double accel = 0.0;
	    if (velocity.size() > 9) {
	    	for (int i = 1; i<10;i++){
	    		accel -= velocity.get(velocity.size()-i);
	    	}
	    	acceleration.add(accel);
	    } else {
	    	acceleration.add(accel);
	    }
	    
	    return new Pair<ArrayList<Double> ,ArrayList<Double>>(velocity,acceleration);
	}
	
	public void run() {
		logger.info("velocity function ready to go ! calculating the velocity");
		pose.add(robot.getCurrentCartesianPosition(gripper.getFrame("/TCP")));
		ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);
		Runnable beeper = new Runnable() {
	        public void run() {
	        	Pair<ArrayList<Double> ,ArrayList<Double> > velAcc = distance(pose.get(pose.size()-1), velocity, acceleration);
	        	velocity = velAcc.getA();
	        	vel = velocity.get(velocity.size()-1);
	        	acceleration = velAcc.getB();
	        	acc = acceleration.get(acceleration.size()-1);
	        	
	    		logger.info("cartesian velocity : "+ vel);
	    		logger.info("cartesian acceleration : "+ acc);
	    		pose.add(robot.getCurrentCartesianPosition(gripper.getFrame("/TCP")));
	        }
		};
		beeperHandle = scheduler.scheduleAtFixedRate(beeper, 10, 10, TimeUnit.MILLISECONDS);
	}
	
	public void stop(){
		beeperHandle.cancel(true);
	}
	
}

