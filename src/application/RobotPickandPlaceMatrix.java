package application;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.circ;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.spl;
import java.util.Arrays;
import java.util.List;
import java.util.ListIterator;
import java.util.concurrent.TimeUnit;
import javax.inject.Inject;
import javax.inject.Named;
import com.kuka.common.Pair;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.math.geometry.Vector3D;
import com.kuka.nav.geometry.Vector2D;
import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ForceComponentCondition;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.kmp.SunriseOmniMoveMobilePlatform;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.CartPlane;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ITransformationProvider;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.geometricModel.math.ITransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.OrientationReferenceSystem;
import com.kuka.roboticsAPI.motionModel.SPL;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.SplineOrientationType;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.persistenceModel.IPersistenceEngine;
import com.kuka.roboticsAPI.persistenceModel.XmlApplicationDataSource;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.task.ITaskLogger;

 
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
public class RobotPickandPlaceMatrix extends RoboticsAPIApplication {
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
	CartesianImpedanceControlMode springRobot2;
	@Override
	public void initialize() {
		gripper.attachTo(robot.getFlange());
		gripper2F1.initalise();
		gripper2F1.setSpeed(130);
		gripper2F1.open();
		mF.setLEDBlue(true);
		ThreadUtil.milliSleep(200);
		mF.setLEDBlue(false);
		ThreadUtil.milliSleep(200);
		
		springRobot = new CartesianImpedanceControlMode(); 
		// Set stiffness
		// TODO: Stiff in every direction except plane perpendicular to flange
		springRobot.parametrize(CartDOF.X).setStiffness(800);
		springRobot.parametrize(CartDOF.Y).setStiffness(800);
		springRobot.parametrize(CartDOF.Z).setStiffness(2500);
		// Stiff rotation
		springRobot.parametrize(CartDOF.C).setStiffness(100);
		springRobot.parametrize(CartDOF.B).setStiffness(100);
		springRobot.parametrize(CartDOF.A).setStiffness(100);
		springRobot.setReferenceSystem(World.Current.getRootFrame());
		springRobot.parametrize(CartDOF.ALL).setDamping(1);
		//FORCE CONDITIONS EXAMPLE
		//USAGE, will move to next line when triggered
		//LOOK at pipecutting.java for examples on analysing the break condition. 
		//gripper.move(linRel(0, 0, -30, World.Current.getRootFrame()).setCartVelocity(50).breakWhen(touch10)); 
		
	}
	
	public Frame calibrate() {
		springRobot2 = CartesianSineImpedanceControlMode.createSpiralPattern(CartPlane.XY, 1, 20, 400, 10);
		springRobot2.parametrize(CartDOF.Z).setStiffness(1500);
		// Stiff rotation
		springRobot2.parametrize(CartDOF.C).setStiffness(100);
		springRobot2.parametrize(CartDOF.B).setStiffness(200);
		springRobot2.parametrize(CartDOF.A).setStiffness(200);
		springRobot2.setReferenceSystem(World.Current.getRootFrame());
		springRobot2.parametrize(CartDOF.ALL).setDamping(1);
		
		ForceComponentCondition calibrate_inverted =  new ForceComponentCondition( gripper.getFrame("/TCP"),World.Current.getRootFrame(), CoordinateAxis.Z,15.0,25.0);
		ICondition calibrateForce =  calibrate_inverted.invert();
		gripper.moveAsync(positionHold(springRobot2, 20, TimeUnit.SECONDS).breakWhen(calibrateForce));
		IMotionContainer motion1 = gripper.move(linRel(0,0,-150, World.Current.getRootFrame()).setCartVelocity(30).breakWhen(calibrateForce));
		if (motion1.getFiredBreakConditionInfo() == null){
			logger.info("No Collision Detected in x y z");
		}
		else{
			logger.info("Collision Detected");
		}
		return robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"),World.Current.getRootFrame());
	}
 
	@Override
	public void run() {
//		IHonkCapability honkCapability = kmp.getCapability(IHonkCapability.class);
//		honkCapability.honk();
		gripper2F1.close();
		Frame pickMain = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		Frame pick1 = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"));
		mF.setLEDBlue(false);
		ThreadUtil.milliSleep(200);
		gripper.move(ptp(getApplicationData().getFrame("/P6")).setJointVelocityRel(0.3));
		//gripper.move(ptp(getApplicationData().getFrame("/P5")).setJointVelocityRel(0.3));
		
		
		logger.info("calibrating");
		//gripper.move(linRel(0, 0, -130, World.Current.getRootFrame()).setJointVelocityRel(0.3));
		
		
		pickMain = calibrate();
		logger.info("calibrating done");
		ForceCondition touch = ForceCondition.createSpatialForceCondition(gripper.getFrame("/TCP"), 10);
		IMotionContainer motion1 = gripper.move(linRel(0,-5, 0, World.Current.getRootFrame()).setCartVelocity(30).breakWhen(touch));
		gripper.move(linRel(0,1,0, World.Current.getRootFrame()).setJointVelocityRel(0.3));
		ThreadUtil.milliSleep(200);
		IMotionContainer motion2 = gripper.move(linRel(-5,0, 0, World.Current.getRootFrame()).setCartVelocity(30).breakWhen(touch));
		gripper.move(linRel(1,0,0, World.Current.getRootFrame()).setJointVelocityRel(0.3));
		ThreadUtil.milliSleep(200);
		
		IMotionContainer motion3 = gripper.move(linRel(0,0, -5, World.Current.getRootFrame()).setCartVelocity(30).breakWhen(touch));
		gripper.move(linRel(0,0,1, World.Current.getRootFrame()).setJointVelocityRel(0.3));
		ThreadUtil.milliSleep(200);

		if (motion1.getFiredBreakConditionInfo() == null && motion2.getFiredBreakConditionInfo() == null && motion3.getFiredBreakConditionInfo() == null){
			logger.info("No Collision Detected in x y z");
		}
		else{
			logger.info("Collision Detected");
			pickMain = robot.getCurrentCartesianPosition(gripper.getFrame("/TCP"),World.Current.getRootFrame());
		}
		ThreadUtil.milliSleep(500);
		logger.info("100 up");
		gripper.move(linRel(0,0,100, World.Current.getRootFrame()).setJointVelocityRel(0.3));
		ThreadUtil.milliSleep(500);
		logger.info("going back to p5");
		gripper.move(ptp(pickMain).setJointVelocityRel(0.3));
		ThreadUtil.milliSleep(500);
		logger.info("150 down");
		gripper.move(linRel(0,0,150, World.Current.getRootFrame()).setJointVelocityRel(0.3));
		
		pick1.setX(pickMain.getX()+180);
		pick1.setY(pickMain.getY()+20);
		pick1.setZ(pickMain.getZ()+150);
		pick1.setAlphaRad(pickMain.getAlphaRad());
		pick1.setBetaRad(pickMain.getBetaRad());
		pick1.setGammaRad(pickMain.getGammaRad());
		
		gripper.move(ptp(pick1).setJointVelocityRel(0.4).setMode(springRobot));
		
		gripper2F1.setPos(150);
		gripper.move(linRel(0, 0, -130, World.Current.getRootFrame()).setCartVelocity(50).setMode(springRobot));
		gripper2F1.close();
		
		Spline mySpline = new Spline(
				spl(getApplicationData().getFrame("/P6")),
				spl(getApplicationData().getFrame("/P7")),
				spl(getApplicationData().getFrame("/P2"))
		);
		
		gripper.move(mySpline.setJointVelocityRel(0.4).setMode(springRobot));
		gripper2F1.setPos(150);

	}
}