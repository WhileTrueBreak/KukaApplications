package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.task.ITaskLogger;

public class Drawerer extends RoboticsAPIApplication{
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
	private List<Position> path;
	
	@Override
	public void initialize() {
		
		// Initializes the boing boing
		springRobot = new CartesianImpedanceControlMode(); 
		
		// Set stiffness

		// TODO: Stiff in every direction except plane perpendicular to flange
		springRobot.parametrize(CartDOF.X).setStiffness(50);
		springRobot.parametrize(CartDOF.Y).setStiffness(1000);
		springRobot.parametrize(CartDOF.Z).setStiffness(1000);

		// Stiff rotation
		springRobot.parametrize(CartDOF.C).setStiffness(300);
		springRobot.parametrize(CartDOF.B).setStiffness(300);
		springRobot.parametrize(CartDOF.A).setStiffness(300);
		springRobot.setReferenceSystem(World.Current.getRootFrame());
		springRobot.parametrize(CartDOF.ALL).setDamping(0.4);
		
		// Inits the Robot
		gripper.attachTo(robot.getFlange());
		gripper2F1.initalise();
		gripper2F1.setSpeed(189);
		gripper2F1.open();
		mF.setLEDBlue(true);
		ThreadUtil.milliSleep(200);
		mF.setLEDBlue(false);
		ThreadUtil.milliSleep(200);
	}
	
	
	IMotionContainer m1;
	@Override
	public void run() {
		// Move to bottom left
		gripper.move(ptp(getApplicationData().getFrame("/bottom_left")).setMode(springRobot).setJointVelocityRel(0.2));
		path = getPath(); // have a file here probably
		Position current_pos = new Position(0, 0);

		for (Position position:path){
			current_pos = springyMove(current_pos, position);
		}
		
		ThreadUtil.milliSleep(120000);
	}
	
	private List<Position> getPath(){
		// TODO: Take in a file and return the points here.

		// temp path
		Position[] path = {new Position(100,50), new Position(300,300)};
		return Arrays.asList(path);
	}

	private Position springyMove(Position current_pos, Position next_pos){
		double vel = 0.2;
		Position move_pos = next_pos.getRelDistance(current_pos);
		gripper.move(linRel(move_pos.x, move_pos.y, 0).setJointVelocityRel(vel));
		return next_pos;
	}
}
