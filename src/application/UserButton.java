package application;


import javax.inject.Inject;
import java.util.concurrent.TimeUnit;

import com.kuka.generated.ioAccess.Gripper2FIOGroup;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;

/**
 * Implementation of a cyclic background task.
 * <p>
 * It provides the {@link RoboticsAPICyclicBackgroundTask#runCyclic} method 
 * which will be called cyclically with the specified period.<br>
 * Cycle period and initial delay can be set by calling 
 * {@link RoboticsAPICyclicBackgroundTask#initializeCyclic} method in the 
 * {@link RoboticsAPIBackgroundTask#initialize()} method of the inheriting 
 * class.<br>
 * The cyclic background task can be terminated via 
 * {@link RoboticsAPICyclicBackgroundTask#getCyclicFuture()#cancel()} method or 
 * stopping of the task.
 * @see UseRoboticsAPIContext
 * 
 */
public class UserButton extends RoboticsAPICyclicBackgroundTask {
	@Inject
	private Controller kUKA_Sunrise_Cabinet_1;
	
	@Inject
	private Gripper2FIOGroup gripperIO;
	
	@Inject 
	private Gripper2F gripper2F1;
	
	@Override
	public void initialize() {
		// initialize your task here
		initializeCyclic(0, 500, TimeUnit.MILLISECONDS,	CycleBehavior.BestEffort);
		gripper2F1.initalise();
		IUserKeyBar gripperBar = getApplicationUI().createUserKeyBar("Gripper");
		IUserKeyListener openGripperListener = new IUserKeyListener(){
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				key.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Green,UserKeyLEDSize.Small);
				gripper2F1.open(); // Method for opening the gripper
				key.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Grey,UserKeyLEDSize.Small);
			}
		};
		IUserKeyListener closeGripperListener = new IUserKeyListener(){
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				key.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Green,UserKeyLEDSize.Small);
				gripper2F1.close(); // Method for closing the gripper
				key.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Grey,UserKeyLEDSize.Small);
			}
		};
		IUserKeyListener ForceListener = new IUserKeyListener(){
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				int Force = getApplicationData().getProcessData("GripForce").getValue(); // Method for closing the gripper
				if (event==UserKeyEvent.FirstKeyUp && Force<255){
					Force = Force + 10;
				}else if (event==UserKeyEvent.SecondKeyDown && Force>0){
					Force = Force - 10;
				}
				getApplicationData().getProcessData("GripForce").setValue(Force);
				gripper2F1.setForce(Force);
				key.setText(UserKeyAlignment.Middle, Integer.toString(Force));
				
			}
		};
			IUserKey openKey = gripperBar.addUserKey(0, openGripperListener, true);
			IUserKey closeKey = gripperBar.addUserKey(1,closeGripperListener, true);
			IUserKey forceKey = gripperBar.addDoubleUserKey(2, ForceListener, false);
			openKey.setText(UserKeyAlignment.Middle, "OPEN");
			closeKey.setText(UserKeyAlignment.Middle, "CLOSE");
			forceKey.setText(UserKeyAlignment.TopMiddle, "+");
			forceKey.setText(UserKeyAlignment.BottomMiddle, "-");
			forceKey.setText(UserKeyAlignment.TopMiddle, "+");			
			forceKey.setText(UserKeyAlignment.Middle, "Force");	
			gripperBar.publish();
		
	}

	@Override
	public void runCyclic() {
		// your task execution starts here
		
	}
}