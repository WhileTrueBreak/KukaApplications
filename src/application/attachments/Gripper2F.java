package application.attachments;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.Gripper2FIOGroup;
import com.kuka.task.ITaskLogger;

@Singleton  //??
public class Gripper2F {
	@Inject
	private Gripper2FIOGroup gripperIO;
	@Inject
	private ITaskLogger logger;
	
//Level 1 Functions - Individual	
	public void activate(){
 		gripperIO.setRAct(true);
		//logger.info("Griper Activated");
	}
	
	public void deactivate(){
		gripperIO.setRAct(false);;
		//logger.info("Griper Dectivated");
	}
	
	public void enable(){
		gripperIO.setRGTO(true);
		//logger.info("Griper Enabled");
	}

	public void stop(){
		gripperIO.setRGTO(false);;
	}
	
	public void setPos(int pos){
		gripperIO.setR_Pos(pos);
		//logger.info("Gripper Position Set:" + pos);
	}
	
	public void setSpeed(int speed){
 		gripperIO.setR_Speed(speed);
		//logger.info("Gripper Speed Set:" + speed);
	}
	
	public void setForce(int force){
 		gripperIO.setR_Force(force);
		//logger.info("Gripper Force Set:" + force);
	}

	public int readStatus(){
		ThreadUtil.milliSleep(500);
		boolean status1 = gripperIO.getGOBJ1();
		boolean status0 = gripperIO.getGOBJ0();
		if (status1||status0){
			return 1; //Activation Complete
		}else{
			return 0; //Activation in Progress
		}
	}
	
	public int readObjectDetection(){
		boolean status1 = gripperIO.getGOBJ1();
		boolean status0 = gripperIO.getGOBJ0();
		if (status1&&status0){
			//logger.info("Fingers in Requested Position");
			return 3; //Activation Complete
		}else if (status1&&!status0){
			//logger.info("Object Deteted, Fingers Stopped while closing");
			return 2;
		}else if (!status1&&status0){
			//logger.info("Object Deteted, Fingers Stopped while opening");
			return 1;
		}else if (!status1&&!status0){
			//logger.info("Fingers in motion");
			return 0;
		}else{
			//logger.error("Object Deteted ERROR");
			return 10; //Activation in Progress
		}
	}
	
	public int readPos(){
		int pos = gripperIO.getGPos();
 		//logger.info("Detected Position:" + pos);
 		return pos;
	}
	
	public int readCurrent(){
		int current = gripperIO.getGCurrent();
		//logger.info("Detected Current:" + current);
		return current;
	}	

//Level 2 Functions - Combined
	
	public void reset(){
		deactivate();
		activate();
		//logger.info("Gripper Reset");
	}
	
	public void open(){
		//logger.info("Gripper: Opening");
 		setPos(0);
 		int i = 0;
 		while(readStatus()==0){}
 		readObjectDetection();
 		//logger.info("Gripper: Complete");
	}
	
	public void close(){
		//logger.info("Gripper: Closing");
 		setPos(255);
 		while(readStatus()==0){}
		readObjectDetection();
		//logger.info("Gripper: Complete");
	}
	
	public void moveTo(int pos){
		setPos(pos);
		while(readStatus()==0){}
		readObjectDetection();
		//logger.info("Gripper: Complete");
	}
	
	public void initalise(){
		activate();
		stop();
		//setSpeed(0);
		//setForce(0);
		//setPos(0);
		activate();
		enable();
	}
}


