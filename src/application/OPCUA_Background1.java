package application;

import javax.inject.Inject;
import java.util.concurrent.TimeUnit;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;

public class OPCUA_Background1 extends RoboticsAPICyclicBackgroundTask {
	
	@Inject	
	private OPCUA_Client_Background1 OPCUA;
	
	@Override
	public void initialize(){
		try {
			OPCUA.SetUp();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		try {
			OPCUA.ServerUpdate();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		initializeCyclic(0, 10, TimeUnit.MILLISECONDS, CycleBehavior.BestEffort);
	}

	@Override
	public void runCyclic() { 
		if (OPCUA.Disconnect==false) {
			try {
				OPCUA.ServerUpdate();
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		} else if (OPCUA.Disconnect==true) {
			OPCUA.clientDisconnect();
			while (true){
				
			}
		}

	}

}