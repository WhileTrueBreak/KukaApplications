package application;

import javax.inject.Inject;
import java.util.concurrent.TimeUnit;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;

public class OPCUA_Background3 extends RoboticsAPICyclicBackgroundTask {
	
	@Inject	
	private OPCUA_Client_Background3 OPCUA;
	
	@Override
	public void initialize(){

		// Comment Between Here
		
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
		
		
		// AND HERE
		
		initializeCyclic(0, 10, TimeUnit.MILLISECONDS, CycleBehavior.BestEffort);
	}

	@Override
	public void runCyclic() { 
		// Comment Between Here
		
		
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
		
		
		
		
		// AND HERE
	}

}