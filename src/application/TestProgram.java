package application;

import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.ioModel.*;
import com.kuka.roboticsAPI.conditionModel.*;
import com.kuka.task.ITaskLogger;
import com.kuka.common.ThreadUtil;

public class TestProgram extends RoboticsAPIApplication {
    
    @Inject
    private ITaskLogger logger;
    @Inject
    private LBR robot;
    @Inject
    private IApplicationData data;
    
    
    
    @Override
    public void initialize() {
    }
  
    @Override
    public void run() {
        Main();
    }
    
    private Output GetDO(int port) {
        //Map you outputs here
        return null;
    }
    
    private Input GetDI(int port) {
        //Map you inputs here
        return null;
    }
    
    private void Main() {
        robot.move(lin(data.getFrame("/BASE_0/P1")).setCartVelocity(2000));
        robot.move(lin(data.getFrame("/BASE_0/P2")).setCartVelocity(2000));
    }
    
}