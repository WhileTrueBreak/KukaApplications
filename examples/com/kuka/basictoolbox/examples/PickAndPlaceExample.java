package com.kuka.basictoolbox.examples;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.appframework.AppFwApplicationCategory;
import com.kuka.appframework.IAppFwApplication;
import com.kuka.basictoolbox.impl.Container;
import com.kuka.basictoolbox.util.SceneGraphUtil;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.applicationModel.tasks.UseRoboticsAPIContext;
import com.kuka.roboticsAPI.deviceModel.Robot;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.Workpiece;

/**
 * Auto-generated AppFramework application class.
 */
@AppFwApplicationCategory(model = "PickAndPlaceExample.blm")
@UseRoboticsAPIContext
public class PickAndPlaceExample implements IAppFwApplication
{
    private static final int TRAY_NUM_OF_ITEMS_IN_ROW = 2;
    private static final int TRAY_NUM_OF_ITEMS_IN_COL = 3;

    @Inject
    private Robot _robot;

    @Inject
    private IApplicationData _appData;

    @Inject
    @Named("MyVirtualGripper")
    private Tool _gripper;

    private VirtualGripper _handlingGripper;

    @Override
    public void initialize()
    {
        _handlingGripper = new VirtualGripper(_gripper, _robot);
        _gripper.attachTo(_robot.getFlange());

        Container tray = Container.createTray(_appData.getFrame("/PickAndPlaceExample/Tray"), TRAY_NUM_OF_ITEMS_IN_ROW, TRAY_NUM_OF_ITEMS_IN_COL);

        for (int i = 0; i < tray.capacity(); i++)
        {
            Workpiece workpiece = _appData.createFromTemplate("Item");
            tray.addWorkpiece(workpiece, workpiece.getRootFrame());
        }

        Container.createStack(_appData.getFrame("/PickAndPlaceExample/Stack"), tray.capacity());
    }

    @Override
    public void dispose()
    {
        _handlingGripper.dispose();
        SceneGraphUtil.cleanUpSceneGraph();
    }
}
