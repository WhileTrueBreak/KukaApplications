package com.kuka.basictoolbox.examples;

import java.util.concurrent.TimeUnit;

import com.kuka.basictoolbox.gripper.HandlingGripperState;
import com.kuka.basictoolbox.gripper.HandlingGrippers;
import com.kuka.basictoolbox.gripper.IHandlingGripper;
import com.kuka.roboticsAPI.deviceModel.Robot;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.Workpiece;

/**
 * This DummyGripper imitates the behavior of a gripper and implements IHandlingGripper so that it can be used in the
 * BasicToolbox.
 */
public class VirtualGripper implements IHandlingGripper
{
    private HandlingGripperState _handlingGripperState = HandlingGripperState.RELEASED;
    private Workpiece _workpiece;
    private Tool _tool;

    /**
     * Constructor of the DummyGripper. The root frame of the tool is automatically
     * registered with the DummyGripper, i.e. a IHandlingGripper.
     * 
     * @param tool
     *            The tool has to be injected with the correct tool name.
     * @param robot
     *            the injected robot which the tool should be attached to
     */
    public VirtualGripper(Tool tool, Robot robot)
    {
        _tool = tool;
        _tool.attachTo(robot.getFlange());
        HandlingGrippers.register(this);
    }

    @Override
    public void releaseAsync()
    {
        _handlingGripperState = HandlingGripperState.RELEASED;

    }

    @Override
    public void gripAsync()
    {
        _handlingGripperState = HandlingGripperState.GRIPPED_ITEM;
    }

    @Override
    public HandlingGripperState getGripperState()
    {
        return _handlingGripperState;
    }

    @Override
    public boolean waitForGripperState(final int timeout, final TimeUnit timeUnit, final HandlingGripperState... target)
    {
        for (final HandlingGripperState state : target)
        {
            if (state == _handlingGripperState)
            {
                return true;
            }
        }
        return false;
    }

    @Override
    public boolean waitForGripperState(final HandlingGripperState... target)
    {
        for (final HandlingGripperState state : target)
        {
            if (state == _handlingGripperState)
            {
                return true;
            }
        }
        return false;

    }

    @Override
    public void setWorkpiece(final Workpiece workpiece)
    {
        _workpiece = workpiece;
    }

    @Override
    public Workpiece getWorkpiece()
    {
        return _workpiece;
    }

    @Override
    public Tool getTool()
    {
        return _tool;
    }

    /**
     * Unregister the tool from the IHandlingGripper.
     */
    public void dispose()
    {
        HandlingGrippers.unregister(this);
    }

}
