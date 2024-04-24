package com.kuka.generated.flexfellow;

import static com.kuka.flexfellow.FlexFellowIos.IO_GROUP_NAME;
import static com.kuka.flexfellow.FlexFellowIos.MAX_PRESSURE_EXCEEDED;
import static com.kuka.flexfellow.FlexFellowIos.PRESSURE_OK;

import javax.inject.Singleton;

import com.kuka.flexfellow.AbstractFlexFellowComfort;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to facilitate access to the flexFELLOW's functionality.
 * <i>Do not modify this file!</i>
 * 
 * Represents a KUKA Sunrise.flexFELLOW H900 Comfort platform and gives access to its IOs.
 */
@Singleton
public class FlexFellow extends AbstractFlexFellowComfort
{
    private final FlexFellowIOGroup _ioGroup;

    /**
     * Creates a new KUKA flexFELLOW instance.
     * 
     * @param controller
     *            the controller to be used
     * 
     * @param name
     *            the name for the flexFELLOW instance
     */
    public FlexFellow(Controller controller, String name)
    {
        super(controller, name);
        _ioGroup = new FlexFellowIOGroup(controller);
    }

    /**
     * Gets the state of the pressure with respect to the settings of the pressure sensor.
     * 
     * @return <code>true</code>, if the pressure is OK with respect to the settings of the pressure sensor.
     *         <code>false</code>, otherwise.
     */
    public boolean isPressureOk()
    {
        return _ioGroup.getBooleanIOValue(PRESSURE_OK, false);
    }

    /**
     * Gets the state of the pressure with respect to the maximal allowed pressure.
     * 
     * @return <code>true</code>, if the maximum pressure is exceeded. <code>false</code>,
     *         otherwise.
     */
    public boolean isMaximumPressureExceeded()
    {
        return _ioGroup.getBooleanIOValue(MAX_PRESSURE_EXCEEDED, false);
    }

    /**
     * Class to encapsulate access to the KUKA flexFELLOW's IOs.
     */
    private static class FlexFellowIOGroup extends AbstractFlexFellowIOGroup
    {
        public FlexFellowIOGroup(Controller controller)
        {
            super(controller, IO_GROUP_NAME);

            addInput(PRESSURE_OK, IOTypes.BOOLEAN, 1);
            addInput(MAX_PRESSURE_EXCEEDED, IOTypes.BOOLEAN, 1);
        }
    }
}
