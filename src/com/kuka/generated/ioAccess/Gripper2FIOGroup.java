package com.kuka.generated.ioAccess;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>Gripper2F</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * ./.
 */
@Singleton
public class Gripper2FIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'Gripper2F'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'Gripper2F'
	 */
	@Inject
	public Gripper2FIOGroup(Controller controller)
	{
		super(controller, "Gripper2F");

		addInput("GACT", IOTypes.BOOLEAN, 1);
		addInput("GCurrent", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("GFault", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("GGTO", IOTypes.BOOLEAN, 1);
		addInput("GOBJ0", IOTypes.BOOLEAN, 1);
		addInput("GOBJ1", IOTypes.BOOLEAN, 1);
		addInput("GPos", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("GSTA0", IOTypes.BOOLEAN, 1);
		addInput("GSTA1", IOTypes.BOOLEAN, 1);
		addDigitalOutput("R_Force", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("R_Pos", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("R_Speed", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("RAct", IOTypes.BOOLEAN, 1);
		addDigitalOutput("RARD", IOTypes.BOOLEAN, 1);
		addDigitalOutput("RGTO", IOTypes.BOOLEAN, 1);
		addDigitalOutput("RLBP", IOTypes.BOOLEAN, 1);
	}

	/**
	 * Gets the value of the <b>digital input '<i>GACT</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'GACT'
	 */
	public boolean getGACT()
	{
		return getBooleanIOValue("GACT", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>GCurrent</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'GCurrent'
	 */
	public java.lang.Integer getGCurrent()
	{
		return getNumberIOValue("GCurrent", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>GFault</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'GFault'
	 */
	public java.lang.Integer getGFault()
	{
		return getNumberIOValue("GFault", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>GGTO</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'GGTO'
	 */
	public boolean getGGTO()
	{
		return getBooleanIOValue("GGTO", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>GOBJ0</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'GOBJ0'
	 */
	public boolean getGOBJ0()
	{
		return getBooleanIOValue("GOBJ0", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>GOBJ1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'GOBJ1'
	 */
	public boolean getGOBJ1()
	{
		return getBooleanIOValue("GOBJ1", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>GPos</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'GPos'
	 */
	public java.lang.Integer getGPos()
	{
		return getNumberIOValue("GPos", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>GSTA0</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'GSTA0'
	 */
	public boolean getGSTA0()
	{
		return getBooleanIOValue("GSTA0", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>GSTA1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'GSTA1'
	 */
	public boolean getGSTA1()
	{
		return getBooleanIOValue("GSTA1", false);
	}

	/**
	 * Gets the value of the <b>digital output '<i>R_Force</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'R_Force'
	 */
	public java.lang.Integer getR_Force()
	{
		return getNumberIOValue("R_Force", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>R_Force</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'R_Force'
	 */
	public void setR_Force(java.lang.Integer value)
	{
		setDigitalOutput("R_Force", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>R_Pos</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'R_Pos'
	 */
	public java.lang.Integer getR_Pos()
	{
		return getNumberIOValue("R_Pos", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>R_Pos</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'R_Pos'
	 */
	public void setR_Pos(java.lang.Integer value)
	{
		setDigitalOutput("R_Pos", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>R_Speed</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'R_Speed'
	 */
	public java.lang.Integer getR_Speed()
	{
		return getNumberIOValue("R_Speed", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>R_Speed</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'R_Speed'
	 */
	public void setR_Speed(java.lang.Integer value)
	{
		setDigitalOutput("R_Speed", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>RAct</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'RAct'
	 */
	public boolean getRAct()
	{
		return getBooleanIOValue("RAct", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>RAct</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'RAct'
	 */
	public void setRAct(java.lang.Boolean value)
	{
		setDigitalOutput("RAct", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>RARD</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'RARD'
	 */
	public boolean getRARD()
	{
		return getBooleanIOValue("RARD", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>RARD</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'RARD'
	 */
	public void setRARD(java.lang.Boolean value)
	{
		setDigitalOutput("RARD", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>RGTO</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'RGTO'
	 */
	public boolean getRGTO()
	{
		return getBooleanIOValue("RGTO", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>RGTO</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'RGTO'
	 */
	public void setRGTO(java.lang.Boolean value)
	{
		setDigitalOutput("RGTO", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>RLBP</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'RLBP'
	 */
	public boolean getRLBP()
	{
		return getBooleanIOValue("RLBP", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>RLBP</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'RLBP'
	 */
	public void setRLBP(java.lang.Boolean value)
	{
		setDigitalOutput("RLBP", value);
	}

}
