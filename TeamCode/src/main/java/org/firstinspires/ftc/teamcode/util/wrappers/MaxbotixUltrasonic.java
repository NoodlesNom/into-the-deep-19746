package org.firstinspires.ftc.teamcode.util.wrappers;

import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * The class Maxbotix ultrasonic.
 */
public class MaxbotixUltrasonic
{
	//Scaling factor to convert from mV
	private static final double SCALING_FACTOR = 1.9 * 2.54; // Determined empirically, don't know why

	/**
	 * The ultrasonic sensor as an analog input.
	 */
	private AnalogInput ultrasonic;

	/**
	 * Instantiates a new Maxbotix ultrasonic sensor.
	 *
	 * @param ultrasonic the ultrasonic sensor
	 */
	public MaxbotixUltrasonic(AnalogInput ultrasonic)
	{
		this.ultrasonic = ultrasonic;
	}

	/**
	 * Gets the distance ultrasonic sensor senses in inches.
	 *
	 * @return the distance in inches
	 */
	public double getDistance()
	{
		//Converts V to mV and converts it to inches with the scaling factor
		return (ultrasonic.getVoltage() * 1000) / SCALING_FACTOR;
	}
}