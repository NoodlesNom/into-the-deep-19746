package org.firstinspires.ftc.teamcode.autonomous.cancellers.sensors;

import org.firstinspires.ftc.teamcode.util.wrappers.MaxbotixUltrasonic;

/**
 * The type Ultrasonic canceller.
 */
public class UltrasonicCanceller extends SensorCanceller
{
	private MaxbotixUltrasonic ultrasonic;

	/**
	 * Instantiates a new ultrasonic canceller.
	 *
	 * @param target     the target
	 * @param condition  the condition
	 * @param ultrasonic the ultrasonic
	 */
	public UltrasonicCanceller(double target, Condition condition, MaxbotixUltrasonic ultrasonic)
	{
		super(target, condition);

		this.ultrasonic = ultrasonic;
	}

	@Override
	protected double getCurrentValue() {
		return ultrasonic.getDistance();
	}
}
