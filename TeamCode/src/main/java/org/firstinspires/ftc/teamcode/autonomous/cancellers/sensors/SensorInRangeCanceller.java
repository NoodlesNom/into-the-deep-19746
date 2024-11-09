package org.firstinspires.ftc.teamcode.autonomous.cancellers.sensors;

import org.firstinspires.ftc.teamcode.autonomous.cancellers.Canceller;
import org.firstinspires.ftc.teamcode.util.utils.Range;

public abstract class SensorInRangeCanceller implements Canceller
{
	protected Range range;

	protected abstract double getCurrentValue();

	public SensorInRangeCanceller(Range range)
	{
		this.range = range;
	}

	public boolean isConditionMet()
	{
		return range.isInRange(getCurrentValue());
	}

	public Range getRange()
	{
		return range;
	}
}
