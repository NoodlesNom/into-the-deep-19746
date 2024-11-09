package org.firstinspires.ftc.teamcode.autonomous.cancellers.sensors;

import org.firstinspires.ftc.teamcode.autonomous.cancellers.Canceller;

public abstract class SensorCanceller implements Canceller
{
	private double target;
	private Condition condition;

	protected abstract double getCurrentValue();

	public SensorCanceller(double target, Condition condition)
	{
		this.target = target;
		this.condition = condition;
	}

	public boolean isConditionMet()
	{
		double current = getCurrentValue();

		switch (condition)
		{
			case LESS:
				return current < target;
			case LESS_OR_EQUAL:
				return current <= target;
			case GREATER:
				return current > target;
			case GREATER_OR_EQUAL:
				return current >= target;
			case NOT_EQUALS:
				return current != target;
			default:
				return true;
		}
	}

	public double getTarget()
	{
		return target;
	}

	public void setTarget(double target)
	{
		this.target = target;
	}

	public enum Condition
	{
		LESS,
		LESS_OR_EQUAL,
		GREATER,
		GREATER_OR_EQUAL,
		NOT_EQUALS,
		IN_RANGE
	}
}
