package org.firstinspires.ftc.teamcode.util.utils;

/**
 * Created by parvs on 4/12/2019.
 */
public class Range
{
	private double originalLowerBound;
	private double originalUpperBound;
	private double lowerBound;
	private double upperBound;
	
	public Range(double lowerBound, double upperBound)
	{
		this.lowerBound = originalLowerBound = lowerBound;
		this.upperBound = originalUpperBound = upperBound;
	}

	public void setLowerBound(double lowerBound)
	{
		this.lowerBound = lowerBound;
	}

	public void setUpperBound(double upperBound)
	{
		this.upperBound = upperBound;
	}

	public double getLowerBound()
	{
		return lowerBound;
	}

	public double getUpperBound()
	{
		return upperBound;
	}

	public boolean isInRange(double number)
	{
		return number <= upperBound && number >= lowerBound;
	}

	public void shiftRange(double shiftAmount)
	{
		lowerBound += shiftAmount;
		upperBound += shiftAmount;
	}

	public void shiftOriginalRange(double shiftAmount)
	{
		lowerBound = originalLowerBound + shiftAmount;
		upperBound = originalUpperBound + shiftAmount;
	}
}
