package org.firstinspires.ftc.teamcode.util.utils;

import java.util.ArrayDeque;
import java.util.Collections;

/**
 * Created by Parv on 3/30/2018.
 */

public class RollingAverage
{
	private ArrayDeque<Double> values;
	private boolean firstTime;
	private double average;
	private int n;

	public RollingAverage(int n)
	{
		values = new ArrayDeque<>(n);
		firstTime = true;
		average = 0;
		this.n = n;
	}

	public double getAverage(double value)
	{
		if (Double.isNaN(value) || Double.isInfinite(value))
			return average;

		if (firstTime)
		{
			values.addAll(Collections.nCopies(n, value));
			firstTime = false;
			return value;
		} else
		{
			values.remove();
			values.add(value);
		}

		average = calculateAverage(values);

		return average;
	}

	private double calculateAverage(ArrayDeque<Double> list)
	{
		double sum = 0;

		for (Double item : list)
			sum += item;

		return sum / (double) list.size();
	}
}
