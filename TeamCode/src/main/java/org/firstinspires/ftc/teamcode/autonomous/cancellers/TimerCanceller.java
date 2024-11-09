package org.firstinspires.ftc.teamcode.autonomous.cancellers;

/**
 * The class Timer canceller.
 */
public class TimerCanceller implements Canceller
{
	private double ms, start;

	/**
	 * Instantiates a new Timer canceller.
	 *
	 * @param ms the ms
	 */
	public TimerCanceller(double ms)
	{
		reset(ms);
	}

	public void reset(double ms)
	{
		this.ms = ms;
		start = System.currentTimeMillis();
	}

	public void reset()
	{
		reset(this.ms);
	}

	//Returns if the time requirement is met
	public boolean isConditionMet()
	{
		return ((start + ms) < System.currentTimeMillis());
	}

	public double timeRemaining()
	{
		double now = System.currentTimeMillis();

		if( (start + ms) > now ) {
			return ((start + ms) - now);
		} else {
			return(0.0);
		}
	}


	//Waits for timer to finish
	public void waitForCanceller()
	{
		while (!isConditionMet()) {
			try {
				Thread.sleep(100, 0);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
}
