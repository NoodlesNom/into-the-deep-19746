package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;


public class MBUltraSonic {

    public boolean enableUltra = false;
    private AnalogInput V33;
    private AnalogInput UltraS;
    public Deadline sampleTimer = new Deadline(105, TimeUnit.MILLISECONDS);
    private double largestS = 0.0;
    private double lastVcc = 0.0;
    private double lastVobs = 0.0;
    public double lastS = 0.0;
    public String name;
    public double m = 1.0;
    public double b = 0.0;
    public double sensorM = 7.18;
    public double sensorB = -302.0;
    // From the spec.  This does not work:  s = (((Vobs/(Vcc/1024.0))*6.0)-300.0);
    // Experimental results
    // Right Sensor
    // m = 7.1714
    // b = -281.85
    // Left Sensor
    // m = 7.1455
    // b = -299.01
    public MBUltraSonic(HardwareMap map, String V33Str, String UltraStr) {
        V33 = map.tryGet(AnalogInput.class, V33Str);
        UltraS = map.tryGet(AnalogInput.class, UltraStr);
        name = UltraStr;

        reset();
        enableUltra = false;
    }

    public double convert( double Vcc, double Vobs) {
        double s = 0.0;
        // The sensor refreshes every 100ms.
        // If we wait 105ms, we're guaranteed to get a 'new" value since the SW reset event
        if((Vcc != 0) && sampleTimer.hasExpired()) {
            s = ((((Vobs / (Vcc / 1024.0)) * sensorM) + sensorB))*m + b;
        }
        return s;
    }

    public double captureDistance( ) {
        // BotLog.logD("UltraMBInfo:", String.format("Called %s", name));
        if(enableUltra && (V33 != null) && (UltraS != null)) {
            lastVcc = V33.getVoltage();
            lastVobs = UltraS.getVoltage();
            lastS = convert(lastVcc, lastVobs);
            largestS = Math.abs(largestS) > Math.abs(lastS) ? largestS : lastS;
            //BotLog.logD("UltraMBInfo:", String.format("Capture Vcc: %.2f, Obs: %.2f, S: %.2f, maxS: %.2f", lastVcc, lastVobs, lastS, largestS));
            return lastS;
        } else {
            return 0.0;
        }
    }

    public void reset(double m, double b) {
        this.m = m;
        this.b = b;
        reset();
    }

    public void reset( ) {
        largestS = 0.0;
        lastS = 0.0;
        lastVcc = 0.0;
        lastVobs = 0.0;
        sampleTimer.reset();
    }

    public void autoInit(){
        autoInit(1.0,0.0);
    }
    public void autoInit(double m, double b){
        reset();
        this.m = m;
        this.b = b;
        enableUltra = true;
    }

    public void teleopInit(){
    }

    public void calibrationData()
    {
        // UltraSonic calibration data
        double Vcc = V33.getVoltage();
        double Vobs = UltraS.getVoltage();

        BotLog.logD("UltraCal:", "%s: Vcc: %5.3f, Vobs: %5.3f", name, Vcc, Vobs);
    }

}