package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.utils.BotLog;
import org.firstinspires.ftc.teamcode.util.utils.Component;

public class MBUltraSonic implements Component {

    public boolean enableUltra = false;
    public AnalogInput V33;
    public AnalogInput UltraS;
    public BotLog logger = new BotLog();
    public ElapsedTime elapsed = new ElapsedTime();
    public long samplesSinceReset = 0;
    public double maxS = 0.0;
    public double lastVcc = 0.0;
    public double lastVobs = 0.0;
    public double lastS = 0.0;
    public String name;

    public MBUltraSonic(HardwareMap map, String V33Str, String UltraStr) {
        V33 = map.get(AnalogInput.class, V33Str);
        UltraS = map.get(AnalogInput.class, UltraStr);
        name = UltraStr;

        reset();
        enableUltra = false;

        logger.LOGLEVEL = logger.LOGDEBUG;
    }

    public double convert( double Vcc, double Vobs) {
        double s;
        // s = (((Vobs/(Vcc/1024.0))*6.0)-300.0);
        // Sensor #1
        // s = (((Vobs/(Vcc/1024.0))*7.295)-325.0);
        // Sensor #2, #3, #4
        s = (((Vobs/(Vcc/1024.0))*7.18)-302.0);
        return s;
    }

    public double captureDistance( ) {
        //logger.logD("UltraMBInfo:", String.format("Called %s", name));
        if(enableUltra) {
            lastVcc = V33.getVoltage();
            lastVobs = UltraS.getVoltage();
            lastS = convert(lastVcc, lastVobs);
            maxS = Math.max(maxS, lastS);
            samplesSinceReset++;
            //logger.logD("UltraMBInfo:", String.format("Capture %d, %.2f, %.2f, %.2f, %.2f", samplesSinceReset, lastVcc, lastVobs, lastS, maxS));
            return lastS;
        } else {
            return 0.0;
        }
    }

    public void reset( ) {
        samplesSinceReset = 0;
        maxS = 0.0;
        lastS = 0.0;
        lastVcc = 0.0;
        lastVobs = 0.0;
        elapsed.reset();
    }

    @Override
    public void initAuto() {
        reset();
        enableUltra = false;
    }

    @Override
    public void initTeleOp() {
    }

    @Override
    public void updateComponent() {
        //capture();
    }

    public String test() {
        String failures = "";

        return failures;
    }
}