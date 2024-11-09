package org.firstinspires.ftc.teamcode.Subsystems;//package org.firstinspires.ftc.teamcode.components;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.utils.BotLog;
//import org.firstinspires.ftc.teamcode.utils.Component;
//
//import java.util.Arrays;
//import java.util.List;
//
//public class Color5 implements Component {
//
//    public boolean enableColor = true;
//    public BotLog logger = new BotLog();
//    public ElapsedTime elapsed = new ElapsedTime();
//
//    final double BlueThreshold = 2.5;
//    final double RedThreshold = 1.25;
//
//    public Boolean L2OnTape;
//    public Boolean L1OnTape;
//    public Boolean COnTape;
//    public Boolean R1OnTape;
//    public Boolean R2OnTape;
//
//    public double shiftedAmount = 0.0;
//
//    public NormalizedColorSensor L2;
//    public NormalizedColorSensor L1;
//    public NormalizedColorSensor C;
//    public NormalizedColorSensor R1;
//    public NormalizedColorSensor R2;
//    public NormalizedRGBA colorsL2;
//    public NormalizedRGBA colorsL1;
//    public NormalizedRGBA colorsC;
//    public NormalizedRGBA colorsR1;
//    public NormalizedRGBA colorsR2;
//    private List<NormalizedRGBA> colors;
//    private List<Boolean> onTape;
//
//
//    public Color5(HardwareMap map) {
//        L2 = map.get(NormalizedColorSensor.class, "L2");
//        L1 = map.get(NormalizedColorSensor.class, "L1");
//        C = map.get(NormalizedColorSensor.class, "C");
//        R1 = map.get(NormalizedColorSensor.class, "R1");
//        R2 = map.get(NormalizedColorSensor.class, "R2");
//    }
//
//    public boolean measure( ) {
//        int onTapeCount = 0;
//        shiftedAmount = 0.0;
//
//        if(enableColor) {
//            // Read all the sensors
//            colorsL2 = L2.getNormalizedColors();
//            colorsL1 = L1.getNormalizedColors();
//            colorsC = C.getNormalizedColors();
//            colorsR1 = R1.getNormalizedColors();
//            colorsR2 = R2.getNormalizedColors();
//
//            // Make a list for easier processing
//            colors = Arrays.asList(colorsL2, colorsL1, colorsC, colorsR1, colorsR2);
//
//            // Prevent divide by 0 exception
//            for (NormalizedRGBA myColor : colors) {
//                if (myColor.blue == 0.0) {
//                    myColor.blue = (float) 0.000001;
//                }
//                if (myColor.red == 0.0) {
//                    myColor.red = (float) 0.000001;
//                }
//            }
//
//            // Determine who is on the tape and count
//            L2OnTape = !isRGBAGray(colorsL2);
//            L1OnTape = !isRGBAGray(colorsL1);
//            COnTape = !isRGBAGray(colorsC);
//            R1OnTape = !isRGBAGray(colorsR1);
//            R2OnTape = !isRGBAGray(colorsR2);
//            onTape = Arrays.asList(L2OnTape, L1OnTape, COnTape, R1OnTape, R2OnTape);
//
//            for (Boolean myTape : onTape) {
//                if (myTape) {
//                    onTapeCount++;
//                }
//            }
//
//            //if(true) {
//            //    return (false);
//            //}
//
//            // Check all the states that we think are legal
//            // Slightly left or right
//            if (onTapeCount >= 4) {
//                shiftedAmount = 0.0;
//                return (true);
//            }
//
//            // Perfectly aligned
//            if ((onTapeCount == 3) && !L2OnTape && !R2OnTape) {
//                shiftedAmount = 0.0;
//                return (true);
//            }
//
//            // One sensor left or right
//            if ((onTapeCount == 3) && L2OnTape && L1OnTape && COnTape) {
//                shiftedAmount = (16.0 / 25.4);
//                return (true);
//            }
//            if ((onTapeCount == 3) && R2OnTape && R1OnTape && COnTape) {
//                shiftedAmount = (-16.0 / 25.4);
//                return (true);
//            }
//
//            // Two sensors left or right
//            if ((onTapeCount == 2) && L2OnTape && L1OnTape) {
//                shiftedAmount = (32.0 / 25.4);
//                return (true);
//            }
//            if ((onTapeCount == 2) && R2OnTape && R1OnTape) {
//                shiftedAmount = (-32.0 / 25.4);
//                return (true);
//            }
//
//            // 3 sensors left or right
//            if ((onTapeCount == 1) && L2OnTape) {
//                shiftedAmount = (48.0 / 25.4);
//                return (true);
//            }
//            if ((onTapeCount == 1) && R2OnTape) {
//                shiftedAmount = (-48.0 / 25.4);
//                return (true);
//            }
//        }
//
//        // Some weird state
//        return(false);
//    }
//
//    private boolean isRGBARed( NormalizedRGBA myColor ) {
//        if( (myColor.red/myColor.blue) > RedThreshold ) {
//            return(true);
//        } else {
//            return(false);
//        }
//    }
//
//    private boolean isRGBABlue( NormalizedRGBA myColor ) {
//        if( (myColor.blue/myColor.red) > BlueThreshold ) {
//            return(true);
//        } else {
//            return(false);
//        }
//    }
//
//    private boolean isRGBAGray( NormalizedRGBA myColor ) {
//        if( !isRGBARed(myColor) && !isRGBABlue(myColor) ) {
//            return(true);
//        } else {
//            return(false);
//        }
//    }
//
//    public boolean isRed( ) {
//        int red = 0 ;
//        int blue = 0  ;
//
//        for (NormalizedRGBA myColor : colors) {
//            if( isRGBARed(myColor) ) {
//                red++;
//            } else if ( isRGBABlue(myColor) ) {
//                blue++;
//            }
//        }
//        return (red > blue) ;
//    }
//
//    public boolean isBlue( ) {
//        int red = 0 ;
//        int blue = 0  ;
//
//        for (NormalizedRGBA myColor : colors) {
//            if( isRGBARed(myColor) ) {
//                red++;
//            } else if ( isRGBABlue(myColor) ) {
//                blue++;
//            }
//        }
//        return (blue > red) ;
//    }
//
//    public boolean isGray( ) {
//        return( !(isRed() || isBlue()));
//    }
//
//    @Override
//    public void initAuto() {
//        // Do nothing
//    }
//
//    @Override
//    public void initTeleOp() {
//        // Do nothing
//    }
//
//    @Override
//    public void updateComponent() {
//        // Do nothing
//    }
//
//    public String test() {
//        String failures = "";
//
//        return failures;
//    }
//}