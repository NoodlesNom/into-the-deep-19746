package org.firstinspires.ftc.teamcode.debugging;

import org.firstinspires.ftc.teamcode.autonomous.gf.FloatPoint;

import java.text.DecimalFormat;

public class ComputerDebuggingNew {
    //this is what actually sends our messages
    private static UDPServer udpServer;
    //this is what we will use to build the messages
    private static StringBuilder messageBuilder = new StringBuilder();

    //use this to format decimals
    private static DecimalFormat df = new DecimalFormat("#.00");

    /**
     * Initializes udp server and starts it's thread
     */
    public ComputerDebuggingNew(){
        UDPServer.kill = false;
        udpServer = new UDPServer(36929);
        Thread runner = new Thread(udpServer);
        runner.start();//go go go
    }



    /**
     * Sends the robot location to the debug computer
     */
    public static void sendRobotLocation(double x, double y, double angle_rad){
        //return if not using computer
        //if(!Robot.usingComputer){return;}

        //first send the robot location
        messageBuilder.append("ROBOT,");
        messageBuilder.append(df.format(x));
        messageBuilder.append(",");
        messageBuilder.append(df.format(y));
        messageBuilder.append(",");
        messageBuilder.append(df.format(angle_rad));
        messageBuilder.append("%");

    }

    /**
     * Sends the location of any point you would like to send
     * @param floatPoint the point you want to send
     */
    public static void sendKeyPoint(FloatPoint floatPoint) {
        //if(!Robot.usingComputer){return;}


        messageBuilder.append("P,")
                .append(df.format(floatPoint.x))
                .append(",")
                .append(df.format(floatPoint.y))
                .append("%");
    }


    /**
     * This is a point you don't want to clear every update
     * @param floatPoint the point you want to send
     */
    public static void sendLogPoint(FloatPoint floatPoint) {
        //if(!Robot.usingComputer){return;}


        messageBuilder.append("LP,")
                .append(df.format(floatPoint.x))
                .append(",")
                .append(df.format(floatPoint.y))
                .append("%");
    }


    /**
     * Used for debugging lines
     * @param point1
     * @param point2
     */
    public static void sendLine(FloatPoint point1, FloatPoint point2){
        //return if not using the computer
        //if(!Robot.usingComputer){return;}
        messageBuilder.append("LINE,")
                .append(df.format(point1.x))
                .append(",")
                .append(df.format(point1.y))
                .append(",")
                .append(df.format(point2.x))
                .append(",")
                .append(df.format(point2.y))
                .append("%");
    }


    /**
     * This kills the udpServer background thread
     */
    public static void stopAll() {
        //if(!Robot.usingComputer){return;}

        UDPServer.kill = true;
    }

    /**
     * Sends the data accumulated over the update by adding it to the udpServer
     */
    public static void markEndOfUpdate() {
        //if(!Robot.usingComputer){return;}
        messageBuilder.append("CLEAR,%");
        System.out.println("HERE: " + messageBuilder.toString());
        udpServer.splitAndSend(messageBuilder.toString());
        messageBuilder = new StringBuilder();
    }

    /**
     * Forces a clear log
     */
    public static void clearLogPoints() {
        //if(!Robot.usingComputer){return;}
        udpServer.splitAndSend("CLEARLOG,%");

    }
}