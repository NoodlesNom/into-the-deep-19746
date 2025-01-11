package org.firstinspires.ftc.teamcode.autonomous.gf;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.rr.Drawing;

import java.util.ArrayList;

@Config
@Autonomous(name="GF test")
public class GFtest extends OldAutoMaster {

    private enum State {
        sub1,
        pick1,
        human1,
        spec1,
        sub2,
        finished
    }
    public static int gox = 0;
    public static int goy = 0;
    private State auto = State.sub1;

    ElapsedTime test = new ElapsedTime();
    private Rotation2d endAngle;
    private Vector2d endPos;
    private double speed = 0;

    public static double targetAngularPow = .5;
    public static double targetStraightPow = .5;
    public static double targetStrafePow = .5;

    private boolean start = false;

    public static int accelTime = 2;
    public static int slipTime = 1;

    private double stateStartTime = 0;

    private Path sub1()
    {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0, 0, 0, 0, 0, 0, 0, 0));
        allPoints.add(new CurvePoint(5, 24 , 1, 1, 40, 40, Math.toRadians(60), 0.6));
        return new Path(allPoints, Math.toRadians(30),true);
    }

    private Path pick1()
    {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0, 36, 0, 0, 0, 0, 0, 0));
        allPoints.add(new CurvePoint(0, 0, 1, 1, 40, 40, Math.toRadians(60), 0.6));
        return new Path(allPoints, Math.toRadians(180));
    }
    private Path human1()
    {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(32.8, 36.6, 0, 0, 0, 0, 0, 0));
        allPoints.add(new CurvePoint(36.6, 27.2, 1, 1, 40, 40, Math.toRadians(60), 0.6));
        return new Path(allPoints, false);
    }
    private Path spec1()
    {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(32.8, 36.6, 0, 0, 0, 0, 0, 0));
        allPoints.add(new CurvePoint(36, 13, 1, 1, 40, 40, Math.toRadians(60), 0.6));
        allPoints.add(new CurvePoint(36, 12, 1, 1, 40, 40, Math.toRadians(60), 0.6));

        return new Path(allPoints, false);
    }
    private Path sub2()
    {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(36, 12, 0, 0, 0, 0, 0, 0));
        allPoints.add(new CurvePoint(12, 36, 1, 1, 40, 40, Math.toRadians(60), 0.6));

        return new Path(allPoints, false, Math.toRadians(-90));
    }
    
    @Override
    public void setStartPose(GFRobot robot)
    {
        robot.mDrive.localizer.pose = new com.acmerobotics.roadrunner.Pose2d(0,0,Math.toRadians(0));
        robot.mDrive.localizer.pinpoint.setPosition(new com.acmerobotics.roadrunner.Pose2d(0, 0, Math.toRadians(0)));

    }

    @Override
    public void runMain(GFRobot robot, double time)
    {


        if (!start) {
            start = true;
            robot.mDrive.setWantGFPath(sub1());
        }


        switch (auto)
        {
            case sub1: 
            {
                if (robot.mDrive.isDoneWithGF()){
                    if (test.seconds()>0.1) {
                        auto = State.pick1;
                        robot.mDrive.setWantGFPos(0,0,Math.toRadians(0),1, 1    );
                    }
                }else{
                    test.reset();
                }
                break;
            }
            case pick1:
            {
                if (robot.mDrive.isDoneWithGF()){
                    if (test.seconds()>0.1) {
                        auto = State.human1;
                        robot.mDrive.setWantGFPos(5,24,Math.toRadians(30),1, 1    );

                    }
                }else{
                    test.reset();
                }

                break;
            }
            case human1:
            {
                if (robot.mDrive.isDoneWithGF()){
                    if (test.seconds()>0.1) {
                        auto = State.finished;
                        //robot.mDrive.setWantGFPath(spec1());
                    }
                }else{
                    test.reset();
                }
                break;
            }
            case spec1:
            {
                if (robot.mDrive.isDoneWithGF()){
                    if (test.seconds()>0.1) {
                        auto = State.sub2;
                        robot.mDrive.setWantGFPath(sub2());
                    }
                }else{
                    test.reset();
                }
                break;
            }
            case sub2:
            {
                if (robot.mDrive.isDoneWithGF()){
                    if (test.seconds()>0.1) {
                        auto = State.finished;
                    }
                }else{
                    test.reset();
                }
                break;
            }
            case finished:
            {
                robot.mDrive.setOpenLoop(0,0,0,0);
                break;
            }

        }
        robot.update(time);
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), robot.mDrive.mPeriodicIO.currentpose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}