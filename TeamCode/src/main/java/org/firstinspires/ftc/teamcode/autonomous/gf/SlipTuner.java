package org.firstinspires.ftc.teamcode.autonomous.gf;

import static org.firstinspires.ftc.teamcode.autonomous.gf.GFMath.AngleWrap;
import static org.firstinspires.ftc.teamcode.autonomous.gf.GFMath.inToCM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.autonomous.gf.Rotation2d;
import org.firstinspires.ftc.teamcode.autonomous.gf.Translation2d;
import org.firstinspires.ftc.teamcode.autonomous.gf.GFMath;
import org.firstinspires.ftc.teamcode.autonomous.gf.DriveCharacterization;
import org.firstinspires.ftc.teamcode.autonomous.rr.Drawing;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.util;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

import java.util.ArrayList;
import java.util.List;
@Config
@Autonomous(name="slip tuner")
public class SlipTuner extends OldAutoMaster {

    private enum State {
        straight,
        slipStraight,
        wait,
        turn,
        slipTurn,
        strafe,
        slipStrafe,
        finished
    }

    private State auto = State.straight;

    private Rotation2d endAngle;
    private Vector2d endPos;
    private double speed = 0;

    public static double targetAngularPow = .5;
    public static double targetStraightPow = .5;
    public static double targetStrafePow = .5;

    public static int accelTime = 2;
    public static int slipTime = 1;

    private double stateStartTime = 0;

    @Override
    public void setStartPose(GFRobot robot)
    {

        robot.mDrive.localizer.pinpoint.setPosition(new com.acmerobotics.roadrunner.Pose2d(-50, 0, 0));
    }

    @Override
    public void runMain(GFRobot robot, double time)
    {
        switch (auto)
        {
            case straight:
            {
                BotLog.logD("cur", robot.mDrive.mPeriodicIO.currentvels.linearVel.norm()+ ", " + robot.mDrive.mPeriodicIO.currentpose.position.x);

                robot.mDrive.setOpenLoop(targetStraightPow,targetStraightPow,targetStraightPow,targetStraightPow);
                if (time - stateStartTime > accelTime)
                {
                    speed = inToCM(robot.mDrive.mPeriodicIO.currentvels.linearVel.norm());
                    endPos = robot.mDrive.mPeriodicIO.currentpose.position;

                    robot.mDrive.setOpenLoop(0,0,0,0);
                    robot.mDrive.localizer.leftBack.setPower(0);
                    robot.mDrive.localizer.rightBack.setPower(0);
                    robot.mDrive.localizer.leftFront.setPower(0);
                    robot.mDrive.localizer.rightFront.setPower(0);
                    robot.mLift.setClawPos(0);

                    stateStartTime = time;
                    auto = State.slipStraight;
                }
                break;
            }
            case slipStraight:
            {

                if (time - stateStartTime > slipTime)
                {
                    BotLog.logD("original", endPos.y + ", " + endPos.x);
                    BotLog.logD("final", robot.mDrive.mPeriodicIO.currentpose.position.y+ ", " + robot.mDrive.mPeriodicIO.currentpose.position.x);
                    BotLog.logD("speed", String.format("%.2f", speed));
                    BotLog.logD("straightSlip", String.format("%.2f", (inToCM(Math.sqrt(Math.pow(robot.mDrive.mPeriodicIO.currentpose.position.x - endPos.x,2)+Math.pow(robot.mDrive.mPeriodicIO.currentpose.position.y-endPos.y,2)))) / speed));

                    speed = 0;
                    stateStartTime = time;

                    auto = State.wait;

                }else{
                    BotLog.logD("cur", robot.mDrive.mPeriodicIO.currentvels.linearVel.norm()+ ", " + robot.mDrive.mPeriodicIO.currentpose.position.x);

                }
                break;
            }
            case wait:
            {
                stateStartTime = time;
                if (gamepad1.a) {
                    auto = State.strafe;
                    robot.mDrive.localizer.pinpoint.setPositionRR(new com.acmerobotics.roadrunner.Pose2d(0, 0, 0));
                }
                break;
            }
            case turn:
            {
                robot.mDrive.setOpenLoop(-targetAngularPow, -targetAngularPow, targetAngularPow, targetAngularPow);

                if (time - stateStartTime > accelTime)
                {
                    robot.mDrive.readPeriodicInputs(time);
                    speed = Math.toRadians(robot.mDrive.mPeriodicIO.angularVelocity.zRotationRate);

                    robot.mDrive.setOpenLoop(0,0,0,0);

                    endAngle = Rotation2d.fromRadians(robot.mDrive.mPeriodicIO.currentpose.heading.toDouble());

                    stateStartTime = time;
                    auto = State.slipTurn;
                }
                break;
            }
            case slipTurn:
            {
                if (time - stateStartTime > slipTime)
                {
                    BotLog.logD("original", String.format("%.2f", endAngle.getDegrees()));
                    BotLog.logD("final", String.format("%.2f", Math.toDegrees(robot.mDrive.mPeriodicIO.currentpose.heading.toDouble())));
                    BotLog.logD("speed", String.format("%.2f", Math.toDegrees(speed)));

                    BotLog.logD("turnSlip", String.format("%.2f", endAngle.inverse().rotateBy(Rotation2d.fromRadians(robot.mDrive.mPeriodicIO.currentpose.heading.toDouble())).getRadians() / speed));

                    auto = State.finished;
                }
                break;
            }
            case strafe:
            {
                robot.mDrive.setOpenLoop(targetStrafePow, -targetAngularPow, -targetAngularPow, targetAngularPow);

                if (time - stateStartTime > accelTime)
                {
                    speed = inToCM(robot.mDrive.mPeriodicIO.currentvels.linearVel.norm());
                    endPos = robot.mDrive.mPeriodicIO.currentpose.position;

                    robot.mDrive.setOpenLoop(0,0,0,0);

                    stateStartTime = time;
                    auto = State.slipStrafe;
                }
                break;
            }
            case slipStrafe:
            {
                if (time - stateStartTime > slipTime)
                {
                    BotLog.logD("original", endPos.y + ", " + endPos.x);
                    BotLog.logD("final", robot.mDrive.mPeriodicIO.currentpose.position.y+ ", " + robot.mDrive.mPeriodicIO.currentpose.position.x);
                    BotLog.logD("speed", String.format("%.2f", speed));
                    BotLog.logD("straightSlip", String.format("%.2f", (inToCM(Math.sqrt(Math.pow(robot.mDrive.mPeriodicIO.currentpose.position.x - endPos.x,2)+Math.pow(robot.mDrive.mPeriodicIO.currentpose.position.y-endPos.y,2)))) / speed));
                    speed = 0;
                    stateStartTime = time;
                    auto = State.turn;
                }
                break;
            }
            case finished:
            {
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