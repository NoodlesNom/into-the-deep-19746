//package org.firstinspires.ftc.teamcode.autonomous.gf;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Robot;
//import org.firstinspires.ftc.teamcode.autonomous.gf.Pose2d;
//import org.firstinspires.ftc.teamcode.autonomous.gf.Rotation2d;
//import org.firstinspires.ftc.teamcode.autonomous.gf.Translation2d;
//import org.firstinspires.ftc.teamcode.autonomous.gf.GFMath;
//import org.firstinspires.ftc.teamcode.lib.physics.DriveCharacterization;
//import org.firstinspires.ftc.teamcode.util.BotLog;
//import org.firstinspires.ftc.teamcode.util.util;
//import org.firstinspires.ftc.teamcode.Subsystems.Drive;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@Autonomous(name="slip tuner")
//public class SlipTuner extends OldAutoMaster {
//
//    private enum State {
//        straight,
//        slipStraight,
//        turn,
//        slipTurn,
//        finished
//    }
//
//    private State auto = State.straight;
//
//    private Rotation2d endAngle;
//    private double endPos;
//    private double speed = 0;
//
//    private double targetAngularPow = .4;
//    private double targetStraightPow = .5;
//
//    private int accelTime = 3;
//    private int slipTime = 1;
//
//    private double stateStartTime = 0;
//
//    @Override
//    public void setStartPose(Robot robot)
//    {
//        robot.mDrive.setStartPose(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
//    }
//
//    @Override
//    public void runMain(Robot robot, double time)
//    {
//        switch (auto)
//        {
//            case straight:
//            {
//                robot.mDrive.setOpenLoop(new Drive.DriveSignal(targetStraightPow, targetStraightPow));
//                if (time - stateStartTime > accelTime)
//                {
//                    speed = GFMath.inToCM(robot.mDrive.getLinearVelocityInches());
//                    endPos = robot.mDrive.getXPos();
//
//                    robot.mDrive.setOpenLoop(Drive.DriveSignal.BRAKE);
//
//                    stateStartTime = time;
//                    auto = State.slipStraight;
//                }
//                break;
//            }
//            case slipStraight:
//            {
//                if (time - stateStartTime > slipTime)
//                {
//                    BotLog.logD("original", String.format("%.2f", endPos));
//                    BotLog.logD("final", String.format("%.2f", robot.mDrive.getXPos()));
//                    BotLog.logD("speed", String.format("%.2f", speed));
//                    BotLog.logD("straightSlip", String.format("%.2f", (GFMath.inToCM(robot.mDrive.getXPos() - endPos)) / speed));
//
//                    speed = 0;
//                    stateStartTime = time;
//                    auto = State.turn;
//                }
//                break;
//            }
//            case turn:
//            {
//                robot.mDrive.setOpenLoop(new Drive.DriveSignal(-targetAngularPow, targetAngularPow));
//
//                if (time - stateStartTime > accelTime)
//                {
//                    robot.mDrive.readIMU(time);
//                    speed = Math.toRadians(robot.mDrive.getAngularVelocityMotor());
//
//                    robot.mDrive.setOpenLoop(Drive.DriveSignal.BRAKE);
//
//                    endAngle = robot.mDrive.getHeading();
//
//                    stateStartTime = time;
//                    auto = State.slipTurn;
//                }
//                break;
//            }
//            case slipTurn:
//            {
//                if (time - stateStartTime > slipTime)
//                {
//                    BotLog.logD("original", String.format("%.2f", endAngle.getDegrees()));
//                    BotLog.logD("final", String.format("%.2f", robot.mDrive.getHeading().getDegrees()));
//                    BotLog.logD("speed", String.format("%.2f", Math.toDegrees(speed)));
//
//                    BotLog.logD("turnSlip", String.format("%.2f", endAngle.inverse().rotateBy(robot.mDrive.getHeading()).getRadians() / speed));
//
//                    auto = State.finished;
//                }
//                break;
//            }
//            case finished:
//            {
//                break;
//            }
//
//        }
//        robot.update(time);
//    }
//}