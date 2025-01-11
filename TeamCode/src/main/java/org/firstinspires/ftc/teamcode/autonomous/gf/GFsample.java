package org.firstinspires.ftc.teamcode.autonomous.gf;

import static org.firstinspires.ftc.teamcode.teleop.BSTEMTELE.firstwait;
import static org.firstinspires.ftc.teamcode.teleop.BSTEMTELE.intakewait;
import static org.firstinspires.ftc.teamcode.teleop.BSTEMTELE.secondwait;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.autonomous.rr.Drawing;
import org.firstinspires.ftc.teamcode.teleop.BSTEMTELE;

import java.util.ArrayList;

@Config
@Autonomous(name="4 sample")
public class GFsample extends OldAutoMaster {

    private boolean transferready = false;
    private ElapsedTime transfertimer = new ElapsedTime();
    private ElapsedTime generaltimer = new ElapsedTime();


    private enum State {
        bucket1,
        pick1,
        retract1,
        transfer1,
        bucket2,
        pick2,
        retract2,
        transfer2,
        bucket3,
        pick3,
        retract3,
        transfer3,
        bucket4,
        park,
        finished
    }
    public static int gox = 0;
    public static int goy = 0;
    private State auto = State.bucket1;

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

    private Path park()
    {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(-60, 17, 0, 0, 0, 0, 0, 0));
        allPoints.add(new CurvePoint(-48, 48, 1, 1, 40, 40, Math.toRadians(60), 1));
        allPoints.add(new CurvePoint(-35, 60, 1, 1, 40, 40, Math.toRadians(60), 1));
        allPoints.add(new CurvePoint(-23.5, 60, 1, 1, 40, 40, Math.toRadians(60), 1));

        return new Path(allPoints, false);
    }

    private Path pick1()
    {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(7.25, 35, 0, 0, 0, 0, 0, 0));
        allPoints.add(new CurvePoint(24, 35, 1, 1, 40, 40, Math.toRadians(60), 0.6));
        allPoints.add(new CurvePoint(30.5, 30.5, 1, 1, 40, 40, Math.toRadians(60), 0.6));
        allPoints.add(new CurvePoint(30.6, 30.3, 1, 1, 40, 40, Math.toRadians(60), 0.6));

        return new Path(allPoints, false, Math.toRadians(180));
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
        robot.mDrive.localizer.pose = new com.acmerobotics.roadrunner.Pose2d(-48+7.25,7.25,Math.toRadians(90));
        robot.mDrive.localizer.pinpoint.setPosition(new com.acmerobotics.roadrunner.Pose2d(-48+7.25, 7.25, Math.toRadians(90)));

    }

    @Override
    public void runMain(GFRobot robot, double time)
    {



        if (!start) {
            robot.teleopInit();
            start = true;
            robot.mDrive.setWantGFPos(-60,17,Math.toRadians(60),1, 1    );
            robot.mIntake.setExtendoPos(0,time);
            robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRAP.getVal());
            robot.mDeposit.autoInit();

        }


        switch (auto)
        {
            case bucket1:
            {
                if (robot.mDrive.isDoneWithGF()) {
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(),time);
                    if (robot.mLift.closeEnough()) {
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal());
                        robot.mDeposit.setDiffyPos(-30,0);
                        if (test.seconds() > 0.6) {
                            auto = State.pick1;
                            robot.mDrive.setWantGFPos(-60, 17, Math.toRadians(70), 1, 1);
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(),time);
                            robot.mDeposit.setDiffyPos(25,-90);
                        }else if (test.seconds() > 0.3) {
                            robot.mDeposit.setClawPos(0);
                        }
                    }
                    else{
                        test.reset();
                    }
                }else{
                    test.reset();
                }
                break;
            }
            case pick1:
            {
                if (robot.mDrive.isDoneWithGF()){
                    if (!robot.mIntake.closeEnough()){
                        test.reset();
                    }
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal());
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), time);
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    robot.mIntake.setIntakeOpenLoop(1);
                    if (test.seconds()>0.5) {
                        auto = State.retract1;
                        robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), time);
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                        robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
                        robot.mIntake.setIntakeOpenLoop(0);
                    }
                }else{
                    test.reset();
                }
                break;
            }
            case retract1:
            {
                if (robot.mDrive.isDoneWithGF()&&robot.mIntake.closeEnough()){
                    robot.mDrive.setWantGFPos(-60, 17, Math.toRadians(60), 1, 1);
                    transferready = false;
                    robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
                    generaltimer.reset();
                    transfertimer.reset();
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                    robot.mDeposit.setClawPos(0);
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), time);
                    if (test.seconds()>0.1) {
                        auto = State.transfer1;
                    }
                }else{
                    test.reset();
                }
                break;
            }
            case transfer1:
            {
                if (robot.mLift.closeEnough()&&robot.mIntake.closeEnough()&&!transferready){
                    robot.mIntake.setExtendoOpenLoop(-0.1);
                    robot.mIntake.setGatePos(Intake.GATE_POS.OPEN.getVal());
                    if (transfertimer.seconds()>firstwait+0.2){

                        robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), time);
                        transferready = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRAP.getVal());
                        transfertimer.reset();
                        robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), time);

                    }else if (transfertimer.seconds()>intakewait){
                        robot.mIntake.setIntakeOpenLoop(0.7);
                    }
                }else if (transferready){
                    if (robot.mLift.getLiftTicks()<135){
                        robot.mDeposit.setClawPos(1);
                        if (transfertimer.seconds()>secondwait) {
                            auto = State.bucket2;
                            robot.mIntake.setIntakeOpenLoop(0);
                            transfertimer.reset();
                            robot.mIntake.setGatePos(Intake.GATE_POS.OPEN.getVal());
                            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.TRANSFER.getVal(), time);
                            robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), time);

                        }
                    }else{
                        transfertimer.reset();
                    }

                } else {
                    transfertimer.reset();
                    robot.mDeposit.setClawPos(0);
                }
                break;
            }

            case bucket2:
            {
                if (robot.mLift.getLiftTicks()<650){
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
                    robot.mDeposit.setDiffyPos(0, -90);
                }
                if (robot.mLift.closeEnough()) {
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal());
                    robot.mDeposit.setDiffyPos(-30,0);


                    if (robot.mDrive.isDoneWithGF()) {
                        if (test.seconds() > 0.6) {
                            auto = State.pick2;
                            robot.mDrive.setWantGFPos(-60, 17, Math.toRadians(90), 1, 1);
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(),time);

                            robot.mDeposit.setDiffyPos(25,-90);
                        }else if (test.seconds() > 0.3) {
                            robot.mDeposit.setClawPos(0);
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setIntakeOpenLoop(0);
                            robot.mIntake.setExtendoPos(0, time);
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                        }
                    }else{
                        test.reset();
                    }
                }else{
                    test.reset();
                }
                break;
            }
            case pick2:
            {
                if (robot.mDrive.isDoneWithGF()){
                    if (!robot.mIntake.closeEnough()){
                        test.reset();
                    }
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal());
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), time);
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    robot.mIntake.setIntakeOpenLoop(1);
                    if (test.seconds()>0.5) {
                        auto = State.retract2;
                        robot.mIntake.setIntakeOpenLoop(0);

                        robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), time);
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                        robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
                    }
                }else{
                    test.reset();
                }
                break;
            }
            case retract2:
            {
                if (robot.mDrive.isDoneWithGF()&&robot.mIntake.closeEnough()){
                    transferready = false;
                    robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
                    generaltimer.reset();
                    transfertimer.reset();
                    robot.mDrive.setWantGFPos(-60, 17, Math.toRadians(60), 1, 1);

                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                    robot.mDeposit.setClawPos(0);
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), time);
                    if (test.seconds()>0.1) {
                        auto = State.transfer2;
                    }
                }else{
                    test.reset();
                }
                break;
            }
            case transfer2:
            {
                if (robot.mLift.closeEnough()&&robot.mIntake.closeEnough()&&!transferready){
                    robot.mIntake.setExtendoOpenLoop(-0.1);
                    robot.mIntake.setGatePos(Intake.GATE_POS.OPEN.getVal());
                    if (transfertimer.seconds()>firstwait+0.2){

                        robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), time);
                        transferready = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRAP.getVal());
                        transfertimer.reset();
                        robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), time);

                    }else if (transfertimer.seconds()>intakewait){
                        robot.mIntake.setIntakeOpenLoop(0.7);
                    }
                }else if (transferready){
                    if (robot.mLift.getLiftTicks()<135){
                        robot.mDeposit.setClawPos(1);
                        if (transfertimer.seconds()>secondwait) {
                            auto = State.bucket3;
                            robot.mIntake.setIntakeOpenLoop(0);
                            transfertimer.reset();
                            robot.mIntake.setGatePos(Intake.GATE_POS.OPEN.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(),time);
                            robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), time);

                        }
                    }else{
                        transfertimer.reset();
                    }

                } else {
                    transfertimer.reset();
                    robot.mDeposit.setClawPos(0);
                }
                break;
            }
            case bucket3:
            {
                if (robot.mLift.getLiftTicks()<650){
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
                    robot.mDeposit.setDiffyPos(0, -90);
                }
                if (robot.mLift.closeEnough()) {
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal());
                    robot.mDeposit.setDiffyPos(-30,0);


                    if (robot.mDrive.isDoneWithGF()) {
                        if (test.seconds() > 0.6) {
                            auto = State.pick3;
                            robot.mDrive.setWantGFPos(-60, 17, Math.toRadians(110), 1, 1);
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(),time);

                            robot.mDeposit.setDiffyPos(25,-90);
                        }if (test.seconds() > 0.3) {
                            robot.mDeposit.setClawPos(0);
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setIntakeOpenLoop(0);
                            robot.mIntake.setExtendoPos(0, time);
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                        }
                    }else{
                        test.reset();
                    }
                }else{
                    test.reset();
                }
                break;
            }
            case pick3:
            {
                if (robot.mDrive.isDoneWithGF()){
                    if (!robot.mIntake.closeEnough()){
                        test.reset();
                    }
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal());
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), time);
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    robot.mIntake.setIntakeOpenLoop(1);

                    if (test.seconds()>0.5) {
                        auto = State.retract3;
                        robot.mIntake.setIntakeOpenLoop(0);

                        robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), time);
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                        robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
                    }
                }else{
                    test.reset();
                }
                break;
            }
            case retract3:
            {
                if (robot.mDrive.isDoneWithGF()&&robot.mIntake.closeEnough()){
                    transferready = false;
                    robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
                    generaltimer.reset();
                    transfertimer.reset();
                    robot.mDrive.setWantGFPos(-60, 17, Math.toRadians(60), 1, 1);

                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                    robot.mDeposit.setClawPos(0);
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), time);
                    if (test.seconds()>0.1) {
                        auto = State.transfer3;
                    }
                }else{
                    test.reset();
                }
                break;
            }
            case transfer3:
            {
                if (robot.mLift.closeEnough()&&robot.mIntake.closeEnough()&&!transferready){
                    robot.mIntake.setExtendoOpenLoop(-0.1);
                    robot.mIntake.setGatePos(Intake.GATE_POS.OPEN.getVal());
                    if (transfertimer.seconds()>firstwait+0.2){
                        robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), time);
                        transferready = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRAP.getVal());
                        transfertimer.reset();
                        robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), time);

                    }else if (transfertimer.seconds()>intakewait){
                        robot.mIntake.setIntakeOpenLoop(0.7);
                    }
                }else if (transferready){
                    if (robot.mLift.getLiftTicks()<135){
                        robot.mDeposit.setClawPos(1);
                        if (transfertimer.seconds()>secondwait) {
                            auto = State.bucket4;
                            robot.mIntake.setIntakeOpenLoop(0);
                            transfertimer.reset();
                            robot.mIntake.setGatePos(Intake.GATE_POS.OPEN.getVal());
                            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.TRANSFER.getVal(), time);
                            robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(),time);

                        }
                    }else{
                        transfertimer.reset();
                    }

                } else {
                    transfertimer.reset();
                    robot.mDeposit.setClawPos(0);
                }
                break;
            }
            case bucket4:
            {
                if (robot.mLift.getLiftTicks()<650){
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
                    robot.mDeposit.setDiffyPos(0, -90);
                }
                if (robot.mLift.closeEnough()) {
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal());
                    robot.mDeposit.setDiffyPos(-30,0);


                    if (robot.mDrive.isDoneWithGF()) {
                        if (test.seconds() > 0.6) {
                            auto = State.finished;
                            robot.mDrive.setWantGFPath(park());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.DOWN.getVal(),time);
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOEND.getVal());
                            robot.mDeposit.setDiffyPos(25,-90);
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                            robot.mIntake.setExtendoPos(0,time);
                        }if (test.seconds() > 0.3) {
                            robot.mDeposit.setClawPos(0);
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setIntakeOpenLoop(0);
                            robot.mIntake.setExtendoPos(0, time);
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                        }
                    }else{
                        test.reset();
                    }
                }else{
                    test.reset();
                }
                break;
            }
            case finished:
            {
                if (robot.mLift.closeEnough()){
                    robot.mLift.setOpenLoop(-0.8);
                }
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