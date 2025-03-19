package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.gf.OldAutoMaster.team;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.autonomous.rr.localizer.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.StickyButton;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "6 spec and PULL", group = "Autonomous")


public class spec6pull extends LinearOpMode {
    public Robot robot;
    public ElapsedTime timer;
    ElapsedTime generaltimer = new ElapsedTime();
    ElapsedTime pulltimer = new ElapsedTime();
    boolean debug = true;

    boolean retract = false;

    private boolean failed = true;
    private boolean genericboolean = false;
    public static double blockx1 = 0;
    public static int blocky1 = 2;
    public static double blockx2 = 0;
    public static int blocky2 = 2;
    public static double blockx3 = 0;
    public static int blocky3 = 2;
    private PinpointDrive drive;

    private StickyButton intakeposup = new StickyButton();
    private StickyButton intakeposdown = new StickyButton();
    private StickyButton intakeposleft = new StickyButton();
    private StickyButton switchblock = new StickyButton();
    private StickyButton intakeposright = new StickyButton();
    ElapsedTime loopTimer = new ElapsedTime();
    Deadline logging = new Deadline(100, TimeUnit.MILLISECONDS);

    public class robotController {

        public class Update implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.update(timer.seconds());
                if(debug) {
                    if(logging.hasExpired())
                    {
                        logging.reset();
                        String debugMsg = robot.getTelem(timer.seconds());
                        debugMsg += String.format("loopTime = %4.0f\n", loopTimer.milliseconds());
                        debugMsg+="\nlift height: "+ robot.mLift.getLiftTicks();
                        telemetry.addLine(debugMsg);
                        BotLog.logD("BSDbg", debugMsg);
                    }
                    loopTimer.reset();
                }
                telemetry.update();
                return true;
            }
        }

        public Action updateRobot() {
            return new Update();
        }

        public class OpenClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setClawPos(3);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }

        public class CloseClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setClawPos(1);
                //robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class PivotDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal(), 800 , new double[] {1,2,2,2,2,2,2,1,1,1});
                return false;
            }
        }

        public Action pivotDown() {
            return new PivotDown();
        }

        public class PivotUp implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPEC.getVal());
                return false;
            }
        }
        public Action pivotUp() {
            return new PivotUp();
        }

        public class TellRetract implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                retract=true;
                return false;
            }
        }
        public Action tellRetract() {
            return new TellRetract();
        }

        public class RetractReset implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                retract=false;
                return false;
            }
        }
        public Action retractReset() {
            return new RetractReset();
        }

        public class DiffyIntake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setDiffyPos(80,-90);
                return false;
            }
        }
        public Action diffyIntake() {
            return new DiffyIntake();
        }

        public class DiffyPlace implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setDiffyPos(90,90);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOCLEAR.getVal());
                BotLog.logD("BSDbg", "END OF PLACING TRAJECTORY AHHHHHHHHHHH");
                return false;
            }
        }
        public Action diffyPlace() {
            return new DiffyPlace();
        }

        public class Pull implements Action {
            private boolean stopresetting = false;
            private boolean reject = false;
            private boolean detected = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if ((robot.mIntake.detectedBlue()&&team.name().equals("RED"))||(robot.mIntake.detectedRed()&&team.name().equals("BLUE"))){
                    robot.mIntake.setIntakeOpenLoop(-0.8);
                    reject = true;
                }
                if ((robot.mIntake.detectedBlue()&&team.name().equals("BLUE"))||(robot.mIntake.detectedRed()&&team.name().equals("RED"))||robot.mIntake.detectedYellow()){
                    if (!detected){
                        generaltimer.reset();
                    }
                    detected = true;
                    BotLog.logD("deteted", detected+"");
                }
                if (detected){
                    robot.mIntake.pwmenable();

                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());

                    if (generaltimer.seconds() > 0.2) {
                        robot.mIntake.pwmenable();

                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());

                        return false;
                    } else if (generaltimer.seconds() > 0.1) {

                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        robot.mIntake.setIntakeOpenLoop(-0.8);
                    }else{
                        robot.mIntake.setIntakeOpenLoop(1);
                        robot.mIntake.setOutputLimits(-1, 1);
                        robot.mIntake.setClawPos(1);
                    }
                }else {
                    if (generaltimer.seconds() > 1.5) {
                        robot.mIntake.pwmenable();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                        robot.mIntake.setIntakeOpenLoop(0);
                        return false;

                    } else if (generaltimer.seconds() > 1.4) {
                        robot.mIntake.pwmenable();
                        robot.mIntake.setOutputLimits(-1, 1);
                        robot.mIntake.setClawPos(1);
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                    } else if (generaltimer.seconds() > 0.6) {

                        robot.mIntake.setExtendoTicks((int) (((blocky1 + 5)/0.033)), timer.seconds());
                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(1);
                        }
                    } else if (generaltimer.seconds() > 0.4) {
                        //robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());



                    } else if (generaltimer.seconds() > 0.15) {
                        robot.mIntake.pwmdisable();
                    } else if (robot.mIntake.closeEnough() && genericboolean) {
                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(0.6);
                            robot.mIntake.setExtendoOpenLoop(0);
                        }

                        stopresetting = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());

                    } else if (!genericboolean && robot.mIntake.getTargetExtendoIdx() != Intake.EXTEND_POS.INTAKING.getVal()) {
                        robot.mIntake.setExtendoTicks((int) (((blocky1)/0.033)), timer.seconds());

                        genericboolean = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                        robot.mIntake.setOutputLimits(-1, 1);

                    }
                    if (!stopresetting) {
                        generaltimer.reset();
                    }
                }
                return true;
            }
        }

        public Action pull() {
            return new Pull();
        }
        public class PullHopeFirst implements Action {
            private boolean stopresetting = false;
            private boolean reject = false;
            private boolean detected = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if ((robot.mIntake.detectedBlue()&&team.name().equals("RED"))||(robot.mIntake.detectedRed()&&team.name().equals("BLUE"))){
                    robot.mIntake.setIntakeOpenLoop(-0.8);
                    reject = true;
                }
                if ((robot.mIntake.detectedBlue()&&team.name().equals("BLUE"))||(robot.mIntake.detectedRed()&&team.name().equals("RED"))||robot.mIntake.detectedYellow()){
                    if (!detected){
                        generaltimer.reset();
                    }
                    detected = true;
                    BotLog.logD("deteted", detected+"");
                }
                if (detected){
                    robot.mIntake.pwmenable();

                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());

                    if (generaltimer.seconds() > 0.2) {
                        robot.mIntake.pwmenable();

                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());

                        return false;
                    } else if (generaltimer.seconds() > 0.1) {


                        robot.mIntake.setIntakeOpenLoop(-0.8);
                    }else{
                        robot.mIntake.pwmenable();
                        robot.mIntake.setIntakeOpenLoop(1);
                        robot.mIntake.setOutputLimits(-1, 1);
                        robot.mIntake.setClawPos(1);
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    }
                }else {
                    if (drive.pose.position.y<34&&retract){
                        robot.mIntake.pwmenable();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                        robot.mIntake.setIntakeOpenLoop(0);
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        robot.mIntake.setOutputLimits(-1, 1);
                        return false;
                    }
                    if (generaltimer.seconds() > 1.3) {
                        robot.mIntake.pwmenable();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                        robot.mIntake.setIntakeOpenLoop(0);
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        return false;

                    } else if (generaltimer.seconds() > 1.2) {
                        robot.mIntake.pwmenable();
                        robot.mIntake.setOutputLimits(-1, 1);
                        robot.mIntake.setClawPos(1);
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                    } else if (generaltimer.seconds() > 0.3) {

                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(1);
                        }
                        //} else if (generaltimer.seconds() > 0.3) {
                        //robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());



                    } else if (generaltimer.seconds() > 0.1) {
                        robot.mIntake.setExtendoTicks((int) (((blocky2+7)/0.033)), timer.seconds());
                    } else if (pulltimer.seconds()>1 && genericboolean) {
                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(1);
                            robot.mIntake.setExtendoOpenLoop(0);
                        }

                        stopresetting = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());

                    } else if (!genericboolean && robot.mIntake.getTargetExtendoIdx() != Intake.EXTEND_POS.INTAKING.getVal()) {
                        robot.mIntake.setExtendoTicks((int) (((blocky2)/0.033)), timer.seconds());

                        genericboolean = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                        robot.mIntake.setOutputLimits(-1, 1);

                    }
                    if (!stopresetting) {
                        generaltimer.reset();
                    }
                }
                return true;
            }
        }

        public Action pullHopeFirst() {
            return new PullHopeFirst();
        }

        public class ClearHopeFirst implements Action {
            private boolean stopresetting = false;
            private boolean reject = false;
            private boolean detected = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if ((robot.mIntake.detectedBlue()&&team.name().equals("RED"))||(robot.mIntake.detectedRed()&&team.name().equals("BLUE"))){
                    robot.mIntake.setIntakeOpenLoop(-0.8);
                    reject = true;
                }
                if ((robot.mIntake.detectedBlue()&&team.name().equals("BLUE"))||(robot.mIntake.detectedRed()&&team.name().equals("RED"))||robot.mIntake.detectedYellow()){
                    if (!detected){
                        generaltimer.reset();
                    }
                    detected = true;
                    BotLog.logD("deteted", detected+"");
                }
                if (detected){
                    robot.mIntake.pwmenable();

                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());

                    if (generaltimer.seconds() > 0.2) {
                        robot.mIntake.pwmenable();

                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());

                        return false;
                    } else if (generaltimer.seconds() > 0.1) {


                        robot.mIntake.setIntakeOpenLoop(-0.8);
                    }else{
                        robot.mIntake.pwmenable();
                        robot.mIntake.setIntakeOpenLoop(1);
                        robot.mIntake.setOutputLimits(-1, 1);
                        robot.mIntake.setClawPos(1);
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    }
                }else {
                    if (drive.pose.position.y<34&&retract){
                        robot.mIntake.pwmenable();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                        robot.mIntake.setIntakeOpenLoop(0);
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        robot.mIntake.setOutputLimits(-1, 1);
                        return false;
                    }
                    if (generaltimer.seconds() > 1.3) {
                        robot.mIntake.pwmenable();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                        robot.mIntake.setIntakeOpenLoop(0);
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        return false;

                    } else if (generaltimer.seconds() > 1.2) {
                        robot.mIntake.pwmenable();
                        robot.mIntake.setOutputLimits(-1, 1);
                        robot.mIntake.setClawPos(1);
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.BLOCKCLEAR.getVal());
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                    } else if (generaltimer.seconds() > 0.3) {

                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(1);
                        }
                        //} else if (generaltimer.seconds() > 0.3) {
                        //robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());



                    } else if (generaltimer.seconds() > 0.1) {
                        //robot.mIntake.setExtendoTicks((int) (((blockx2+7)/0.033)), timer.seconds());
                    } else if (pulltimer.seconds()>1 && genericboolean) {
                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(0.6);
                            robot.mIntake.setExtendoOpenLoop(0);
                        }

                        stopresetting = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.BLOCKCLEAR.getVal());

                    } else if (!genericboolean && robot.mIntake.getTargetExtendoIdx() != Intake.EXTEND_POS.INTAKING.getVal()) {
                        robot.mIntake.setExtendoTicks((int) (((blocky2+1)/0.033)), timer.seconds());

                        genericboolean = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                        robot.mIntake.setOutputLimits(-1, 1);

                    }
                    if (!stopresetting) {
                        generaltimer.reset();
                    }
                }
                return true;
            }
        }

        public Action clearHopeFirst() {
            return new ClearHopeFirst();
        }

        public class PullHopeSecond implements Action {
            private boolean stopresetting = false;
            private boolean reject = false;
            private boolean detected = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if ((robot.mIntake.detectedBlue()&&team.name().equals("RED"))||(robot.mIntake.detectedRed()&&team.name().equals("BLUE"))){
                    robot.mIntake.setIntakeOpenLoop(-0.8);
                    reject = true;
                }
                if ((robot.mIntake.detectedBlue()&&team.name().equals("BLUE"))||(robot.mIntake.detectedRed()&&team.name().equals("RED"))||robot.mIntake.detectedYellow()){
                    if (!detected){
                        generaltimer.reset();
                    }
                    detected = true;
                    BotLog.logD("deteted", detected+"");
                }
                if (detected){
                    robot.mIntake.pwmenable();

                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());

                    if (generaltimer.seconds() > 0.2) {
                        robot.mIntake.pwmenable();

                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());

                        return false;
                    } else if (generaltimer.seconds() > 0.1) {


                        robot.mIntake.setIntakeOpenLoop(-0.8);
                    }else{
                        robot.mIntake.pwmenable();
                        robot.mIntake.setIntakeOpenLoop(1);
                        robot.mIntake.setOutputLimits(-1, 1);
                        robot.mIntake.setClawPos(1);
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    }
                }else {
                    if (drive.pose.position.y<34&&retract){
                        robot.mIntake.pwmenable();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                        robot.mIntake.setIntakeOpenLoop(0);
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        robot.mIntake.setOutputLimits(-1, 1);
                        return false;
                    }
                    if (generaltimer.seconds() > 1.3) {
                        robot.mIntake.pwmenable();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                        robot.mIntake.setIntakeOpenLoop(0);
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        return false;

                    } else if (generaltimer.seconds() > 1.2) {
                        robot.mIntake.pwmenable();
                        robot.mIntake.setOutputLimits(-1, 1);
                        robot.mIntake.setClawPos(1);
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                    } else if (generaltimer.seconds() > 0.3) {

                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(1);
                        }
                    //} else if (generaltimer.seconds() > 0.3) {
                        //robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());



                    } else if (generaltimer.seconds() > 0.1) {
                        robot.mIntake.setExtendoTicks((int) (((blocky3+7)/0.033)), timer.seconds());
                    } else if (pulltimer.seconds()>1 && genericboolean) {
                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(1);
                            robot.mIntake.setExtendoOpenLoop(0);
                        }

                        stopresetting = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());

                    } else if (!genericboolean && robot.mIntake.getTargetExtendoIdx() != Intake.EXTEND_POS.INTAKING.getVal()) {
                        robot.mIntake.setExtendoTicks((int) (((blocky3)/0.033)), timer.seconds());

                        genericboolean = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                        robot.mIntake.setOutputLimits(-1, 1);

                    }
                    if (!stopresetting) {
                        generaltimer.reset();
                    }
                }
                return true;
            }
        }

        public Action pullHopeSecond() {
            return new PullHopeSecond();
        }

        public class ClearHopeSecond implements Action {
            private boolean stopresetting = false;
            private boolean reject = false;
            private boolean detected = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if ((robot.mIntake.detectedBlue()&&team.name().equals("RED"))||(robot.mIntake.detectedRed()&&team.name().equals("BLUE"))){
                    robot.mIntake.setIntakeOpenLoop(-0.8);
                    reject = true;
                }
                if ((robot.mIntake.detectedBlue()&&team.name().equals("BLUE"))||(robot.mIntake.detectedRed()&&team.name().equals("RED"))||robot.mIntake.detectedYellow()){
                    if (!detected){
                        generaltimer.reset();
                    }
                    detected = true;
                    BotLog.logD("deteted", detected+"");
                }
                if (detected){
                    robot.mIntake.pwmenable();

                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());

                    if (generaltimer.seconds() > 0.2) {
                        robot.mIntake.pwmenable();

                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());

                        return false;
                    } else if (generaltimer.seconds() > 0.1) {


                        robot.mIntake.setIntakeOpenLoop(-0.8);
                    }else{
                        robot.mIntake.pwmenable();
                        robot.mIntake.setIntakeOpenLoop(1);
                        robot.mIntake.setOutputLimits(-1, 1);
                        robot.mIntake.setClawPos(1);
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    }
                }else {
                    if (drive.pose.position.y<34&&retract){
                        robot.mIntake.pwmenable();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                        robot.mIntake.setIntakeOpenLoop(0);
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        robot.mIntake.setOutputLimits(-1, 1);
                        return false;
                    }
                    if (generaltimer.seconds() > 1.3) {
                        robot.mIntake.pwmenable();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                        robot.mIntake.setIntakeOpenLoop(0);
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        return false;

                    } else if (generaltimer.seconds() > 1.2) {
                        robot.mIntake.pwmenable();
                        robot.mIntake.setOutputLimits(-1, 1);
                        robot.mIntake.setClawPos(1);
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.BLOCKCLEAR.getVal());
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                    } else if (generaltimer.seconds() > 0.3) {

                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(1);
                        }
                        //} else if (generaltimer.seconds() > 0.3) {
                        //robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());



                    } else if (generaltimer.seconds() > 0.1) {
                        //robot.mIntake.setExtendoTicks((int) (((blockx2+7)/0.033)), timer.seconds());
                    } else if (pulltimer.seconds()>1 && genericboolean) {
                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(0.6);
                            robot.mIntake.setExtendoOpenLoop(0);
                        }

                        stopresetting = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.BLOCKCLEAR.getVal());

                    } else if (!genericboolean && robot.mIntake.getTargetExtendoIdx() != Intake.EXTEND_POS.INTAKING.getVal()) {
                        robot.mIntake.setExtendoTicks((int) (((blocky3+1)/0.033)), timer.seconds());

                        genericboolean = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                        robot.mIntake.setOutputLimits(-1, 1);

                    }
                    if (!stopresetting) {
                        generaltimer.reset();
                    }
                }
                return true;
            }
        }

        public Action clearHopeSecond() {
            return new ClearHopeSecond();
        }


        public class DiffyPlaceLast implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setDiffyPos(90,90);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECPUSH.getVal());
                BotLog.logD("BSDbg", "END OF PLACING TRAJECTORY AHHHHHHHHHHH");
                return false;
            }
        }
        public Action diffyPlaceLast() {
            return new DiffyPlaceLast();
        }

        public class subPrepareInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setClawPos(0);
                robot.mIntake.setExtendoTicks((int) ((blocky1 /0.033)), timer.seconds());
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                return false;
            }
        }

        public Action subPrepareInstant() {
            return new subPrepareInstant();
        }

        public class intakePrepareInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setClawPos(0);
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                return false;
            }
        }

        public Action intakePrepareInstant() {
            return new intakePrepareInstant();
        }
        public class DiffyRelease implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal());
                return false;
            }
        }
        public Action diffyRelease() {
            return new DiffyRelease();
        }

        public class LiftDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(0, timer.seconds());
                return !robot.mLift.closeEnough();
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }

        public class LiftIntakeInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(2, timer.seconds());
                return false;
            }
        }

        public Action liftIntakeInstant() {
            return new LiftIntakeInstant();
        }


        public class LiftClearInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.SPECCLEAR.getVal(), timer.seconds());
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                robot.mIntake.setIntakeOpenLoop(-0.9);
                return false;
            }
        }

        public Action liftClearInstant() {
            return new LiftClearInstant();
        }

        public class RobotReset implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.DOWN.getVal(), timer.seconds());
                robot.mIntake.setExtendoOpenLoop(-0.4);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
                robot.mDeposit.setDiffyPos(0,0);

                return false;
            }
        }

        public Action robotReset() {
            return new RobotReset();
        }

        public class Intakeing implements Action {

            private boolean intaken = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setOutputLimits(-1,0.8);
                if (robot.mLift.closeEnough()||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()){
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (generaltimer.seconds()>1||(intaken&&generaltimer.seconds()>0.15)){
                            robot.mIntake.setIntakeOpenLoop(0);

                            robot.mIntake.setExtendoPos(0, timer.seconds());
                            robot.mIntake.setOutputLimits(-1,1);
                            return false;
                        }else if  (intaken){
                            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                            robot.mIntake.setClawPos(1);
                        }else if (generaltimer.seconds()>0){
                            robot.mIntake.setIntakeOpenLoop(1);
                            robot.mIntake.setClawPos(0);
                            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                        }

                    }else{
                        generaltimer.reset();
                    }
                    if (((robot.mIntake.detectedBlue()&&team.name().equals("BLUE"))||(robot.mIntake.detectedRed()&&team.name().equals("RED")))&&!intaken){
                        intaken = true;
                        generaltimer.reset();
                    }
                }else{
                    generaltimer.reset();
                }
                return true;
            }
        }


        public Action intakeing() {
            return new Intakeing();
        }

        public class LiftPlaceInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(1, timer.seconds());
                return false;
            }
        }

        public Action liftPlaceInstant() {
            return new LiftPlaceInstant();
        }


        public class ClearWall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (generaltimer.seconds()>1){
                    generaltimer.reset();
                }else if (generaltimer.seconds()>0.4){
                    robot.mDeposit.setDiffyPos(-40, 113);
                    robot.mLift.setTargetPos(Lift.LIFT_POS.SPECIMEN_PLACE.getVal(), timer.seconds());
                    robot.mIntake.setIntakeOpenLoop(0);
                    return false;
                }else if (generaltimer.seconds()>0.2) {
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPEC.getVal(), 800 , new double[] {1,2,2,2,2,2,2,1,1,1});

                }else{
                    robot.mLift.setTargetPos(Lift.LIFT_POS.SPECCLEAR.getVal(), timer.seconds());
                    robot.mIntake.setIntakeOpenLoop(-0.9);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                }
                return true;
            }
        }

        public Action clearWall() {
            return new ClearWall();
        }

        public class IntakeReset implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                //robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                if (generaltimer.seconds()>0.6){
                    //robot.mIntake.zerofinish(timer.seconds());
                    return false;
                }else if (generaltimer.seconds()>0.5){
                    //robot.mIntake.setExtendoOpenLoop(0);
                    //robot.mIntake.rezero();
                    robot.mLift.setTargetPos(Lift.LIFT_POS.SPECINTAKE.getVal(), timer.seconds());
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal(), 900 , new double[] {1,2,2,2,2,2,2,1,1,1});
                    robot.mDeposit.setClawPos(3);

                    robot.mDeposit.setDiffyPos(80,-90);
                }else if (generaltimer.seconds()>0.4){
                    retract=true;
                }else{
                    //robot.mIntake.setExtendoOpenLoop(-0.5);
                }
                return true;
            }
        }

        public Action intakeReset() {
            return new IntakeReset();
        }

        public class ResetTimer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                generaltimer.reset();
                pulltimer.reset();
                genericboolean = false;
                return false;
            }
        }

        public Action resetTimer() {
            return new ResetTimer();
        }

        public class LiftUpInstant implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(2, timer.seconds());
                return false;
            }
        }

        public Action liftUpInstant() {
            return new LiftUpInstant();
        }

        public class SpecPlace implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.SPECIMEN_PLACE.getVal(), timer.seconds());
                robot.mDeposit.setDiffyPos(-40, 67);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPEC.getVal(), 600, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                return false;
            }
        }

        public Action specPlace() {
            return new SpecPlace();
        }

        public class PivotShoot implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                return false;
            }
        }

        public Action pivotShoot() {
            return new PivotShoot();
        }

        public class ExtendoPrepareInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setClawPos(0);
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                return false;
            }
        }

        public Action extendoPrepareInstant() {
            return new ExtendoPrepareInstant();
        }
        public class ExtendoInInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setClawPos(0);
                robot.mDeposit.setClawPos(0);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal());
                robot.mDeposit.setDiffyPos(80,-90);
                robot.mIntake.setExtendoPos(0, timer.seconds());
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                return false;
            }
        }

        public Action extendoInInstant() {
            return new ExtendoInInstant();
        }


        public class Shoot implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                robot.mDeposit.setDiffyPos(80,-90);
                if (!robot.mIntake.closeEnoughAuto()){
                    generaltimer.reset();
                }
                if (generaltimer.seconds()>0.4) {
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                    robot.mIntake.setIntakeOpenLoop(0);
                    //robot.mLift.setTargetPos(1, timer.seconds());

                    return false;
                }else  if (generaltimer.seconds()>0.1) {
                    robot.mIntake.setIntakeOpenLoop(-0.9);
                }else if   (robot.mIntake.closeEnoughAuto()){

                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setExtendoOpenLoop(-0.6);
                }else {
                    robot.mIntake.setClawPos(1);
                    robot.mIntake.setIntakeOpenLoop(-0.8);
                    robot.mLift.setTargetPos(2, timer.seconds());
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());

                }


                return true;
            }
        }


        public Action shoot() {
            return new Shoot();
        }
        public class ShootLast implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                if (!robot.mIntake.closeEnoughAuto()){
                    generaltimer.reset();
                }
                if (generaltimer.seconds()>0.3) {
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    robot.mIntake.setIntakeOpenLoop(0);

                    return false;
                }else  if   (robot.mIntake.closeEnoughAuto()){
                    robot.mIntake.setIntakeOpenLoop(-0.7);
                    robot.mIntake.setClawPos(0);
                }

                return true;
            }
        }

        public Action shootLast() {
            return new ShootLast();
        }

        public class LiftDownInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(0, timer.seconds());
                return false;
            }
        }

        public Action liftDownInstant() {
            return new LiftDownInstant();
        }
    }

    @Override
    public void runOpMode() {
        timer = new ElapsedTime();
        robotController controller = new robotController();
        Pose2d initialPose = new Pose2d((10-7.25), 7.25, Math.toRadians(90));
        //claw is 1.5 inches to the right
        robot = new Robot(this, initialPose , hardwareMap);
        drive = new PinpointDrive(this.hardwareMap, initialPose);

        // vision here that outputs position
        int visionOutputPosition = 1;



        Action sub1human1 = drive.actionBuilder(new Pose2d(39,6, Math.toRadians(90)))
                .setReversed(false)
                .afterTime(0.8,
                        new SequentialAction(
                                controller.resetTimer(),
                                controller.clearHopeFirst()
                        )
                )
                .setTangent(Math.toRadians(145))

                .splineToLinearHeading(new Pose2d(blockx2+2, 31 ,Math.toRadians(90)), Math.toRadians(145),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 80))

                .splineToLinearHeading(new Pose2d(blockx2, 36 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 80))
                .splineToLinearHeading(new Pose2d(blockx2, 40.5 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 80))
                .stopAndAdd(new SequentialAction(
                        controller.openClaw(),
                        controller.diffyPlace(),
                        controller.resetTimer()

                ))
                .afterTime(0.01,controller.intakeReset())
                .waitSeconds(0.01)
                .setReversed(true)
                .setTangent(Math.toRadians(90-180))
                .splineToLinearHeading(new Pose2d(33, 20 ,Math.toRadians(90)), Math.toRadians(145-180),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                .splineToLinearHeading(new Pose2d(36, 12 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                .splineToLinearHeading(new Pose2d(36, 6.5 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw()
                ))
                .waitSeconds(0.05)
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.liftClearInstant(),
                        controller.retractReset()
                ))
                .build();

        Action sub2human2 = drive.actionBuilder(new Pose2d(36,6.5, Math.toRadians(90)))
                .setReversed(false)
                .afterTime(0.8,
                        new SequentialAction(
                                controller.resetTimer(),
                                controller.pullHopeFirst()
                        )
                )
                .setTangent(Math.toRadians(145))

                .splineToLinearHeading(new Pose2d(blockx2+2, 31 ,Math.toRadians(90)), Math.toRadians(145),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 80))

                .splineToLinearHeading(new Pose2d(blockx2, 36 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 80))
                .splineToLinearHeading(new Pose2d(blockx2, 40.5 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 80))
                .stopAndAdd(new SequentialAction(
                        controller.openClaw(),
                        controller.diffyPlace(),
                        controller.resetTimer()

                ))
                .afterTime(0.01,controller.intakeReset())
                .waitSeconds(0.01)
                .setReversed(true)
                .setTangent(Math.toRadians(90-180))
                .splineToLinearHeading(new Pose2d(33, 20 ,Math.toRadians(90)), Math.toRadians(145-180),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                .splineToLinearHeading(new Pose2d(36, 12 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                .splineToLinearHeading(new Pose2d(36, 6.5 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw()
                ))
                .waitSeconds(0.05)
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.liftClearInstant(),
                        controller.retractReset()
                ))
                .build();
        Action sub3human3 = drive.actionBuilder(new Pose2d(36,6.5, Math.toRadians(90)))
                .setReversed(false)
                .afterTime(0.8,
                        new SequentialAction(
                                controller.resetTimer(),
                                controller.clearHopeSecond()
                        )
                )
                .setTangent(Math.toRadians(145))

                .splineToLinearHeading(new Pose2d(blockx3+2, 31 ,Math.toRadians(90)), Math.toRadians(145),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 80))

                .splineToLinearHeading(new Pose2d(blockx3, 36 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 80))
                .splineToLinearHeading(new Pose2d(blockx3, 40.5 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 80))
                .stopAndAdd(new SequentialAction(
                        controller.openClaw(),
                        controller.diffyPlace(),
                        controller.resetTimer()

                ))
                .afterTime(0.01,controller.intakeReset())
                .waitSeconds(0.01)
                .setReversed(true)
                .setTangent(Math.toRadians(90-180))
                .splineToLinearHeading(new Pose2d(33, 20 ,Math.toRadians(90)), Math.toRadians(145-180),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                .splineToLinearHeading(new Pose2d(36, 12 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                .splineToLinearHeading(new Pose2d(36, 6.5 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw()
                ))
                .waitSeconds(0.05)
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.liftClearInstant(),
                        controller.retractReset()
                ))
                .build();
        Action sub4human4 = drive.actionBuilder(new Pose2d(36,6.5, Math.toRadians(90)))
                .setReversed(false)
                .afterTime(0.8,
                        new SequentialAction(
                                controller.resetTimer(),
                                controller.pullHopeSecond()
                        )
                )
                .setTangent(Math.toRadians(145))

                .splineToLinearHeading(new Pose2d(blockx3+2, 31 ,Math.toRadians(90)), Math.toRadians(145),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 80))

                .splineToLinearHeading(new Pose2d(blockx3, 36 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 80))
                .splineToLinearHeading(new Pose2d(blockx3, 40.5 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 80))
                .stopAndAdd(new SequentialAction(
                        controller.openClaw(),
                        controller.diffyPlace(),
                        controller.resetTimer()

                ))
                .afterTime(0.01,controller.intakeReset())
                .waitSeconds(0.01)
                .setReversed(true)
                .setTangent(Math.toRadians(90-180))
                .splineToLinearHeading(new Pose2d(33, 20 ,Math.toRadians(90)), Math.toRadians(145-180),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                .splineToLinearHeading(new Pose2d(36, 12 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                .splineToLinearHeading(new Pose2d(36, 6.5 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw()
                ))
                .waitSeconds(0.05)
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.liftClearInstant(),
                        controller.retractReset()
                ))
                .build();

        Action sub5 = drive.actionBuilder(new Pose2d(36,6.5, Math.toRadians(90)))
                .setReversed(false)
                .setTangent(Math.toRadians(145))
                .splineToLinearHeading(new Pose2d(5, 31 ,Math.toRadians(90)), Math.toRadians(145),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                .splineToLinearHeading(new Pose2d(3, 36 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                .splineToLinearHeading(new Pose2d(3, 40.5 ,Math.toRadians(90)), Math.toRadians(90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                .stopAndAdd(new SequentialAction(
                        controller.openClaw(),
                        controller.diffyPlaceLast(),
                        controller.resetTimer()
                ))
                .build();

        Action human5 = drive.actionBuilder(new Pose2d(3,40.5, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(48, 12), Math.toRadians(-15),new TranslationalVelConstraint(90 ), new ProfileAccelConstraint(-100, 90))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw()
                ))
                .waitSeconds(0.2)
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.robotReset()
                ))

                .build();








        //.stopAndAdd()




        // actions that need to happen on init; for instance, a claw tightening.

        int block = 0;
        while (!isStopRequested() && !opModeIsActive()) {
            intakeposleft.update(gamepad1.dpad_left);
            intakeposright.update(gamepad1.dpad_right);
            intakeposup.update(gamepad1.dpad_up);
            intakeposdown.update(gamepad1.dpad_down);
            switchblock.update(gamepad1.x);
            robot.update(timer.seconds());
            timer.reset();
            robot.autoInit();
            robot.mDeposit.setLiveLed(team);
            telemetry.addLine("PRESS A (BOTTOM) FOR BLUE");
            telemetry.addLine("PRESS B (RIGHT) FOR RED");
            telemetry.addLine("CURRENT COLOR: " + team.name());
            if (gamepad1.a){
                team = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            }
            if (gamepad1.b){
                team = RevBlinkinLedDriver.BlinkinPattern.RED;
            }
            if (switchblock.getState()){
                block++;
            }
            telemetry.addLine("USE DPAD TO ADJUST INTAKE POS BY INCHES");
            telemetry.addLine("PRESS X TO ADJUST WHICH INTAKE");
            //telemetry.addLine("IF DONT ADJUST INTAKE 2, WILL BE SAME AS 1. 2 ADJUSTED = " +has_adjust_2);
            if (block%3==0){
                telemetry.addLine("ADJUSTING FIRST INTAKE");
                if (intakeposup.getState()){
                    blocky1 +=1;
                }else if (intakeposdown.getState()){
                    blocky1 -=1;
                }else if (intakeposleft.getState()){
                    blockx1 -=1;
                }else if (intakeposright.getState()){
                    blockx1 +=1;
                }
            }else if  (block % 3 == 1){
                //has_adjust_2 = true;
                telemetry.addLine("ADJUSTING SECOND INTAKE");
                if (intakeposup.getState()){
                    blocky2 +=1;
                }else if (intakeposdown.getState()){
                    blocky2 -=1;
                }else if (intakeposleft.getState()){
                    blockx2 -=1;
                }else if (intakeposright.getState()){
                    blockx2 +=1;
                }
            }else if  (block % 3 == 2){
                telemetry.addLine("ADJUSTING FALLBACK INTAKE");
                if (intakeposup.getState()){
                    blocky3 +=1;
                }else if (intakeposdown.getState()){
                    blocky3 -=1;
                }else if (intakeposleft.getState()){
                    blockx3 -=1;
                }else if (intakeposright.getState()){
                    blockx3 +=1;
                }
            }

            telemetry.addLine("PULL 1: "+ (blockx1) + ", " + blocky1);
            telemetry.addLine("PULL 2: "+ (blockx2) + ", " + blocky2);
            telemetry.addLine("PULL 3: "+ (blockx3) + ", " + blocky3);
            //telemetry.addLine("DO NOT MAKE FALLBACK FIRST NUMBER SAME AS ANY OTHERS, WILL ADD ONE IF YOU DO");


            telemetry.update();

        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();
        robot.mIntake.setExtendoPos(0, timer.seconds());

        blocky1 -=2;
        Action preload;
        if (blockx1 -initialPose.position.x<-3) {
            preload = drive.actionBuilder(initialPose)
                    .stopAndAdd(controller.specPlace())
                    .afterTime(0.75, controller.subPrepareInstant())
                    .afterDisp(Math.sqrt(Math.pow(blockx1 -initialPose.position.x,2)+Math.pow(41.5-initialPose.position.y,2)),
                            new SequentialAction(
                                    controller.resetTimer(),
                                    controller.pull()
                            ))
                    .setTangent(Math.toRadians(135))
                    .splineToLinearHeading(new Pose2d(blockx1 -1, 24, Math.toRadians(90)), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-30, 85))
                    .splineToLinearHeading(new Pose2d(blockx1, 41.5, Math.toRadians(90)), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-30, 85))
                    .stopAndAdd(new SequentialAction(
                            controller.openClaw(),
                            controller.diffyPlace(),
                            controller.resetTimer()
                    ))
                    .build();
        } else if (blockx1 -initialPose.position.x>3) {
             preload = drive.actionBuilder(initialPose)
                    .stopAndAdd(controller.specPlace())
                    .afterTime(0.75, controller.subPrepareInstant())
                     .setTangent(Math.toRadians(45))
                     .afterDisp(Math.sqrt(Math.pow(blockx1 -initialPose.position.x,2)+Math.pow(41.5-initialPose.position.y,2)),
                             new SequentialAction(
                                     controller.resetTimer(),
                                     controller.pull()
                             ))
                     .splineToLinearHeading(new Pose2d(blockx1, 24, Math.toRadians(90)), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-30, 85))
                    .splineToLinearHeading(new Pose2d(blockx1, 41.5, Math.toRadians(90)), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-30, 85))
                    .stopAndAdd(new SequentialAction(
                            controller.openClaw(),
                            controller.diffyPlace(),
                            controller.resetTimer()
                    ))
                    .build();
        }else {
            preload = drive.actionBuilder(initialPose)
                    .stopAndAdd(controller.specPlace())
                    .afterTime(0.75, controller.subPrepareInstant())
                    .afterDisp(Math.sqrt(Math.pow(blockx1 -initialPose.position.x,2)+Math.pow(41.5-initialPose.position.y,2)),
                            new SequentialAction(
                                    controller.resetTimer(),
                                    controller.pull()
                            ))
                    .splineToLinearHeading(new Pose2d(blockx1, 24, Math.toRadians(90)), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-30, 85))
                    .splineToLinearHeading(new Pose2d(blockx1, 41.5, Math.toRadians(90)), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-30, 85))
                    .stopAndAdd(new SequentialAction(
                            controller.openClaw(),
                            controller.diffyPlace(),
                            controller.resetTimer()
                    ))
                    .build();
        }
        Action shoot1 = drive.actionBuilder(new Pose2d(blockx1, 41, Math.toRadians(90)))
                .setReversed(true)
                .afterTime(0.5,new SequentialAction(
                        controller.liftUpInstant(),
                        controller.pivotShoot()
                ))
                .afterDisp(62, new SequentialAction(
                        controller.resetTimer(),
                        controller.shoot()
                ))
                .splineToLinearHeading(new Pose2d(57.5,17, Math.toRadians(90)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))

                .build();
        Action intake1 = drive.actionBuilder(new Pose2d(57.5, 17, Math.toRadians(90)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(58,21), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .build();

        Action shoot2 = drive.actionBuilder(new Pose2d(58, 21, Math.toRadians(90)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(61,17), Math.toRadians(75),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .build();
        Action intake2 = drive.actionBuilder(new Pose2d(61, 17, Math.toRadians(75)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(63,23), Math.toRadians(75),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .build();
        Action shoot3 = drive.actionBuilder(new Pose2d(63, 23, Math.toRadians(75)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(48,17), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .build();
        Action intake3 = drive.actionBuilder(new Pose2d(48, 17, Math.toRadians(90)))
                .setReversed(false)
                .stopAndAdd(controller.pivotDown())

                .strafeToLinearHeading(new Vector2d(48,22), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .build();
        Action wall = drive.actionBuilder(new Pose2d(48, 22, Math.toRadians(90)))
                .setReversed(false)
                .stopAndAdd(controller.extendoInInstant())
                .strafeToLinearHeading(new Vector2d(39,6), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw()
                ))
                .waitSeconds(0.1)
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.liftClearInstant()
                ))
                .waitSeconds(0.05)
                .build();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        controller.updateRobot(),
                        new SequentialAction(
                                preload,
                                controller.pull(),
                                shoot1,
                                controller.resetTimer(),
                                new ParallelAction(
                                        intake1,

                                        controller.intakeing()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        shoot2,
                                        controller.shoot()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        intake2,
                                        controller.intakeing()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        shoot3,
                                        controller.shoot()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        intake3,
                                        controller.intakeing()
                                ),
                                controller.resetTimer(),

                                wall,
                                controller.resetTimer(),
                                new ParallelAction(
                                        sub1human1,
                                        controller.clearWall()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        sub2human2,
                                        controller.clearWall()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        sub3human3,
                                        controller.clearWall()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        sub4human4,
                                        controller.clearWall()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        sub5,
                                        controller.clearWall()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        human5,
                                        controller.liftDownInstant()
                                )

                        )

                )
        );
        robot.stop();
    }
}