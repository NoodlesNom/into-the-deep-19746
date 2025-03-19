package org.firstinspires.ftc.teamcode.util.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "intake servo tester")
public class intake_test extends LinearOpMode {

    private ServoImplEx pivot;

    private DcMotorEx intake;
    public static double gatepos = 0;
    public static double pivotpos = 0;

    public static double intakepower = 0;
    public static boolean move = false;
    public static boolean powered = true;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        pivot = hardwareMap.get(ServoImplEx .class, "pivot");
        pivot.setPosition(0);


        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);



        waitForStart();
        timer.reset();

        if (isStopRequested()) return;
        while (!isStopRequested()) {
            telemetry.update();
            if (move) {
                if (powered) {
                    pivot.setPosition(pivotpos);
                }
                intake.setPower(intakepower);
                telemetry.addData("current", intake.getCurrent(CurrentUnit.AMPS));
            }
            if (powered){
                if (!pivot.isPwmEnabled()) {
                    pivot.setPwmEnable();
                }
            }else{
                if (pivot.isPwmEnabled()) {
                    pivot.setPwmDisable();
                }
            }
        }
    }
}