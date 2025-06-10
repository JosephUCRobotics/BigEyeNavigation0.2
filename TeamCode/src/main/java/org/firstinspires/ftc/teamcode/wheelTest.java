package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

// adb connect 192.168.43.1:5555

@TeleOp(name = "Wheel Test", group = "A")
@Config
public class wheelTest extends LinearOpMode {
    private DcMotorEx front;
    private DcMotorEx left;
    private DcMotorEx back;
    private DcMotorEx right;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        front = hardwareMap.get(DcMotorEx.class, "front");
        //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        front.setDirection(DcMotorSimple.Direction.REVERSE);

        left = hardwareMap.get(DcMotorEx.class, "left");
        //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        back = hardwareMap.get(DcMotorEx.class, "back");
        //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back.setDirection(DcMotorSimple.Direction.REVERSE);

        right = hardwareMap.get(DcMotorEx.class, "right");
        //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                back.setPower(.3);
            }else{
                back.setPower(0);
            }
            if (gamepad1.x) {
                left.setPower(.3);
            }else{
                left.setPower(0);
            }
            if (gamepad1.y) {
                front.setPower(.3);
            }else{
                front.setPower(0);
            }
            if (gamepad1.b) {
                right.setPower(.3);
            }else{
                right.setPower(0);
            }

            telemetry.update();

        }
    }
}
