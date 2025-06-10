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

@TeleOp(name = "AutoPickUpBlock", group = "A")
@Config
public class AutoPickUpBlock extends LinearOpMode {
    private DcMotorEx front;
    private DcMotorEx left;
    private DcMotorEx back;
    private DcMotorEx right;

    boolean moveArmWithPath = false;
    boolean pathMoveSet = false;


    public static double KpS = 0.002;
    public static double KiS = 0.000;
    public static double KdS = 0.0001;

    boolean clawTargetUp = true;

    double clawWheelSpeed = 0;

    boolean shoulderTargetPosSet = false;
    boolean elbowTargetPosSet = false;

    private DcMotorEx shoulder;
//    private DcMotorEx shoulderBreak;
    private DcMotorEx elbow;

    boolean clawDown = false;
    double manualClawPos = 0;

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

        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoulder.setDirection(DcMotorSimple.Direction.FORWARD);

//        shoulderBreak = hardwareMap.get(DcMotorEx.class, "brake");
//        //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shoulderBreak.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        //shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        shoulderBreak.setDirection(DcMotorSimple.Direction.REVERSE);

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        //elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setDirection(DcMotorSimple.Direction.FORWARD);

        Arm arm = new Arm(hardwareMap);

        boolean lastLeftBumper = false;

        Map<String, Double> moveS = new HashMap<>();;
        Map<String, Double> moveE = new HashMap<>();;

        CameraVariables.hardwareMap = hardwareMap;
        MainVision vision = new MainVision();

        while (opModeInInit()) {
            vision.setManualExposure(CameraVariables.exposure, CameraVariables.gain);
        }

        waitForStart();

        while (opModeIsActive()) {

            double angle1 = -CameraVariables.distanceX1 / 14 + 90;
            double angle2 = CameraVariables.distanceX2 / 14 + 90;
            double c = 3;
            double C = 180 - angle1 - angle2;
            double a = Math.sin(Math.toRadians(angle1)) * (c/Math.sin(Math.toRadians(C)));
            telemetry.addData("X1: ", angle1);
            telemetry.addData("X2: ", angle2);
            telemetry.addData("Z: ", Math.sin(Math.toRadians(angle2))*a);

            if(gamepad1.b){
                if(!lastLeftBumper){
                    if (clawWheelSpeed == 1){
                        clawWheelSpeed = -1;
                    } else {
                        clawWheelSpeed = 1;
                    }
                }
                lastLeftBumper = true;
            } else {
                lastLeftBumper = false;
            }

            arm.levelClaw(manualClawPos);


            arm.clawWheels(clawWheelSpeed);

            if (gamepad1.a) {
                if (!pathMoveSet){
                    moveS = arm.setMove(-shoulder.getCurrentPosition(), 1455, 1500, 700);
                    moveE = arm.setMove(-elbow.getCurrentPosition(), -5695, 1500, 700);
                    pathMoveSet = true;
                    moveArmWithPath = true;
                }
                elbowTargetPosSet = false;
                shoulderTargetPosSet = false;
                clawTargetUp = true;
                clawDown = false;

            } else if (gamepad1.y) {
                if (!pathMoveSet){
                    moveS = arm.setMove(-shoulder.getCurrentPosition(), 2700, 1500, 700);
                    moveE = arm.setMove(-elbow.getCurrentPosition(), -5430, 1500, 700); //3000
                    pathMoveSet = true;
                    moveArmWithPath = true;
                }
                elbowTargetPosSet = false;
                shoulderTargetPosSet = false;
                clawTargetUp = true;
                clawDown = false;
            } else {
                moveArmWithPath = false;
                pathMoveSet = false;
            }



            if (moveArmWithPath && gamepad1.left_bumper) {
                double shoulderTarget = arm.calcCurrentTargetPos(moveS);
                double elbowTarget = arm.calcCurrentTargetPos(moveE);

                arm.shoulderTargetForPath(shoulderTarget);
                arm.elbowTargetForPath(elbowTarget);
            } else {
                shoulder.setPower(0);
                elbow.setPower(0);
            }

            if (gamepad1.left_bumper) {
                drive((Math.sin(Math.toRadians(angle2))*a)-12, -angle1+90 + angle2-90);
            } else {
                drive(0, 0);
            }

            telemetry.update();

        }
    }

    public void drive(double forwardTarget, double angleTarget){
        double frontPow = angleTarget * CameraVariables.angleMultiplier;
        double backPow = frontPow;
        double leftPow = frontPow - forwardTarget * CameraVariables.driveMultiplier;
        double rightPow = frontPow + forwardTarget * CameraVariables.driveMultiplier;

        double maxPow = CameraVariables.maxPow;
        if (frontPow > maxPow){
            frontPow = maxPow;
        }
        if (leftPow > maxPow){
            leftPow = maxPow;
        }
        if (backPow > maxPow){
            backPow = maxPow;
        }
        if (rightPow > maxPow){
            rightPow = maxPow;
        }
        front.setPower(frontPow);
        left.setPower(leftPow);
        back.setPower(backPow);
        right.setPower(rightPow);
    }

    public class Arm {
        private DcMotorEx shoulder;
        private DcMotorEx elbow;
//        Servo clawClose;
        CRServo clawWheels;
        Servo clawUp;
        public Arm(HardwareMap hardwareMap){
            shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
            //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shoulder.setDirection(DcMotorSimple.Direction.FORWARD);

            elbow = hardwareMap.get(DcMotorEx.class, "elbow");
            //elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elbow.setDirection(DcMotorSimple.Direction.FORWARD);

//            clawClose = hardwareMap.get(Servo.class, "clawClose");
            clawWheels = hardwareMap.get(CRServo.class, "clawWheels");
            clawUp = hardwareMap.get(Servo.class, "clawUp");
        }
        public double calcCurrentTargetPos(Map<String, Double> move) {

            // Set the previous, current and total move time
            double moveTime = getRuntime() - move.get("moveStartTime");

            double moveEndTime = move.get("moveEndTime");
            double accel = move.get("accel");
            double P1 = move.get("P1");
            double P2 = move.get("P2");
            double P3 = move.get("P3");
            double peakVel = move.get("peakVel");
            telemetry.addData("P1", P1);
            // if the move is over set the move to go to the last pos by setting the time to the last time
            if (moveTime > moveEndTime) {
                moveTime = moveEndTime;
            }

            double Pmove;
            if (Math.abs(move.get("length")) <= 2 * Math.abs(P1)) {
                // Triangle
                if (moveTime <= moveEndTime / 2) {
                    Pmove =  0.5 * accel * Math.pow(moveTime, 2);
                } else {
                    Pmove = -0.5 * accel * Math.pow(moveTime, 2) + P2 * moveTime + P3;
                }
            } else {
                // Trapezoid
                double timeA = peakVel / accel;
                if (moveTime <= timeA) {
                    Pmove = 0.5 * accel * Math.pow(moveTime, 2);
                } else if (moveTime <= moveEndTime - timeA) {
                    Pmove = peakVel * moveTime + P1;
                } else {
                    Pmove = -0.5 * accel * Math.pow(moveTime, 2) + P2 * moveTime + P3;
                }
            }
            double targetPos = move.get("startPos") + Pmove;
            return targetPos;
        }

        public void clawWheels(double pow){
            clawWheels.setPower(pow);
        }
        double endArmAngle;
        double servoTarget;
        public void levelClaw(double pos){
            if (pos ==0){
                endArmAngle = ((-shoulder.getCurrentPosition() - 1457)/8192.0 * 2*Math.PI)+ ((-elbow.getCurrentPosition() + 3344)/8192.0 * 2*Math.PI);
                servoTarget = -endArmAngle/(Math.PI*2)*1.333;
            } else {
                servoTarget = pos; //69
            }
            clawUp.setPosition(servoTarget);
        }


        double integralSumS = 0;
        double integralSumE = 0;
        double lastErrorS = 0;
        double lastErrorE = 0;
        ElapsedTime timerE = new ElapsedTime();
        ElapsedTime timerS = new ElapsedTime();

        double elbowGravity = 0;

        int ticksPerRev = 8192;
        double shoulderAngle = 0;

        public void shoulderTargetForPath(double reference) {
            // obtain the encoder position
            double encoderPosition = -shoulder.getCurrentPosition();
            //telemetry.addData("shoulder pos: ", encoderPosition);

            // calculate the error
            double error = reference - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastErrorS) / timerS.seconds();

            // sum of all error over time
            integralSumS = integralSumS + (error * timerS.seconds());

            shoulderAngle = ((encoderPosition - 1350)/ticksPerRev)*(2*Math.PI);

            double gravity = Math.cos(shoulderAngle) * .02 + elbowGravity;

            double out = (0.002 * error) + (0 * integralSumS) + (0.0001 * derivative);

            out = Math.max(Math.min(out, 1), -1);

            out += gravity;

            shoulder.setPower(out);

            lastErrorS = error;

            // reset the timer for next time
            timerS.reset();
            telemetry.addData("Target Pos: ", reference/5000);
            telemetry.addData("Current Pos: ", encoderPosition/5000);
            telemetry.addData("P Pow: ", Math.max(Math.min(KpS * error, 1), -1));
            telemetry.addData("I Pow: ", Math.max(Math.min(KiS * integralSumS, 1), -1));
            telemetry.addData("D Pow: ", Math.max(Math.min(KdS * derivative, 1), -1));
            telemetry.addData("G Pow: ", gravity);
            telemetry.addData("out Pow: ",out);
        }

        double GravityKE = 0.15;
        public void elbowTargetForPath( double reference) {
            // obtain the encoder position
            double encoderPosition = -elbow.getCurrentPosition();
            // calculate the error
            double error = reference - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastErrorE) / timerE.seconds();

            // sum of all error over time
            integralSumE = integralSumE + (error * timerE.seconds());

            double elbowAngle = ((encoderPosition + 3344)/ticksPerRev)*(2*Math.PI) + shoulderAngle;

            elbowGravity = Math.cos(elbowAngle) * GravityKE;

            double out = (.003 * error) + (0 * integralSumE) + (0.0001 * derivative) + elbowGravity;

            out = Math.max(Math.min(out, 1), -1);

            elbow.setPower(out);

            lastErrorE = error;

            // reset the timer for next time
            timerE.reset();

            telemetry.addData("target", reference);
            telemetry.addData("pos", encoderPosition);

        }
        public Map<String, Double> setMove(double currentPos, double targetPos, double accel, double peakVel) {

            Map<String, Double> move = new HashMap<>();

            // Set the total distance of the move
            double length = targetPos - currentPos;

            accel = Math.abs(accel);
            peakVel = Math.abs(peakVel);


            // Set the pos at T1 = 1/2 Vmax * t1
            // If the triangle was the biggest size is just length / 2
            double P1 = -0.5 * (Math.pow(peakVel, 2) / accel);

            double moveEndTime;

            // Decide if the move is a trapezoid or a triangle
            if (Math.abs(length) <= 2 * -P1) {
                // Triangle
                // Calc the move end time 2 + Vmax/A ????????
                moveEndTime = 2 * Math.sqrt(Math.abs(length) / accel);

            } else {
                // Trapezoid
                // Calc the move end time L/Vmax + Vmax/A
                moveEndTime = Math.abs(length) / peakVel + peakVel / accel;
            }
            double P2 = accel * moveEndTime;
            double P3 = Math.abs(length) - 0.5 * accel * Math.pow(moveEndTime, 2);

            if (length < 0){
                accel *= -1;
                peakVel *= -1;
                P1 *= -1;
                P2 *= -1;
                P3 *= -1;
            }

            move.put("P1", P1);
            move.put("P2",P2);
            move.put("P3",P3);
            move.put("length", length);
            move.put("moveEndTime", moveEndTime);
            move.put("moveStartTime",  getRuntime());
            move.put("startPos", currentPos);
            move.put("accel", accel);
            move.put("peakVel", peakVel);

            return move;
        }
    }

}
