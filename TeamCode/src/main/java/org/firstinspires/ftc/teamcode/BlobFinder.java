package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="Blob Finder", group = "A")
//@Disabled
public class BlobFinder extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        
        CameraVariables.hardwareMap = hardwareMap;
        MainVision vision = new MainVision();

        while (opModeInInit()) {
            vision.setManualExposure(CameraVariables.exposure, CameraVariables.gain);
            telemetry.addData("Mat Type: ", CameraVariables.matType);
            telemetry.update();
        }

        waitForStart();

        while (CameraVariables.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Thread.sleep(25);
        }

        while (opModeIsActive()){
            double angle1 = -CameraVariables.distanceX1 / 14 + 90;
            double angle2 = CameraVariables.distanceX2 / 14 + 90;
            double c = 3;
            double C = 180 - angle1 - angle2;
            double a = Math.sin(Math.toRadians(angle1)) * (c/Math.sin(Math.toRadians(C)));
            telemetry.addData("X1: ", angle1);
            telemetry.addData("X2: ", angle2);
            telemetry.addData("Z: ", Math.sin(Math.toRadians(angle2))*a);

            telemetry.update();

//            ((DcMotorEx) front).setVelocity(CameraVariables.wheelSpeed * CameraVariables.distanceX);
//            ((DcMotorEx) left).setVelocity(CameraVariables.wheelSpeed * CameraVariables.distanceX);
//            ((DcMotorEx) back).setVelocity(CameraVariables.wheelSpeed * CameraVariables.distanceX);
//            ((DcMotorEx) right).setVelocity(CameraVariables.wheelSpeed * CameraVariables.distanceX);
        }
    }

}
