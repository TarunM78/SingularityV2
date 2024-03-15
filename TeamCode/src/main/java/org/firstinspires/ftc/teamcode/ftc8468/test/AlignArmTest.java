package org.firstinspires.ftc.teamcode.ftc8468.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ftc8468.auto.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.ftc8468.auto.temp.ServoAutoDrive;
import org.firstinspires.ftc.teamcode.ftc8468.auto.temp.VisionAutoDrive;


@Autonomous(name = "AlignArmTest")
public class AlignArmTest extends LinearOpMode {
    ServoAutoDrive drive;
    private static int position = 0;
    boolean isReached = false;

    protected Servo turretServo;

    public void runOpMode() {
        initialize();

        waitForStart();
        if (isStopRequested()) return;

        drive.activateArm();
        isReached = drive.checkAndStopArm(telemetry);

        while (opModeIsActive() && !isStopRequested()) {
            if(!isReached) {
                isReached = drive.checkAndStopArm(telemetry);
            } else {
//                drive.deactivateArm();
                drive.stopArm();
            }
            telemetry.addData("isReached:", isReached ? "true" : "false"+" | distance: "+drive.getDistance());
            telemetry.update();
//            drive.update();
        }
    }

    private void initialize() {
        drive = new ServoAutoDrive(hardwareMap);
        telemetry.addData("Status:", "initialize() - Robot and Camera are initialized!");
        telemetry.update();
    }

}
