package org.firstinspires.ftc.teamcode.ftc8468.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "TestServoTemp")
public class TestServoTemp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "armServo");


        ElapsedTime elapsedTime = new ElapsedTime();
        waitForStart();
        elapsedTime.reset();
        while (opModeIsActive()) {
            if (elapsedTime.seconds() > .2 && (gamepad1.dpad_up || gamepad1.dpad_down)) {
                if (gamepad1.dpad_up) {
                    servo.setPosition(Range.clip(servo.getPosition() + .005, 0, 1));
                } else if (gamepad1.dpad_down) {
                    servo.setPosition(Range.clip(servo.getPosition() - .005, 0, 1));
                }
                elapsedTime.reset();
            }
            telemetry.addData("Servo position", servo.getPosition());
            telemetry.update();
            idle();

        }
    }
}
