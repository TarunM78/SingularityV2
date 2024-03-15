package org.firstinspires.ftc.teamcode.ftc8468.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class TestServo extends LinearOpMode {
    boolean isButtonPressed = false;
    boolean isX = false;
    boolean isY = false;
    boolean isA = false;
    boolean isB = false;
    boolean isLeftBumper = false;
    boolean isRightBumper = false;
    boolean isDpadLeft = false;
    boolean isDpadRight = false;
    boolean isLeftJoystick = false;

    Servo transfer;
    Servo transferLock;
    Servo flap;
    Servo intakeServo;
    Servo redClaw;
    Servo blueClaw;
    Servo armServo;
    Servo wrist;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = null;
        telemetry.addData("Choose the button ", "for desired Servo");
        telemetry.addData("X: ", "\"transfer\"");
        telemetry.addData("Y: ", "\"transferLock\"");
        telemetry.addData("A: ", "\"flap\"");
        telemetry.addData("B: ", "\"intakeServo\"");
        telemetry.addData("Left Bumper: ", "\"redClaw\""); // left
        telemetry.addData("Right Bumper: ", "\"blueClaw\""); // right
        telemetry.addData("Dpad Left: ", "\"armServo\"");
        telemetry.addData("Dpad Right: ", "\"wrist\"");
        telemetry.addData("Left Stick Button: ", "\"intakeServo\"");

        telemetry.update();

        while (opModeInInit()) {
            if (gamepad1.x) {
                isButtonPressed = true;
                isX = true;
                servo = hardwareMap.get(Servo.class, "transfer");
            }
            if (gamepad1.y) {
                isButtonPressed = true;
                isY = true;
                servo = hardwareMap.get(Servo.class, "transferLock");
            }
            if (gamepad1.a) {
                isButtonPressed = true;
                isA = true;
                servo = hardwareMap.get(Servo.class, "flap");
            }
            if (gamepad1.b) {
                isButtonPressed = true;
                isB = true;
                servo = hardwareMap.get(Servo.class, "intakeServo");
            }
            if (gamepad1.left_bumper) {
                isButtonPressed = true;
                isLeftBumper = true;
                servo = hardwareMap.get(Servo.class, "redClaw");
            }
            if (gamepad1.right_bumper) {
                isButtonPressed = true;
                isRightBumper = true;
                servo = hardwareMap.get(Servo.class, "blueClaw");
            }
            if (gamepad1.dpad_left) {
                isButtonPressed = true;
                isDpadLeft = true;
                servo = hardwareMap.get(Servo.class, "armServo");
            }
            if (gamepad1.dpad_right) {
                isButtonPressed = true;
                isDpadRight = true;
                servo = hardwareMap.get(Servo.class, "wrist");
            }
        }

        ElapsedTime elapsedTime = new ElapsedTime();
        waitForStart();
        elapsedTime.reset();

        if(isButtonPressed) {
            servo = getServo();
        }

        if(servo != null) {
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
        } else {
            telemetry.addData("No Button is Pressed! ", "Press the correct Button for a Servo!!");

            telemetry.update();
        }
    }

    Servo getServo() {
        Servo servo = null;
        if(isX) {
            servo = hardwareMap.get(Servo.class, "transfer");
        }
        if(isY) {
            servo = hardwareMap.get(Servo.class, "transferLock");
        }
        if(isA) {
            servo = hardwareMap.get(Servo.class, "flap");
        }
        if(isB) {
            servo = hardwareMap.get(Servo.class, "intakeServo");
        }
        if(isLeftBumper) {
            servo = hardwareMap.get(Servo.class, "redClaw");
        }
        if(isRightBumper) {
            servo = hardwareMap.get(Servo.class, "blueClaw");
        }
        if(isDpadLeft) {
            servo = hardwareMap.get(Servo.class, "armServo");
        }
        if(isDpadRight) {
            servo = hardwareMap.get(Servo.class, "wrist");
        }
        return servo;
    }

}
