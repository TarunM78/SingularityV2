package org.firstinspires.ftc.teamcode.ftc8468.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "VR_IntakeTest")
public class VR_IntakeTest extends LinearOpMode {

    // RPM = 1620; TICKS_PER_REV = 103.8; Gear Ratio = 5;
//    int TICKS_FOR_ONE = 519; // Ticks for 1 rotation is 103.8 * 5 = 58;
//    int TICKS_FOR_TWO = 1038; // Ticks for 2 rotation is 28 * 10 = 1538;
//    int TICKS_FOR_THREE = 1557; // Ticks for 3 rotation is 28 * 15 = 2307;
    private int liftMotorTicks = 425;

    //    private RobotDrive drive = new RobotDrive(this);
    RobotDriveTest drive = new RobotDriveTest();

    boolean driveSlow = false;
    boolean isLiftActivated = false;
    boolean isBottomReached = false;
    boolean isVertSensorTouchedOnce = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
//        drive.downLift();
//        drive.activateIntakeServo();
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            operateTeleOp();
        }
    }

    private void operateTeleOp() {
        double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
        double strafe = gamepad1.left_stick_x * 1;
        double rotate = gamepad1.right_stick_x * 1;

        double forward2 = gamepad2.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
        double strafe2 = gamepad2.left_stick_x * 1;
        double rotate2 = gamepad2.right_stick_x * 1;

        drive.driveMecanum(forward + forward2, strafe + strafe2, rotate + rotate2, driveSlow);

        if(gamepad1.x){
            drive.deactivateArm();
            drive.activateWristVert();
            drive.deactivateTransfer();
            drive.activateFlap();
            drive.deactivateTransferLock();
            drive.activateIntake();
        }
        if(gamepad1.y){
            drive.stopIntake();
        }

        if(gamepad1.a){
            drive.reverseIntake();
        }

        if(gamepad1.b){
            drive.deactivateFlap();
            drive.activateTransferLock();
            drive.activateTransfer();
        }

        if(gamepad1.right_bumper){
            drive.activateRedClaw();
            drive.activateBlueClaw();
            drive.deactivateTransferLock();
        }

        if(gamepad1.left_bumper){
            drive.deactivateRedClaw();
            drive.deactivateBlueClaw();
        }

        if(gamepad1.dpad_down){
            drive.activateArm();
            drive.deactivateTransfer();
//            drive.activateWristHoriz();
        }
    }

}