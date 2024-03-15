package org.firstinspires.ftc.teamcode.ftc8468.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "VR_MainTeleOp_One_Player", group = "ftc8468")
public class VR_MainTeleOp_One_Player extends LinearOpMode {

    // RPM = 1620; TICKS_PER_REV = 103.8; Gear Ratio = 5;
//    int TICKS_FOR_ONE = 519; // Ticks for 1 rotation is 103.8 * 5 = 58;
//    int TICKS_FOR_TWO = 1038; // Ticks for 2 rotation is 28 * 10 = 1538;
//    int TICKS_FOR_THREE = 1557; // Ticks for 3 rotation is 28 * 15 = 2307;
    private int liftMotorTicks = 425;

    RobotDrive drive = new RobotDrive();

    boolean driveSlow = false;
    boolean isLiftActivated = false;
    boolean isBottomReached = false;
    boolean isVertSensorTouchedOnce = false;
    boolean isArmUp = false;
    boolean isWristActivatedHoriz = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
//        drive.downLift();
        drive.deactivateIntakeServo();
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

        //check lift sensors and decide whether to stop it or not...
        if(!isLiftActivated) {
            isBottomReached = drive.checkLiftMotion();
            if(isBottomReached) {
                isVertSensorTouchedOnce = true;
            }
        }

        double wristPosition = 0.0;
        if(isArmUp) {
            isWristActivatedHoriz = drive.checkAndActivateWristHoriz();
            wristPosition = drive.getWristPosition();
        }

        if(gamepad1.left_bumper){
            drive.deactivateRedClaw();
        }
        if(gamepad1.right_bumper) {
            drive.deactivateBlueClaw();
        }
        if(gamepad1.a) {
            drive.deactivateTransferLock();
            drive.activateRedClaw();
            drive.activateBlueClaw();
            drive.stopIntake();
            drive.stopIntakeBottom();

//            drive.activateIntakeServo();
        }

        if((gamepad1.x) && (isVertSensorTouchedOnce)){
            drive.activateWristVert();
            drive.deactivateArm();
            drive.deactivateTransfer();
            drive.activateFlap();
            drive.deactivateTransferLock();
            drive.activateIntake();
            drive.activateIntakeBottom();
            drive.deactivateRedClaw();
            drive.deactivateBlueClaw();
        }
        if(gamepad1.y){
            drive.reverseIntake();
            drive.reverseIntakeBottom();
        }

        if(gamepad1.b){
            drive.deactivateFlap();
            drive.activateTransferLock();
            drive.activateTransfer();
            drive.deactivateRedClaw();
            drive.deactivateBlueClaw();
            drive.stopIntake();
        }

        if ((gamepad1.dpad_left) && (isBottomReached || isVertSensorTouchedOnce)) {
            if (!isLiftActivated) {
                drive.deactivateTransferLock();
                drive.deactivateTransfer();
                drive.activateLift(400);
                drive.activateArm();
                drive.stopIntake();
                drive.stopIntakeBottom();
//                drive.activateIntakeServo();
                driveSlow = true;
                isLiftActivated = true;
                isBottomReached = false;
                isVertSensorTouchedOnce = false;
                isArmUp = true;
            }
        }

        if ((gamepad1.dpad_up) && (isBottomReached || isVertSensorTouchedOnce)) {
            if (!isLiftActivated) {
                drive.deactivateTransferLock();
                drive.deactivateTransfer();
                drive.activateLift(800);
                drive.activateArm();
                drive.stopIntake();
                drive.stopIntakeBottom();
//                drive.activateIntakeServo();
                driveSlow = true;
                isLiftActivated = true;
                isBottomReached = false;
                isVertSensorTouchedOnce = false;
                isArmUp = true;
            }
        }

        if ((gamepad1.dpad_right) && (isBottomReached || isVertSensorTouchedOnce)) {
            if (!isLiftActivated) {
                drive.deactivateTransferLock();
                drive.deactivateTransfer();
                drive.activateLift(1250);
                drive.activateArm();
                drive.stopIntake();
                drive.stopIntakeBottom();;
//                drive.activateIntakeServo();
                driveSlow = true;
                isLiftActivated = true;
                isBottomReached = false;
                isVertSensorTouchedOnce = false;
                isArmUp = true;
            }
        }

        if (gamepad1.dpad_down) {
            if (isLiftActivated) {
                drive.deactivateLift();
                drive.deactivateArm();
                drive.deactivateTransfer();
                drive.activateWristVert();
//                drive.activateIntakeServo();
                driveSlow = false;
                isLiftActivated = false;
            }
        }

        if(gamepad2.left_bumper){
            drive.activateLeftClimb();
            drive.activateRightClimb();
        }
        if(gamepad2.right_bumper) {
            drive.deactivateLeftClimb();
            drive.deactivateRightClimb();
        }
//        if(gamepad2.y) {
//            drive.activateRightRaiseClimb();
//            drive.activateLeftRaiseClimb();
//            drive.deactivateIntakeServo();
//            drive.stopIntake();
//        }
//        if(gamepad2.x) {
//            drive.deactivateRightRaiseClimb();
//            drive.deactivateLeftRaiseClimb();
//            drive.deactivateIntakeServo();
//            drive.stopIntake();
//        }
//        if(gamepad2.a) {
//            drive.activateDroneServo();
//            drive.deactivateIntakeServo();
//            drive.stopIntake();
//        }
//        if(gamepad2.b) {
//            drive.activateLeftRaiseClimbDrone();
//            drive.activateRightRaiseClimbDrone();
//            drive.deactivateIntakeServo();
//            drive.stopIntake();
//        }


        telemetry.addData("isBottomReached: ", isBottomReached);
        telemetry.addData("isLiftActivated: ", isLiftActivated);
        telemetry.addData("wristPosition: ", wristPosition+" | isWristActivatedHoriz"+isWristActivatedHoriz);
        telemetry.update();
    }

}