package org.firstinspires.ftc.teamcode.ftc8468.teleop;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.virtualrobot.MecanumDrive;
import org.firstinspires.ftc.teamcode.ftc8468.RobotConstants;


public class RobotDriveTest extends MecanumDrive {

//    private DcMotorEx liftMotorL;
//    private DcMotorEx liftMotorR;

    private DcMotorEx intakeMotor;
    private DcMotorEx intakeMotorBottom;

    protected Servo flap;
    protected Servo transfer;
    protected Servo transferLock;

    protected Servo arm;
    protected Servo wrist;
    protected Servo redClaw; // left
    protected Servo blueClaw; // right

//    protected Servo leftClimb;
//    protected Servo rightClimb;
//
//    protected Servo leftRaiseClimb;
//    protected Servo rightRaiseClimb;

//    protected Servo intakeServo;
//
//    protected Servo droneServo;

//    protected DigitalChannel horizSensorBottom;
//
//    RevColorSensorV3 leftColorSensor, rightColorSensor;

    final int LIFT_TOLERANCE = 0;

    protected VoltageSensor batteryVoltageSensor;

    protected float speedMultiplier = 0.3f; //0.93f;

    enum PixelCount
    {
        ZERO,
        ONE,
        TWO
    }

    PixelCount pixelCount = PixelCount.ZERO;
    final int VERT_TOLERANCE = 5;
//    final int HORIZ_TOLERANCE = 20;

    // Servo Positions...

    // RPM = 435; TICKS PER REV = 384.5
    // RPM = 1620; TICKS_PER_REV = 103.8;
    // RPM = 1150; TICKS_PER_REV = 145.1;
    public static PIDFCoefficients LIFT_VELO_PID = new PIDFCoefficients(2.0, 0, 0, (32767 / (1150 / 60 * 145.1)));

//    public static PIDFCoefficients HORIZ_VELO_PID = new PIDFCoefficients(2.0, 0, 0, (32767 / (1150 / 60 * 145.1)));

//    OpMode opMode;
//    RobotDrive() {
//        super();
//    }
//
//    RobotDrive(OpMode _opMode) {
//        super();
//        opMode = _opMode;
//    }

    public void init(HardwareMap hwMap) {
        super.init(hwMap);
        intakeMotor = hwMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotorBottom = hwMap.get(DcMotorEx.class, "intakeBottom");
        intakeMotorBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flap = hwMap.get(Servo.class, "flap");
        transfer = hwMap.get(Servo.class, "transfer");
        transferLock = hwMap.get(Servo.class, "transferLock");

        arm = hwMap.get(Servo.class, "armServo");

        wrist = hwMap.get(Servo.class, "wrist");

        redClaw = hwMap.get(Servo.class, "redClaw"); // left
        blueClaw = hwMap.get(Servo.class, "blueClaw"); // right
//
//        leftClimb = hwMap.get(Servo.class, "leftClimb");
//        rightClimb = hwMap.get(Servo.class, "rightClimb");
//
//        leftRaiseClimb = hwMap.get(Servo.class, "leftRaiseClimb");
//        rightRaiseClimb = hwMap.get(Servo.class, "rightRaiseClimb");
//
//        intakeServo = hwMap.get(Servo.class, "intakeServo");
//
//        droneServo = hwMap.get(Servo.class, "drone");
//
//        horizSensorBottom = hwMap.get(DigitalChannel.class, "horizTouchBottom");
//        horizSensorBottom.setMode(DigitalChannel.Mode.INPUT);
//
//        leftColorSensor = hwMap.get(RevColorSensorV3.class, "leftColorSensor");
//        rightColorSensor = hwMap.get(RevColorSensorV3.class, "rightColorSensor");

        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();

//        liftMotorL = hwMap.get(DcMotorEx.class, "liftMotorL");
//        liftMotorR = hwMap.get(DcMotorEx.class, "liftMotorR");
//
//        liftMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        liftMotorR.setDirection(DcMotor.Direction.REVERSE);
//        liftMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        liftMotorL.setDirection(DcMotor.Direction.REVERSE);


//        if(LIFT_VELO_PID != null) {
//            setLiftPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LIFT_VELO_PID);
//        }
    }

    // ***** custom code *****

    void driveMecanum(double forward, double strafe, double rotate, boolean driveSlow) {
        double frontLeftSpeed = forward + strafe + rotate;
        double frontRightSpeed = forward - strafe - rotate;
        double backLeftSpeed = forward - strafe + rotate;
        double backRightSpeed = forward + strafe - rotate;

        if(driveSlow) {
            frontLeftSpeed = Range.clip((frontLeftSpeed+strafe*.5) * speedMultiplier,-1,1);
            frontRightSpeed = Range.clip((frontRightSpeed-strafe*.5) * speedMultiplier,-1,1);
            backLeftSpeed = Range.clip((backLeftSpeed-strafe*.5) * speedMultiplier,-1,1);
            backRightSpeed = Range.clip((backRightSpeed+strafe*.5) * speedMultiplier,-1,1);
        }

        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

//    public void setLiftPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
//        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
//                coefficients.p, coefficients.i, coefficients.d,
//                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
//        );
//        liftMotorR.setPIDFCoefficients(runMode, compensatedCoefficients);
//        liftMotorL.setPIDFCoefficients(runMode, compensatedCoefficients);
//    }


    public void activateIntake() {
        intakeMotor.setPower(1.0);
        intakeMotorBottom.setPower(1.0);
    }
    public void reverseIntake() {
        intakeMotor.setPower(-1.0);
        intakeMotorBottom.setPower(-1.0);
    }
    public void stopIntake() {
        intakeMotor.setPower(0);
        intakeMotorBottom.setPower(0);
    }

    public void activateFlap() {
        flap.setPosition(RobotConstants.FLAP_POSITION_ACTIVE);
    }
    public void deactivateFlap() {
        flap.setPosition(RobotConstants.FLAP_POSITION_DEACTIVE);
    }

    public void activateTransfer() {
        transfer.setPosition(RobotConstants.TRANSFER_POSITION_ACTIVE);
    }
    public void deactivateTransfer() {
        transfer.setPosition(RobotConstants.TRANSFER_POSITION_REST);
    }

    public void activateTransferLock() {
        transferLock.setPosition(RobotConstants.TRANSFERLOCK_POSITION_ACTIVE);
    }
    public void deactivateTransferLock() {
        transferLock.setPosition(RobotConstants.TRANSFERLOCK_POSITION_DEACTIVE);
    }

    public void activateArm() {
        arm.setPosition(RobotConstants.ARM_POSITION_ACTIVE);
    }
    public void restArm() {
        arm.setPosition(RobotConstants.ARM_POSITION_REST);
    }
    public void deactivateArm() {
        arm.setPosition(RobotConstants.ARM_POSITION_DEACTIVE);
    }

    public void activateRedClaw() {
        redClaw.setPosition(RobotConstants.RED_CLAW_POSITION_ACTIVE);
    }
    public void deactivateRedClaw() {
        redClaw.setPosition(RobotConstants.RED_CLAW_POSITION_DEACTIVE);
    }

    public void activateBlueClaw() {
        blueClaw.setPosition(RobotConstants.BLUE_CLAW_POSITION_ACTIVE);
    }
    public void deactivateBlueClaw() {
        blueClaw.setPosition(RobotConstants.BLUE_CLAW_POSITION_DEACTIVE);
    }

    public void activateWristHoriz() {
        wrist.setPosition(RobotConstants.WRIST_POSITION_HORIZ);
    }
    public void activateWristVert() {
        wrist.setPosition(RobotConstants.WRIST_POSITION_VERT);
    }

//    public void activateLeftClaw() {
//        leftClaw.setPosition(RobotConstants.LEFT_CLAW_POSITION_ACTIVE);
//    }
//    public void deactivateLeftClaw() {
//        leftClaw.setPosition(RobotConstants.LEFT_CLAW_POSITION_DEACTIVE);
//    }
//
//    public void activateRightClaw() {
//        rightClaw.setPosition(RobotConstants.RIGHT_CLAW_POSITION_ACTIVE);
//    }
//    public void deactivateRightClaw() {
//        rightClaw.setPosition(RobotConstants.RIGHT_CLAW_POSITION_DEACTIVE);
//    }
//
//
//    public void activateLeftClimb() {
//        leftClimb.setPosition(RobotConstants.LEFT_CLIMB_POSITION_ACTIVE);
//    }
//    public void activateRightClimb() {
//        rightClimb.setPosition(RobotConstants.RIGHT_CLIMB_POSITION_ACTIVE);
//    }
//
//    public void deactivateLeftClimb() {
//        leftClimb.setPosition(RobotConstants.LEFT_CLIMB_POSITION_DEACTIVE);
//    }
//    public void deactivateRightClimb() {
//        rightClimb.setPosition(RobotConstants.RIGHT_CLIMB_POSITION_DEACTIVE);
//    }
//
//    public void activateLeftRaiseClimb() {
//        leftRaiseClimb.setPosition(RobotConstants.LEFT_RAISE_CLIMB_POSITION_ACTIVE);
//    }
//
//    public void activateLeftRaiseClimbDrone() {
//        leftRaiseClimb.setPosition(RobotConstants.LEFT_RAISE_CLIMB_POSITION_DRONE);
//    }
//
//    public void activateRightRaiseClimb() {
//        rightRaiseClimb.setPosition(RobotConstants.RIGHT_RAISE_CLIMB_POSITION_ACTIVE);
//    }
//
//    public void activateRightRaiseClimbDrone() {
//        rightRaiseClimb.setPosition(RobotConstants.RIGHT_RAISE_CLIMB_POSITION_DRONE);
//    }
//
//    public void deactivateLeftRaiseClimb() {
//        leftRaiseClimb.setPosition(RobotConstants.LEFT_RAISE_CLIMB_POSITION_DEACTIVE);
//    }
//    public void deactivateRightRaiseClimb() {
//        rightRaiseClimb.setPosition(RobotConstants.RIGHT_RAISE_CLIMB_POSITION_DEACTIVE);
//    }
//    public void activateIntakeServo() {
//        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_ACTIVE);
//    }
//    public void deactivateIntakeServo() {
//        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_REST);
//    }
//
//    public void activateDroneServo() {
//        droneServo.setPosition(RobotConstants.DRONE_POSITION_ACTIVE);
//    }
//    public void deactivateDroneServo() {
//        droneServo.setPosition(RobotConstants.DRONE_POSITION_REST);
//    }
//    public void activateLift(int ticks) {
//        // activateDumpServoHalf();
//        liftMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//
//        setLiftPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,LIFT_VELO_PID);
//
//        double power = 1.0; //1.0;
//        liftMotorR.setTargetPosition(ticks);
//        liftMotorL.setTargetPosition(-ticks);
//        liftMotorR.setTargetPositionTolerance(LIFT_TOLERANCE);
//        liftMotorL.setTargetPositionTolerance(LIFT_TOLERANCE);
//        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotorR.setPower(power);
//        liftMotorL.setPower(power);
//    }
//
//    public void downLift() {
//        int ticks = 1000;
//        double power = -1.0; //-1.0;
//        liftMotorR.setTargetPosition(-ticks); // negative ticks for opposite direction
//        liftMotorL.setTargetPosition(ticks); // negative ticks for opposite direction
//        liftMotorR.setTargetPositionTolerance(LIFT_TOLERANCE);
//        liftMotorL.setTargetPositionTolerance(LIFT_TOLERANCE);
//        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotorR.setPower(power);
//        liftMotorL.setPower(power);
//    }
//
//    public void deactivateLift() {
//        int ticks = 150;
//        double power = -1.0; //-1.0;
//        liftMotorR.setTargetPosition(-ticks); // negative ticks for opposite direction
//        liftMotorL.setTargetPosition(ticks); // negative ticks for opposite direction
//        liftMotorR.setTargetPositionTolerance(LIFT_TOLERANCE);
//        liftMotorL.setTargetPositionTolerance(LIFT_TOLERANCE);
//        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotorR.setPower(power);
//        liftMotorL.setPower(power);
//    }
//
//    public void stopLift() {
//        liftMotorL.setPower(0);
//        liftMotorR.setPower(0);
//    }
//
//    public void moveLift(double power) {
//        liftMotorR.setPower(power);
//        liftMotorL.setPower(-power);
//    }
//
//    public boolean isLiftSensorTouched() {
//        // ***** if state == true, then "Not Pressed", Otherwise "Pressed" *****
////        if (sensor1.getState() == true) {
////            telemetry.addData("Digital Touch", "Is Not Pressed");
////        } else {
////            telemetry.addData("Digital Touch", "Is Pressed");
////        }
//
//        boolean isTouched = false;
//        if(horizSensorBottom.getState() == false) {
//            isTouched = true;
//        }
//        return isTouched;
//    }
//
//    public boolean checkLiftMotion() {
//        boolean isBottomReached = false;
//        if (isLiftSensorTouched()) {
//            stopLift();
//            //restArm();
//            //restKicker();
//            //startIntake();
//            isBottomReached = true;
//        }
//        return isBottomReached;
//    }
//
//    public PixelCount getPixelCount()
//    {
//        if (leftColorSensor.getDistance(DistanceUnit.CM) > 1.0 && rightColorSensor.getDistance(DistanceUnit.CM) > 1.0)
//        {
//            pixelCount = PixelCount.ZERO;
//        }
//        else if (leftColorSensor.getDistance(DistanceUnit.CM) <= 1.0 && rightColorSensor.getDistance(DistanceUnit.CM) <= 1.0)
//        {
//            pixelCount = PixelCount.TWO;
//        }
//        else
//        {
//            pixelCount = PixelCount.ONE;
//        }
//        return pixelCount;
//    }
//
//    public boolean leftPixelContained()
//    {
//        if (leftColorSensor.getDistance(DistanceUnit.CM) <= 1.0)
//            return true;
//        else
//            return false;
//    }
//
//    public boolean rightPixelContained()
//    {
//        if (rightColorSensor.getDistance(DistanceUnit.CM) <= 1.0)
//            return true;
//        else
//            return false;
//    }

}