package org.firstinspires.ftc.teamcode.ftc8468.auto;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ftc8468.RobotConstants;



public class RRAutoDrive extends SampleMecanumDrive {

    private DcMotorEx liftMotorL;
    private DcMotorEx liftMotorR;

    private DcMotorEx intakeMotor;
    private DcMotorEx intakeMotorBottom;

    protected Servo arm;
    protected Servo wrist;
    protected Servo redClaw;
    protected Servo blueClaw;
    protected Servo intakeServo;

    protected DigitalChannel horizSensorBottom;
    protected VoltageSensor batteryVoltageSensor;

    RevColorSensorV3 leftColorSensor, rightColorSensor;

    public enum PixelCount
    {
        ZERO,
        ONE,
        TWO
    }

    RRAutoDrive.PixelCount pixelCount = RRAutoDrive.PixelCount.ZERO;

    private ElapsedTime elapsedTime;

    final int LIFT_TOLERANCE = 0;
    protected float speedMultiplier = 0.6f;

    // RPM = 435; TICKS PER REV = 384.5
    // RPM = 1150; TICKS_PER_REV = 145.1;
    // RPM = 1620; TICKS_PER_REV = 103.8;//
//    public static PIDFCoefficients LIFT_VELO_PID = new PIDFCoefficients(2.0, 0, 0, (32767 / (1150 / 60 * 145.1)));
    public static PIDFCoefficients LIFT_VELO_PID = new PIDFCoefficients(2.0, 0, 0, (32767 / (435 / 60 * 384.5)));

    public RRAutoDrive (HardwareMap hwMap) {
        super(hwMap);
        intakeMotor = hwMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotorBottom = hwMap.get(DcMotorEx.class, "intakeBottom");
        intakeMotorBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hwMap.get(Servo.class, "armServo");

        wrist = hwMap.get(Servo.class, "wrist");

        redClaw = hwMap.get(Servo.class, "redClaw"); // left
        blueClaw = hwMap.get(Servo.class, "blueClaw"); // right

        intakeServo = hwMap.get(Servo.class, "intakeServo");

        horizSensorBottom = hwMap.get(DigitalChannel.class, "horizTouchBottom");
        horizSensorBottom.setMode(DigitalChannel.Mode.INPUT);

        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();

        liftMotorL = hwMap.get(DcMotorEx.class, "liftMotorL");
        liftMotorR = hwMap.get(DcMotorEx.class, "liftMotorR");

//        leftColorSensor = hwMap.get(RevColorSensorV3.class, "leftColorSensor");
//        rightColorSensor = hwMap.get(RevColorSensorV3.class, "rightColorSensor");

        liftMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorR.setDirection(DcMotor.Direction.REVERSE);
        liftMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorL.setDirection(DcMotor.Direction.REVERSE);


        if(LIFT_VELO_PID != null) {
            setLiftPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LIFT_VELO_PID);
        }
        elapsedTime = new ElapsedTime();

    }

    // ***** FTC 18305 custom code *****

    //    void driveMecanum(double forward, double strafe, double rotate, boolean driveSlow) {
    //        double frontLeftSpeed = forward + strafe + rotate;
    //        double frontRightSpeed = forward - strafe - rotate;
    //        double backLeftSpeed = forward - strafe + rotate;
    //        double backRightSpeed = forward + strafe - rotate;
    //
    //        if(driveSlow) {
    //            frontLeftSpeed = frontLeftSpeed * speedMultiplier;
    //            frontRightSpeed = frontRightSpeed * speedMultiplier;
    //            backLeftSpeed = backLeftSpeed * speedMultiplier;
    //            backRightSpeed = backRightSpeed * speedMultiplier;
    //        }
    //
    //        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    //    }


    public void setLiftPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        liftMotorR.setPIDFCoefficients(runMode, compensatedCoefficients);
        liftMotorL.setPIDFCoefficients(runMode, compensatedCoefficients);
    }

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

    public void activateArm() {
        arm.setPosition(RobotConstants.ARM_POSITION_ACTIVE);
    }
    public void initArm() {
        arm.setPosition(RobotConstants.ARM_POSITION_INIT);
    }

    public void restArm() {
        arm.setPosition(RobotConstants.ARM_POSITION_REST);
    }
    public void restArmAuto() {
        arm.setPosition(RobotConstants.ARM_POSITION_AUTO);
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

    public void activateIntakeServo() {
        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_ACTIVE);
    }
    public void activateIntakeServoFive() {
        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_FIVE);
    }
    public void activateIntakeServoFour() {
        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_FOUR);
    }
    public void activateIntakeServoThree() {
        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_THREE);
    }
    public void activateIntakeServoTwo() {
        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_TWO);
    }

    public void deactivateIntakeServo() {
        intakeServo.setPosition(RobotConstants.INTAKE_POSITION_REST);
    }

    public void activateWristHoriz() {
        wrist.setPosition(RobotConstants.WRIST_POSITION_HORIZ);
    }
    public void activateWristVert() {
        wrist.setPosition(RobotConstants.WRIST_POSITION_VERT);
    }

    public void activateLift(int ticks) {
        // activateDumpServoHalf();
        liftMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        setLiftPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,LIFT_VELO_PID);

        double power = 1.0; //1.0;
        liftMotorR.setTargetPosition(ticks);
        liftMotorL.setTargetPosition(-ticks);
        liftMotorR.setTargetPositionTolerance(LIFT_TOLERANCE);
        liftMotorL.setTargetPositionTolerance(LIFT_TOLERANCE);
        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorR.setPower(power);
        liftMotorL.setPower(power);
    }

    public void deactivateLift() {
        int ticks = 150;
        double power = -1.0; //-1.0;

        liftMotorR.setTargetPosition(-ticks); // negative ticks for opposite direction
        liftMotorL.setTargetPosition(ticks); // negative ticks for opposite direction
        liftMotorR.setTargetPositionTolerance(LIFT_TOLERANCE);
        liftMotorL.setTargetPositionTolerance(LIFT_TOLERANCE);
        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorR.setPower(power);
        liftMotorL.setPower(power);
    }

    public void stopLift() {
        liftMotorL.setPower(0);
        liftMotorR.setPower(0);
    }

    public boolean isLiftSensorTouched() {
        // ***** if state == true, then "Not Pressed", Otherwise "Pressed" *****
//        if (sensor1.getState() == true) {
//            telemetry.addData("Digital Touch", "Is Not Pressed");
//        } else {
//            telemetry.addData("Digital Touch", "Is Pressed");
//        }

        boolean isTouched = false;
        if(horizSensorBottom.getState() == false) {
            isTouched = true;
        }
        return isTouched;
    }

    public boolean checkLiftMotion() {
        boolean isBottomReached = false;
        if (isLiftSensorTouched()) {
            stopLift();
            isBottomReached = true;
        }
        return isBottomReached;
    }

    public RRAutoDrive.PixelCount getPixelCount()
    {
        if (leftColorSensor.getDistance(DistanceUnit.CM) > 0.7 && rightColorSensor.getDistance(DistanceUnit.CM) > 0.7)
        {
            pixelCount = RRAutoDrive.PixelCount.ZERO;
        }
        else if (leftColorSensor.getDistance(DistanceUnit.CM) <= 0.7 && rightColorSensor.getDistance(DistanceUnit.CM) <= 0.7)
        {
            pixelCount = RRAutoDrive.PixelCount.TWO;
        }
        else
        {
            pixelCount = RRAutoDrive.PixelCount.ONE;
        }
        return pixelCount;
    }

    public boolean leftPixelContained()
    {
        if (leftColorSensor.getDistance(DistanceUnit.CM) <= 0.7)
            return true;
        else
            return false;
    }

    public boolean rightPixelContained()
    {
        if (rightColorSensor.getDistance(DistanceUnit.CM) <= 0.7)
            return true;
        else
            return false;
    }

    public void resetRuntime()
    {
        elapsedTime.reset();
    }

    public double elapsedMilliseconds()
    {
        return elapsedTime.milliseconds();
    }

    public double elapsedSeconds()
    {
        return elapsedTime.seconds();
    }

}