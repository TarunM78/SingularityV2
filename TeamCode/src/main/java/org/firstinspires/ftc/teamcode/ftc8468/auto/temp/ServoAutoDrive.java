package org.firstinspires.ftc.teamcode.ftc8468.auto.temp;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ftc8468.RobotConstants;
import org.firstinspires.ftc.teamcode.ftc8468.auto.RRAutoDrive;


public class ServoAutoDrive {

    protected Servo arm;
    protected DistanceSensor distanceSensor;
    public static final float sensor_position_0 = 0.25f;
    public static final float sensor_position_1 = 0.23f;
    public static final float sensor_position_2 = 0.21f;
    public static final float sensor_position_3 = 0.19f;
    public static final float sensor_position_4 = 0.17f;
    public static final float sensor_position_5 = 0.15f;
    public static final float sensor_position_6 = 0.13f;
    public static final float sensor_position_7 = 0.12f;
    public static final float sensor_position_8 = 0.11f;
    public static final float sensor_position_9 = 0.10f;

    public static final float arm_position_active = 0.60f;
    public static final float arm_position_inactive = 0.80f;



//    RRAutoDrive(HardwareMap hwMap, OpMode _opMode) {
//        this(hwMap);
//        opMode = _opMode;
//    }

    public ServoAutoDrive(HardwareMap hwMap) {
//        super(hwMap);
        arm = hwMap.get(Servo.class, "armServo");
        distanceSensor = hwMap.get(DistanceSensor.class, "colorSensor");

    }

    public boolean alignArm(Telemetry telemetry) {
        boolean isAligned = false;
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        while(distance > 5) {
            if(distance > 13) {
                arm.setPosition(sensor_position_1);
            } else if(distance > 12) {
                arm.setPosition(sensor_position_2);
            } else if(distance > 11) {
                arm.setPosition(sensor_position_3);
            } else if(distance > 10) {
                arm.setPosition(sensor_position_4);
            } else if(distance > 9) {
                arm.setPosition(sensor_position_5);
            } else if(distance > 8) {
                arm.setPosition(sensor_position_6);
            } else if(distance > 7) {
                arm.setPosition(sensor_position_7);
            } else if(distance > 6) {
                arm.setPosition(sensor_position_8);
            } else if(distance > 5) {
                arm.setPosition(sensor_position_9);
            }
            telemetry.addData("Distance: ", distance);
            telemetry.update();
            distance = distanceSensor.getDistance(DistanceUnit.CM);
        }
        isAligned = true;
        telemetry.addData("Distance - Before: ", distance+ " | Current: "+ distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();

        return isAligned;
    }

    public void activateArm() {
        arm.setPosition(arm_position_active);
    }
    public void deactivateArm() {
        arm.setPosition(arm_position_inactive);
    }
    public void stopArm() {
        arm.setPosition(arm.getPosition());
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    public boolean checkAndStopArm(Telemetry telemetry) {
        boolean isReached = false;
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        if(distance < 6) {
            isReached = true;
        }
        telemetry.addData("Distance: ", distance);
        return isReached;
    }

}