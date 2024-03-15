package org.firstinspires.ftc.teamcode.ftc8468;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class RobotConstants {
    public enum ZONE {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum ZONE_VIEW {
        LEFT,
        RIGHT
    }

    public static final String ALLIANCE_BLUE = "BLUE";
    public static final String ALLIANCE_RED = "RED";

    public static final float ARM_POSITION_ACTIVE = 0.07f;
    public static final float ARM_POSITION_DEACTIVE = 0.805f;
    public static final float ARM_POSITION_REST = .47f;
    public static final float ARM_POSITION_AUTO = .54f;
    public static final float ARM_POSITION_INIT = .70f;

    public static final float RED_CLAW_POSITION_ACTIVE = 0.30f;  // left
    public static final float RED_CLAW_POSITION_DEACTIVE = 0.65f;

    public static final float BLUE_CLAW_POSITION_ACTIVE = .67f;  // right
    public static final float BLUE_CLAW_POSITION_DEACTIVE = 0.32f;

    public static final float WRIST_POSITION_HORIZ = 0.34f;
    public static final float WRIST_POSITION_VERT = 0.07f;

    public static final float LEFT_CLIMB_POSITION_ACTIVE = 1.0f;
    public static final float LEFT_CLIMB_POSITION_DEACTIVE = -1.0f;


    public static final float RIGHT_CLIMB_POSITION_ACTIVE = -1.0f;
    public static final float RIGHT_CLIMB_POSITION_DEACTIVE = 1.0f;

    public static final float RIGHT_RAISE_CLIMB_POSITION_ACTIVE = 0.55f;
    public static final float RIGHT_RAISE_CLIMB_POSITION_DRONE = 0.80f;
    public static final float RIGHT_RAISE_CLIMB_POSITION_DEACTIVE = 0.92f;

    public static final float LEFT_RAISE_CLIMB_POSITION_ACTIVE = 0.88f;
    public static final float LEFT_RAISE_CLIMB_POSITION_DRONE = 0.68f;
    public static final float LEFT_RAISE_CLIMB_POSITION_DEACTIVE = 0.53f;

    public static final float INTAKE_POSITION_FIVE = 0.77f;
    public static final float INTAKE_POSITION_FOUR = 0.81f;
    public static final float INTAKE_POSITION_THREE = 0.842f;
    public static final float INTAKE_POSITION_TWO = 0.858f;
    public static final float INTAKE_POSITION_ACTIVE = 0.26f;
    public static final float INTAKE_POSITION_REST = 0.62f;

    public static final float DRONE_POSITION_ACTIVE = 1.0f;
    public static final float DRONE_POSITION_REST = 0.55f;

    public static final float FLAP_POSITION_ACTIVE = 0.57f;
    public static final float FLAP_POSITION_DEACTIVE = 0.2f;

    public static final float TRANSFERLOCK_POSITION_ACTIVE = 0.27f;
    public static final float TRANSFERLOCK_POSITION_DEACTIVE = 0.325f;

    public static final float TRANSFER_POSITION_ACTIVE = 0.265f;
    public static final float TRANSFER_POSITION_REST = 0.565f;

//    public static final float INTAKE_SERVO_POSITION_ACTIVE = 0.0f;
//    public static final float INTAKE_SERVO_POSITION_REST = 0.5f;

    public static final float LEFT_LINKAGE_POSITION_ACTIVE = 0.20f;
    public static final float LEFT_LINKAGE_POSITION_REST = 0.70f;
    public static final float LEFT_LINKAGE_POSITION_HALF = 0.55f;
    public static final float LEFT_LINKAGE_POSITION_SHARED = 0.65f;

    public static final float RIGHT_LINKAGE_POSITION_ACTIVE = 0.62f;
    public static final float RIGHT_LINKAGE_POSITION_REST = 0.20f;
    public static final float RIGHT_LINKAGE_POSITION_HALF = 0.35f;
    public static final float RIGHT_LINKAGE_POSITION_SHARED = 0.25f;

    public static final float RIGHT_ARM_POSITION_ACTIVE = 0.62f;
    public static final float RIGHT_ARM_POSITION_REST = 0.11f;
    public static final float RIGHT_ARM_POSITION_HALF = 0.52f;
    public static final float RIGHT_ARM_POSITION_EXACTHALF = 0.32f;
    public static final float RIGHT_ARM_POSITION_LITTLE = 0.21f;

    public static final float LEFT_ARM_POSITION_ACTIVE = 0.10f;
    public static final float LEFT_ARM_POSITION_REST = 0.60f;
    public static final float LEFT_ARM_POSITION_HALF = 0.20f;
    public static final float LEFT_ARM_POSITION_EXACTHALF = 0.30f;
    public static final float LEFT_ARM_POSITION_LITTLE = 0.50f;

    public static final float DUMP_SERVO_POSITION_REST = 0.20f;
    public static final float DUMP_SERVO_POSITION_HALF = 0.65f;
    public static final float DUMP_SERVO_POSITION_ACTIVE = 0.95f;

    public static final float KICKER_POSITION_OUT = 0.75f;
    public static final float KICKER_POSITION_REST = 0.28f;
    public static final float KICKER_POSITION_SOFTOUT = 0.48f;
    public static final float KICKER_POSITION_CLOSE = 0.10f;

    // RPM = 435; TICKS PER REV = 384.5
    // RPM = 1150; TICKS_PER_REV = 145.1;
    // RPM = 1620; TICKS_PER_REV = 103.8;
//    public static PIDFCoefficients LIFT_VELO_PID = new PIDFCoefficients(2.0, 0, 0, (32767 / (1150 / 60 * 145.1)));
//    public static PIDFCoefficients LIFT_VELO_PID = new PIDFCoefficients(2.0, 0, 0, (32767 / (435 / 60 * 384.5)));

}
