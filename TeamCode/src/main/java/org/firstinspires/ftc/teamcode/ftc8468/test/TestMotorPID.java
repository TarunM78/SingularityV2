package org.firstinspires.ftc.teamcode.ftc8468.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class TestMotorPID extends OpMode {

//    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;

    private DcMotorEx liftMotorL;
    private DcMotorEx liftMotorR;

    private final double valueF = (32767 / (435 / 60 * 384.5));
//
//    @Override
    public void init() {
//        controller = new PIDController(p, i, d);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        liftMotorL = hardwareMap.get(DcMotorEx.class, "liftMotorL");
        liftMotorR = hardwareMap.get(DcMotorEx.class, "liftMotorR");
    }

    public void loop() {
//        controller.setPID(p, i , d);
//        int positionL = liftMotorL.getCurrentPosition();
//        int positionR = liftMotorR.getCurrentPosition();
//        double pidL = controller.calculate(positionL, target);
//        double pidR = controller.calculate(positionR, target);
//        double ffL = Math.cos(Math.toRadians(target/valueF)) * f;
//        double ffR = Math.cos(Math.toRadians(target/valueF)) * f;
//
//        liftMotorL.setPower(pidL * ffL);
//        liftMotorR.setPower(pidR * ffR);
//
//        telemetry.addData("posL | posR: ", positionL + " | " + positionR);
//        telemetry.addData("target: ", target);
    }
}
