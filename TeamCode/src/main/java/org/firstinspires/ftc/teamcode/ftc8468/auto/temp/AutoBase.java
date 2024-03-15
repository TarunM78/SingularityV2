package org.firstinspires.ftc.teamcode.ftc8468.auto.temp;

import android.util.Size;

import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ftc8468.RobotConstants;
import org.firstinspires.ftc.teamcode.ftc8468.auto.RRAutoDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public abstract class AutoBase extends LinearOpMode {
    String currentAlliance = null;
    protected RobotConstants.ZONE zone = RobotConstants.ZONE.RIGHT;
    protected RobotConstants.ZONE_VIEW zoneView = RobotConstants.ZONE_VIEW.LEFT;
    int camW = 1280;
    int camH = 720;
    Size size = new Size(camW, camH);
    final double DESIRED_DISTANCE = 12.0;

    protected final int LIFT_MOTOR_TICKS = 425;
    protected RRAutoDrive drive;
    TrajectoryVelocityConstraint velCon = SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accCon = SampleMecanumDrive.getAccelerationConstraint(40);
    TrajectoryVelocityConstraint velConPixel = SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accConPixel = SampleMecanumDrive.getAccelerationConstraint(20);

    protected VisionPortal visionPortal;
    protected VisionPortal stackVisionPortal;
    protected VisionPortal tagVisionPortal;
    protected AprilTagProcessor aprilTagProcessor;
    protected TeamElementProcessor teamElementProcessor;
    protected StackPipelineProcessor stackPipelineProcessor;
    protected WebcamName webCam1;
    protected WebcamName webCam2;

//    protected TeamElementSubsystem teamElementDetection;

    protected ElapsedTime runtime = new ElapsedTime();

    // *** For moveRobot() ***
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    protected DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    protected DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    protected DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    protected DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    protected double manualDriveSpeed = 0.8;
    // *** For moveRobot() ***

    public AutoBase() {
    }

    protected abstract String getCurrentAlliance();

    protected abstract RobotConstants.ZONE_VIEW getZoneView();

    protected void resetTime() {
        runtime.reset();
    }

    protected void initialize() {
        drive = new RRAutoDrive(hardwareMap);
        initRobotDrive();
//        drive.activateLeftClaw();
//        drive.activateRightClaw();

//        initVisionPortal();
        initCV();
        telemetry.addData("Status:", "initialize() - Robot and Vision are initialized!");
        telemetry.update();
    }

    protected void initRobotDrive() {
        // *** For moveRobot() ***
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftRear");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        // *** For moveRobot() ***
    }

    protected void initVisionPortal() {
        webCam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webCam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        teamElementProcessor = new TeamElementProcessor(getCurrentAlliance(), getZoneView(), telemetry);

//        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
//        aprilTagProcessor.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(webCam2)
                .setCameraResolution(size)
                .addProcessor(teamElementProcessor)
//                .addProcessor(aprilTagProcessor)
                .build();
    }

    protected void initCV() {
        // Front Camera
        webCam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Rear Camera
        webCam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        teamElementProcessor = new TeamElementProcessor(getCurrentAlliance(), getZoneView(), telemetry);

        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        aprilTagProcessor.setDecimation(2);

        int[] visionViewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        visionPortal = new VisionPortal.Builder()
                .setCamera(webCam1)
                .setCameraResolution(size)
                .setLiveViewContainerId(visionViewIds[0])
//                .enableLiveView(false)
//                .setAutoStopLiveView(true)
                .addProcessor(teamElementProcessor)
                .build();

        tagVisionPortal = new VisionPortal.Builder()
                .setCamera(webCam2)
                .setCameraResolution(size)
                .setLiveViewContainerId(visionViewIds[1])
//                .enableLiveView(false)
//                .setAutoStopLiveView(true)
                .addProcessor(aprilTagProcessor)
                .build();
//        stopSreaming(tagVisionPortal);

    }

    protected void stopSreaming(VisionPortal portal) {
        while(!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING) ) {
            sleep(20);
        }
        portal.stopStreaming();
        while(!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_READY) ) {
            sleep(20);
        }
    }

//    protected void initTagVisionPortal() {
//        webCam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
//        aprilTagProcessor = new AprilTagProcessor.Builder().build();
//
//        tagVisionPortal = new VisionPortal.Builder()
//                .setCamera(webCam2)
//                .setCameraResolution(size)
//                .addProcessor(aprilTagProcessor)
//                .build();
//    }

//    protected void initSwitchableVisionPortal() {
//        webCam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
//        webCam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
//        Size size = new Size(camW, camH);
//
//        aprilTagProcessor = new AprilTagProcessor.Builder().build();
//        // Adjust Image Decimation to trade-off detection-range for detection-rate.
//        // eg: Some typical detection data using a Logitech C920 WebCam
//        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
//        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
//        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
//        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
//        // Note: Decimation can be changed on-the-fly to adapt during a match.
//        aprilTagProcessor.setDecimation(2);
//
//        CameraName switchableCamera = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(webCam1, webCam2);
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(switchableCamera)
//                .setCameraResolution(size)
//                .addProcessor(aprilTagProcessor)
//                .build();
//    }

//    protected void initVisionPortal() {
//        webCam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
//        stackPipelineProcessor = new StackPipelineProcessor();
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(webCam2)
//                .setCameraResolution(new Size(1920, 1080))
//                .addProcessor(stackPipelineProcessor)
//                .build();
//    }

//    protected void initStackVisionPortal() {
//        webCam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
//        Size size = new Size(camW, camH);
//        stackPipelineProcessor = new StackPipelineProcessor();
//
//        stackVisionPortal = new VisionPortal.Builder()
//                .setCamera(webCam2)
//                .setCameraResolution(new Size(1920, 1080))
//                .addProcessor(stackPipelineProcessor)
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .enableLiveView(true)
//                .setAutoStopLiveView(true)
//                .build();
////        FtcDashboard.getInstance().startCameraStream(stackPipelineProcessor, 0);
//    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER initializing the AprilTagProcessor, and only works for Webcams;
    */
    protected void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    public RobotConstants.ZONE getElementZone() {
        this.zone = teamElementProcessor.getElementZone();
        return this.zone;
    }


    public int getTargetTagId(RobotConstants.ZONE identifiedZone) {
        int tagId = 0;
        if(RobotConstants.ALLIANCE_BLUE.equalsIgnoreCase(getCurrentAlliance())) {
            switch (identifiedZone) {
                case LEFT:
                    tagId = 1;
                    break;
                case CENTER:
                    tagId = 2;
                    break;
                case RIGHT:
                    tagId = 3;
                    break;
                default:
                    tagId = 1;
            }
        } else { // RED
            switch (identifiedZone) {
                case LEFT:
                    tagId = 4;
                    break;
                case CENTER:
                    tagId = 5;
                    break;
                case RIGHT:
                    tagId = 6;
                    break;
                default:
                    tagId = 4;
            }
        }
    return tagId;
    }

    protected AprilTagDetection getDesiredTagForZone(RobotConstants.ZONE identifiedZone) {
        AprilTagDetection matchingTag = null;
        // get the targetId of the tag for the zone...
        int targetId = getTargetTagId(identifiedZone);
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("identifiedZone: ", identifiedZone+" TargetTagId: "+targetId+" Count: "+currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((targetId < 0) || (detection.id == targetId)) {
//                    targetFound = true;
                    matchingTag = detection;
                    telemetry.addData("Matching Tag Found, Id is: ", matchingTag.id);
                    break;
                }
//                else {
//                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
//                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        telemetry.update();
        return matchingTag;
    }

    protected AprilTagPoseFtc getDesiredTagPose(RobotConstants.ZONE identifiedZone) {
        AprilTagPoseFtc tagPoseFtc;
        AprilTagDetection desiredTag = getDesiredTagForZone(identifiedZone);
        if(desiredTag != null ) {
            tagPoseFtc = desiredTag.ftcPose;
            telemetry.addData("Tag Id: ", desiredTag.id+" Range: "+desiredTag.ftcPose.range);
            telemetry.update();
        } else {
            // return hardcoded PoseFtc for Blue or Red...
            if(RobotConstants.ALLIANCE_BLUE.equals(getCurrentAlliance())) {
                tagPoseFtc = new AprilTagPoseFtc(0,0,0,0,0,0,0,0,0);
            } else{
                tagPoseFtc = new AprilTagPoseFtc(0,0,0,0,0,0,0,0,0);
            }
        }
        return tagPoseFtc;
    }

    protected void telemetryAprilTag(AprilTagDetection aprilTag) {
        if(aprilTag != null && aprilTag.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", aprilTag.id, aprilTag.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", aprilTag.ftcPose.x, aprilTag.ftcPose.y, aprilTag.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", aprilTag.ftcPose.pitch, aprilTag.ftcPose.roll, aprilTag.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", aprilTag.ftcPose.range, aprilTag.ftcPose.bearing, aprilTag.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("AprilTag is null or it has no metadata"));
        }
        telemetry.update();
    }

    public void closeVision() {
        visionPortal.close();
//        stackVisionPortal.close();
    }

    // *** For moveRobot() ***
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
//        leftFrontDrive.setPower(leftFrontPower);
//        rightFrontDrive.setPower(rightFrontPower);
//        leftBackDrive.setPower(leftBackPower);
//        rightBackDrive.setPower(rightBackPower);
        leftFrontDrive.setPower(leftFrontPower * manualDriveSpeed);
        rightFrontDrive.setPower(rightFrontPower * manualDriveSpeed);
        leftBackDrive.setPower(leftBackPower * manualDriveSpeed);
        rightBackDrive.setPower(rightBackPower * manualDriveSpeed);
    }

    public void stopRobot() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    // *** For moveRobot() ***

}
