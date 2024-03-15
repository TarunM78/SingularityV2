package org.firstinspires.ftc.teamcode.ftc8468.auto.temp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ftc8468.RobotConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Autonomous (name = "AutoBlueVisionTest")
public class AutoBlueVisionTest extends AutoBase {
    TrajectorySequence trajSeqCenter1, trajSeqCenter2, trajSeqLeft1, trajSeqLeft2, trajSeqRight1, trajSeqRight2;

    protected String getCurrentAlliance() {
        return RobotConstants.ALLIANCE_BLUE;
    }

    protected RobotConstants.ZONE getZone() {
        return zone;
    }

    protected RobotConstants.ZONE_VIEW getZoneView() {
        return RobotConstants.ZONE_VIEW.LEFT;
    }

    public void runOpMode() {
        // initialize Drive and Vision Subsystems...
        initialize();

//        visionPortal.setProcessorEnabled(aprilTagProcessor, false);

        while (!isStarted() && !isStopRequested()) {
//            visionPortal.resumeLiveView();
//            visionPortal.resumeStreaming();
//            visionPortal.resumeLiveView();
            zone = getElementZone();
            telemetry.addData("Current Alliance: ", getCurrentAlliance()+" Zone: "+zone);
            telemetry.update();
        }

        if (getCurrentAlliance() == "BLUE") {
            Pose2d startPose = new Pose2d(18, 64, Math.toRadians(270));
            drive.setPoseEstimate(startPose);

//            trajSeqCenter1 = drive.trajectorySequenceBuilder(startPose)
//                    .lineTo(new Vector2d(18, 30), velConPixel, accConPixel)
//                    .lineTo(new Vector2d(54, 34), velConPixel, accConPixel)
//                    .build();

            trajSeqCenter1 = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(18, 31), velConPixel, accConPixel)
                    .lineToLinearHeading(new Pose2d(35, 32, Math.toRadians(180)), velConPixel, accConPixel)
                    .waitSeconds(1)
                    .build();

//            trajSeqCenter2 = drive.trajectorySequenceBuilder(trajSeqCenter1.end())
//                    .lineTo(new Vector2d(53, 34), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
//                    .lineTo(new Vector2d(48, 62))
//                    .lineTo(new Vector2d(60, 62))
//                    .build();
        }
        else {
            // do nothing!
        }

        waitForStart();

//        visionPortal.stopLiveView();
        resetTime();

        if (isStopRequested()) return;

        visionPortal.stopStreaming();
        visionPortal.stopLiveView();
        tagVisionPortal.resumeStreaming();
        tagVisionPortal.resumeLiveView();

        // place the Purple Pixel on the correct zone and move close to the Backdrop.
        if (zone == RobotConstants.ZONE.LEFT)
//            drive.followTrajectorySequence(trajSeqLeft);
            telemetry.addData("Zone: ", zone);
        else if (zone == RobotConstants.ZONE.CENTER) {
            telemetry.addData("Zone: ", zone);
            drive.followTrajectorySequence(trajSeqCenter1);
        }
        else
//            drive.followTrajectorySequence(trajSeqRight);
            telemetry.addData("Zone: ", zone);

        telemetry.update();
        drive.update();
        Pose2d currentPose = drive.getPoseEstimate();

        sleep(2000);

        // get the desired AprilTag
        AprilTagDetection desiredTag = getDesiredTagForZone(zone);
        telemetryAprilTag(desiredTag);
        if(desiredTag != null) {
            AprilTagPoseFtc poseFtc = desiredTag.ftcPose;
            // check to see if the robot needs to be moved..
            double rangeError = (poseFtc.range - 5);
            double headingError = poseFtc.bearing;
            double yawError = poseFtc.yaw;
            // Decide to move the robot either by tolerance or for certain time (Ex: 1000 ms)
            telemetry.addData("Tag Data", "Range %5.2f, Yaw %5.2f, Heading %5.2f ", rangeError, yawError, headingError);
            telemetry.update();

            resetTime();
//            if (Math.abs(rangeError) > 3) {
//        if(rangeError > 3 || headingError > 1 || yawError > 5) { // if(runtime.milliseconds() < 1000) {
                // move the robot...
                // move using moveRobot() or see if you can use TrajectorySequence...
                // Use the speed and turn "gains" to calculate how we want the robot to move.
                double driveDistance = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", driveDistance, strafe, turn);
                telemetry.update();
            moveRobot(driveDistance, strafe, turn);
//            }
        }

        // Drop the Pixel...

        // Decide on the next move...
        drive.update();
        currentPose = drive.getPoseEstimate();
        // Decide on the next move - Park or Go for Cycles...
//        trajSeqCenter2 = drive.trajectorySequenceBuilder(currentPose)
//                .lineTo(new Vector2d(53, 34), SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(5))
//                .lineTo(new Vector2d(48, 62))
//                .lineTo(new Vector2d(60, 62))
//                .build();
//
//        if (zone == RobotConstants.ZONE.LEFT)
////            drive.followTrajectorySequence(trajSeqLeft);
//            telemetry.addData("Zone: ", zone);
//        else if (zone == RobotConstants.ZONE.CENTER) {
//            drive.followTrajectorySequence(trajSeqCenter2);
//        }
//        else
////            drive.followTrajectorySequence(trajSeqRight);
//            telemetry.addData("Zone: ", zone);

        telemetry.update();
//        closeVision();
    }
}
