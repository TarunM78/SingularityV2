package org.firstinspires.ftc.teamcode.ftc8468.auto.temp;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.ftc8468.RobotConstants;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import java.util.Arrays;
import java.util.List;

public class TeamElementProcessor implements VisionProcessor {

    RobotConstants.ZONE_VIEW zoneView;
    public RobotConstants.ZONE color_zone = RobotConstants.ZONE.LEFT; //1;
    String alliance;
    Telemetry telemetry;

    // *** Element Color is defaulted to RED ***
    List<Integer> ELEMENT_COLOR = Arrays.asList(255, 0, 0); //(red, green, blue)

    Mat zone1;
    Mat zone2;
    Scalar avgColor1;
    Scalar avgColor2;

    double distance1 = 1;
    double distance2 = 1;
    double max_distance = 0;

//    public enum ZONE {
//        LEFT,
//        CENTER,
//        RIGHT
//    }
//
//    public enum ZONE_VIEW {
//        LEFT,
//        RIGHT
//    }

    public TeamElementProcessor(RobotConstants.ZONE_VIEW zoneView)
    {
        this.zoneView = zoneView;
    }

    public TeamElementProcessor(String alliance, RobotConstants.ZONE_VIEW zoneView)
    {
        this(zoneView);
        setAlliance(alliance);
    }

    public TeamElementProcessor(String alliance, RobotConstants.ZONE_VIEW zoneView, Telemetry telemetry)
    {
        this(alliance, zoneView);
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return processFrame(frame);
    }

    public void setAlliance(String alliance){
        this.alliance = alliance;
        if (alliance.equalsIgnoreCase("RED")){
            ELEMENT_COLOR = Arrays.asList(255, 0, 0);
        }else{
            ELEMENT_COLOR = Arrays.asList(0, 0, 255);
        }
    }

    public Mat processFrame(Mat input) {

        //Creating duplicate of original frame with no edits
//        original = input.clone();

        //input = input.submat(new Rect(0));

        //Defining Zones
        //Rect(top left x, top left y, bottom right x, bottom right y)
//        zone1 = input.submat(new Rect(60, 170, 356, 285));
//        zone2 = input.submat(new Rect(735, 170, 253, 230));
        if (zoneView == RobotConstants.ZONE_VIEW.LEFT) {
            zone1 = input.submat(new Rect(900, 280, 200, 250));
            zone2 = input.submat(new Rect(420, 270, 200, 250));
        }
        else {
            zone1 = input.submat(new Rect(950, 290, 200, 250));
            zone2 = input.submat(new Rect(480, 270, 200, 250));
        }

        //Averaging the colors in the zones
        avgColor1 = Core.mean(zone1);
        avgColor2 = Core.mean(zone2);

        //Putting averaged colors on zones (we can see on camera now)
        zone1.setTo(avgColor1);
        zone2.setTo(avgColor2);

        distance1 = color_distance(avgColor1, ELEMENT_COLOR);
        distance2 = color_distance(avgColor2, ELEMENT_COLOR);

        // *** Positions ***
        // distance1 is for Middle (2)
        // distance2 is for Right (1)
        // Left = 3;    Middle = 2;     Right = 1;
        // Robot is positioned to view the Middle (2) and Right (1) Elements.
        // If the Element is in the Left, then the position is 3
        // Default position is also 3
        if (zoneView == RobotConstants.ZONE_VIEW.LEFT) {
            if ((distance1 > 195) && (distance2 > 190)) {
                color_zone = RobotConstants.ZONE.RIGHT; // Left
                max_distance = -1;
            } else {
                max_distance = Math.min(distance1, distance2);

                if (max_distance == distance1) {
                    //telemetry.addData("Zone 1 Has Element", distance1);
                    color_zone = RobotConstants.ZONE.CENTER; // Middle

                } else {
                    //telemetry.addData("Zone 2 Has Element", distance2);
                    color_zone = RobotConstants.ZONE.LEFT; // Right
                }
            }
        }
        else {
            if ((distance1 > 195) && (distance2 > 190)) {
                color_zone = RobotConstants.ZONE.LEFT; // Left
                max_distance = -1;
            } else {
                max_distance = Math.min(distance1, distance2);

                if (max_distance == distance1) {
                    //telemetry.addData("Zone 1 Has Element", distance1);
                    color_zone = RobotConstants.ZONE.RIGHT; // Middle

                } else {
                    //telemetry.addData("Zone 2 Has Element", distance2);
                    color_zone = RobotConstants.ZONE.CENTER; // Right
                }
            }
        }
        return input;
    }

    public double color_distance(Scalar color1, List color2) {
        double r1 = color1.val[0];
        double g1 = color1.val[1];
        double b1 = color1.val[2];

        int r2 = (int) color2.get(0);
        int g2 = (int) color2.get(1);
        int b2 = (int) color2.get(2);

        return Math.sqrt(Math.pow((r1 - r2), 2) + Math.pow((g1 - g2), 2) + Math.pow((b1 - b2), 2));
    }

    public RobotConstants.ZONE getElementZone(){
        if(telemetry != null) {
            telemetry.addData("Alliance: ", alliance);
            telemetry.addData("ZoneView: ", zoneView);
            telemetry.addData("Zone: ", color_zone);
//            telemetry.addData("distance1: ", distance1);
//            telemetry.addData("distance2: ", distance2);
//            telemetry.addData("max_distance: ", max_distance);
        }
        return color_zone;
    }
}
