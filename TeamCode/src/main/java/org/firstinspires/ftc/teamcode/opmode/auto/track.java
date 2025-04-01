package org.firstinspires.ftc.teamcode.opmode.auto;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.Arrays;
import java.util.List;

import android.util.Size;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;


public class track extends OpMode {

    private double adjust_x = 0;
    private double adjust_y = 0;
    public double fix = 0;
    private long time2 = 0;
    public static boolean check = false;
    ColorBlobLocatorProcessor.Builder myColorBlobLocatorProcessorBuilder;
    VisionPortal.Builder myVisionPortalBuilder;
    ColorBlobLocatorProcessor myColorBlobLocatorProcessor;
    VisionPortal myVisionPortal;
    List<ColorBlobLocatorProcessor.Blob> myBlobs;
    ColorBlobLocatorProcessor.Blob myBlob;
    RotatedRect myBoxFit;
    org.opencv.core.Rect mySize;
    boolean vertical = true;
    org.opencv.core.Point[] myPoints;
    public double max_area = 6000;
    public double min_area = 4500;
    public double Perfect_X = 142;
    public double Perfect_Y = 199;
    public double ti_me = 100.0;

    public List data_right;
    List data_return = Arrays.asList(0,0,0.0,0.0);

    public double right_x = 0;
    public double right_y = 0;

    public track(HardwareMap hardwareMap , ColorRange colorRange1) {
        this.hardwareMap = hardwareMap;
        initialize(colorRange1);
    }

    public void initialize(ColorRange colorRange1) {

        myColorBlobLocatorProcessorBuilder = new ColorBlobLocatorProcessor.Builder();
        myColorBlobLocatorProcessorBuilder.setTargetColorRange(colorRange1);
        myColorBlobLocatorProcessorBuilder.setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1));
        myColorBlobLocatorProcessorBuilder.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY);
        myColorBlobLocatorProcessorBuilder.setDrawContours(true);
        myColorBlobLocatorProcessorBuilder.setBlurSize(5);
        myColorBlobLocatorProcessorBuilder.setErodeSize(5);
        myColorBlobLocatorProcessorBuilder.setDilateSize(5);
        myColorBlobLocatorProcessor = myColorBlobLocatorProcessorBuilder.build();

        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.addProcessor(myColorBlobLocatorProcessor);
        myVisionPortalBuilder.setCameraResolution(new Size(320, 240));
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        myVisionPortal = myVisionPortalBuilder.build();
        data_right = JavaUtil.createListWith();
        // Speed up telemetry updates, Just use for debugging.
        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {}


    public List track(boolean check_verticle){
        data_right.removeAll(data_right);
        data_return = Arrays.asList(0,0,0.0,0.0,0);

        time2 = System.currentTimeMillis();
        while ((System.currentTimeMillis() - time2) <= ti_me) {
            myBlobs = myColorBlobLocatorProcessor.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByArea(min_area, 40000, myBlobs);
            for (ColorBlobLocatorProcessor.Blob myBlob_item : myBlobs) {
                myBlob = myBlob_item;
                myBoxFit = myBlob.getBoxFit();
                mySize = myBoxFit.boundingRect();
                {
                    myPoints = new org.opencv.core.Point[4];
                    myBoxFit.points(myPoints);
                }
                if (mySize.width / mySize.height > 1) {vertical = false;}
                else if (mySize.width / mySize.height < 1){vertical = true;}

                if (check_verticle == vertical){
                    data_right.add(JavaUtil.createListWith(Math.round(Math.sqrt(Math.pow(Perfect_X - myBoxFit.center.x, 2) + Math.pow(Perfect_Y - myBoxFit.center.y, 2))), vertical, myBoxFit.center.x, myBoxFit.center.y));
                }



            }
            if (JavaUtil.listLength(data_right) >= 1) {
                data_right = JavaUtil.sort(data_right, JavaUtil.SortType.NUMERIC, JavaUtil.SortDirection.ASCENDING);
                break;
            }
        }



        if (JavaUtil.listLength(data_right) != 0) {
            if (((Boolean) JavaUtil.inListGet((((List) JavaUtil.inListGet(data_right, JavaUtil.AtMode.FROM_START, (int) 0, false))), JavaUtil.AtMode.FROM_START, (int) 1, false)).booleanValue() == true) {
                data_return.set(0,0);

                if ((160.0 - ((Double) JavaUtil.inListGet((((List) JavaUtil.inListGet(data_right, JavaUtil.AtMode.FROM_START, (int) 0, false))), JavaUtil.AtMode.FROM_START, (int) 2, false)).doubleValue()) > 0){
                   //right
                    if ((120.0 - ((Double) JavaUtil.inListGet((((List) JavaUtil.inListGet(data_right, JavaUtil.AtMode.FROM_START, (int) 0, false))), JavaUtil.AtMode.FROM_START, (int) 3, false)).doubleValue()) > 0){
                        //Front
                        data_return.set(4,0);
                        right_x = 0.8;
                        right_y = 0.8;
                    }
                    else{
                        //Back
                        data_return.set(4,1);
                        right_x = 1;
                        right_y = 1;
                    }
                }
                else{
                    //left
                    if ((120.0 - ((Double) JavaUtil.inListGet((((List) JavaUtil.inListGet(data_right, JavaUtil.AtMode.FROM_START, (int) 0, false))), JavaUtil.AtMode.FROM_START, (int) 3, false)).doubleValue()) > 0){
                        //Front
                        data_return.set(4,2);
                        right_x = 0.8;
                        right_y = 0.9;
                    }
                    else{
                        //Back
                        data_return.set(4,3);
                        right_x = 0.85;
                        right_y = 0.9;
                    }
                }
            }

            if ((Long)JavaUtil.inListGet((((List) JavaUtil.inListGet(data_right, JavaUtil.AtMode.FROM_START, (int) 0, false))), JavaUtil.AtMode.FROM_START, (int) 0, false) <= 20) {
                data_return.set(1,1);
            }
            else {
                data_return.set(1, 2);
                adjust_y = ((Perfect_Y - ((Double) JavaUtil.inListGet((((List) JavaUtil.inListGet(data_right, JavaUtil.AtMode.FROM_START, (int) 0, false))), JavaUtil.AtMode.FROM_START, (int) 3, false)).doubleValue()) * 0.029166 * right_y) * -0.5;
                adjust_x = ((Perfect_X - ((Double) JavaUtil.inListGet((((List) JavaUtil.inListGet(data_right, JavaUtil.AtMode.FROM_START, (int) 0, false))), JavaUtil.AtMode.FROM_START, (int) 2, false)).doubleValue()) * 0.01875 * right_x) * -0.71;
            }

        }
        else {
            data_return.set(1,3);
        }
        data_return.set(2,adjust_x);
        data_return.set(3,adjust_y);
        return data_return ;
    }
}


