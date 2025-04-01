package org.firstinspires.ftc.teamcode.opmode.auto;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import java.util.List;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import java.util.List;
import java.util.concurrent.TransferQueue;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;


@Autonomous(name = "S5")
public class S5 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private int mec;
    private double adjust_x;
    private double adjust_y;
    private Servo rightservo;
    private Servo leftservo;
    private Servo neep;
    private Servo spin;
    private Servo wrist;
    private Servo P3;
    private DcMotor L2 ;
    private DcMotor L1 ;
    private long time2;
    private boolean check = false;
    private boolean debug = false;
    ColorBlobLocatorProcessor.Builder myColorBlobLocatorProcessorBuilder;
    VisionPortal.Builder myVisionPortalBuilder;
    ColorBlobLocatorProcessor myColorBlobLocatorProcessor;
    VisionPortal myVisionPortal;
    List<ColorBlobLocatorProcessor.Blob> myBlobs;
    ColorBlobLocatorProcessor.Blob myBlob;
    RotatedRect myBoxFit;
    Rect mySize;
    boolean vertical;
    org.opencv.core.Point[] myPoints;
    double angle = 0;
    double count = 0;
    double area = 6000;
    double Perfect_X;
    double Perfect_Y;
    List data_right;
    List data_middle;

    double middle_y = 0;
    double middle_x = 0;
    double right_x = 0;
    double right_y = 0;

    private track track_color ;
    private List keeper = JavaUtil.createListWith();
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose set_Sample = new Pose(-18, 4, Math.toRadians(45));
    private final Pose b_final = new Pose(-21.5, 55, Math.toRadians(55));
    private  Pose final0 = new Pose(7.28,55.72,Math.toRadians(0));
    private Pose keep_S5 = new Pose(6.28,51.72,Math.toRadians(0));
    private final Pose set_Sample5 = new Pose(-25, 5, Math.toRadians(45));
    private  Pose final1 = new Pose(7.28,55.72,Math.toRadians(0));
    private Pose keep_S6 = new Pose(7.28,55.72,Math.toRadians(0));
    private final Pose set_Sample6 = new Pose(-25, 5, Math.toRadians(45));






    private Path Point1_set;
    private PathChain keepS2, keepS3, Point2_set,  Point3_set, keepS4,Point4_set,finale,keep6,set_Sample_6,hangSp2,keepSp3,hangSp3,hangSp4,keep6_setkeep,keepSp1,keepSp2,Finish,keep5,set_Sample_5;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        Point1_set = new Path(new BezierLine(new Point(startPose), new Point(set_Sample)));
        Point1_set.setLinearHeadingInterpolation(startPose.getHeading(), set_Sample.getHeading());

        finale = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(set_Sample),new Point(b_final), new Point(final0)))
                .setLinearHeadingInterpolation(set_Sample.getHeading(), final0.getHeading())
                .build();

        keep5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(final0), new Point(keep_S5)))
                .setLinearHeadingInterpolation(final0.getHeading(), keep_S5.getHeading())
                .build();
        set_Sample_5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(keep_S5),new Point(b_final), new Point(set_Sample5)))
                .setLinearHeadingInterpolation(keep_S5.getHeading(), set_Sample5.getHeading())
                .build();
        keep6 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(set_Sample5),new Point(b_final), new Point(final1)))
                .setLinearHeadingInterpolation(set_Sample5.getHeading(), final1.getHeading())
                .build();
        keep6_setkeep = follower.pathBuilder()
                .addPath(new BezierLine(new Point(final1), new Point(keep_S6)))
                .setLinearHeadingInterpolation(final1.getHeading(), keep_S6.getHeading())
                .build();

        set_Sample_6 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(keep_S6),new Point(b_final), new Point(set_Sample6)))
                .setLinearHeadingInterpolation(keep_S6.getHeading(), set_Sample6.getHeading())
                .build();



    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.setMaxPower(1);
                Servo_kan(0.4);
                wrist.setPosition(1);
                neep.setPosition(0.3);
                spin.setPosition(0);
                follower.followPath(Point1_set);
                setPathState(1);
            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(finale);
                    Servo_kan(0.15);
                    setPathState(80);
                }

            case 80:
                if (!follower.isBusy()) {
                    upread();
                    Thread.sleep(800);
                    final0 = new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(0));
                    follower.setMaxPower(0.8);

                    Thread.sleep(300);
                    keeper = track_color.track(true);
                    telemetry.addData("Read",keeper);
                    if ((Integer) keeper.get(0) == 0){
                        spin.setPosition(0);
                    }
                    else if (((Integer) keeper.get(0) == 1)){
                        spin.setPosition(0.5);
                    }
                    Thread.sleep(300);
                    if ((Integer) keeper.get(1) == 1){
                            setPathState(802);
                    } else{
                        keep_S5 = new Pose(follower.getPose().getX() + (double)keeper.get(3),follower.getPose().getY() + (double)keeper.get(2),0);
                        keep5 = follower.pathBuilder()
                                .addPath(new BezierLine(new Point(final0), new Point(keep_S5)))
                                .setLinearHeadingInterpolation(final0.getHeading(), keep_S5.getHeading())
                                .build();
                        follower.followPath(keep5);
                        if ((Integer) keeper.get(1) == 2){
                            setPathState(802);
                        }
                        else{
                            keep_S5 = new Pose(follower.getPose().getX(),follower.getPose().getY() + 3,0);
                            keep5 = follower.pathBuilder()
                                    .addPath(new BezierLine(new Point(final0), new Point(keep_S5)))
                                    .setLinearHeadingInterpolation(final0.getHeading(), keep_S5.getHeading())
                                    .build();
                            follower.followPath(keep5);
                            setPathState(80);
                        }
                    }

                }
                break;
            case 802:
                if (!follower.isBusy()) {
                    downkeep();
                    setPathState(-8);

                }
                break;
            case 803:
                P3.setPosition(0.16);
                Thread.sleep(1000);
                setPathState(-1);
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(set_Sample_5);
                    setPathState(201);
                }
                break;
            case 201:
                if (Math.abs(follower.getPose().getX()) > Math.abs(set_Sample5.getX()) - 16 && follower.getPose().getY() < (set_Sample5.getY() + 30)) {
                    setMec(0);
                    setPathState(202);
                }
                break;
            case 202:
                if (!follower.isBusy()) {
                    Thread.sleep(500);
                    neep.setPosition(0);
                    Thread.sleep(500);
                    Servo_kan(0.15);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    Servo_kan(0.15);
                    wrist.setPosition(1);
                    follower.followPath(keep6);
                    setMec(1);
                    setPathState(80);
                }


        }
    }
    public void mecpath() {
        switch (mec) {
            case 0:
                upliftset(1450);
                break;
            case 1:
                downlift();
                break;
        }
    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void setMec(int pMec) {
        mec = pMec;
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {


        // These loop the movements of the robot
        follower.update();

        try {
            autonomousPathUpdate();
            mecpath();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("lift1",L1.getCurrentPosition());
        telemetry.addData("lift2",L2.getCurrentPosition());
        telemetry.addData("Position",(Integer) keeper.get(4));
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        track_color = new track(hardwareMap,ColorRange.YELLOW);
        follower.setStartingPose(startPose);

        buildPaths();
        rightservo = hardwareMap.get(Servo.class, "rightservo");
        leftservo = hardwareMap.get(Servo.class, "leftservo");
        wrist = hardwareMap.get(Servo.class, "wrist");
        neep = hardwareMap.get(Servo.class, "neep");
        spin = hardwareMap.get(Servo.class, "spin");
        L1 = hardwareMap.get(DcMotor.class, "L1");
        L2 = hardwareMap.get(DcMotor.class, "L2");
        P3 = hardwareMap.get(Servo.class, "P3");
        wrist.setDirection(Servo.Direction.REVERSE);
        L1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L1.setDirection(DcMotor.Direction.REVERSE);
        L1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

    }

    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(80);
        setMec(-1);

    }

    private void Servo_kan(double right) {
        rightservo.setPosition(right);
        leftservo.setPosition(1 - right);
    }


    private void upread(){
        while (true) {
            if (L2.getCurrentPosition() <= 400) {
                L1.setPower(0.6);
                L2.setPower(0.6);
                wrist.setPosition(0.8);
                neep.setPosition(0.2);
                Servo_kan(0);
            } else {
                L1.setPower(0.05);
                L2.setPower(0.05);
                L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            }}
    }
    private void downkeep() throws InterruptedException{
        wrist.setPosition(1);
        Thread.sleep(500);
        while (true) {
            if (Math.abs(L2.getCurrentPosition()) >= 15) {
                L1.setPower(-0.4);
                L2.setPower(-0.4);
            } else {
                L1.setPower(0);
                L2.setPower(0);
                L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            }
        }
        Thread.sleep(700);
        neep.setPosition(1);
        Thread.sleep(500);
        Servo_kan(0.15);
        Thread.sleep(300);
        setPathState(-1);
    }
    private void upliftset(int up) {
        wrist.setPosition(0.43);
        spin.setPosition(0);
        if (L2.getCurrentPosition() < up) {
            Servo_kan(0.3);
            L1.setPower(1);
            L2.setPower(1);
        } else {
            Servo_kan(0.51);
            L1.setPower(0.05);
            L2.setPower(0.05);
            L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            setMec(-1);
        }
    }
    private void downlift(){
        if (Math.abs(L2.getCurrentPosition()) >= 25) {
            L1.setPower(-1);
            L2.setPower(-1);
        }
        else {
            L1.setPower(0);
            L2.setPower(0);
            L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            setMec(-1);
        }

    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        ;
    }
}
