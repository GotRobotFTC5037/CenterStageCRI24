package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous(name = "Far Auto", group = "Auto")
public class farAuto extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Hardware robot = new Hardware();
    private propPositions propPosition;

    boolean isBlue = true;
    int yMultiplier = 1;
    double sideSpecificHeading = 0;
    double tangentAdd = 0;

    public enum propPositions {
        LEFT,
        RIGHT,
        CENTER
    }

    private static int DESIRED_TAG_ID = -1;
    private static final int BLUELEFTTAG = 1;
    private static final int BLUECENTERTAG = 2;
    private static final int BLUERIGHTTAG = 3;

    private static  int REDLEFTTAG = 4;
    private static  int REDCENTERTAG = 5;
    private static  int REDRIGHTTAG = 6;

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private boolean targetFound = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.droneAngle.setPosition(robot.droneAngleDown);
        robot.launcherRelease.setPosition(robot.launchClosed);
        robot.stripper.setPosition(robot.stripperOpen);
        robot.hook.setPosition(robot.hookDown);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //isBlue = robot.marker.getVoltage() < 1;

        if (isBlue) {
            sideSpecificHeading = (3*Math.PI)/2;
            tangentAdd = 0;
            yMultiplier = 1;
        } else {
            sideSpecificHeading = Math.PI/2;
            tangentAdd = Math.PI;
            yMultiplier = -1;
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Pose2d startPose;
        if (isBlue) {
             startPose = new Pose2d(-38.5, 60, sideSpecificHeading);
        } else {
             startPose = new Pose2d(-37.5, -60, sideSpecificHeading);
        }

        Pose2d globalPose = new Pose2d(-30, 10 * yMultiplier, Math.PI);
        Pose2d backboardPose = new Pose2d(0, 0 * yMultiplier, Math.toRadians(90));

        Vector2d detectionPoint = new Vector2d(-34, 35 * yMultiplier);

        drive.setPoseEstimate(startPose);

        TrajectorySequence toDetection = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> robot.intake.setPower(1))
                .waitSeconds(0.05)
                .lineToConstantHeading(detectionPoint)
                .waitSeconds(0.3)
                .build();

        TrajectorySequence centerSpikePlacement = drive.trajectorySequenceBuilder(toDetection.end())
                .lineToConstantHeading(new Vector2d(-41, 36 * yMultiplier)) //Line up to spike mark
                .addTemporalMarker(() -> robot.transfer.setPower(-.23))
                .waitSeconds(1)
                .addTemporalMarker(() -> robot.transfer.setPower(0))
//                .lineToConstantHeading(new Vector2d(-49, 36 * yMultiplier))
//                .splineToConstantHeading(new Vector2d(-52,10 * yMultiplier), sideSpecificHeading)
//                .lineToSplineHeading(new Pose2d(-30, 10 * yMultiplier, Math.PI))  //Global endpoint
                .build();
        TrajectorySequence rightSpikePlacement = drive.trajectorySequenceBuilder(toDetection.end())
                .lineToLinearHeading(new Pose2d(-47, 38 * yMultiplier, sideSpecificHeading))
                .addTemporalMarker(() -> robot.transfer.setPower(-.23))
                .waitSeconds(1)
                .addTemporalMarker(() -> robot.transfer.setPower(0))
//                .lineTo(new Vector2d(-35, 35 * yMultiplier))
//                .lineTo(new Vector2d(-30, 10 * yMultiplier))
//                .turn((-Math.PI/2) * yMultiplier)
                .build();
        TrajectorySequence leftSpikePlacement = drive.trajectorySequenceBuilder(toDetection.end())
                .lineToConstantHeading(new Vector2d(-23, 35 * yMultiplier))
                .addTemporalMarker(() -> robot.transfer.setPower(-.23))
                .waitSeconds(1)
                .addTemporalMarker(() -> robot.transfer.setPower(0))
//                .lineToConstantHeading(new Vector2d(-49, 36.5 * yMultiplier))
//                .splineToConstantHeading(new Vector2d(-52,10 * yMultiplier),sideSpecificHeading)
//                .lineToSplineHeading(new Pose2d(-30, 10 * yMultiplier, Math.PI))
                .build();


        // Tell the driver that initialization is complete.
        if (isBlue) {
            telemetry.addData("Initialized", "Running Blue");
            telemetry.update();
        } else {
            telemetry.addData("Initialized", "Running Red");
            telemetry.update();

        }

        waitForStart();
        runtime.reset();
        if (isStopRequested()) return;

        drive.followTrajectorySequence(toDetection);

        if ((robot.rightDistance.getDistance(DistanceUnit.CM) > 2) && (robot.rightDistance.getDistance(DistanceUnit.CM) < 15)) {
            if (isBlue) {
                DESIRED_TAG_ID = BLUERIGHTTAG;
                propPosition = farAuto.propPositions.RIGHT;
            } else {
                DESIRED_TAG_ID = REDRIGHTTAG;
                propPosition = farAuto.propPositions.LEFT;
            }
            telemetry.addData("Running:", "Right");
        } else if ((robot.leftDistance.getDistance(DistanceUnit.CM) > 2) && (robot.leftDistance.getDistance(DistanceUnit.CM) < 15)) {
            if (isBlue) {
                DESIRED_TAG_ID = BLUELEFTTAG;
                propPosition = farAuto.propPositions.LEFT;
            } else {
                DESIRED_TAG_ID = REDLEFTTAG;
                propPosition = farAuto.propPositions.RIGHT;
            }
            telemetry.addData("Running:", "Left");
        } else {
            propPosition = farAuto.propPositions.CENTER;
            if (isBlue) {DESIRED_TAG_ID = BLUECENTERTAG;
            } else {DESIRED_TAG_ID = REDCENTERTAG; }
            telemetry.addData("Running:", "Center");
        }
        telemetry.update();

        switch (propPosition) {
            case CENTER:
                drive.followTrajectorySequence(centerSpikePlacement);
                break;
            case LEFT:
                drive.followTrajectorySequence(leftSpikePlacement);
                break;
            case RIGHT:
                drive.followTrajectorySequence(rightSpikePlacement);
                break;
        }
//        TrajectorySequence trussToBackboard = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .addTemporalMarker(() -> robot.intake.setPower(0))
//                .lineToLinearHeading(new Pose2d(24, 12 * yMultiplier, Math.PI))
//                .splineToConstantHeading(new Vector2d(38, 34 * yMultiplier),Math.PI)
//                .build();
//
//        TrajectorySequence centerBackboardPlacement = drive.trajectorySequenceBuilder(trussToBackboard.end())
//                .lineToConstantHeading(new Vector2d(54.5,35.5 * yMultiplier))
//                .back(2)
//                .build();
//
//        TrajectorySequence rightBackboardPlacement = drive.trajectorySequenceBuilder(trussToBackboard.end())
//                .lineToConstantHeading(new Vector2d(54.5,30.5 * yMultiplier))
//                .back(2)
//                .build();
//
//        TrajectorySequence leftBackboardPlacement = drive.trajectorySequenceBuilder(trussToBackboard.end())
//                .lineToLinearHeading(new Pose2d(54.5,40.5 * yMultiplier, Math.PI))
//                .back(2)
//                .build();
//        drive.followTrajectorySequence(trussToBackboard);
//        switch (propPosition) {
//            case CENTER:
//                drive.followTrajectorySequence(centerBackboardPlacement);
//                backboardPose = centerBackboardPlacement.end();
//                break;
//            case LEFT:
//                drive.followTrajectorySequence(leftBackboardPlacement);
//                backboardPose = leftBackboardPlacement.end();
//                break;
//            case RIGHT:
//                drive.followTrajectorySequence(rightBackboardPlacement);
//                backboardPose = rightBackboardPlacement.end();
//                break;
//        }
//        int liftPlacementPosition = 850;
//        robot.lift.setTargetPosition(liftPlacementPosition);
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lift.setPower(0.6);
//
//        while (robot.lift.getCurrentPosition() < liftPlacementPosition * 0.9 || robot.lift.getCurrentPosition() > liftPlacementPosition * 1.1) {
//            sleep(50);
//        }
//        robot.stripper.setPosition(robot.stripperSecondRelease);
//        sleep(1500);
//        robot.stripper.setPosition(robot.stripperOpen);
//        sleep(500);
//        robot.lift.setTargetPosition(0);
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lift.setPower(0.4);
//        TrajectorySequence park = drive.trajectorySequenceBuilder(backboardPose)
//                .lineToLinearHeading(new Pose2d(45,36 * yMultiplier, Math.PI))
//                .splineToConstantHeading(new Vector2d(55,13 * yMultiplier), Math.PI)
//                .build();
//        drive.followTrajectorySequence(park);
//        robot.lift.setPower(0);
        stop();
    }
}
