package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Autonomous(name = "Blue Auto", group = "Auto")
public class blueAuto extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Hardware robot = new Hardware();
    private propPositions propPosition;

    boolean isBlue = true;
    int xMultiplier = 1;
    double headingAdd = 0;
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

        isBlue = robot.marker.getVoltage() < 1;

        if (isBlue) {
            headingAdd = 0;
            tangentAdd = 0;
            xMultiplier = 1;
        } else {
            headingAdd = -Math.PI;
            tangentAdd = Math.PI;
            xMultiplier = -1;
        }

        initAprilTag();

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Pose2d startPose = new Pose2d(-38.5 * xMultiplier, 59.5, ((3 * Math.PI) / 2) + headingAdd);
        Pose2d globalPose = new Pose2d(-30 * xMultiplier, 10, Math.PI);
        Pose2d lineUpToTagPos = new Pose2d(0, 0, Math.toRadians(90));

        Vector2d detectionPoint = new Vector2d(-35 * xMultiplier, 36);

        drive.setPoseEstimate(startPose);

        TrajectorySequence toDetection = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> robot.intake.setPower(1))
                .waitSeconds(0.05)
                .lineToConstantHeading(detectionPoint)
                .waitSeconds(0.3)
                .build();

        TrajectorySequence centerSpikePlacement = drive.trajectorySequenceBuilder(toDetection.end())
                .lineToConstantHeading(new Vector2d(-41 * xMultiplier, 36)) //Line up to spike mark
                .addTemporalMarker(() -> robot.transfer.setPower(-.23))
                .waitSeconds(1.5)
                .addTemporalMarker(() -> robot.transfer.setPower(0))
                .splineToConstantHeading(new Vector2d(-52 * xMultiplier, 36), 0)
                .splineToConstantHeading(new Vector2d(-50 * xMultiplier, 10), 0)
                .lineToSplineHeading(new Pose2d(-30 * xMultiplier, 12, Math.PI))  //Global endpoint
                .build();
//        TrajectorySequence rightSpikePlacement = drive.trajectorySequenceBuilder(toDetection.end())
//                .build();
//        TrajectorySequence leftSpikePlacement = drive.trajectorySequenceBuilder(toDetection.end())
//                .build();

        TrajectorySequence trussToBackboard = drive.trajectorySequenceBuilder(globalPose)
                .addTemporalMarker(() -> robot.intake.setPower(0))
                .lineToConstantHeading(new Vector2d(24 * xMultiplier, 12))
                .splineToConstantHeading(new Vector2d(38 * xMultiplier, 34),0)
                .lineToConstantHeading(new Vector2d(54.5,30))
                //.lineToLinearHeading(new Pose2d(38 * xMultiplier, 38, Math.PI)) //Line up to backboard for camera detection

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
            propPosition = blueAuto.propPositions.RIGHT;
            if (isBlue) {DESIRED_TAG_ID = BLUERIGHTTAG;
            } else {DESIRED_TAG_ID = REDRIGHTTAG; }
            telemetry.addData("Running:", "Right");
        } else if ((robot.leftDistance.getDistance(DistanceUnit.CM) > 2) && (robot.leftDistance.getDistance(DistanceUnit.CM) < 15)) {
            propPosition = blueAuto.propPositions.LEFT;
            if (isBlue) {DESIRED_TAG_ID = BLUELEFTTAG;
            } else {DESIRED_TAG_ID = REDLEFTTAG; }
            telemetry.addData("Running:", "Left");
        } else {
            propPosition = blueAuto.propPositions.CENTER;
            if (isBlue) {DESIRED_TAG_ID = BLUECENTERTAG;
            } else {DESIRED_TAG_ID = REDCENTERTAG; }
            telemetry.addData("Running:", "Center");
        }
        telemetry.update();

        DESIRED_TAG_ID = BLUECENTERTAG;

        switch (propPosition) {
            case CENTER:
                drive.followTrajectorySequence(centerSpikePlacement);
                break;
            case LEFT:
                //drive.followTrajectorySequence(leftSpikePlacement);
                break;
            case RIGHT:
                //drive.followTrajectorySequence(rightSpikePlacement);
                break;
        }
        drive.followTrajectorySequence(trussToBackboard);
        //Camera Detection code

//        double oldTime = getRuntime();
//
//        while (getRuntime() < oldTime + 2 && opModeIsActive()) {
//            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//            for (AprilTagDetection detection : currentDetections) {
//                // Look to see if we have size info on this tag.
//                if (detection.metadata != null) {
//                    //  Check to see if we want to track towards this tag.
//                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
//                        // Yes, we want to use this tag.
//                        targetFound = true;
//                        desiredTag = detection;
//                        break;  // don't look any further.
//                    } else {
//                        // This tag is in the library, but we do not want to track it right now.
//                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
//                        telemetry.update();
//                    }
//                } else {
//                    // This tag is NOT in the library, so we don't have enough information to track to it.
//                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
//                    telemetry.update();
//                }
//            }
//        }



        switch (propPosition) {
            case CENTER:

                break;
            case LEFT:

                break;
            case RIGHT:

                break;
        }

       // drive.followTrajectorySequence(toPlacement);
        int liftPlacementPosition = 1000;
        robot.lift.setTargetPosition(liftPlacementPosition);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.6);

        while (robot.lift.getCurrentPosition() < liftPlacementPosition * 0.9 || robot.lift.getCurrentPosition() > liftPlacementPosition * 1.1) {
            sleep(50);
        }
        robot.stripper.setPosition(robot.stripperSecondRelease);
        sleep(1000);
        robot.lift.setPower(0);




    }

    private void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(1087.05, 1087.05, 725.72, 454.44) /**Parameters for Arducam**/

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

    }

    private void setManualExposure(int exposureMS, int gain) {
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
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
