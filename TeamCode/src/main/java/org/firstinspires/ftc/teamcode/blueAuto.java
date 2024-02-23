package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;


@Autonomous(name = "Blue Auto", group = "Auto")
public class blueAuto extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Hardware robot = new Hardware();
    private propPositions propPosition;
    private static int DESIRED_TAG_ID;
    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    boolean targetFound     = false;    // Set to true when an AprilTag target is detected

    public enum propPositions {
        LEFT,
        RIGHT,
        CENTER
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        initAprilTag();
        robot.droneAngle.setPosition(robot.droneAngleDown);
        robot.launcherRelease.setPosition(robot.launchClosed);
        robot.stripper.setPosition(robot.stripperOpen);
        robot.hook.setPosition(robot.hookDown);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        Pose2d globalPose = new Pose2d(0, 0, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory toDetection = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-7, 22),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

        waitForStart();
        visionPortal.stopStreaming();
        runtime.reset();
        if (isStopRequested()) return;

        robot.intake.setPower(0);

        drive.followTrajectory(toDetection);

        if ((robot.rightDistance.getDistance(DistanceUnit.CM) > 10.5) && (robot.rightDistance.getDistance(DistanceUnit.CM) < 15)) {
            propPosition = propPositions.RIGHT;
            DESIRED_TAG_ID = 6;
            telemetry.addData("Running:", "Right");
        } else if ((robot.leftDistance.getDistance(DistanceUnit.CM) > 8) && (robot.leftDistance.getDistance(DistanceUnit.CM) < 15)) {
            propPosition = propPositions.LEFT;
            DESIRED_TAG_ID = 4;
            telemetry.addData("Running:", "Left");
        } else {
            propPosition = propPositions.CENTER;
            DESIRED_TAG_ID = 5;
            telemetry.addData("Running:", "Center");
        }
        propPosition = propPositions.LEFT;
        telemetry.update();

        switch (propPosition) {
            case CENTER:
                robot.stripper.setPosition(robot.stripperOpen);
                Trajectory placePixelCenter = drive.trajectoryBuilder(toDetection.end())
                        .strafeTo(new Vector2d(0,22))
                        .build();
                Trajectory globalPositionCenter1 = drive.trajectoryBuilder(placePixelCenter.end())
                        .splineToConstantHeading(new Vector2d(15, 18), 0)
                        .splineToConstantHeading(new Vector2d(10,43),0)
                        .splineToConstantHeading(new Vector2d(-10,43),0)
                        .build();
                Trajectory globalPositionCenter2 = drive.trajectoryBuilder(globalPositionCenter1.end())
                        .lineToLinearHeading(new Pose2d(-47, 43, 0))
                        .build();
                Trajectory globalPositionCenter3 = drive.trajectoryBuilder(globalPositionCenter2.end(), true)
                        .back(40)
                        .build();
                Trajectory globalPositionCenter4 = drive.trajectoryBuilder(globalPositionCenter3.end(), true)
                        .strafeRight(28.5)
                        .build();
                Trajectory globalPositionCenter5 = drive.trajectoryBuilder(globalPositionCenter4.end())
                        .back(5)
                        .build();

                robot.transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.transfer.setPower(-0.25);

                drive.followTrajectory(placePixelCenter);
                sleep(250);
                drive.followTrajectory(globalPositionCenter1);
                drive.followTrajectory(globalPositionCenter2);
                drive.followTrajectory(globalPositionCenter3);

                robot.lift.setTargetPosition(750);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(.7);

                telemetry.addData("Lift position: ", robot.lift.getCurrentPosition());
                telemetry.update();

                drive.followTrajectory(globalPositionCenter4);
                sleep(350);
                robot.stripper.setPosition(robot.stripperSecondRelease);
                drive.followTrajectory(globalPositionCenter5);

                break;
            case LEFT:
                Trajectory moveBack = drive.trajectoryBuilder(toDetection.end())
                        .back(8)
                        .build();
                Trajectory globalPosition1 = drive.trajectoryBuilder((moveBack.end()))
                        .strafeRight(10)
                        .build();
                Trajectory globalPosition2 = drive.trajectoryBuilder((globalPosition1.end()))
                        .forward(30)
                        .build();
                Trajectory globalPosition3 = drive.trajectoryBuilder(globalPosition2.end())
                        .lineToLinearHeading(new Pose2d(0,-10,90))
                        .build();
                Trajectory globalPosition4 = drive.trajectoryBuilder(globalPosition3.end())
                        .back(26)

                        .build();


                robot.intake.setPower(0.3);
                drive.turn(Math.toRadians(60));
                robot.transfer.setPower(-0.25);
                sleep(1000);
                robot.leftBackDrive.setPower(50);
                robot.leftDrive.setPower(50);
                robot.rightDrive.setPower(50);
                robot.rightBackDrive.setPower(50);
                drive.followTrajectory((moveBack));
                drive.followTrajectory(globalPosition1);
                drive.followTrajectory(globalPosition2);
                drive.followTrajectory(globalPosition3);
                //drive.followTrajectory(globalPosition4);
                //drive.turn(Math.toRadians(-92));
                //robot.lift.setTargetPosition(1350);
                //robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //robot.lift.setPower(.7);
                //sleep(1000);
                //robot.stripper.setPosition(robot.stripperSecondRelease);


                sleep(450);
                break;
            case RIGHT:
                Trajectory placePixelRight = drive.trajectoryBuilder(toDetection.end())
                        .splineToConstantHeading(new Vector2d(-7, 15), 0)
                        .build();

                drive.followTrajectory(placePixelRight);
                robot.transfer.setPower(-.32);
                sleep(450);
                break;
        }

    }


    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(911.384, 911.384, 605.325, 382.676) /**Parameters for Arducam**/

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(robot.webcam);


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 800));

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

    }   // end method initAprilTag()

}
