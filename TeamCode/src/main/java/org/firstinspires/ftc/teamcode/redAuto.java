package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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


@Autonomous(name = "Red Auto", group = "Auto")
public class redAuto extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Hardware robot = new Hardware();
    private propPositions propPosition;

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
        robot.droneAngle.setPosition(robot.droneAngleDown);
        robot.launcherRelease.setPosition(robot.launchClosed);
        robot.stripper.setPosition(robot.stripperOpen);
        robot.hook.setPosition(robot.hookDown);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        Pose2d globalPose = new Pose2d(0, 0, Math.toRadians(90));
        Pose2d lineUpToTagPos = new Pose2d(0, 0, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory toDetection = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(4, 29),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

        waitForStart();
        runtime.reset();
        if (isStopRequested()) return;

        robot.intake.setPower(1);

        drive.followTrajectory(toDetection);

        sleep(500);

        if ((robot.rightDistance.getDistance(DistanceUnit.CM) < 15)) {
            propPosition = redAuto.propPositions.RIGHT;
            telemetry.addData("Running:", "Right");
        } else if ((robot.leftDistance.getDistance(DistanceUnit.CM) < 15)) {
            propPosition = redAuto.propPositions.LEFT;
            telemetry.addData("Running:", "Left");
        } else {
            propPosition = redAuto.propPositions.CENTER;
            telemetry.addData("Running:", "Center");
        }
        telemetry.update();
        switch (propPosition) {
            case CENTER:
                Trajectory placePixelCenter = drive.trajectoryBuilder(toDetection.end())
                        .strafeTo(new Vector2d(-3, 25))
                        .build();
                Trajectory globalPositionCenter1 = drive.trajectoryBuilder(placePixelCenter.end())
                        .splineToConstantHeading(new Vector2d(-15, 18), 0)
                        .splineToConstantHeading(new Vector2d(-10,52),0)
                        .splineToConstantHeading(new Vector2d(10,50),0)
                        .build();
                Trajectory globalPositionCenter2 = drive.trajectoryBuilder(globalPositionCenter1.end())
                        .lineToLinearHeading(new Pose2d(35, 50, 0))
                        .build();
                drive.followTrajectory(placePixelCenter);
                robot.transfer.setPower(-.32);
                sleep(450);
                robot.transfer.setPower(0);
                drive.followTrajectory(globalPositionCenter1);
                drive.followTrajectory(globalPositionCenter2);
                globalPose = globalPositionCenter2.end();
                break;
            case RIGHT:
                Trajectory placePixelRight = drive.trajectoryBuilder(toDetection.end())
                        .strafeTo(new Vector2d(15, 24))
                        .build();
                Trajectory underTruss = drive.trajectoryBuilder(placePixelRight.end())
                        .strafeTo(new Vector2d(27, 24))
                        .build();
                Trajectory throughTruss = drive.trajectoryBuilder(underTruss.end())
                        .lineToLinearHeading(new Pose2d(27, 52, Math.toRadians(90)))
                        .build();
                Trajectory globalPositionRight = drive.trajectoryBuilder(throughTruss.end())
                        .lineToLinearHeading(new Pose2d(50, 48, Math.toRadians(0)))
                        .build();
                drive.followTrajectory(placePixelRight);
                robot.transfer.setPower(-.28);
                sleep(650);
                drive.followTrajectory(underTruss);
                drive.followTrajectory(throughTruss);
                drive.followTrajectory(globalPositionRight);
                globalPose = globalPositionRight.end();
                break;
            case LEFT:
                Trajectory placePixelLeft = drive.trajectoryBuilder(toDetection.end())
                        .splineToConstantHeading(new Vector2d(-9, 15), 0)
                        .build();
                Trajectory lineUp = drive.trajectoryBuilder(placePixelLeft.end())
                                .strafeTo(new Vector2d(1,15))
                                        .build();
                Trajectory lineUpStageDoor = drive.trajectoryBuilder(lineUp.end())
                                .lineTo(new Vector2d(1, 48))
                                        .build();
                Trajectory globalPositionLeft = drive.trajectoryBuilder(lineUpStageDoor.end())
                                .splineToLinearHeading(new Pose2d(35,48, Math.toRadians(0)),0)
                                        .build();
                drive.followTrajectory(placePixelLeft);
                robot.transfer.setPower(-.32);
                sleep(450);
                drive.followTrajectory(lineUp);
                drive.followTrajectory(lineUpStageDoor);
                drive.followTrajectory(globalPositionLeft);
                globalPose = globalPositionLeft.end();
                break;
        }
        robot.intake.setPower(0);
        robot.transfer.setPower(0);
        Trajectory throughStageDoor = drive.trajectoryBuilder(globalPose)
                .lineToLinearHeading(new Pose2d(50,45, 2.9))
                .build();
        Trajectory nextToBackboard = drive.trajectoryBuilder(throughStageDoor.end())
                .back(35)
                .build();
        drive.followTrajectory(throughStageDoor);
        drive.followTrajectory(nextToBackboard);

    }
}
