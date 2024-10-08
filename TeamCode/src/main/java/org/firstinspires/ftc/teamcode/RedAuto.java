package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import java.util.List;

@Autonomous(name="Red Auto", group="Auto")
public class RedAuto extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // CENTER, RIGHT, or LEFT
        String propPosition = "CENTER";

        // MOVEMENT:
        // ----------------
        // FORWARD: +X
        // LEFT: +Y
        // RIGHT: -Y
        // BACKWARD: -X

        Trajectory driveToDetectionPosition = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(34,0),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        waitForStart();
        if(isStopRequested()) return;

        robot.intake.setPower(0.5);
        drive.followTrajectory(driveToDetectionPosition);
        robot.intake.setPower(0);
        sleep(100);

        if (robot.rightDistance.getDistance(DistanceUnit.CM) < 15) {
            propPosition = "RIGHT";
        } else if (robot.leftDistance.getDistance(DistanceUnit.CM) < 15) {
            propPosition = "LEFT";
        } else {
            propPosition = "CENTER";
        }

        telemetry.addData("Prop Position", propPosition);
        telemetry.update();
        sleep(100);

        switch (propPosition) {
            case "CENTER":
                Trajectory moveRight = drive.trajectoryBuilder(new Pose2d())
                        .strafeRight(4)
                        .build();
                drive.followTrajectory(moveRight);
                sleep(200);
                robot.transfer.setPower(-0.31);
                sleep(500);
                robot.transfer.setPower(0);
                drive.turn(Math.toRadians(105));
                Trajectory moveToBackboard = drive.trajectoryBuilder(new Pose2d())
                        .back(32)
                        .build();
                Trajectory adjustRight = drive.trajectoryBuilder(new Pose2d())
                        .strafeRight(3)
                        .build();
                drive.followTrajectory(adjustRight);
                drive.followTrajectory(moveToBackboard);
                Trajectory slowlyApproachBackboard = drive.trajectoryBuilder(new Pose2d())
                        .back(6)
                        .build();
                robot.leftDrive.setPower(0.2);
                robot.leftBackDrive.setPower(0.2);
                robot.rightDrive.setPower(0.2);
                robot.rightBackDrive.setPower(0.2);
                drive.followTrajectory(slowlyApproachBackboard);
                sleep(2000);
                break;

            case "RIGHT":
                Trajectory adjustLeft = drive.trajectoryBuilder(new Pose2d())
                        .strafeLeft(5)
                        .build();
                drive.followTrajectory(adjustLeft);
                drive.turn(Math.toRadians(-95));
                robot.transfer.setPower(-0.55);
                sleep(1000);
                robot.transfer.setPower(0);
                Trajectory adjustRightSlowly = drive.trajectoryBuilder(new Pose2d())
                        .strafeRight(8)
                        .build();
                drive.followTrajectory(adjustRightSlowly);
                drive.turn(Math.toRadians(187));
                Trajectory ToBackboard = drive.trajectoryBuilder(new Pose2d())
                        .back(42)
                        .build();
                drive.followTrajectory(ToBackboard);
                Trajectory adjustForPlacement = drive.trajectoryBuilder(new Pose2d())
                        .back(7)
                        .build();
                Trajectory adjustToPlace = drive.trajectoryBuilder(new Pose2d())
                        .strafeRight(6)
                        .build();
                drive.followTrajectory(adjustToPlace);
                drive.followTrajectory(adjustForPlacement);
                break;

            case "LEFT":
                drive.turn(Math.toRadians(90));
                robot.transfer.setPower(-0.2);
                sleep(350);
                drive.turn(Math.toRadians(33));
                robot.transfer.setPower(0);
                Trajectory toBackboard = drive.trajectoryBuilder(new Pose2d())
                        .back(38)
                        .build();
                Trajectory slowlyAdjustToPlace = drive.trajectoryBuilder(new Pose2d())
                        .back(5)
                        .build();
                Trajectory adjustToRightForPlacement = drive.trajectoryBuilder(new Pose2d())
                        .strafeRight(5)
                        .build();
                drive.followTrajectory(adjustToRightForPlacement);
                drive.followTrajectory(toBackboard);
                robot.leftDrive.setPower(0.2);
                robot.leftBackDrive.setPower(0.2);
                robot.rightDrive.setPower(0.2);
                robot.rightBackDrive.setPower(0.2);
                drive.followTrajectory(slowlyAdjustToPlace);
                sleep(1000);
                break;
        }

        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setTargetPosition(950);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.7);

        sleep(750);

        robot.stripper.setPosition(robot.stripperFirstRelease);
        sleep(100);
        robot.stripper.setPosition(robot.stripperSecondRelease);

        sleep(1000);

        robot.stripper.setPosition(robot.stripperOpen);
        sleep(500);
        robot.lift.setTargetPosition(0);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.3);

        sleep(1000);

        Trajectory moveToParkForward = drive.trajectoryBuilder(new Pose2d())
                .forward(10)
                .build();
        if (propPosition == "CENTER") {
            Trajectory moveToParkLeft = drive.trajectoryBuilder(new Pose2d())
                    .strafeLeft(13)
                    .build();
            Trajectory moveToParkBack = drive.trajectoryBuilder(new Pose2d())
                    .back(12)
                    .build();
            drive.followTrajectory(moveToParkForward);
            drive.followTrajectory(moveToParkLeft);
            drive.followTrajectory(moveToParkBack);
        } else if (propPosition == "RIGHT") {
            Trajectory moveToParkLeft = drive.trajectoryBuilder(new Pose2d())
                    .strafeLeft(11)
                    .build();
            Trajectory moveToParkBack = drive.trajectoryBuilder(new Pose2d())
                    .back(15)
                    .build();
            drive.followTrajectory(moveToParkForward);
            drive.followTrajectory(moveToParkLeft);
            drive.followTrajectory(moveToParkBack);
        } else {
            Trajectory moveToParkLeft = drive.trajectoryBuilder(new Pose2d())
                    .strafeLeft(21)
                    .build();
            Trajectory moveToParkBack = drive.trajectoryBuilder(new Pose2d())
                    .back(14)
                    .build();
            drive.followTrajectory(moveToParkForward);
            drive.followTrajectory(moveToParkLeft);
            drive.followTrajectory(moveToParkBack);
        }

        sleep(1000);

    }
}
