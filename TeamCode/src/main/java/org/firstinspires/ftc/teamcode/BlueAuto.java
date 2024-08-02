package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Blue Auto", group="Auto")
public class BlueAuto extends LinearOpMode {
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
                .lineToLinearHeading(new Pose2d(27,0,Math.toRadians(0)))
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
                        .strafeRight(3)
                        .build();
                drive.followTrajectory(moveRight);
                sleep(200);
                robot.transfer.setPower(-0.25);
                sleep(500);
                robot.transfer.setPower(0);
                drive.turn(Math.toRadians(-100));
                Trajectory moveToBackboard = drive.trajectoryBuilder(new Pose2d())
                        .back(42)
                        .build();
               // Trajectory adjustRight = drive.trajectoryBuilder(new Pose2d())
                 //       .strafeRight(3)
                   //     .build();
                //drive.followTrajectory(adjustRight);
                drive.followTrajectory(moveToBackboard);
                Trajectory slowlyApproachBackboard = drive.trajectoryBuilder(new Pose2d())
                        .back(6)
                        .build();
                robot.leftDrive.setPower(0.2);
                robot.leftBackDrive.setPower(0.2);
                robot.rightDrive.setPower(0.2);
                robot.rightBackDrive.setPower(0.2);
                drive.followTrajectory(slowlyApproachBackboard);
                sleep(1000);
                break;

            case "RIGHT":
                Trajectory adjustLeft = drive.trajectoryBuilder(new Pose2d())
                        .strafeLeft(4)
                        .build();
                drive.followTrajectory(adjustLeft);
                drive.turn(Math.toRadians(-95));
                robot.transfer.setPower(-0.4);
                sleep(1000);
                robot.transfer.setPower(0);
                Trajectory adjustLeftForBackboard = drive.trajectoryBuilder(new Pose2d())
                        .strafeRight(5)
                        .build();
                drive.followTrajectory(adjustLeftForBackboard);
                drive.turn(Math.toRadians(-18));
                Trajectory ToBackboard = drive.trajectoryBuilder(new Pose2d())
                        .back(35)
                        .build();
                drive.followTrajectory(ToBackboard);
                Trajectory adjustForPlacement = drive.trajectoryBuilder(new Pose2d())
                        .back(6)
                        .build();
                drive.followTrajectory(adjustForPlacement);
                sleep(1000);
                break;

            case "LEFT":
                Trajectory adjustRightSlightly = drive.trajectoryBuilder(new Pose2d())
                        .strafeRight(3)
                        .build();
                drive.followTrajectory(adjustRightSlightly);
                drive.turn(Math.toRadians(95));
                robot.transfer.setPower(-0.48);
                sleep(1000);
                robot.transfer.setPower(0);
                Trajectory adjustLeftSlightly = drive.trajectoryBuilder(new Pose2d())
                        .strafeLeft(6)
                        .build();
                drive.followTrajectory(adjustLeftSlightly);
                drive.turn(Math.toRadians(-170));
                Trajectory toBackboardBigMovement = drive.trajectoryBuilder(new Pose2d())
                        .back(42)
                        .build();
                drive.followTrajectory(toBackboardBigMovement);
                Trajectory adjustForPlacementSlightly = drive.trajectoryBuilder(new Pose2d())
                        .back(6)
                        .build();
                drive.followTrajectory(adjustForPlacementSlightly);
                Trajectory adjustToPlace = drive.trajectoryBuilder(new Pose2d())
                        .strafeLeft(2)
                        .build();
                drive.followTrajectory(adjustToPlace);
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
                .forward(13)
                .build();
        if (propPosition == "CENTER") {
            Trajectory moveToParkRight = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(18)
                    .build();
            Trajectory moveToParkBack = drive.trajectoryBuilder(new Pose2d())
                    .back(19)
                    .build();
            drive.followTrajectory(moveToParkForward);
            drive.followTrajectory(moveToParkRight);
            drive.followTrajectory(moveToParkBack);
        } else if (propPosition == "RIGHT") {
            Trajectory moveToParkRight = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(20)
                    .build();
            Trajectory moveToParkBack = drive.trajectoryBuilder(new Pose2d())
                    .back(20)
                    .build();
            drive.followTrajectory(moveToParkForward);
            drive.followTrajectory(moveToParkRight);
            drive.followTrajectory(moveToParkBack);
        } else {
            Trajectory moveToParkRight = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(12)
                    .build();
            Trajectory moveToParkBack = drive.trajectoryBuilder(new Pose2d())
                    .back(16)
                    .build();
            drive.followTrajectory(moveToParkForward);
            drive.followTrajectory(moveToParkRight);
            drive.followTrajectory(moveToParkBack);
        }

        sleep(1000);

    }
}
