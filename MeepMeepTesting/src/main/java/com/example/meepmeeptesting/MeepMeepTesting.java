package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(720);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(40, 30, Math.toRadians(190), Math.toRadians(60), 15.45)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38.5, 59.5, (3*Math.PI)/2))
                                .waitSeconds(2) //Wait For start
                                .lineToConstantHeading(new Vector2d(-35, 36)) //Go to detection point - Detect then set to run specific placement - render all before hand
                                //Start of position specific placement
                                .lineToConstantHeading(new Vector2d(-41, 36)) //Line up to spike mark
                                .waitSeconds(2) // Place pixel
                                .splineToConstantHeading(new Vector2d(-52, 36), 0)
                                .splineToConstantHeading(new Vector2d(-50,10),0)
                                //End of position specific placement
                                .lineToSplineHeading(new Pose2d(-30, 12, Math.PI)) //Global endpoint
                                .lineToConstantHeading(new Vector2d(24,12))
                                .splineToConstantHeading(new Vector2d(38,35),Math.PI/2)
                                //.lineToLinearHeading(new Pose2d(38,30, Math.PI)) //Line up to backboard for camera detection
                                .back(20)
                                //Start of position specific placement
                                .build()
                );
        RoadRunnerBotEntity myBotLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(40, 30, Math.toRadians(190), Math.toRadians(60), 15.45)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, 64, (3*Math.PI)/2))
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(-35, 36))
                                .lineToConstantHeading(new Vector2d(-24, 36))
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(-48, 36))
                                .splineToConstantHeading(new Vector2d(-53,10),0)
                                .lineToSplineHeading(new Pose2d(-30, 10, Math.PI))
                                .lineToConstantHeading(new Vector2d(24,10))
                                .splineToConstantHeading(new Vector2d(38,15),Math.PI/2)
                                .lineToConstantHeading(new Vector2d(38,38))
                                .build()
                );
        RoadRunnerBotEntity myBotRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(40, 30, Math.toRadians(190), Math.toRadians(60), 15.45)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, 64, (3*Math.PI)/2))
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(-35, 36))
                                .lineToConstantHeading(new Vector2d(-47, 36))
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(-35, 36))
                                .lineToConstantHeading(new Vector2d(-30, 10))
                                .turn(-Math.PI/2)
                                .lineToConstantHeading(new Vector2d(24,10))
                                .splineToConstantHeading(new Vector2d(38,15),Math.PI/2)
                                .lineToConstantHeading(new Vector2d(38,38))
                                .build()
                );


        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(40, 30, Math.toRadians(190), Math.toRadians(60), 15.45)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, -64, (Math.PI)/2))
                                .waitSeconds(2)
                                .lineTo(new Vector2d(-35, -36))
                                .strafeTo(new Vector2d(-41, -36))
                                .waitSeconds(2)
                                .splineToConstantHeading(new Vector2d(-56, -36), 0)
                                .splineToConstantHeading(new Vector2d(-53,-10),0)
                                .lineToSplineHeading(new Pose2d(-30, -10, Math.PI))
                                .lineToConstantHeading(new Vector2d(24,-10))
                                .splineToConstantHeading(new Vector2d(38,-15),(3*Math.PI)/2)
                                .lineToConstantHeading(new Vector2d(38,-38))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBotLeft)
                .addEntity(myBotRight)
                .addEntity(mySecondBot)
                .start();
    }
}