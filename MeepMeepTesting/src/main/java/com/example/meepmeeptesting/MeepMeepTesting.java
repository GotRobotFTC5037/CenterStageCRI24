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
                                .lineToConstantHeading(new Vector2d(-35, 35)) //Go to detection point - Detect then set to run specific placement - render all before hand
                                //Start of position specific placement
                                .lineToConstantHeading(new Vector2d(-41, 35)) //Line up to spike mark
                                .waitSeconds(2) // Place pixel
                                .lineToConstantHeading(new Vector2d(-49, 36))
                                .splineToConstantHeading(new Vector2d(-52,10), (3*Math.PI)/2)
                                .lineToSplineHeading(new Pose2d(-30, 10, Math.PI))
                                //End of position specific placement
                                .lineToConstantHeading(new Vector2d(24, 12))
                                .splineToConstantHeading(new Vector2d(38, 34),0)
                                .lineToConstantHeading(new Vector2d(54.5,35.5))
                                .back(2)
                                .waitSeconds(3)
                                .lineToLinearHeading(new Pose2d(45,36, Math.PI))
                                .splineToConstantHeading(new Vector2d(55,10), Math.PI)
                                //Start of position specific placement
                                .build()
                );
        RoadRunnerBotEntity myBotLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(40, 30, Math.toRadians(190), Math.toRadians(60), 15.45)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38.5, 59.5, (3*Math.PI)/2))
                                .waitSeconds(2) //Wait For start
                                .lineToConstantHeading(new Vector2d(-35, 35)) //Go to detection point - Detect then set to run specific placement - render all before hand
                                //Start of position specific placement
                                .lineToConstantHeading(new Vector2d(-23, 35)) //Line up to spike mark
                                .waitSeconds(2) // Place pixel
                                .lineToConstantHeading(new Vector2d(-49, 36.5))
                                .splineToConstantHeading(new Vector2d(-52,10),(3*Math.PI)/2)
                                .lineToSplineHeading(new Pose2d(-30, 10, Math.PI))
                                //End of position specific placement
                                .lineToConstantHeading(new Vector2d(24, 12))
                                .splineToConstantHeading(new Vector2d(38, 34),0)
                                .lineToConstantHeading(new Vector2d(54.5,35.5))
                                .back(2)
                                .waitSeconds(3)
                                .lineToLinearHeading(new Pose2d(45,36, Math.PI))
                                .splineToConstantHeading(new Vector2d(55,10), Math.PI)
                                //Start of position specific placement
                                .build()
                );
        RoadRunnerBotEntity myBotRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(40, 30, Math.toRadians(190), Math.toRadians(60), 15.45)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38.5, 59.5, (3*Math.PI)/2))
                                .waitSeconds(2) //Wait For start
                                .lineToConstantHeading(new Vector2d(-35, 35)) //Go to detection point - Detect then set to run specific placement - render all before hand
                                //Start of position specific placement
                                .lineToConstantHeading(new Vector2d(-47, 36))//Line up to spike mark
                                .waitSeconds(2) // Place pixel
                                .lineToConstantHeading(new Vector2d(-35, 35))
                                .lineToConstantHeading(new Vector2d(-30, 10))
                                .turn(-Math.PI/2)
                                //End of position specific placement
                                .lineToConstantHeading(new Vector2d(24, 12))
                                .splineToConstantHeading(new Vector2d(38, 34),0)
                                .lineToConstantHeading(new Vector2d(54.5,35.5))
                                .back(2)
                                .waitSeconds(3)
                                .lineToLinearHeading(new Pose2d(45,36, Math.PI))
                                .splineToConstantHeading(new Vector2d(55,10), Math.PI)
                                //Start of position specific placement
                                .build()
                );


        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(40, 30, Math.toRadians(190), Math.toRadians(60), 15.45)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15.5, -59.5, (Math.PI)/2))
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(13, -36))
                                .lineToConstantHeading(new Vector2d(19, -36))
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(19, -46))
                                .splineToLinearHeading(new Pose2d(54.5,-35.5, Math.PI), 0)
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