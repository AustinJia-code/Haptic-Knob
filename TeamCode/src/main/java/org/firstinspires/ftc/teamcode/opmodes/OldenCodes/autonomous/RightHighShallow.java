/*package org.firstinspires.ftc.teamcode.opmodes.OldenCodes.autonomous;

import com.acmerobotics.roadrunner.geometry.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import static java.lang.Math.*;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

//TUNED, 6.75 setup
@Autonomous(name = "\uD83D\uDC17 Right High Shallow \uD83D\uDC17", group = "Final")
public class RightHighShallow extends High {

    private double waitAtStorage = 0.2;
    private double waitAtScore = 0.1;
    public static Pose2d INIT = new Pose2d(31, -65, toRadians(-90));
    public static Pose2d PARK_LEFT = new Pose2d(9, -12, toRadians(-90));
    public static Pose2d PARK_MIDDLE = new Pose2d(32, -11, toRadians(-90));
    public static Pose2d PARK_RIGHT = new Pose2d(58, -12, toRadians(-90));

    public void build(){
        SCORING_POSITION = new Pose2d(26,-4, toRadians(155));
        STORAGE_POSITION = new Pose2d(48, -11, toRadians(0));

        drive.setPoseEstimate(INIT);

        ScorePreload = drive.trajectorySequenceBuilder(INIT)
                .addTemporalMarker(0, () -> bot.setPosition(State.LOW))
                .addTemporalMarker(0.1, () -> bot.setPosition(State.LOW))
                .addTemporalMarker(0.7, () -> bot.setPosition(State.MIDDLE))
                .addTemporalMarker(0.75, () -> bot.setPosition(State.HIGH))
                .addTemporalMarker(0.8, () -> bot.slide.setTarget(LinearSlides.spoolChange(1420)))
                .addTemporalMarker(2.2, () -> {
                    bot.slide.incrementSlides(-1);
                    bot.arm.slamThatJawn();
                })
                .back(34)
                .splineTo(new Vector2d(SCORING_POSITION.getX()-2.75, SCORING_POSITION.getY()+1), SCORING_POSITION.getHeading())
                .build();
        WaitAtScore1 = waitSequence(ScorePreload, waitAtScore, false);

        ScoreToStorage1 = ScoreToStorage(ScorePreload, -1, -1, 0);
        WaitAtStorage1 = waitSequence(ScoreToStorage1, waitAtStorage, true);
        StorageToScore1 = StorageToScore(WaitAtStorage1, 0.9, 3.8, 0);
        WaitAtScore2 = waitSequence(StorageToScore1, waitAtScore, false);

        ScoreToStorage2 = ScoreToStorage(WaitAtScore2, -0.8, -0.5, 0);
        WaitAtStorage2 = waitSequence(ScoreToStorage2, waitAtStorage, true);
        StorageToScore2 = StorageToScore(WaitAtStorage2, 0.75, 3.9 , 0);
        WaitAtScore3 = waitSequence(StorageToScore2, waitAtScore, false);

        ScoreToStorage3 = ScoreToStorage(WaitAtScore3, -0.55, 0, 0);
        WaitAtStorage3 = waitSequence(ScoreToStorage3, waitAtStorage, true);
        StorageToScore3 = StorageToScore(WaitAtStorage3, 1, 5.5, 0);
        WaitAtScore4 = waitSequence(StorageToScore3, waitAtScore, false);

        ScoreToStorage4 = ScoreToStorage(WaitAtScore4, -1.25, 1, 0);
        WaitAtStorage4 = waitSequence(ScoreToStorage4, waitAtStorage, true);
        StorageToScore4 = StorageToScore(WaitAtStorage4, 1.5, 6.0, 0);
        WaitAtScore5 = waitSequence(StorageToScore4, waitAtScore, false);

        ScoreToStorage5 = ScoreToStorage(WaitAtScore5, -1.75, 2, 0);
        WaitAtStorage5 = waitSequence(ScoreToStorage5, waitAtStorage, true);
        StorageToScore5 = StorageToScoreLast(WaitAtStorage5, 2.25, 7.5 , 0);
        WaitAtScore6 = waitSequence(StorageToScore5, waitAtScore, false);

        ParkMiddle = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(true)
                .addTemporalMarker(0, ()->{
                    bot.arm.setPosition(State.LIFTED);
                    bot.claw.close();
                })
                .lineToLinearHeading(PARK_MIDDLE)
                .forward(8)
                .waitSeconds(1)
                .build();
        ParkLeft = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(false)
                .addTemporalMarker(0, ()->{
                    bot.arm.setPosition(State.LIFTED);
                    bot.claw.close();
                })
                .lineToLinearHeading(PARK_LEFT)
                .forward(8)
                .waitSeconds(1)
                .build();
        ParkRight = drive.trajectorySequenceBuilder(WaitAtScore6.end())
                .setReversed(true)
                .addTemporalMarker(0, ()->{
                    bot.arm.setPosition(State.LIFTED);
                    bot.claw.close();
                })
                .lineToLinearHeading(PARK_MIDDLE)
                .strafeLeft(24)
                .forward(12)
                .waitSeconds(1)
                .build();
    }

    @Override
    public void setCameraPosition(){
        webcamName = "Right";
    }
}

 */