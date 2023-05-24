package org.firstinspires.ftc.teamcode.opmodes.OldenCodes.autonomous;

import com.acmerobotics.dashboard.config.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.*;

import static org.firstinspires.ftc.teamcode.commands.SleeveDetection.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.roadrunner.drive.*;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.openftc.easyopencv.*;

@Disabled
@Config
public abstract class SafeHigh extends LinearOpMode {

    Robot bot;
    SampleMecanumDrive drive;
    SleeveDetection sleeveDetection;
    OpenCvCamera camera;
    String webcamName;

    TrajectorySequence ScorePreload;
    TrajectorySequence ScoreToStorage1, ScoreToStorage2, ScoreToStorage3, ScoreToStorage4, ScoreToStorage5;
    TrajectorySequence StorageToScore1, StorageToScore2, StorageToScore3, StorageToScore4, StorageToScore5;
    TrajectorySequence WaitAtScore1, WaitAtScore2, WaitAtScore3, WaitAtScore4, WaitAtScore5, WaitAtScore6, WaitAtStorage1, WaitAtStorage2, WaitAtStorage3, WaitAtStorage4, WaitAtStorage5;
    TrajectorySequence ParkLeft, ParkRight, ParkMiddle;
    Pose2d SCORING_POSITION, STORAGE_POSITION;

    @Override
    public void runOpMode(){
        telemetry.addLine("Status: Initializing");
        telemetry.update();

        bot = new Robot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        sleeveDetection = new SleeveDetection();

        bot.slide.setModeToPosition();
        bot.claw.close();
        bot.arm.setAutoPositions(182);

        DriveConstants.MAX_VEL = 45;
        DriveConstants.MAX_ACCEL = 45;
        DriveConstants.MAX_ANG_VEL = Math.toRadians(90);
        DriveConstants.MAX_ANG_ACCEL = Math.toRadians(90);

        build();

        setCameraPosition();
        initCam();

        while (!isStarted()) {
            telemetry.addData("STATUS:", "INITIALIZED");
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }
        waitForStart();

        execute(sleeveDetection.getPosition());

        bot.slide.setPosition(State.ONE);
    }

    public abstract void setCameraPosition();
    public abstract void build();
    public void execute(TSEPosition position){
        camera.closeCameraDevice();
        bot.arm.setArms(192);
        bot.arm.setArms(192);

        //Score 1+0
        drive.followTrajectorySequence(ScorePreload);
        bot.claw.open();
        drive.followTrajectorySequence(WaitAtScore1);

        //Intake from five
        bot.slide.setPosition(State.FIVE);
        drive.followTrajectorySequence(ScoreToStorage1);
        bot.claw.close();
        drive.followTrajectorySequence(WaitAtStorage1);

        //Score 1+1
        drive.followTrajectorySequence(StorageToScore1);
        bot.claw.open();
        drive.followTrajectorySequence(WaitAtScore2);

        //Intake from four
        bot.slide.setPosition(State.FOUR);
        drive.followTrajectorySequence(ScoreToStorage2);
        bot.claw.close();
        drive.followTrajectorySequence(WaitAtStorage2);

        //Score 1+2
        drive.followTrajectorySequence(StorageToScore2);
        bot.claw.open();
        drive.followTrajectorySequence(WaitAtScore3);

        //Intake from three
        bot.slide.setPosition(State.THREE);
        drive.followTrajectorySequence(ScoreToStorage3);
        bot.claw.close();
        drive.followTrajectorySequence(WaitAtStorage3);

        //Score 1+3
        drive.followTrajectorySequence(StorageToScore3);
        bot.claw.open();
        drive.followTrajectorySequence(WaitAtScore4);

        //Intake from two
        bot.slide.setPosition(State.TWO);
        drive.followTrajectorySequence(ScoreToStorage4);
        bot.claw.close();
        drive.followTrajectorySequence(WaitAtStorage4);

        //Score 1+4
        drive.followTrajectorySequence(StorageToScore4);
        bot.claw.open();
        drive.followTrajectorySequence(WaitAtScore5);

        //Intake from one
        bot.slide.setPosition(State.ONE);
        drive.followTrajectorySequence(ScoreToStorage5);
        bot.claw.close();
        drive.followTrajectorySequence(WaitAtStorage5);

        //Score 1+5
        drive.followTrajectorySequence(StorageToScore5);
        bot.claw.open();

        drive.followTrajectorySequence(WaitAtScore6);

        bot.setPosition(State.INTAKING);
        bot.slide.setPosition(State.ONE);
        //drive.followTrajectorySequence(ParkMiddle);

        switch(position) {
            case LEFT:
                drive.followTrajectorySequence(ParkLeft);
                break;
            case RIGHT:
                drive.followTrajectorySequence(ParkRight);
                break;
            case MIDDLE:
                drive.followTrajectorySequence(ParkMiddle);
        }
        bot.slide.setPosition(State.ONE);
    }
    public TrajectorySequence waitSequence(TrajectorySequence preceding, double time){
        return drive.trajectorySequenceBuilder(preceding.end())
                .waitSeconds(time)
                .build();

    }
    public TrajectorySequence ScoreToStorage(TrajectorySequence preceding, double xOffset, double yOffset, double headingOffset){
        return drive.trajectorySequenceBuilder(preceding.end())
                .setReversed(false)
                .addTemporalMarker(0.2, () -> {
                    bot.arm.setPosition(State.INTAKING);
                    bot.claw.setPosition(State.INTAKING);
                })
                .addTemporalMarker(2.1, () -> {
                    bot.claw.close();
                })
                .addTemporalMarker(2.25, () -> {
                    bot.slide.setPosition(State.MIDDLE);
                    bot.slide.midLilHigher();
                })
                .splineTo(new Vector2d(STORAGE_POSITION.getX()+xOffset, STORAGE_POSITION.getY()+yOffset), STORAGE_POSITION.getHeading()+headingOffset)
                .forward(25.5)
                .build();
    }
    public TrajectorySequence StorageToScore(TrajectorySequence preceding, double xOffset, double yOffset, double headingOffset){
        return drive.trajectorySequenceBuilder(preceding.end())
                .addTemporalMarker(0.2, ()->{
                    bot.arm.setPosition(State.HIGH);
                })
                .addTemporalMarker(0.7, ()->{
                    bot.setPosition(State.HIGH);
                })
                /*
                .addTemporalMarker(1.8, () -> {
                    bot.claw.outtakeUpdate(State.HIGH, gamepad1, gamepad2, 10);
                })
                */
                .addTemporalMarker(2.4, () -> {
                    bot.slide.setPosition(State.MIDDLE);
                    bot.arm.slamThatJawn();
                })
                .back(35.5)
                .splineTo(new Vector2d(SCORING_POSITION.getX()+xOffset,SCORING_POSITION.getY()+yOffset), SCORING_POSITION.getHeading()+headingOffset)
                .build();
    }

    private void initCam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }
}
