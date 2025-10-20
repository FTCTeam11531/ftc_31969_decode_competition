package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.system.drivetrain.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.system.indexer.Indexer;
import org.firstinspires.ftc.teamcode.system.intake.Intake;
import org.firstinspires.ftc.teamcode.system.shooter.Shooter;
import org.firstinspires.ftc.teamcode.system.sound.Sound;
import org.firstinspires.ftc.teamcode.system.vision.Vision;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Locale;

@Autonomous(name="Depot Launch Zone", group="_main", preselectTeleOp="DriverControl")
public class DepotLaunchZone extends LinearOpMode {

    // System - Drivetrain
    DrivetrainMecanum drivetrain;

    // System - Intake
    Intake intake = new Intake(this);

    // System - Indexer
    Indexer indexer = new Indexer(this);

    // System - Shooter
    Shooter shooter = new Shooter(this);

    // System - Vision
    Vision vision = new Vision(this);

    // System - Sound
    Sound sound = new Sound(this);

    @Override
    public void runOpMode() throws InterruptedException {

        // -------------------------------------------------
        // Misc - OpMode Variables
        // -------------------------------------------------
        ElapsedTime opModeRunTime = new ElapsedTime();
        Pose2d initialPose = RobotConstants.OpModeTransition.getPoseFinalOpMode();

        HuskyLens.Block[] listTargetAIObjects = null;
        HuskyLens.Block targetAIObject = null;

        String detectedAprilTagIds;

        int patternIdObelisk, pathAllianceAdj, headingAllianceAdj;

        AprilTagDetection localizationData = null;


        // Setup Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ------------------------------------------------------------
        // Initialize System(s)
        // ------------------------------------------------------------
        // System - Drivetrain
        DrivetrainMecanum drivetrain = new DrivetrainMecanum(hardwareMap, initialPose);

        // System - Indexer

        // System - Shooter
        shooter.init();

        // System - Shooter
        intake.init();

        // System - Indexer
        indexer.init();

        // System - Vision
        vision.init();
        vision.setAICameraMode(RobotConstants.Vision.HuskyLens.kLabelCameraModeAprilTag);

        // System - Sound
        sound.init();

        // -- Configuration - Get Initial Pose for Drivetrain
        if(vision.getDetectedAllianceTagColor().equals("blue")) {
            pathAllianceAdj = 1;
            headingAllianceAdj = 0;
//            initialPose = RobotConstants.Drivetrain.Autonomous.Pose.kInitialPoseHangmanBlue;
//            sysLighting.setLightPattern(RobotConstants.Lighting.Pattern.Default.kAutonomousAllianceBlueHangman);
        }
        else {
            pathAllianceAdj = -1;
            headingAllianceAdj = 180;
//            initialPose = RobotConstants.Drivetrain.Autonomous.Pose.kInitialPoseHangmanRed;
//            sysLighting.setLightPattern(RobotConstants.Lighting.Pattern.Default.kAutonomousAllianceRedHangman);
        }


        // Clear all telemetry
        telemetry.clearAll();

        // Loop while opMode is in initialize
        while (opModeInInit() && !isStopRequested()) {

            if (vision.getDetectedLocalization() != null) {
                localizationData = vision.getDetectedLocalization();

                Pose2d visionPose = new Pose2d(
                          Math.round(localizationData.robotPose.getPosition().x)
                        , Math.round(localizationData.robotPose.getPosition().y)
                        , Math.round(localizationData.robotPose.getOrientation().getYaw()));

                drivetrain.setPose(visionPose);
            }

            Pose2d robotPose = drivetrain.localizer.getPose();

            // Set Initial Pose to robot pose to be used when active
            initialPose = robotPose;

            // Vision - get apriltag target data
            listTargetAIObjects = vision.getListAICameraObject();

            if (listTargetAIObjects.length > 0) {
                targetAIObject = vision.getAICameraObject(listTargetAIObjects);
            }

            detectedAprilTagIds = vision.getDetectedAprilTagIds();
            patternIdObelisk = vision.getDetectedObeliskId();

            // ------------------------------------------------------------
            // Send telemetry message to signify robot completed initialization and waiting to start;
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------------");
            telemetry.addData("-", "All Systems Ready - Waiting to Start");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("run time", "%.1f seconds", opModeRunTime.seconds());
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("drivetrain", String.format(Locale.US,"{mode: %s, speed: %s}", drivetrain.getDrivetrainMode().getLabel(), drivetrain.getDrivetrainOutputPower().getLabel()));
            telemetry.addData("robot pose", String.format(Locale.US,"{x: %s, y: %s, heading: %s}"
                    , robotPose.position.x
                    , robotPose.position.y
                    , robotPose.heading.toDouble()));
            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("light mode", sysLighting.getLightPatternCurrent().toString());

            // ------------------------------------------------------------
            // - Vision telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Vision");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Alliance", vision.getDetectedAllianceTagColor());
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Detected April Tag ID    --");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Webcam");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Obelisk", String.format(Locale.US,"{id: %s, patten: %s}", patternIdObelisk, vision.getObeliskPattern(patternIdObelisk)));

            if (localizationData != null) {
                telemetry.addData("Localization", String.format(Locale.US,"{id: %s, zone: %s}", localizationData.id, localizationData.metadata.name));
            }

            telemetry.addData("-", "------------------------------");
            telemetry.addData("Target ID", detectedAprilTagIds);
            vision.telemetryAprilTag();

            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- HuskyLens");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Camera Block Count", vision.getListAICameraObject().length);
            if (targetAIObject != null) {
                telemetry.addData("-", "------------------------------");
                telemetry.addData("-", "-- Target Object");
                telemetry.addData("-", "------------------------------");
                telemetry.addData("Target ID", targetAIObject.id);
                telemetry.addData("Target x:", targetAIObject.x);
                telemetry.addData("Target y:", targetAIObject.y);
                telemetry.addData("Target width:", targetAIObject.width);
                telemetry.addData("Target height:", targetAIObject.height);
                telemetry.addData("Target top:", targetAIObject.top);
                telemetry.addData("Target left:", targetAIObject.left);
            }


            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------
            telemetry.update();
            idle();

            // FTC Dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), robotPose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        // Wait for Start state (from driver station) - (disable if using an init loop)
//        waitForStart();

        // Reset runtime timer
        opModeRunTime.reset();

        // Clear all telemetry
        telemetry.clearAll();

        // Actions to run during Autonomous Play
        if(opModeIsActive() && !isStopRequested()) {

            telemetry.addData("-", "------------------------------------");
            telemetry.addData("-", "Autonomous Mode Started");
            telemetry.addData("-","--------------------------------------");
            telemetry.update();

            TrajectoryActionBuilder waitPeriodSecondsHalf = drivetrain.actionBuilder(initialPose)
                    .waitSeconds(0.5);

            // Path - Move to Shoot - Position One
            TrajectoryActionBuilder pathStart = drivetrain.actionBuilder(initialPose)
                    .strafeTo(new Vector2d(0 * pathAllianceAdj, 33 * pathAllianceAdj))
                    .waitSeconds(0.5);

            // Log start of action(s)
            telemetry.addData("timestamp", "%.1f seconds", opModeRunTime.seconds());
            telemetry.addData("action", "Starting Run");
            telemetry.addData("-","--------------------------------------");
            telemetry.update();

            Actions.runBlocking(
                    new SequentialAction(

                            // Initial
                            waitPeriodSecondsHalf.build()

                            // Move to shooting Position
                            , pathStart.build()

                            // Shoot loaded Artifacts
                            , shooter.actionActivateShooter(
                                      RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft
                                    , RobotConstants.Shooter.Configuration.kMotorOutputPowerHigh)
                            , shooter.actionActivateShooter(
                                      RobotConstants.HardwareConfiguration.kLabelShooterMotorRight
                                    , RobotConstants.Shooter.Configuration.kMotorOutputPowerHigh)
                            , indexer.actionActivateIndexer(
                                      RobotConstants.HardwareConfiguration.kLabelIndexServoLeft
                                    , RobotConstants.Indexer.Servo.Setpoint.kForward)
                            , indexer.actionActivateIndexer(
                                      RobotConstants.HardwareConfiguration.kLabelIndexServoRight
                                    , RobotConstants.Indexer.Servo.Setpoint.kForward)


                    )
            );



            // Log timestamp for completion
            telemetry.addData("timestamp", "%.1f seconds", opModeRunTime.seconds());
            telemetry.addData("action", "All Actions Completed");
            telemetry.addData("-","--------------------------------------");
            telemetry.update();

        }

        // Wait for Auto to complete if not completed already
        while(!isStopRequested() && opModeIsActive()) {

            RobotConstants.OpModeTransition.setPoseFinalOpMode(drivetrain.getRobotPosition());
        }


    }


}
