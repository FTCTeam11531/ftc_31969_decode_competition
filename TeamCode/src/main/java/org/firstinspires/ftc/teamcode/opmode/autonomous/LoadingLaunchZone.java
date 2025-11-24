package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.system.drivetrain.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.system.indexer.Indexer;
import org.firstinspires.ftc.teamcode.system.intake.Intake;
import org.firstinspires.ftc.teamcode.system.lighting.Lighting;
import org.firstinspires.ftc.teamcode.system.shooter.Shooter;
import org.firstinspires.ftc.teamcode.system.sound.Sound;
import org.firstinspires.ftc.teamcode.system.vision.Vision;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Locale;

@Autonomous(name="Loading Launch Zone", group="_main", preselectTeleOp="DriverControl")
public class LoadingLaunchZone extends LinearOpMode {

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

    // System - Lighting
    Lighting lighting = new Lighting(this);

    @Override
    public void runOpMode() throws InterruptedException {

        // -------------------------------------------------
        // Misc - OpMode Variables
        // -------------------------------------------------
        ElapsedTime opModeRunTime = new ElapsedTime();
        ElapsedTime driveTime = new ElapsedTime();

        Pose2d initialPose = new Pose2d(0,0,0);// RobotConstants.OpModeTransition.getPoseFinalOpMode();

        String detectedAprilTagIds, labelAlliance = "unknown";

        int patternIdObelisk, pathAllianceAdjX, pathAllianceAdjY, headingAllianceAdj;

        AprilTagDetection localizationData = null;

        Pose2d robotPose;


        // Setup Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ------------------------------------------------------------
        // Initialize System(s)
        // ------------------------------------------------------------
        // System - Drivetrain
        DrivetrainMecanum drivetrain = new DrivetrainMecanum(hardwareMap, initialPose);
        drivetrain.resetZeroRobotHeading();

        // System - Indexer

        // System - Shooter
        shooter.init();

        // System - Shooter
        intake.init();

        // System - Indexer
        indexer.init();

        // System - Vision
        vision.init();
//        vision.setAICameraMode(RobotConstants.Vision.HuskyLens.kLabelCameraModeAprilTag);

        // System - Sound
        sound.init();

        // System - Lighting
        lighting.init();

        // Clear all telemetry
        telemetry.clearAll();


        // Loop while opMode is in initialize
        while (opModeInInit() && !isStopRequested()) {

            if (vision.getDetectedLocalization() != null) {
                localizationData = vision.getDetectedLocalization();

                Pose2d visionPose = new Pose2d(
                        Math.round(localizationData.robotPose.getPosition().x)
                        , Math.round(localizationData.robotPose.getPosition().y)
                        , Math.round(localizationData.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)));

                drivetrain.setPose(visionPose);
//                drivetrain.updatePoseEstimate();
                drivetrain.updateOdometry();
//                drivetrain.localizer.setPose(visionPose);
                drivetrain.localizer.update();

            }

            robotPose = drivetrain.localizer.getPose();

            // Set Initial Pose to robot pose to be used when active
            initialPose = robotPose;

            detectedAprilTagIds = vision.getDetectedAprilTagIds();
            patternIdObelisk = vision.getDetectedObeliskId();

            // -- Configuration - Get Initial Pose for Drivetrain
            if(vision.getDetectedAllianceColor(labelAlliance).equals("blue")) {
                pathAllianceAdjX = 1;
                pathAllianceAdjY = -1;
                headingAllianceAdj = 0;
                labelAlliance = "blue";
            }
            else {
                pathAllianceAdjX = 1;
                pathAllianceAdjY = 1;
                headingAllianceAdj = 180;
                labelAlliance = "red";
            }

            // ------------------------------------------------------------
            // Send telemetry message to signify robot completed initialization and waiting to start;
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------------");
            telemetry.addData("-", "All Systems Ready - Waiting to Start");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("run time", "%.1f seconds", opModeRunTime.seconds());
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Alliance", vision.getDetectedAllianceColor(labelAlliance));
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("drivetrain", String.format(Locale.US,"{mode: %s, speed: %s}", drivetrain.getDrivetrainMode().getLabel(), drivetrain.getDrivetrainOutputPower().getLabel()));
            telemetry.addData("robot pose", String.format(Locale.US,"{x: %s, y: %s, heading: %s}"
                    , robotPose.position.x
                    , robotPose.position.y
                    , robotPose.heading));
            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("light mode", sysLighting.getLightPatternCurrent().toString());

            // Show imu / odometry information
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Inertia Measurement Unit");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("imu-status", drivetrain.getImuStatus());
            telemetry.addData("imu-heading", String.format(Locale.US,"{raw: %.3f, adj: %.3f}", drivetrain.getRobotHeadingRaw(), drivetrain.getRobotHeadingAdj()));
            telemetry.addData("imu-heading-diff", String.format(Locale.US,"{onboard: %.3f, pinpoint: %.3f}", drivetrain.getRobotHeadingOnBoard(), drivetrain.getRobotHeadingRaw()));
            telemetry.addData("position", drivetrain.getImuPositionDetail());
            telemetry.addData("velocity", drivetrain.getImuVelocityDetail());
            telemetry.addData("robot-pos", drivetrain.getRobotPosition());

            // ------------------------------------------------------------
            // - Vision telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Vision");
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

            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------
            telemetry.update();
            idle();

            // ------------------------------------------------------------
            // Lighting
            // ------------------------------------------------------------
            if(shooter.checkShooterVelocityLevel(RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft
                    , RobotConstants.Shooter.Configuration.kVelocityMin)
                    || shooter.checkShooterVelocityLevel(RobotConstants.HardwareConfiguration.kLabelShooterMotorRight
                    , RobotConstants.Shooter.Configuration.kVelocityMin)) {

                lighting.setLightPattern(RobotConstants.Lighting.Pattern.kReadyToShoot);
            }
            else if(vision.checkTargetBearing()) {
                if(labelAlliance == "blue") {
                    lighting.setLightPattern(RobotConstants.Lighting.Pattern.kOnTargetBlue);
                }
                else if(labelAlliance == "red") {
                    lighting.setLightPattern(RobotConstants.Lighting.Pattern.kOnTargetRed);
                }
                else {
                    lighting.setLightPattern(RobotConstants.Lighting.Pattern.kOnTarget);
                }
            }
            else if(opModeRunTime.time() >= 90.00 && opModeRunTime.time() <= 120.00) {
                lighting.setLightPattern(RobotConstants.Lighting.Pattern.kEndgame);
            }
            else {
                lighting.setLightPattern(RobotConstants.Lighting.Pattern.kTeleop);
            }

            // FTC Dashboard
//            TelemetryPacket packet = new TelemetryPacket();
//            packet.fieldOverlay().setStroke("#3F51B5");
//            Drawing.drawRobot(packet.fieldOverlay(), robotPose);
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
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

            TrajectoryActionBuilder waitPeriodSecondsOneHalf = drivetrain.actionBuilder(initialPose)
                    .waitSeconds(1.5);

            TrajectoryActionBuilder waitPeriodSecondsThree = drivetrain.actionBuilder(initialPose)
                    .waitSeconds(3.0);

            TrajectoryActionBuilder waitPeriodSecondsSix = drivetrain.actionBuilder(initialPose)
                    .waitSeconds(6.0);

            // Path - Move to Shoot - Position One
//            TrajectoryActionBuilder pathStart = drivetrain.actionBuilder(initialPose)
//                    .splineToLinearHeading(new Pose2d(
//                             25 * pathAllianceAdjX
//                            ,20 * pathAllianceAdjY
//                            , Math.toRadians(RobotConstants.UnitConversion.addTwoDegreeValuesTogether(180, headingAllianceAdj))), Math.PI/2)
//                    .waitSeconds(0.5);

//            TrajectoryActionBuilder pathStart;
//            if (labelAlliance == "blue") {
//                pathStart = drivetrain.actionBuilder(initialPose)
////                        .splineToLinearHeading(new Pose2d(
////                                25 * pathAllianceAdjX
////                                , 20 * pathAllianceAdjY
////                                , Math.toRadians(RobotConstants.UnitConversion.addTwoDegreeValuesTogether(180, headingAllianceAdj))), Math.PI / 2)
////                        .strafeTo(new Vector2d(25 * pathAllianceAdjX, 20 * pathAllianceAdjY))
////                    .splineToLinearHeading(new Pose2d(
////                             25 //* pathAllianceAdjX
////                            ,20 //* pathAllianceAdjY
////                            , Math.toRadians(RobotConstants.UnitConversion.addTwoDegreeValuesTogether(180, headingAllianceAdj))), Math.PI/2)
//                        .strafeTo(new Vector2d(20, -20))
//
//                        .waitSeconds(0.5);
//            }
//            else {
//                pathStart = drivetrain.actionBuilder(initialPose)
////                        .splineToLinearHeading(new Pose2d(
////                                25 * pathAllianceAdjX
////                                , 20 * pathAllianceAdjY
////                                , Math.toRadians(RobotConstants.UnitConversion.addTwoDegreeValuesTogether(180, headingAllianceAdj))), Math.PI / 2)
////                        .splineToLinearHeading(new Pose2d(
////                                25
////                                ,20
////                                , initialPose.heading.real), Math.PI / 2)
////                        .lineToY(20)
//                        .strafeTo(new Vector2d(20, 20))
//
//
//                        .waitSeconds(0.5);
//            }

            // Log start of action(s)
            telemetry.addData("timestamp", "%.1f seconds", opModeRunTime.seconds());
            telemetry.addData("action", "Starting Run");
            telemetry.addData("-","--------------------------------------");
            telemetry.update();

            Actions.runBlocking(
                    new SequentialAction(

                            // Initial
                            waitPeriodSecondsHalf.build()

                            // Shoot loaded Artifacts
                            , new ParallelAction(
                                    shooter.actionActivateShooterVelocity(
                                            RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft
                                        ,   RobotConstants.Shooter.Setpoint.Velocity.kLongRange)
                                    , shooter.actionActivateShooterVelocity(
                                            RobotConstants.HardwareConfiguration.kLabelShooterMotorRight
                                        ,   RobotConstants.Shooter.Setpoint.Velocity.kLongRange)
                                    , waitPeriodSecondsThree.build()
                            )
                            , waitPeriodSecondsThree.build()
                            , waitPeriodSecondsHalf.build()

                            , new ParallelAction(
                                shooter.actionActivateShooterVelocity(
                                        RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft
                                    ,   RobotConstants.Shooter.Setpoint.Velocity.kLongRange)
                                , shooter.actionActivateShooterVelocity(
                                        RobotConstants.HardwareConfiguration.kLabelShooterMotorRight
                                    ,   RobotConstants.Shooter.Setpoint.Velocity.kLongRange)
                                , indexer.actionActivateIndexer(
                                        RobotConstants.HardwareConfiguration.kLabelIndexServoLeft
                                    ,   RobotConstants.Indexer.Servo.Setpoint.kForward)
                                , indexer.actionActivateIndexer(
                                        RobotConstants.HardwareConfiguration.kLabelIndexServoRight
                                    ,   RobotConstants.Indexer.Servo.Setpoint.kForward)
                                , waitPeriodSecondsThree.build()
                            )
                            , waitPeriodSecondsThree.build()

                            , new ParallelAction(
                                shooter.actionActivateShooterVelocity(
                                        RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft
                                    ,   RobotConstants.Shooter.Setpoint.Velocity.kLongRange)
                                , shooter.actionActivateShooterVelocity(
                                        RobotConstants.HardwareConfiguration.kLabelShooterMotorRight
                                    ,   RobotConstants.Shooter.Setpoint.Velocity.kLongRange)
                                , indexer.actionActivateIndexer(
                                        RobotConstants.HardwareConfiguration.kLabelIndexServoLeft
                                    ,   RobotConstants.Indexer.Servo.Setpoint.kForward)
                                , indexer.actionActivateIndexer(
                                    RobotConstants.HardwareConfiguration.kLabelIndexServoRight
                                    ,   RobotConstants.Indexer.Servo.Setpoint.kForward)
                                , intake.actionActivateIntake(
                                    RobotConstants.Intake.Configuration.kMotorOutputPowerHigh)
                                , waitPeriodSecondsSix.build()
                            )
                            , waitPeriodSecondsThree.build()
                            , waitPeriodSecondsThree.build()

                            // Turn Off
                            , waitPeriodSecondsOneHalf.build()
                            , new ParallelAction(
                                    indexer.actionActivateIndexer(
                                        RobotConstants.HardwareConfiguration.kLabelIndexServoLeft
                                    ,   RobotConstants.Indexer.Servo.Setpoint.kInit)
                                ,   indexer.actionActivateIndexer(
                                        RobotConstants.HardwareConfiguration.kLabelIndexServoRight
                                    ,   RobotConstants.Indexer.Servo.Setpoint.kInit)
                                , shooter.actionActivateShooter(
                                        RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft, 0)
                                ,     shooter.actionActivateShooter(
                                        RobotConstants.HardwareConfiguration.kLabelShooterMotorRight, 0)
                                , intake.actionActivateIntake(
                                        0)

                            // Move away from launch zone
//                            , pathStart.build()

                            , waitPeriodSecondsOneHalf.build()

                    )

                    )
            );

            // Drive to Park
            driveTime.reset();
            if (labelAlliance == "blue") {
                while (opModeIsActive() && (driveTime.seconds() < 1)) {
                    drivetrain.driveMecanum(0, -.8, .10, .5);
                }
            }
            else {
                while (opModeIsActive() && (driveTime.seconds() < 1)) {
                    drivetrain.driveMecanum(0, .8, .10, .5);
                }
            }


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
