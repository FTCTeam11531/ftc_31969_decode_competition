package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.system.drivetrain.DrivetrainPinpoint;
import org.firstinspires.ftc.teamcode.system.indexer.Indexer;
import org.firstinspires.ftc.teamcode.system.intake.Intake;
import org.firstinspires.ftc.teamcode.system.kickstand.Kickstand;
import org.firstinspires.ftc.teamcode.system.lighting.Lighting;
import org.firstinspires.ftc.teamcode.system.shooter.Shooter;
import org.firstinspires.ftc.teamcode.system.drivetrain.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.system.sound.Sound;
import org.firstinspires.ftc.teamcode.system.vision.Vision;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Locale;

@TeleOp(name="Driver Control", group="_main")
public class DriverControl extends LinearOpMode {

    // System - Drivetrain
    DrivetrainPinpoint drivetrain = new DrivetrainPinpoint(this);

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

    // System - Kickstand
    Kickstand kickstand = new Kickstand(this);

    @Override
    public void runOpMode() throws InterruptedException {

        // -------------------------------------------------
        // Misc - OpMode Variables
        // -------------------------------------------------
        ElapsedTime opModeRunTime = new ElapsedTime();
        Pose2D initialPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        Pose2D robotPose;

        Gamepad currDriver = new Gamepad();
        Gamepad currOperator = new Gamepad();
        Gamepad prevDriver = new Gamepad();
        Gamepad prevOperator = new Gamepad();

        double inputAxial, inputLateral, inputYaw, restrictionExtensionMax,
                  shooterOutputLeft = 0.80, shooterOutputRight = 0.80
                , shooterVelocityLeft = RobotConstants.Shooter.Setpoint.Velocity.kLongRange
                , shooterVelocityRight = RobotConstants.Shooter.Setpoint.Velocity.kLongRange
                , intakeOutputLeft = 0.80, intakeOutputRight = 0.80
                , indexerOutputLeft = 1.0, indexerOutputRight = 1.0
                , kickstandPosition = RobotConstants.Kickstand.Setpoint.kInitial;

        HuskyLens.Block[] listTargetAIObjects = null;
        HuskyLens.Block targetAIObject = null;

        String detectedAprilTagIds, labelAlliance = "unknown";

        int patternIdObelisk, pathAllianceAdj, headingAllianceAdj;

        AprilTagDetection localizationData = null;

        // Setup Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ------------------------------------------------------------
        // Initialize System(s)
        // ------------------------------------------------------------
        // System - Drivetrain
        drivetrain.init();

        // System - Shooter
        intake.init();

        // System - Indexer
        indexer.init();

        // System - Shooter
        shooter.init();

        // System - Vision
        vision.init();
//        vision.setAICameraMode(RobotConstants.Vision.HuskyLens.kLabelCameraModeAprilTag);

        // System - Sound
        sound.init();

        // System - Lighting
        lighting.init();

        // System - Kickstand
        kickstand.init();
        kickstandPosition = RobotConstants.Kickstand.Setpoint.kEndgame;

        // -- Configuration - Get Initial Pose for Drivetrain
//        if(vision.getDetectedAllianceColor().equals("blue")) {
//            pathAllianceAdj = 1;
//            headingAllianceAdj = 0;
////            initialPose = RobotConstants.Drivetrain.Autonomous.Pose.kInitialPoseHangmanBlue;
////            sysLighting.setLightPattern(RobotConstants.Lighting.Pattern.Default.kAutonomousAllianceBlueHangman);
//        }
//        else {
//            pathAllianceAdj = -1;
//            headingAllianceAdj = 180;
////            initialPose = RobotConstants.Drivetrain.Autonomous.Pose.kInitialPoseHangmanRed;
////            sysLighting.setLightPattern(RobotConstants.Lighting.Pattern.Default.kAutonomousAllianceRedHangman);
//        }

        // Clear all telemetry
        telemetry.clearAll();

        // Loop while opMode is in initialize
        while (opModeInInit() && !isStopRequested()) {

            // Vision - get apriltag target data
//            listTargetAIObjects = vision.getListAICameraObject();
//
//            if (listTargetAIObjects.length > 0) {
//                targetAIObject = vision.getAICameraObject(listTargetAIObjects);
//            }

            if (vision.getDetectedLocalization() != null) {
                localizationData = vision.getDetectedLocalization();

                Pose2D visionPose = new Pose2D(DistanceUnit.INCH
                        , Math.round(localizationData.robotPose.getPosition().x)
                        , Math.round(localizationData.robotPose.getPosition().y)
                        , AngleUnit.DEGREES
                        , Math.round(localizationData.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

                drivetrain.setRobotPose(visionPose);
            }

            robotPose = drivetrain.getRobotPose();

            // Update Odometry Reading(s)
            drivetrain.updateOdometry();

            // Set Initial Pose to robot pose to be used when active
            initialPose = robotPose;

            // Vision - get apriltag target data
//            listTargetAIObjects = vision.getListAICameraObject();

//            if (listTargetAIObjects.length > 0) {
//                targetAIObject = vision.getAICameraObject(listTargetAIObjects);
//            }

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
            telemetry.addData("drivetrain", String.format(Locale.US,"{mode: %s, speed: %s}"
                    , drivetrain.getDrivetrainMode().getLabel()
                    , drivetrain.getDrivetrainOutputPower().getLabel()));
            telemetry.addData(">", String.format(Locale.US,"{x: %.3f, y: %.3f, heading: %.3f}"
                    , robotPose.getX(DistanceUnit.INCH)
                    , robotPose.getY(DistanceUnit.INCH)
                    , robotPose.getHeading(AngleUnit.DEGREES)));
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("light mode", lighting.getLightPatternCurrent().toString());

            // Show imu / odometry information
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Inertia Measurement Unit");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("imu", drivetrain.getImuStatus());
            telemetry.addData("heading", String.format(Locale.US,"{raw: %.3f, adj: %.3f}", drivetrain.getRobotHeadingRaw(), drivetrain.getRobotHeadingAdj()));
            telemetry.addData("position", drivetrain.getTelemetryImuPositionDetail());
            telemetry.addData("position-alt", drivetrain.getTelemetryImuPositionAltDetail());
            telemetry.addData("velocity", drivetrain.getTelemetryImuVelocityDetail());

            // ------------------------------------------------------------
            // - Vision telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Vision");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Alliance", vision.getDetectedAllianceColor(labelAlliance));
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

//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("-", "-- HuskyLens");
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Camera Block Count", vision.getListAICameraObject().length);
//            if (targetAIObject != null) {
//                telemetry.addData("-", "------------------------------");
//                telemetry.addData("-", "-- Target Object");
//                telemetry.addData("-", "------------------------------");
//                telemetry.addData("Target ID", targetAIObject.id);
//                telemetry.addData("Target x:", targetAIObject.x);
//                telemetry.addData("Target y:", targetAIObject.y);
//                telemetry.addData("Target width:", targetAIObject.width);
//                telemetry.addData("Target height:", targetAIObject.height);
//                telemetry.addData("Target top:", targetAIObject.top);
//                telemetry.addData("Target left:", targetAIObject.left);
//            }

            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Detected April Tag ID    --");
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
            if(vision.checkTargetBearing()) {
                lighting.setLightPattern(RobotConstants.Lighting.Pattern.kOnTarget);
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

        // About to play battle cry
        sound.playSoundFileByName(RobotConstants.Sound.kSoundFileWookie);

        // Loop while opMode is active
        while (opModeIsActive() && !isStopRequested()) {

            if(isStopRequested()) {

                // Update the Transition Adjustment Value for the IMU
                RobotConstants.Drivetrain.Odometry.Transition.setImuTransitionAdjustment(drivetrain.getRobotHeadingRaw());

                // Stop any sounds that might be playing
                sound.stopAllSoundPlayback();

                return;
            }

            // Store current controller output to previous
            prevDriver.copy(currDriver);
            prevOperator.copy(currOperator);

            // get Current controller output
            currDriver.copy(gamepad1);
            currOperator.copy(gamepad2);

            // Vision - get apriltag target data
//            listTargetAIObjects = vision.getListAICameraObject();
//
//            if (listTargetAIObjects.length > 0) {
//                targetAIObject = vision.getAICameraObject(listTargetAIObjects);
//            }

            detectedAprilTagIds = vision.getDetectedAprilTagIds();

            // ------------------------------------------------------------
            // Controls
            // ------------------------------------------------------------
            // Gamepad1 = Main Driver
            // ------------------------------------------------------------
            // -- Robot Movement
            // -- -- Axis (left_stick_x, left_stick_y): Drive
            // -- -- Axis (right_stick_x): Rotate
            //

            // ------------------------------------------------------------
            // Drivetrain
            // ------------------------------------------------------------
            // Assign gamepad control to motion in relation to:
            // -- gamepad input, direction
            // -- robot orientation to field
            // -- installed direction of control hub
            // -- orientation of drivetrain/motors
            if (vision.getDetectedLocalization() != null) {
                localizationData = vision.getDetectedLocalization();

                Pose2D visionPose = new Pose2D(DistanceUnit.INCH
                        , Math.round(localizationData.robotPose.getPosition().x)
                        , Math.round(localizationData.robotPose.getPosition().y)
                        , AngleUnit.DEGREES
                        , Math.round(localizationData.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

                if (visionPose.getX(DistanceUnit.INCH) < 0) {
                    shooterVelocityLeft = RobotConstants.Shooter.Setpoint.Velocity.kMidRange;
                    shooterVelocityRight = RobotConstants.Shooter.Setpoint.Velocity.kMidRange;
                }
                else {
                    shooterVelocityLeft = RobotConstants.Shooter.Setpoint.Velocity.kLongRange;
                    shooterVelocityRight = RobotConstants.Shooter.Setpoint.Velocity.kLongRange;
                }
            }

            robotPose = drivetrain.getRobotPose();

            // Update Odometry Reading(s)
            drivetrain.updateOdometry();

            if(kickstand.getKickstandPosition() >= RobotConstants.Kickstand.Setpoint.kPrelift) {
                inputAxial = 0;
                inputLateral = 0;
                inputYaw = 0;
            }
            else {
                inputAxial = -(currDriver.left_stick_y);
                inputLateral = (currDriver.left_stick_x);
                inputYaw = (currDriver.right_stick_x);
            }

            // Endgame Notification
//            if(opModeRunTime.time() >= RobotConstants.CommonSettings.GameSettings.kEndgameStartTime && opModeRunTime.time() <= RobotConstants.CommonSettings.GameSettings.kEndgameEndTime) {
//                sysLighting.setLightPattern(RobotConstants.Lighting.Pattern.Default.kEndgame);
//            }
//            else if(opModeRunTime.time() >= RobotConstants.CommonSettings.GameSettings.kEndgameEndTime) {
//                sysLighting.setLightPattern(RobotConstants.Lighting.Pattern.Default.kEnd);
//            }

            // Drivetrain Type determined by 'Drivetrain Mode' enumeration selection (Default to Field Centric)
            if(drivetrain.getDrivetrainMode().equals(DrivetrainPinpoint.DrivetrainMode.ROBOT_CENTRIC)) {

                // Set Robot Centric Drivetrain
                drivetrain.driveMecanum(inputAxial, inputLateral, inputYaw, drivetrain.getDrivetrainOutputPower().getValue());
            }
            else {

                // Set Field Centric Drivetrain
                drivetrain.driveMecanumFieldCentric(inputAxial, inputLateral, inputYaw, drivetrain.getDrivetrainOutputPower().getValue());
            }

            // ------------------------------------
            // Drivetrain Speed Options
            // ------------------------------------
            if(currDriver.right_bumper) {
                drivetrain.setDrivetrainOutputPower(DrivetrainPinpoint.DrivetrainSpeed.MEDIUM);
            }

            if(!currDriver.right_bumper) {
                drivetrain.setDrivetrainOutputPower(DrivetrainPinpoint.DrivetrainSpeed.LOW);
            }

//            if(gamepad1.dpad_up) {
//                sysDrivetrain.setDrivetrainOutputPower(SysDrivetrainPinpoint.DrivetrainSpeed.HIGH);
//            }
//
//            if(gamepad1.dpad_down) {
//                sysDrivetrain.setDrivetrainOutputPower(SysDrivetrainPinpoint.DrivetrainSpeed.LOW);
//            }
//
//            if(gamepad1.dpad_left) {
//                sysDrivetrain.setDrivetrainOutputPower(SysDrivetrainPinpoint.DrivetrainSpeed.MEDIUM);
//            }
//
//            if(gamepad1.dpad_right) {
//                sysDrivetrain.setDrivetrainOutputPower(SysDrivetrainPinpoint.DrivetrainSpeed.SNAIL);
//            }

            // ------------------------------------
            // Drivetrain Mode Options
            // ------------------------------------
            if(currDriver.start && currDriver.dpad_up) {
                drivetrain.setDrivetrainMode(DrivetrainPinpoint.DrivetrainMode.FIELD_CENTRIC);
            }

//            if(currDriver.start && currDriver.dpad_down) {
//                drivetrain.setDrivetrainMode(DrivetrainPinpoint.DrivetrainMode.ROBOT_CENTRIC);
//            }

            // ------------------------------------
            // Intake
            // Driver Control
            // ------------------------------------
//            if(currDriver.dpad_up && !prevDriver.dpad_up) {
//                if(intakeOutputLeft < 1.0) {
//                    intakeOutputLeft = intakeOutputLeft + 0.05;
//                    intakeOutputRight = intakeOutputLeft;
//                }
//            }
//
//            if(currDriver.dpad_down && !prevDriver.dpad_down) {
//                if(intakeOutputLeft > 0.0) {
//                    intakeOutputLeft = intakeOutputLeft - 0.05;
//                    intakeOutputRight = intakeOutputLeft;
//                }
//            }

            if(currDriver.left_trigger >= 0.20) {
                intake.activateIntake(intakeOutputRight);
            }
            else if (currDriver.right_trigger >= 0.20) {
                intake.activateIntake(-intakeOutputRight);
            }
            else {
                intake.deactivateIntake();
            }

            // ------------------------------------
            // Indexer
            // Operator control
            // ------------------------------------
//            if(currOperator.dpad_left && !prevOperator.dpad_left) {
//                if(indexerOutputLeft < 1.0) {
//                    indexerOutputLeft = indexerOutputLeft + 0.05;
//                    indexerOutputRight = indexerOutputLeft;
//                }
//            }
//
//            if(currOperator.dpad_right && !prevOperator.dpad_right) {
//                if(shooterOutputLeft > 0.0) {
//                    indexerOutputLeft = indexerOutputLeft - 0.05;
//                    indexerOutputRight = indexerOutputLeft;
//                }
//            }

//            if(currOperator.left_bumper && shooter.checkShooterVelocityLevel(RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft, RobotConstants.Shooter.Configuration.kVelocityMin)) {
//                indexer.activateIndexer(RobotConstants.HardwareConfiguration.kLabelIndexServoLeft, indexerOutputLeft);
//            }
//            else {
//                indexer.deactivateIndexer(RobotConstants.HardwareConfiguration.kLabelIndexServoLeft);
//            }
//
//
//            if(currOperator.right_bumper && shooter.checkShooterVelocityLevel(RobotConstants.HardwareConfiguration.kLabelShooterMotorRight, RobotConstants.Shooter.Configuration.kVelocityMin)) {
//                indexer.activateIndexer(RobotConstants.HardwareConfiguration.kLabelIndexServoRight, indexerOutputRight);
//            }
//            else {
//                indexer.deactivateIndexer(RobotConstants.HardwareConfiguration.kLabelIndexServoRight);
//            }

            // ------------------------------------
            // Shooter
            // Operator control
            // ------------------------------------
//            if(currOperator.dpad_up && !prevOperator.dpad_up) {
//                if(shooterVelocityLeft < RobotConstants.Shooter.Setpoint.Velocity.kMaxRange) {
//                    shooterVelocityLeft = shooterVelocityLeft + 20;
//                    shooterVelocityRight = shooterVelocityLeft;
//                }
//            }
//
//            if(currOperator.dpad_down && !prevOperator.dpad_down) {
//                if(shooterVelocityLeft > RobotConstants.Shooter.Setpoint.Velocity.kMinRange) {
//                    shooterVelocityLeft = shooterVelocityLeft - 20;
//                    shooterVelocityRight = shooterVelocityLeft;
//                }
//            }

            if(currOperator.left_trigger >= 0.20) {
                shooter.activateShooterVelocity(RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft, shooterVelocityLeft);

                if(shooter.checkShooterVelocityLevel(RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft
                        , shooterVelocityLeft)) {

                    indexer.activateIndexer(RobotConstants.HardwareConfiguration.kLabelIndexServoLeft, indexerOutputLeft);
                }
            }
            else if(currOperator.left_bumper) {
                shooter.activateShooterVelocity(RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft, -RobotConstants.Shooter.Setpoint.Velocity.kMinRange);
                indexer.activateIndexer(RobotConstants.HardwareConfiguration.kLabelIndexServoLeft, -indexerOutputLeft);
                intake.activateIntake(-intakeOutputRight);
            }
            else {
                shooter.deactivateShooter(RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft);
                indexer.deactivateIndexer(RobotConstants.HardwareConfiguration.kLabelIndexServoLeft);
            }

            if(currOperator.right_trigger >= 0.20) {
                shooter.activateShooterVelocity(RobotConstants.HardwareConfiguration.kLabelShooterMotorRight, shooterVelocityRight);

                if(shooter.checkShooterVelocityLevel(RobotConstants.HardwareConfiguration.kLabelShooterMotorRight
                        , shooterVelocityRight)) {

                    indexer.activateIndexer(RobotConstants.HardwareConfiguration.kLabelIndexServoRight, indexerOutputLeft);
                }
            }
            else if(currOperator.right_bumper) {
                shooter.activateShooterVelocity(RobotConstants.HardwareConfiguration.kLabelShooterMotorRight, -RobotConstants.Shooter.Setpoint.Velocity.kMinRange);
                indexer.activateIndexer(RobotConstants.HardwareConfiguration.kLabelIndexServoRight, -indexerOutputLeft);
                intake.activateIntake(-intakeOutputRight);
            }
            else {
                shooter.deactivateShooter(RobotConstants.HardwareConfiguration.kLabelShooterMotorRight);
                indexer.deactivateIndexer(RobotConstants.HardwareConfiguration.kLabelIndexServoRight);
            }

            // ------------------------------------------------------------
            // Lighting
            // ------------------------------------------------------------
            if(shooter.checkShooterVelocityLevel(RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft
                    , shooterVelocityLeft)
                || shooter.checkShooterVelocityLevel(RobotConstants.HardwareConfiguration.kLabelShooterMotorRight
                    , shooterVelocityRight)) {

                lighting.setLightPattern(RobotConstants.Lighting.Pattern.kReadyToShoot);
            }
            else if(vision.checkTargetBearing()) {
                lighting.setLightPattern(RobotConstants.Lighting.Pattern.kOnTarget);
            }
            else if(opModeRunTime.time() >= 90.00 && opModeRunTime.time() <= 120.00) {
                lighting.setLightPattern(RobotConstants.Lighting.Pattern.kEndgame);
            }
            else {
                lighting.setLightPattern(RobotConstants.Lighting.Pattern.kTeleop);
            }

            // ------------------------------------------------------------
            // Endgame
            // ------------------------------------------------------------
            if(currDriver.dpad_up && !prevDriver.dpad_up) {
                if(kickstandPosition < 1.0) {
                    kickstandPosition = kickstandPosition + 0.05;
                }
            }

            if(currDriver.dpad_down && !prevDriver.dpad_down) {
                if(kickstandPosition > 0.0) {
                    kickstandPosition = kickstandPosition - 0.05;
                }
            }

            if(currDriver.a && currOperator.a) {
                kickstand.setKickstandPosition(kickstandPosition);
            }

            if(currDriver.b && currOperator.b) {
                kickstand.setKickstandPosition(RobotConstants.Kickstand.Setpoint.kInitial);
            }

            // ------------------------------------
            // Override
            // ------------------------------------
            if((currDriver.start && currDriver.back) && !(prevDriver.start && prevDriver.back)) {

                // Reset the Robot Heading (normally done on init of Drivetrain system)
                drivetrain.resetZeroRobotHeading();

                // Confirmation
                sound.playSoundFileByName(RobotConstants.Sound.kSoundFileRogerRoger);
            }

            // ------------------------------------
            // Driver Hub Feedback
            // ------------------------------------
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Teleop - Main");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("run time", "%.1f seconds", opModeRunTime.seconds());
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("drivetrain", String.format(Locale.US,"{mode: %s, speed: %s}"
                    , drivetrain.getDrivetrainMode().getLabel()
                    , drivetrain.getDrivetrainOutputPower().getLabel()));
            telemetry.addData(">", String.format(Locale.US,"{x: %s, y: %s, heading: %s}"
                    , robotPose.getX(DistanceUnit.INCH)
                    , robotPose.getY(DistanceUnit.INCH)
                    , robotPose.getHeading(AngleUnit.DEGREES)));
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("light mode", lighting.getLightPatternCurrent().toString());
            telemetry.addData("-","--------------------------------------");

            // Show imu / odometry information
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Inertia Measurement Unit");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("imu", drivetrain.getImuStatus());
            telemetry.addData("heading", String.format(Locale.US,"{raw: %.3f, adj: %.3f}", drivetrain.getRobotHeadingRaw(), drivetrain.getRobotHeadingAdj()));
            telemetry.addData("position", drivetrain.getTelemetryImuPositionDetail());
            telemetry.addData("position-alt", drivetrain.getTelemetryImuPositionAltDetail());
            telemetry.addData("velocity", drivetrain.getTelemetryImuVelocityDetail());

            // Shooter
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Shooting Setting(s)");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("main", String.format(Locale.US,"{left: %.3f, Right: %.3f}", shooterVelocityLeft, shooterVelocityRight));
            telemetry.addData("P,I,D,F (left)", "%.04f, %.04f, %.04f, %.04f"
                    , shooter.getShooterPIDFCoefficient(RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft).p
                    , shooter.getShooterPIDFCoefficient(RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft).i
                    , shooter.getShooterPIDFCoefficient(RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft).d
                    , shooter.getShooterPIDFCoefficient(RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft).f);
            telemetry.addData("P,I,D,F (right)", "%.04f, %.04f, %.04f, %.04f"
                    , shooter.getShooterPIDFCoefficient(RobotConstants.HardwareConfiguration.kLabelShooterMotorRight).p
                    , shooter.getShooterPIDFCoefficient(RobotConstants.HardwareConfiguration.kLabelShooterMotorRight).i
                    , shooter.getShooterPIDFCoefficient(RobotConstants.HardwareConfiguration.kLabelShooterMotorRight).d
                    , shooter.getShooterPIDFCoefficient(RobotConstants.HardwareConfiguration.kLabelShooterMotorRight).f);
            telemetry.addData("power"
                    , String.format(Locale.US,"{left: %.3f, Right: %.3f}"
                            , shooter.getMotorPower(RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft)
                            , shooter.getMotorPower(RobotConstants.HardwareConfiguration.kLabelShooterMotorRight)));
            telemetry.addData("velocity"
                    , String.format(Locale.US,"{left: %.3f, Right: %.3f}"
                            , shooter.getMotorVelocity(RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft)
                            , shooter.getMotorVelocity(RobotConstants.HardwareConfiguration.kLabelShooterMotorRight)));

            // Intake
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Intake Setting(s)");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("main", String.format(Locale.US,"{left: %.3f, Right: %.3f}", intakeOutputLeft, intakeOutputRight));
            telemetry.addData("power"
                    , String.format(Locale.US,"{left: %.3f, Right: %.3f}"
                            , 0.0
                            , intake.getMotorPower(RobotConstants.HardwareConfiguration.kLabelIntakeMotorRight)));
            telemetry.addData("velocity"
                    , String.format(Locale.US,"{left: %.3f, Right: %.3f}"
                            , 0.0
                            , intake.getMotorVelocity(RobotConstants.HardwareConfiguration.kLabelIntakeMotorRight)));


            // Show Arm and Intake Telemetry
//            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("-","-- Arm / Intake");
//            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("arm position - motor", String.format(Locale.US,"{pivot: %d, extend left: %d, extend right: %d, shuttle: %d}", sysArm.getArmMotorCurrentPosition(RobotConstants.HardwareConfiguration.kLabelArmMotorPivot), sysArm.getArmMotorCurrentPosition(RobotConstants.HardwareConfiguration.kLabelArmMotorExtendLeft), sysArm.getArmMotorCurrentPosition(RobotConstants.HardwareConfiguration.kLabelArmMotorExtendRight), sysArm.getArmMotorCurrentPosition(RobotConstants.HardwareConfiguration.kLabelArmMotorShuttle)));
//            telemetry.addData("arm position - tolerance", String.format(Locale.US,"{pivot: %d, extend left: %d, extend right: %d}", sysArm.getArmMotorPositionTolerance(RobotConstants.HardwareConfiguration.kLabelArmMotorPivot), sysArm.getArmMotorPositionTolerance(RobotConstants.HardwareConfiguration.kLabelArmMotorExtendLeft), sysArm.getArmMotorPositionTolerance(RobotConstants.HardwareConfiguration.kLabelArmMotorExtendRight)));
//            telemetry.addData("arm position - extension", String.format(Locale.US,"{left: %.3f, right: %.3f, max: %.3f}", sysArm.getArmExtensionSensorCurrentPosition(RobotConstants.HardwareConfiguration.kLabelArmSensorExtendLimitLeft), sysArm.getArmExtensionSensorCurrentPosition(RobotConstants.HardwareConfiguration.kLabelArmSensorExtendLimitRight), sysArm.getArmExtensionPosition()));
//            telemetry.addData("intake position", String.format(Locale.US,"{rotate: %.3f, in left: %.3f, in right: %.3f}", sysIntake.getIntakeRotationPosition(), sysIntake.getIntakeCollectionPower(RobotConstants.HardwareConfiguration.kLabelIntakeServoCollectionLeft), sysIntake.getIntakeCollectionPower(RobotConstants.HardwareConfiguration.kLabelIntakeServoCollectionRight)));
//            telemetry.addData("arm extension - lower limit", sysArm.getArmLimitSensorTripped(RobotConstants.HardwareConfiguration.kLabelArmSensorExtendLimitLower));
//            telemetry.addData("arm pivot - lower limit", sysArm.getArmLimitSensorTripped(RobotConstants.HardwareConfiguration.kLabelArmSensorPivotLimitLower));
//            telemetry.addData("arm pivot - upper limit", sysArm.getArmLimitSensorTripped(RobotConstants.HardwareConfiguration.kLabelArmSensorPivotLimitUpper));
//            telemetry.addData("arm shuttle - lower limit", sysArm.getArmLimitSensorTripped(RobotConstants.HardwareConfiguration.kLabelArmSensorExtendLimitLower));
//            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("-","-- Arm State");
//            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("Extend Position", sysArm.getArmPositionMode(RobotConstants.HardwareConfiguration.kLabelArmMotorExtendLeft));
//            telemetry.addData("Extend Direction", sysArm.getArmTravelDirectionMode(RobotConstants.HardwareConfiguration.kLabelArmMotorExtendLeft));

            // Show Vision
//            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("-","-- Vision");
//            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("alliance", String.format(Locale.US,"{color: %s, red: %d, blue: %d, green: %d}", sysVision.getAllianceColor(), sysVision.getAllianceColorValueRed(), sysVision.getAllianceColorValueBlue(), sysVision.getAllianceColorValueGreen()));
//            telemetry.addData("Camera Block Count", vision.getListAICameraObject().length);
//            if (targetAIObject != null) {
//                telemetry.addData("-", "------------------------------");
//                telemetry.addData("-", "-- Target Object");
//                telemetry.addData("-", "------------------------------");
//                telemetry.addData("Target ID", targetAIObject.id);
//                telemetry.addData("Target x:", targetAIObject.x);
//                telemetry.addData("Target y:", targetAIObject.y);
//                telemetry.addData("Target width:", targetAIObject.width);
//                telemetry.addData("Target height:", targetAIObject.height);
//                telemetry.addData("Target top:", targetAIObject.top);
//                telemetry.addData("Target left:", targetAIObject.left);
//            }

            // Intake
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Kickstand");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("main", String.format(Locale.US,"{current: %.3f, setpoint: %.3f}", kickstand.getKickstandPosition(), kickstandPosition));



//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("-", "-- Detected April Tag ID    --");
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Target ID", detectedAprilTagIds);
//
//            vision.telemetryAprilTag();

            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------
            telemetry.update();

            // FTC Dashboard
//            TelemetryPacket packet = new TelemetryPacket();
//            packet.fieldOverlay().setStroke("#3F51B5");
//            Drawing.drawRobot(packet.fieldOverlay(), robotPose);
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }


    }


}
