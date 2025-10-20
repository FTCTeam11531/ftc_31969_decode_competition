package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.system.indexer.Indexer;
import org.firstinspires.ftc.teamcode.system.intake.Intake;
import org.firstinspires.ftc.teamcode.system.shooter.Shooter;
import org.firstinspires.ftc.teamcode.system.drivetrain.DrivetrainMecanum;
import org.firstinspires.ftc.teamcode.system.sound.Sound;
import org.firstinspires.ftc.teamcode.system.vision.Vision;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;

import java.util.Locale;

@TeleOp(name="Driver Control", group="_main")
public class DriverControl extends LinearOpMode {

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

        Gamepad currDriver = new Gamepad();
        Gamepad currOperator = new Gamepad();
        Gamepad prevDriver = new Gamepad();
        Gamepad prevOperator = new Gamepad();

        double inputAxial, inputLateral, inputYaw, restrictionExtensionMax,
                shooterOutputLeft = 0.80, shooterOutputRight = 0.80,
                intakeOutputLeft = 0.80, intakeOutputRight = 0.80,
                indexerOutputLeft = 1.0, indexerOutputRight = 1.0;

        HuskyLens.Block[] listTargetAIObjects = null;
        HuskyLens.Block targetAIObject = null;

        String detectedAprilTagIds;

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

        // Clear all telemetry
        telemetry.clearAll();

        // Loop while opMode is in initialize
        while (opModeInInit() && !isStopRequested()) {

            // Vision - get apriltag target data
            listTargetAIObjects = vision.getListAICameraObject();

            if (listTargetAIObjects.length > 0) {
                targetAIObject = vision.getAICameraObject(listTargetAIObjects);
            }

            detectedAprilTagIds = vision.getDetectedAprilTagIds();

            // ------------------------------------------------------------
            // Send telemetry message to signify robot completed initialization and waiting to start;
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------------");
            telemetry.addData("-", "All Systems Ready - Waiting to Start");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("run time", "%.1f seconds", opModeRunTime.seconds());
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("drivetrain", String.format(Locale.US,"{mode: %s, speed: %s}", drivetrain.getDrivetrainMode().getLabel(), drivetrain.getDrivetrainOutputPower().getLabel()));
            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("light mode", sysLighting.getLightPatternCurrent().toString());

            // Show joystick information
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Controller Input");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("main", String.format(Locale.US,"{left X: %.3f, Left Y: %.3f, Right X: %.3f, Right Y: %.3f}", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y));
            telemetry.addData("alt", String.format(Locale.US,"{left X: %.3f, Left Y: %.3f, Right X: %.3f, Right Y: %.3f}", gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_x, gamepad2.right_stick_y));
            telemetry.addData("trigger", String.format(Locale.US,"{left: %.3f, Right: %.3f}", gamepad2.left_trigger, gamepad2.right_trigger));

            // Show imu / odometry information
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Inertia Measurement Unit");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("imu", drivetrain.getImuStatus());
            telemetry.addData("heading", String.format(Locale.US,"{raw: %.3f, adj: %.3f}", drivetrain.getRobotHeadingRaw(), drivetrain.getRobotHeadingAdj()));
            telemetry.addData("position", drivetrain.getImuPositionDetail());
            telemetry.addData("velocity", drivetrain.getImuVelocityDetail());

            // ------------------------------------------------------------
            // - Vision telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Vision");
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
        }

        // Wait for Start state (from driver station) - (disable if using an init loop)
//        waitForStart();


        // Reset runtime timer
        opModeRunTime.reset();

        // Clear all telemetry
        telemetry.clearAll();

        // About to play battle cry
        sound.playSoundFileByName(RobotConstants.Sound.kSoundFileLightSaberLong);

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
            listTargetAIObjects = vision.getListAICameraObject();

            if (listTargetAIObjects.length > 0) {
                targetAIObject = vision.getAICameraObject(listTargetAIObjects);
            }

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
            Pose2d robotPose = drivetrain.localizer.getPose();

            inputAxial = -(currDriver.left_stick_y);
            inputLateral = (currDriver.left_stick_x);
            inputYaw =  (currDriver.right_stick_x);

//            Vector2d inputAdjustment = new Vector2d(
//                      inputAxial
//                    , inputLateral
//            ).
//            ).angleCast(poseRobot.heading.inverse());



            // Update Odometry Reading(s)
//            drivetrain.updateOdometry();
            drivetrain.updatePoseEstimate();

            // Endgame Notification
//            if(opModeRunTime.time() >= RobotConstants.CommonSettings.GameSettings.kEndgameStartTime && opModeRunTime.time() <= RobotConstants.CommonSettings.GameSettings.kEndgameEndTime) {
//                sysLighting.setLightPattern(RobotConstants.Lighting.Pattern.Default.kEndgame);
//            }
//            else if(opModeRunTime.time() >= RobotConstants.CommonSettings.GameSettings.kEndgameEndTime) {
//                sysLighting.setLightPattern(RobotConstants.Lighting.Pattern.Default.kEnd);
//            }

            // Drivetrain Type determined by 'Drivetrain Mode' enumeration selection (Default to Field Centric)
            if(drivetrain.getDrivetrainMode().equals(DrivetrainMecanum.DrivetrainMode.ROBOT_CENTRIC)) {

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
                drivetrain.setDrivetrainOutputPower(DrivetrainMecanum.DrivetrainSpeed.MEDIUM);
            }

            if(!currDriver.right_bumper) {
                drivetrain.setDrivetrainOutputPower(DrivetrainMecanum.DrivetrainSpeed.LOW);
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
                drivetrain.setDrivetrainMode(DrivetrainMecanum.DrivetrainMode.FIELD_CENTRIC);
            }

            if(currDriver.start && currDriver.dpad_down) {
                drivetrain.setDrivetrainMode(DrivetrainMecanum.DrivetrainMode.ROBOT_CENTRIC);
            }

            // ------------------------------------
            // Intake
            // Driver Control
            // ------------------------------------
            if(currDriver.dpad_up && !prevDriver.dpad_up) {
                if(intakeOutputLeft < 1.0) {
                    intakeOutputLeft = intakeOutputLeft + 0.05;
                    intakeOutputRight = intakeOutputLeft;
                }
            }

            if(currDriver.dpad_down && !prevDriver.dpad_down) {
                if(intakeOutputLeft > 0.0) {
                    intakeOutputLeft = intakeOutputLeft - 0.05;
                    intakeOutputRight = intakeOutputLeft;
                }
            }

            if(currDriver.left_trigger >= 0.20) {
                intake.activateIntake(intakeOutputRight);
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

            if(currOperator.left_bumper) {
                indexer.activateIndexer(RobotConstants.HardwareConfiguration.kLabelIndexServoLeft, indexerOutputLeft);
            }
            else {
                indexer.deactivateIndexer(RobotConstants.HardwareConfiguration.kLabelIndexServoLeft);
            }

            if(currOperator.right_bumper) {
                indexer.activateIndexer(RobotConstants.HardwareConfiguration.kLabelIndexServoRight, indexerOutputRight);
            }
            else {
                indexer.deactivateIndexer(RobotConstants.HardwareConfiguration.kLabelIndexServoRight);
            }

            // ------------------------------------
            // Shooter
            // Operator control
            // ------------------------------------
            if(currOperator.dpad_up && !prevOperator.dpad_up) {
                if(shooterOutputLeft < 1.0) {
                    shooterOutputLeft = shooterOutputLeft + 0.05;
                    shooterOutputRight = shooterOutputLeft;
                }
            }

            if(currOperator.dpad_down && !prevOperator.dpad_down) {
                if(shooterOutputLeft > 0.0) {
                    shooterOutputLeft = shooterOutputLeft - 0.05;
                    shooterOutputRight = shooterOutputLeft;
                }
            }

            if(currOperator.left_trigger >= 0.20) {
                shooter.activateShooter(RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft, shooterOutputLeft);
            }
            else {
                shooter.deactivateShooter(RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft);
            }

            if(currOperator.right_trigger >= 0.20) {
                shooter.activateShooter(RobotConstants.HardwareConfiguration.kLabelShooterMotorRight, shooterOutputRight);
            }
            else {
                shooter.deactivateShooter(RobotConstants.HardwareConfiguration.kLabelShooterMotorRight);
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
                            , robotPose.position.x
                            , robotPose.position.y
                            , Math.toDegrees(robotPose.heading.toDouble())));
            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("light mode", sysLighting.getLightPatternCurrent().toString());
            telemetry.addData("-","--------------------------------------");

            // Show joystick information
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Controller Input");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("main", String.format(Locale.US,"{left X: %.3f, Left Y: %.3f, Right X: %.3f, Right Y: %.3f}", currDriver.left_stick_x, currDriver.left_stick_y, currDriver.right_stick_x, currDriver.right_stick_y));
            telemetry.addData("alt", String.format(Locale.US,"{left X: %.3f, Left Y: %.3f, Right X: %.3f, Right Y: %.3f}", currOperator.left_stick_x, currOperator.left_stick_y, currOperator.right_stick_x, currOperator.right_stick_y));
            telemetry.addData("trigger", String.format(Locale.US,"{left: %.3f, Right: %.3f}", currOperator.left_trigger, currOperator.right_trigger));

            // Show imu / odometry information
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Inertia Measurement Unit");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("imu", drivetrain.getImuStatus());
            telemetry.addData("heading", String.format(Locale.US,"{raw: %.3f, adj: %.3f}", drivetrain.getRobotHeadingRaw(), drivetrain.getRobotHeadingAdj()));
            telemetry.addData("position", drivetrain.getImuPositionDetail());
            telemetry.addData("velocity", drivetrain.getImuVelocityDetail());

            // Shooter
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Shooting Setting(s)");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("main", String.format(Locale.US,"{left: %.3f, Right: %.3f}", shooterOutputLeft, shooterOutputRight));

            // Intake
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Intake Setting(s)");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("main", String.format(Locale.US,"{left: %.3f, Right: %.3f}", intakeOutputLeft, intakeOutputRight));


            // Show Arm and Intake Telemetry
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Arm / Intake");
            telemetry.addData("-","--------------------------------------");
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
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("-","-- Vision");
            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("alliance", String.format(Locale.US,"{color: %s, red: %d, blue: %d, green: %d}", sysVision.getAllianceColor(), sysVision.getAllianceColorValueRed(), sysVision.getAllianceColorValueBlue(), sysVision.getAllianceColorValueGreen()));
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

            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Detected April Tag ID    --");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Target ID", detectedAprilTagIds);

            vision.telemetryAprilTag();

            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------
            telemetry.update();

            // FTC Dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), robotPose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }


    }


}
