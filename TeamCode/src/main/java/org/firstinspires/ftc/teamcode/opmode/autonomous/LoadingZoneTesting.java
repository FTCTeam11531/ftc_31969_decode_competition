package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.system.drivetrain.DriveToPoint;
import org.firstinspires.ftc.teamcode.system.drivetrain.DrivetrainPinpoint;
import org.firstinspires.ftc.teamcode.system.indexer.Indexer;
import org.firstinspires.ftc.teamcode.system.intake.Intake;
import org.firstinspires.ftc.teamcode.system.lighting.Lighting;
import org.firstinspires.ftc.teamcode.system.shooter.Shooter;
import org.firstinspires.ftc.teamcode.system.sound.Sound;
import org.firstinspires.ftc.teamcode.system.vision.Vision;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Locale;

@Disabled
@Autonomous(name="Loading Zone - Testing!", group="_testing", preselectTeleOp="DriverControl")
public class LoadingZoneTesting extends LinearOpMode {

    // System- Drivetrain
    DrivetrainPinpoint drivetrain = new DrivetrainPinpoint(this);
    DrivetrainPinpoint.RobotStateDrive robotStateDrive;

    DriveToPoint navControl = new DriveToPoint(this);

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
        Pose2D initialPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

        String detectedAprilTagIds;

        int patternIdObelisk, pathAllianceAdjX, pathAllianceAdjY, headingAllianceAdj;

        AprilTagDetection localizationData = null;

        String labelAlliance;
        Boolean isBlueAlliance;

        // Setup Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ------------------------------------------------------------
        // Initialize System(s)
        // ------------------------------------------------------------
        // System - Drivetrain
        drivetrain.init();

        // Starting!
//        navControl.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
//        navControl.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);

        navControl.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        navControl.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        navControl.setDriveType(DriveToPoint.DriveType.MECANUM);

        robotStateDrive = DrivetrainPinpoint.RobotStateDrive.RobotStart;

        // System - Shooter
        intake.init();

        // System - Indexer
        indexer.init();

        // System - Shooter
        shooter.init();

        // System - Vision
        vision.init();

        // System - Sound
        sound.init();

        // System - Lighting
        lighting.init();

        // -- Configuration - Get Initial Pose for Drivetrain
        if(vision.getDetectedAllianceColor().equals("blue")) {
            pathAllianceAdjX = 1;
            pathAllianceAdjY = -1;
            headingAllianceAdj = 0;
            labelAlliance = "blue";
            isBlueAlliance = true;
//            initialPose = RobotConstants.Drivetrain.Autonomous.Pose.kInitialPoseHangmanBlue;
//            sysLighting.setLightPattern(RobotConstants.Lighting.Pattern.Default.kAutonomousAllianceBlueHangman);
        }
        else {
            pathAllianceAdjX = 1;
            pathAllianceAdjY = 1;
            headingAllianceAdj = 180;
            labelAlliance = "red";
            isBlueAlliance = false;
//            initialPose = RobotConstants.Drivetrain.Autonomous.Pose.kInitialPoseHangmanRed;
//            sysLighting.setLightPattern(RobotConstants.Lighting.Pattern.Default.kAutonomousAllianceRedHangman);
        }

        // Clear all telemetry
        telemetry.clearAll();

        // Loop while opMode is in initialize
        while (opModeInInit() && !isStopRequested()) {

            if (vision.getDetectedLocalization() != null) {
                localizationData = vision.getDetectedLocalization();

                Pose2D visionPose = new Pose2D(DistanceUnit.INCH
                        , Math.round(localizationData.robotPose.getPosition().x)
                        , Math.round(localizationData.robotPose.getPosition().y)
                        , AngleUnit.DEGREES
                        , Math.round(localizationData.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

                drivetrain.setRobotPose(visionPose);
            }

            Pose2D robotPose = drivetrain.getRobotPose();

            // Update Odometry Reading(s)
            drivetrain.updateOdometry();

            // Set Initial Pose to robot pose to be used when active
            initialPose = robotPose;


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
            telemetry.addData("match check", String.format(Locale.US,"{alliance: %s, on-target: %s}", vision.getDetectedAllianceColor(), vision.checkTargetBearing()));
            telemetry.addData("imu status", drivetrain.getImuStatus());
            telemetry.addData("-","--------------------------------------");
//            telemetry.addData("drivetrain", String.format(Locale.US,"{mode: %s, speed: %s}", drivetrain.getDrivetrainMode().getLabel(), drivetrain.getDrivetrainOutputPower().getLabel()));
            telemetry.addData("robot pose", String.format(Locale.US,"{x: %.3f, y: %.3f, heading: %.3f}"
                    , robotPose.getX(DistanceUnit.INCH)
                    , robotPose.getY(DistanceUnit.INCH)
                    , robotPose.getHeading(AngleUnit.DEGREES)));
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Detected April Tag ID    --");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Obelisk", String.format(Locale.US,"{id: %s, patten: %s}", patternIdObelisk, vision.getObeliskPattern(patternIdObelisk)));
            if (localizationData != null) {
                telemetry.addData("Localization", String.format(Locale.US,"{id: %s, zone: %s}", localizationData.id, localizationData.metadata.name));
            }
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
                lighting.setLightPattern(RobotConstants.Lighting.Pattern.kAutonomous);
            }

        }

        // Wait for Start state (from driver station) - (disable if using an init loop)
//        waitForStart();


        // Reset runtime timer
        opModeRunTime.reset();

        // Clear all telemetry
        telemetry.clearAll();

        // About to play battle cry
//        sound.playSoundFileByName(RobotConstants.Sound.kSoundFileWookie);

        // Loop while opMode active
        while (opModeIsActive() && !isStopRequested()) {

            if(isStopRequested()) {

                // Stop any sounds that might be playing
                sound.stopAllSoundPlayback();

                return;
            }

            Pose2D robotPose = drivetrain.getRobotPose();

            // Update Odometry Reading(s)
            drivetrain.updateOdometry();

            switch (robotStateDrive) {

                case RobotStart:

                    robotStateDrive = DrivetrainPinpoint.RobotStateDrive.TargetLaunchZone;
//                    telemetry.addLine("Start!");
                    break;

                case TargetLaunchZone:

//                    shooter.activateShooterVelocity(
//                              RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft
//                            , RobotConstants.Shooter.Setpoint.Velocity.kLongRange
//                    );
//
//                    shooter.activateShooterVelocity(
//                            RobotConstants.HardwareConfiguration.kLabelShooterMotorRight
//                            , RobotConstants.Shooter.Setpoint.Velocity.kLongRange
//                    );

                    if(navControl.driveTo(
                              drivetrain.getRobotPosition()
                            , drivetrain.getRobotPosition()
                            , 0.7
                            , 0
                    )) {
//                        telemetry.addLine("Shoot And Score!");
                        robotStateDrive = DrivetrainPinpoint.RobotStateDrive.ParkAuto;
                    }

                    break;

                case ParkAuto:

                    if(navControl.driveTo(
                            drivetrain.getRobotPosition()
                            , robotStateDrive.getValue()
                            , 0.7
                            , 0
                    )) {
//                        telemetry.addLine("Auto Park!");
                    }
                    break;

            }


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
                lighting.setLightPattern(RobotConstants.Lighting.Pattern.kOnTarget);
            }
            else {
                lighting.setLightPattern(RobotConstants.Lighting.Pattern.kAutonomous);
            }


            // ------------------------------------------------------------
            // Send telemetry message to signify robot completed initialization and waiting to start;
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------------");
            telemetry.addData("-", "All Systems Ready - Waiting to Start");
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("run time", "%.1f seconds", opModeRunTime.seconds());
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("match check", String.format(Locale.US,"{alliance: %s, on-target: %s}", vision.getDetectedAllianceColor(), vision.checkTargetBearing()));
            telemetry.addData("imu status", drivetrain.getImuStatus());
            telemetry.addData("-","--------------------------------------");

            telemetry.addData("robot pose", String.format(Locale.US,"{x: %.3f, y: %.3f, heading: %.3f}"
                    , robotPose.getX(DistanceUnit.INCH)
                    , robotPose.getY(DistanceUnit.INCH)
                    , robotPose.getHeading(AngleUnit.DEGREES)));

            telemetry.addData("state pose", String.format(Locale.US,"{x: %.3f, y: %.3f, heading: %.3f}"
                    , robotStateDrive.getValue().getX(DistanceUnit.INCH)
                    , robotStateDrive.getValue().getY(DistanceUnit.INCH)
                    , robotStateDrive.getValue().getHeading(AngleUnit.DEGREES)));
            telemetry.addData("-","--------------------------------------");
            telemetry.addData("auto state", robotStateDrive);
            telemetry.addData("auto state value", robotStateDrive.getValue());



            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------
            telemetry.update();
            idle();

        }
    }
}
