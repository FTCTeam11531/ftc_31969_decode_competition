package org.firstinspires.ftc.teamcode.system.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.utility.RobotConstants;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

public class DrivetrainPinpoint {

    // Enum: Drive mode
    public enum DrivetrainMode {

        FIELD_CENTRIC {

            // Label for setting
            @Override
            public String getLabel() { return "Field Centric"; }

        },
        ROBOT_CENTRIC {

            // Label for setting
            @Override
            public String getLabel() { return "Robot Centric"; }

        };

        public abstract String getLabel();
    }

    // Enum: Drive Speed
    public enum DrivetrainSpeed {

        HIGH {

            // Value for setting
            @Override
            public double getValue() { return RobotConstants.Drivetrain.Configuration.kMotorOutputPowerHigh; }

            // Label for setting
            @Override
            public String getLabel() { return "High"; }

        },
        MEDIUM {

            // Value for setting
            @Override
            public double getValue() { return RobotConstants.Drivetrain.Configuration.kMotorOutputPowerMedium; }

            // Label for setting
            @Override
            public String getLabel() { return "Medium"; }

        },
        LOW {

            // Value for setting
            @Override
            public double getValue() { return RobotConstants.Drivetrain.Configuration.kMotorOutputPowerLow; }

            // Label for setting
            @Override
            public String getLabel() { return "Low"; }

        },
        SNAIL {

            // Value for setting
            @Override
            public double getValue() { return RobotConstants.Drivetrain.Configuration.kMotorOutputPowerSnail; }

            // Label for setting
            @Override
            public String getLabel() { return "Snail"; }

        };

        public abstract double getValue();
        public abstract String getLabel();
    }

    public enum RobotStateDrive {
        RobotStart {
            @Override
            public Pose2D getValue() { return RobotConstants.Drivetrain.Autonomous.Pose.kRobotStart; }
        },
        TargetLaunchZone {
            @Override
            public Pose2D getValue() { return RobotConstants.Drivetrain.Autonomous.Pose.kTargetLaunchZone; }
        },
        TargetDepotZone {
            @Override
            public Pose2D getValue() { return RobotConstants.Drivetrain.Autonomous.Pose.kTargetDepotZone; }
        },
        ParkAuto {
            @Override
            public Pose2D getValue() { return RobotConstants.Drivetrain.Autonomous.Pose.kParkAuto; }
        },

        ParkTeleop {
            @Override
            public Pose2D getValue() { return RobotConstants.Drivetrain.Autonomous.Pose.kParkTeleop; }
        };

        public abstract Pose2D getValue();
    }



    // System Opmode - Calling opmode
    private LinearOpMode opMode;

    // Drivetrain Settings
    private DrivetrainPinpoint.DrivetrainMode drivetrainMode = DrivetrainPinpoint.DrivetrainMode.FIELD_CENTRIC;
    private DrivetrainPinpoint.DrivetrainSpeed drivetrainSpeed = DrivetrainPinpoint.DrivetrainSpeed.MEDIUM;


    // Drivetrain Setting(s)


    // Define Hardware for subsystems
    private DcMotorEx drive_left_front, drive_left_back, drive_right_front, drive_right_back;
    private List<DcMotorEx> listMotorDrivetrain;

    private GoBildaPinpointDriver imu_pinpoint;
    private DriveToPoint robotNav;

    public DrivetrainPinpoint(LinearOpMode opMode) { this.opMode = opMode; }

    public void init() {

        // Telemetry - Initialize - Start
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.addData(">", "System: Drivetrain");
        opMode.telemetry.addData(">", "------------------------------------");

        // Define and Initialize Motor(s)
        drive_left_front = opMode.hardwareMap.get(DcMotorEx.class, RobotConstants.HardwareConfiguration.kLabelDrivetrainMotorLeftFront);
        drive_left_back = opMode.hardwareMap.get(DcMotorEx.class, RobotConstants.HardwareConfiguration.kLabelDrivetrainMotorLeftBack);
        drive_right_front = opMode.hardwareMap.get(DcMotorEx.class, RobotConstants.HardwareConfiguration.kLabelDrivetrainMotorRightFront);
        drive_right_back = opMode.hardwareMap.get(DcMotorEx.class, RobotConstants.HardwareConfiguration.kLabelDrivetrainMotorRightBack);

        // Add Motors to Array for like configuration(s)
        listMotorDrivetrain = Arrays.asList(drive_left_front, drive_left_back, drive_right_front, drive_right_back);

        // Set common configuration for drive motor(s)
        for (DcMotorEx itemMotor : listMotorDrivetrain) {

            // clone motor configuration
            MotorConfigurationType motorConfigurationType = itemMotor.getMotorType().clone();

            // Set motor configuration properties
            motorConfigurationType.setAchieveableMaxRPMFraction(RobotConstants.Drivetrain.Configuration.kMotorAchievableMaxRpmFraction);

            // Write out motor configuration to motor
            itemMotor.setMotorType(motorConfigurationType);
        }

        // Set Zero Power mode for drivetrain
        setDriveMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set Non-common motor configuration(s)
        drive_left_front.setDirection(DcMotorEx.Direction.REVERSE);
        drive_left_back.setDirection(DcMotorEx.Direction.FORWARD);
        drive_right_front.setDirection(DcMotorEx.Direction.FORWARD);
        drive_right_back.setDirection(DcMotorEx.Direction.FORWARD);

        // Set motor mode(s)


        // Define IMU Device
        imu_pinpoint = opMode.hardwareMap.get(GoBildaPinpointDriver.class, RobotConstants.HardwareConfiguration.kLabelDrivetrainIMUDevicePinpoint);

        // Configure IMU Device
        imu_pinpoint.setOffsets(RobotConstants.Drivetrain.Odometry.kOdometryPodOffsetXMillimeters, RobotConstants.Drivetrain.Odometry.kOdometryPodOffsetYMillimeters, DistanceUnit.MM);

        imu_pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        imu_pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // Initialize IMU (reset heading)
        resetZeroRobotHeading();

        // Telemetry - Initialize - End
//        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.addData(">", "System: Drivetrain (Initialized)");
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.update();

    }

    // -----------------------------------------
    // Action Method(s)
    // -----------------------------------------

    /**
     * <h2>Drivetrain Method: driveMecanumFieldCentric</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#kAboutAuthorName}<br>
     * <b>Season Game:</b> {@value RobotConstants.About#kAboutSeasonGameName}<br>
     * <hr>
     * <p><br>
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     * </p>
     * <br>
     * <p>
     * This is a 'Field Centric' variation of the mecanum drivetrain
     * </p>
     * <br>
     * <i>[Y]</i> <b>Axial:</b>   Driving forward and backward<br>
     * <i>[X]</i> <b>Lateral:</b> Strafing right and left<br>
     * <i>[R]</i> <b>Yaw:</b>   Rotating Clockwise and counter clockwise<br>
     *
     * @param inputAxial   [Y] Driving forward and backward
     * @param inputLateral [X] Strafing right and left
     * @param inputYaw     [R] Rotating Clockwise and counter clockwise
     * @param maxOutputPowerPercent Percent of power to apply to motors
     *
     * <br>
     */
    public void driveMecanumFieldCentric(double inputAxial, double inputLateral, double inputYaw, double maxOutputPowerPercent) {

        double modMaintainMotorRatio;

        // Axial = input y
        // Lateral = input x
        // Yaw = rotational

        double modAxial = (inputAxial * maxOutputPowerPercent);  // Note: pushing stick forward gives negative value
        double modLateral = (inputLateral * maxOutputPowerPercent);
        double modYaw = (inputYaw * maxOutputPowerPercent);

        // Get heading value from the IMU
//        updateOdometry();
        double botHeading = getRobotHeadingRaw();

        // Adjust the lateral and axial movements based on heading
        double adjLateral = modLateral * Math.cos(-botHeading) - modAxial * Math.sin(-botHeading);
        double adjAxial = modLateral * Math.sin(-botHeading) + modAxial * Math.cos(-botHeading);

        adjLateral = adjLateral * RobotConstants.Drivetrain.Configuration.kMotorLateralMovementStrafingCorrection; // Mod to even out strafing

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        modMaintainMotorRatio = Math.max(Math.abs(modAxial) + Math.abs(modLateral) + Math.abs(modYaw), RobotConstants.Drivetrain.Configuration.kMotorOutputPowerMax);

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = (adjAxial + adjLateral + modYaw) / modMaintainMotorRatio;
        double leftBackPower   = (adjAxial - adjLateral + modYaw) / modMaintainMotorRatio;
        double rightFrontPower = (adjAxial - adjLateral - modYaw) / modMaintainMotorRatio;
        double rightBackPower  = (adjAxial + adjLateral - modYaw) / modMaintainMotorRatio;

        // Use existing function to drive both wheels.
        setDriveMotorPower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }


    /**
     * <h2>Drivetrain Method: driveMecanum</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#kAboutAuthorName}<br>
     * <b>Season:</b> {@value RobotConstants.About#kAboutSeasonGameName}<br>
     * <hr>
     * <p>
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     * </p>
     * <p>
     * This is a standard mecanum drivetrain or a 'Robot Centric' drivetrain
     * </p>
     * <br>
     * <i>[Y]</i> <b>Axial:</b>   Driving forward and backward<br>
     * <i>[X]</i> <b>Lateral:</b> Strafing right and left<br>
     * <i>[R]</i> <b>Yaw:</b>   Rotating Clockwise and counter clockwise<br>
     *
     * @param inputAxial   [Y] Driving forward and backward
     * @param inputLateral [X] Strafing right and left
     * @param inputYaw     [R] Rotating Clockwise and counter clockwise
     * @param maxOutputPowerPercent Percent of power to apply to motors
     *
     * <br>
     */
    public void driveMecanum(double inputAxial, double inputLateral, double inputYaw, double maxOutputPowerPercent) {

        double modMaintainMotorRatio;

        double modAxial   = (inputAxial * maxOutputPowerPercent);  // Note: pushing stick forward gives negative value
        double modLateral = (inputLateral * maxOutputPowerPercent) * RobotConstants.Drivetrain.Configuration.kMotorLateralMovementStrafingCorrection; // Mod to even out strafing
        double modYaw     = (inputYaw * maxOutputPowerPercent);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        modMaintainMotorRatio = Math.max(Math.abs(modAxial) + Math.abs(modLateral) + Math.abs(modYaw), RobotConstants.Drivetrain.Configuration.kMotorOutputPowerMax);

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = (modAxial + modLateral + modYaw) / modMaintainMotorRatio;
        double leftBackPower   = (modAxial - modLateral + modYaw) / modMaintainMotorRatio;
        double rightFrontPower = (modAxial - modLateral - modYaw) / modMaintainMotorRatio;
        double rightBackPower  = (modAxial + modLateral - modYaw) / modMaintainMotorRatio;

        // Use existing function to drive both wheels.
        setDriveMotorPower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    public void updateOdometry() {
        imu_pinpoint.update();
    }

    public void resetZeroRobotHeading() {
        imu_pinpoint.resetPosAndIMU();
    }


    // -----------------------------------------
    // Get Method(s)
    // -----------------------------------------
    public String getImuStatus() {
        return imu_pinpoint.getDeviceStatus().toString();
    }

    public Pose2D getRobotPose() {
        return imu_pinpoint.getPosition();
    }

    public Pose2D getRobotPosition() {

        return imu_pinpoint.getPosition();
    }

    public double getRobotPositionXInches() {

        return imu_pinpoint.getPosX(DistanceUnit.INCH);
    }

    public double getRobotPositionYInches() {

        return imu_pinpoint.getPosY(DistanceUnit.INCH);
    }

    public double getRobotVelocityXInches() {
        return imu_pinpoint.getVelX(DistanceUnit.INCH);
    }

    public double getRobotVelocityYInches() {
        return imu_pinpoint.getVelY(DistanceUnit.INCH);
    }

    public double getRobotVelocityHeadingDegrees() {
        return imu_pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
    }

    public String getTelemetryImuPositionDetail() {
        return String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", getRobotPosition().getX(DistanceUnit.INCH), getRobotPosition().getY(DistanceUnit.INCH), getRobotPosition().getHeading(AngleUnit.DEGREES));
    }

    public String getTelemetryImuPositionAltDetail() {
        return String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", getRobotPositionXInches(), getRobotPositionYInches(), getRobotPosition().getHeading(AngleUnit.DEGREES));
    }

    public String getTelemetryImuVelocityDetail() {
        return String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", getRobotVelocityXInches(), getRobotVelocityYInches(), getRobotVelocityHeadingDegrees());
    }

    /**
     * <h2>Drivetrain Method: getRobotHeadingRaw</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#kAboutAuthorName}<br>
     * <b>Season Game:</b> {@value RobotConstants.About#kAboutSeasonGameName}<br>
     * <hr>
     * <p>
     * Get the output heading value from the IMU.
     * </p>
     * @return double - Output heading value from the IMU - raw reading
     */
    public double getRobotHeadingRaw() {

//        return imu_pinpoint.getHeading();
        return imu_pinpoint.getHeading(AngleUnit.RADIANS);
    }

    /**
     * <h2>Drivetrain Method: getRobotHeadingAdj</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#kAboutAuthorName}<br>
     * <b>Season Game:</b> {@value RobotConstants.About#kAboutSeasonGameName}<br>
     * <hr>
     * <p>
     * Get the output heading value from the IMU (with offset adjustment).
     * </p>
     * @return double - Output heading value from the IMU (with offset adjustment)
     * <br>
     */
    public double getRobotHeadingAdj() {

        // Variable for output heading value
        double outRobotHeadingValue;

        // Get heading value from the IMU
        // Read inverse IMU heading, as the IMU heading is CW positive
        outRobotHeadingValue = getRobotHeadingRaw() * -1; //+ RobotConstants.CommonSettings.getImuTransitionAdjustment();

        // Should the IMU heading be inversed? Does it matter?
        // Will need to view the heading readout on the driver hub

        //imuUnit.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle

        return outRobotHeadingValue;
    }

    public DrivetrainPinpoint.DrivetrainMode getDrivetrainMode() {
        return drivetrainMode;
    }

    public DrivetrainPinpoint.DrivetrainSpeed getDrivetrainOutputPower() {
        return drivetrainSpeed;
    }


    // -----------------------------------------
    // Set Method(s)
    // -----------------------------------------
    /**
     * <h2>Drivetrain Method: setDriveMotorPower</h2>
     * <hr>
     * <b>Author:</b> {@value RobotConstants.About#kAboutAuthorName}<br>
     * <b>Season Game:</b> {@value RobotConstants.About#kAboutSeasonGameName}<br>
     * <hr>
     * <p>
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     * </p>
     * @param powerLeftFront  Power to Left Front Wheel
     * @param powerRightFront Power to Right Front Wheel
     * @param powerLeftBack   Power to Left Back Wheel
     * @param powerRightBack  Power to Right Back Wheel
     *
     * <br>
     */
    private void setDriveMotorPower(double powerLeftFront, double powerLeftBack, double powerRightFront, double powerRightBack) {

        // Send calculated power to wheels
        drive_left_front.setPower(powerLeftFront);
        drive_left_back.setPower(powerLeftBack);
        drive_right_front.setPower(powerRightFront);
        drive_right_back.setPower(powerRightBack);
    }

    public void setRobotPose(Pose2D robotPose) {
        imu_pinpoint.setPosition(robotPose);
    }

    public void setRobotPositionXInches(double robotPositionX) {
        imu_pinpoint.setPosX(robotPositionX, DistanceUnit.INCH);
    }

    public void setRobotPositionYInches(double robotPositionY) {
        imu_pinpoint.setPosY(robotPositionY, DistanceUnit.INCH);
    }

    public void setRobotPositionHeadingDegrees(double robotPositionHeading) {
        imu_pinpoint.setHeading(robotPositionHeading, AngleUnit.DEGREES);
    }

    /**
     * <h2>Drivetrain Method: setDriveMotorZeroPowerBehavior</h2>
     * <hr>
     * <b>Author: </b><br>
     * <b>Season: </b><br>
     * <hr>
     * <p><br>
     * Set the behavior of each drivetrain motor when zero power is supplied to the motor(s).<br>
     * - Mode(s): Brake / Coast<br>
     * </p>
     *
     * @param zeroPowerMode DcMotorEx.ZeroPowerBehavior - Behavior when zero power sent to motor
     *
     */
    public void setDriveMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerMode) {

        for (DcMotorEx itemMotor : listMotorDrivetrain) {
            itemMotor.setZeroPowerBehavior(zeroPowerMode);
        }
    }

    public void setDrivetrainMode(DrivetrainPinpoint.DrivetrainMode newDrivetrainMode) {
        drivetrainMode = newDrivetrainMode;
    }

    public void setDrivetrainOutputPower(DrivetrainPinpoint.DrivetrainSpeed newDrivetrainSpeed) {
        drivetrainSpeed = newDrivetrainSpeed;
    }


}
