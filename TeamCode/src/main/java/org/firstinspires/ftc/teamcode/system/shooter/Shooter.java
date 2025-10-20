package org.firstinspires.ftc.teamcode.system.shooter;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.utility.RobotConstants;

import java.util.Arrays;
import java.util.List;

public class Shooter {

    public enum ShootingMode {

        Neutral {

            @Override
            public double getValue() { return 0; }

            @Override
            public String getLabel() { return "Neutral"; }
        },

        Long_Range_Shot {

            @Override
            public double getValue() { return 80; }

            @Override
            public String getLabel() { return "Long Range"; }
        },

        Mid_Range_Shot {

            @Override
            public double getValue() { return 50; }

            @Override
            public String getLabel() { return "Mid Range"; }
        },

        Close_Range_Shot {

            @Override
            public double getValue() { return 30; }

            @Override
            public String getLabel() { return "Close Range"; }
        },

        Variable_Shot {

            @Override
            public double getValue() { return 10; }

            @Override
            public String getLabel() { return "Variable"; }
        };

        public abstract  double getValue();
        public abstract  String getLabel();

    }

    // System OpMode
    private LinearOpMode opMode;

    // Shooter Settings
    private ShootingMode shootingModeLeft = ShootingMode.Neutral;
    private ShootingMode shootingModeRight = ShootingMode.Neutral;

    // Define Hardware for subsystem
    private DcMotorEx shooterLeft, shooterRight;
    private List<DcMotorEx> listMotorShooter;


    // Constructor
    public Shooter(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    // Initialize
    public void init() {

        // Telemetry - Initialize - Start
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.addData(">", "System: Shooter");
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.update();

        // Define and Initialize Motor(s)
        shooterLeft = opMode.hardwareMap.get(DcMotorEx.class, RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft);
        shooterRight = opMode.hardwareMap.get(DcMotorEx.class, RobotConstants.HardwareConfiguration.kLabelShooterMotorRight);

        listMotorShooter = Arrays.asList(shooterLeft, shooterRight);

        // Set common configuration for drive motor(s)
        for (DcMotorEx itemMotor : listMotorShooter) {

            // clone motor configuration
            MotorConfigurationType motorConfigurationType = itemMotor.getMotorType().clone();

            // Set motor configuration properties
            motorConfigurationType.setAchieveableMaxRPMFraction(RobotConstants.Shooter.Configuration.kMotorAchievableMaxRpmFraction);

            // Write out motor configuration to motor
            itemMotor.setMotorType(motorConfigurationType);
        }

        // Set Zero Power mode for drivetrain
        setMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Set Non-common motor configuration(s)
        shooterLeft.setDirection(DcMotorEx.Direction.REVERSE);
        shooterRight.setDirection(DcMotorEx.Direction.FORWARD);

        // Telemetry - Initialize - End
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.addData(">", "System: Shooter (Initialized)");
        opMode.telemetry.addData(">", "------------------------------------");
        opMode.telemetry.update();

    }

    // ----------------------------------------------
    // Action Methods
    // ----------------------------------------------

    public void activateShooter(String hardwareLabel, double setpoint) {

        switch (hardwareLabel) {
            case RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft:
                shooterLeft.setPower(setpoint);
                break;

            case RobotConstants.HardwareConfiguration.kLabelShooterMotorRight:
                shooterRight.setPower(setpoint);
                break;

        }

    }

    public void deactivateShooter(String hardwareLabel) {
        double setpoint = 0;

        activateShooter(hardwareLabel, setpoint);
    }


    public boolean checkShooterVelocityLevel(String hardwareLabel, double setpoint) {

        boolean atSetpoint = false;

        switch (hardwareLabel) {
            case RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft:
                if(shooterLeft.getVelocity() >= setpoint) {
                    atSetpoint = true;
                }
                break;

            case RobotConstants.HardwareConfiguration.kLabelShooterMotorRight:
                if(shooterRight.getVelocity() >= setpoint) {
                    atSetpoint = true;
                }
                break;

        }

        return atSetpoint;
    }




    // ----------------------------------------------
    // Action Methods - Road Runner
    // ----------------------------------------------

    // Road Runner - Action - Activate Indexer
    public class ActionActivateShooter implements Action {
        private String hardwareLabel;
        private double setpoint;

        // Action class constructor
        public ActionActivateShooter(String hardwareLabel, double setpoint) {
            this.hardwareLabel = hardwareLabel;
            this.setpoint = setpoint;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            activateShooter(hardwareLabel, setpoint);
            return false;
        }
    }

    public Action actionActivateShooter(String hardwareLabel, double setpoint) {
        return new ActionActivateShooter(hardwareLabel, setpoint);
    }


    // ----------------------------------------------
    // Get Methods
    // ----------------------------------------------

    public double getMotorPower(String motorLabel) {
        double outputPower;

        switch (motorLabel) {
            case RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft:
                outputPower = shooterLeft.getPower();
                break;

            case RobotConstants.HardwareConfiguration.kLabelShooterMotorRight:
                outputPower = shooterRight.getPower();
                break;

            default:
                outputPower = 0;
        }

        return outputPower;
    }

    public double getMotorVelocity(String motorLabel) {
        double outputVelocity;

        switch (motorLabel) {
            case RobotConstants.HardwareConfiguration.kLabelShooterMotorLeft:
                outputVelocity = shooterLeft.getVelocity();
                break;

            case RobotConstants.HardwareConfiguration.kLabelShooterMotorRight:
                outputVelocity = shooterRight.getVelocity();
                break;

            default:
                outputVelocity = 0;
        }

        return outputVelocity;
    }

    public ShootingMode getShootingModeLeft() {
        return shootingModeLeft;
    }

    public ShootingMode getShootingModeRight() {
        return shootingModeRight;
    }


    // ----------------------------------------------
    // Set Methods
    // ----------------------------------------------

    public void setMotorRunMode(DcMotorEx.RunMode inRunMode) {
        for (DcMotorEx itemMotor: listMotorShooter) {
            itemMotor.setMode(inRunMode);
        }
    }
    public void setMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior inZeroPowerBehavior) {
        for (DcMotorEx itemMotor : listMotorShooter) {
            itemMotor.setZeroPowerBehavior(inZeroPowerBehavior);
        }
    }


    public void setShootingModeLeft(ShootingMode newShootingMode) {
        shootingModeLeft = newShootingMode;
    }


}
