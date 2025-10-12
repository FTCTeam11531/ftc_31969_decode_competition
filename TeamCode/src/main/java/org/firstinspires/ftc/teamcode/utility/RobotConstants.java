package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.Pose2d;

public class RobotConstants {

    public static final class About {

        // Documentation Comments Reference(s)
        public static final String kAboutAuthorName = "Techno Trojans - Competition Robot";

        public static final String kAboutSeasonGameName = "Decode";
        public static final String kAboutSeasonPeriod = "2025 / 2026";
    }


    public static final class UnitConversion {

        // Conversion: use conversion factor below to calculate conversion from one
        // - Millimeters to Inches
        // - - Divide Millimeters by ConversionFactor to return Inches
        // - Inches to Millimeters
        // - - Multiple Inches by ConversionFactor to return Millimeters
        public static final double kConversionFactorMillimeterInches = 25.4;

        /**
         *
         * @param valueMillimeter
         *
         * @return
         *
         */
        public static double convertMillimeterToInch(double valueMillimeter) {

            // Return Inch value (converted from Millimeter)
            return (valueMillimeter / kConversionFactorMillimeterInches);
        }

        /**
         *
         * @param valueInch
         *
         * @return
         *
         */
        public static double convertInchToMillimeter(double valueInch) {

            // Return Millimeter value (converted from Inches)
            return (valueInch * kConversionFactorMillimeterInches);
        }

        public static double addTwoDegreeValuesTogether(double degreeOne, double degreeTwo) {

            double degreeNew = degreeOne + degreeTwo;

            if(degreeNew >= 360) {
                degreeNew -= 360;
            }
            else if(degreeNew < 0) {
                degreeNew += 360;
            }

            return degreeNew;
        }

    }

    public static final class UnitRangeConversion {

        /**
         * <h3>Unit Range Conversion: Scale value within a range to another range</h3>
         * <hr>
         * Reference: <a href="https://stats.stackexchange.com/questions/281162/scale-a-number-between-a-range">Source for formula used in this unit conversion method</a>
         * <hr>
         * <p>
         * Scaling will need to take into account the possible range of the original number.<br>
         * There is a difference if your 200 could have been in the range [200,201] or in [0,200] or in [0,10000].
         * </p><br>
         * <b>The Formula is defined as such:</b>
         * <ul>
         * <li><b>rmin</b><br>denote the minimum of the range of your measurement</li>
         * <li><b>rmax</b><br>denote the maximum of the range of your measurement</li>
         * <li><b>tmin</b><br>denote the minimum of the range of your desired target scaling</li>
         * <li><b>tmax</b><br>denote the maximum of the range of your desired target scaling</li>
         * <li><b>m ∈ [rmin,rmax]</b><br>denote your measurement to be scaled</li>
         * </ul><br>
         * <b>Full Formula</b><br>
         * <pre>
         *       m − rmin
         * m ↦  -----------  × (tmax − tmin) + tmin
         *      rmax − rmin
         *
         * This formula will scale m linearly into [tmin,tmax] as desired.
         * </pre>
         *
         * <b>step by step:</b>
         * <ol>
         * <li>map m to [0,rmax − rmin]<pre>m ↦ m − rmin</pre></li>
         * <li>map m to the interval [0,1], with m = rmin mapped to 0 and m = rmax mapped to 1
         * <pre>
         * m ↦  m − rmin
         *     -----------
         *     rmax − rmin</pre></li>
         * <li><p>Multiplying this by (tmax − tmin) maps m to [0,tmax − tmin]</p></li>
         * <li><p>Finally, adding tmin shifts everything and maps m to [tmin,tmax] as desired.</p></li>
         * </ol>
         *
         * @param sourceValue value within source range
         * @param sourceMax source value range max
         * @param sourceMin source value range min
         * @param targetMax target value range max
         * @param targetMin target value range min
         *
         * @return double - return the scaled value within the new target range
         */
        public static double scaleUnitRangeToAlternateRange(double sourceValue, double sourceMax, double sourceMin, double targetMax, double targetMin) {

            return ((sourceValue - sourceMin) / (sourceMax - sourceMin)) * (targetMax - targetMin) + targetMin;
        }
    }

    public static final class OpModeTransition {

        private static Pose2d poseFinalOpMode = new Pose2d(0,0, Math.toRadians(0));

        public static Pose2d getPoseFinalOpMode() {

            return poseFinalOpMode;
        }

        public static void setPoseFinalOpMode(Pose2d poseTransition) {

            poseFinalOpMode = poseTransition;
        }

    }

    public static final class HardwareConfiguration {

        // Hardware Configuration from Control Hub and/or Expansion Hub
        // ---------------------------------------------------------------


        // ---------------------------------------------------------------
        // Motor(s)
        // ---------------------------------------------------------------

        // Drivetrain - Motor(s)
        public static final String kLabelDrivetrainMotorLeftFront = "drive_front_left";
        public static final String kLabelDrivetrainMotorRightFront = "drive_front_right";
        public static final String kLabelDrivetrainMotorLeftBack = "drive_back_left";
        public static final String kLabelDrivetrainMotorRightBack = "drive_back_right";

        // Drivetrain - imu
        public static final String kLabelDrivetrainIMUDeviceNavX = "imu_navx";
        public static final String kLabelDrivetrainIMUDevicePinpoint = "imu_pinpoint";
        public static final String kLabelDrivetrainIMUDeviceDefault = "imu_ch";

        // Drivetrain - Odometry


        // Intake


        // Indexer


        // Shooter - Motor(s)
        public static final String kLabelShooterMotorLeft = "shooter_left";
        public static final String kLabelShooterMotorRight = "shooter_right";



        // Camera




    }

    public static final class CommonSettings {

        public static final boolean kRoadRunnerTunerOpModeDisable = true;

        public static final class GameSettings {

            public static final double kEndgameStartTime = 90.0;
            public static final double kEndgameEndTime = 120.0;
        }

    }


    public static final class Drivetrain {

        public static final class Configuration {

            // Motor Output Settings
            public static final double kMotorLateralMovementStrafingCorrection = 1.1;
            public static final double kMotorOutputPowerMax = 1;

            public static final double kMotorAchievableMaxRpmFraction = 1.0;

            // TODO: test and set final speed value(s)
            public static final double kMotorOutputPowerHigh = 1.0;
            public static final double kMotorOutputPowerMedium = .80;
            public static final double kMotorOutputPowerLow = .50;
            public static final double kMotorOutputPowerSnail = .20;
        }

        public static final class Odometry {

            // Configuration Settings
            // TODO: adjust values to match actual robot setting(s)
            public static final double kOdometryPodOffsetXMillimeters = -53.975;
            public static final double kOdometryPodOffsetYMillimeters = -146.05;

            public static final double kDriveInchPerTick = 1;
            public static final double kTrackWidthTick = 12.769966979946723;

            public static final double kFeedForwardTicksVValue = 0.1800743324232725;
            public static final double kFeedForwardTicksSValue = 0.9185997050940498;
            public static final double kFeedForwardTicksAValue = 0.0211111;

            public static final double kMaxWheelVelocity = 50;
            public static final double kMinProfileAcceleration = -30;
            public static final double kMaxProfileAcceleration = 50;

            public static final double kAxialGain = 4.0;
            public static final double kLateralGain = 4.0;
            public static final double kHeadingGain = 2.0;

            public static final double kAxialVelGain = 0.0;
            public static final double kLateralVelGain = 0.0;
            public static final double kHeadingVelGain = 0.0;

            public static final class Transition {
                public static double imuTransitionAdjustment = 0;

                public static double getImuTransitionAdjustment() {
                    return imuTransitionAdjustment;
                }

                public static void setImuTransitionAdjustment(double heading) {
                    imuTransitionAdjustment = heading;
                }
            }

        }

        public static final class Mecanum {

        }

        public static final class Autonomous {

            public static final class Pose {



            }

        }

    }

    public static final class Vision {

        public static final class Alliance {

            public static final String kAllianceLabelRed = "red";
            public static final String kAllianceLabelBlue = "blue";
        }

    }


    public static final class Lighting {

        // Lighting Enabled (True/False)
        // -- Only enabled if a lighting controller and lighting is installed on the robot
        public static final boolean kEnableLighting = true;

        public static final class Pattern {

        }


    }



}
