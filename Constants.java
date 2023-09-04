package org.firstinspires.ftc.teamcode.freightFrenzy;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    /* ------------------------------------ AUTONOMOUS ------------------------------------ */
    // Drive motor
    public static final double TICKS_PER_REV = 537.7;
    public static final double CIRCUMFERENCE_IN_INCHES = 96 / 25.4 * Math.PI;
    public static final double TICKS_PER_INCH = TICKS_PER_REV / CIRCUMFERENCE_IN_INCHES;

    // SOMETHING IMPORTANT
    public static double sideOffset = 0;
    public static double dropSide = 39;

    // Autonomous turn PID
    public static double kR = 0.084; // PID turn kR
    public static double kD = 0.0072; // PID turn kD


    // Distance sensor drive
    public static double tkR = 0.036; // Distance drive turn correction kR

    public static double setVerticalMinSpeed = 0.18;
    public static double setVerticalMaxSpeed = 1;
    public static double setHorizontalMinSpeed = 0.3;
    public static double setHorizontalMaxSpeed = 1;
    public static double setDiagonalMinSpeed = 0.3;
    public static double setDiagonalMaxSpeed = 1;
    public static double setReadjustMinSpeed = 0.12;
    public static double setReadjustMaxSpeed = 0.;


    public static double setHorizontalDisCap = 24;
    public static double setVerticalDisCap = 36;
    public static double setReadjustDisCap = 24;
    public static double vertHorRatio = 1.22348112;


    public static double minConfidence = 0.6;
    public static double magnification = 1.0; // Original: 2.5
    public static double aspectRatio = 1.0; // Original: 16.0/9.0


    public static double testTargetDistance = 6;
//    public static AutonomousMethods.Direction a = AutonomousMethods.Direction.FORWARD;

    public static double testVertTargetDistance = 8; // 6;
    public static double testHorTargetDistance = 5; // 30;

    public static double duckDropDis = 31.2;


    /* ------------------------------------ TELE-OP ------------------------------------ */
    // Arm rotation (counterclockwise)(751.8 PPR supposedly)
    public static double setRotateMultiplier = 0.36;
    public static double armDriveLimitRatio = -0.36 * 3600; // divide by the current slide position
    public static double forwardLimitRatio = 0.84;

    public static int armL45 = -240; // Left 45
    public static int armR45 = 240; // Right 45
    public static int armL90 = -703; // Left 90
    public static int armR90 = 703; // Right 90
    public static int armL180 = -1406; // Left 180
    public static int armR180 = 1406; // Right 180
    public static int armRLimit = 1406+154;


    // Motor ratios
//    public static double armRatio = 1;
    public static double liftRatio = 1;
    public static double liftDownRatio = 0.72;
    public static double extRatio = -0.0053;


    // Random motor and servo powers
    public static double collectPower = -1;
    public static double ejectPower = 0.9;
    public static double weakEjectPower = 0.36;

    public static double autoEjectPower = 0.72;

    public static int liftOff = 108;
    public static int liftTop = -3306;

    public static final int automationDelay = 96;
    public static final int buttonDelay = 18;


    // Random servo positions
    public static double dropClosePosition = 0.62;
    public static double dropFrontPosition = 0.58;
    public static double dropOpenPosition =  0.76;


    public static double extIn = 0.2;
    public static double extSide = 0.53;
    public static double extOut = 0.72;
    public static double extTop = 0.45;
    public static double extMed = 0.42;
    public static double extBot = 0.36;

    public static double extTopEarly = 240; // when to extend the extender relative to arm encoder
    public static double extMedEarly = 180; // when to extend the extender relative to arm encoder
    public static double extBotEarly = 180; // when to extend the extender relative to arm encoder


    /* ------------------------------------ BOTH ------------------------------------ */
    // Arm raise (5,281.1 PPR supposedly)
    // -2250 is perfect top, -1825 and -2675 it stops dropping
    public static int armAdjustment = 0;

    public static int armCollect = 0; // Almost touching ground to collect
    public static int armDrive = -120 + armAdjustment; // Barely off ground, for driving
    public static int armBot = -720 + armAdjustment; // Reaches bottom level
    //    public static int armOpp = -3630; // Barely off the ground on the OPPosite side
    public static int armMed = -1860 + armAdjustment; // Reaches middle level
    public static int armNeutral = -1620;
    public static int armStop = -2160;
    public static int armStep = -1560;
    public static int armTop = -3279 + armAdjustment; // -2880 before // Reaches top level
    public static int armTopAuto = -2880 + armAdjustment; // -2880 before // Reaches top level
    public static double armMove = 1; // Power for moving arm

    public static int armSpin = -972 + armAdjustment; // High enough for arm rotation
    public static int armSpinOpt = -1560 + armAdjustment; // High enough for arm rotation optimization
    public static double extSpin = 0.53;

//    public static int armMinSpin = armSpin - 0 - 12;


    public static double liftkP = 4.8;
    public static double liftMin = 0.18;
    public static double liftMax = 3600;

    public static double rotationkP = 4.8;
    public static double rotationMin = 0.12;
    public static double rotationMax = 2900;

    public static double duckSpinPower = 1;
    public static double duckAutoSpinPower = 0.6;

    public static boolean driverSide = true;

    public static double testSpeed = 0.9;
    public static int testPosition = -1200;
}
