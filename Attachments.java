package org.firstinspires.ftc.teamcode.freightFrenzy;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.baseBot.Drivetrain;

public class Attachments extends Drivetrain {
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    public Configuration names = new Configuration();
    public DcMotor collectionMotor, rotateMotor, liftMotor;
    public Servo /* cameraServo ,*/ dropServo, extServo;
    public CRServo leftServo, rightServo;
    public Rev2mDistanceSensor frontDistance, backDistance, leftDistance, rightDistance,
            frontLeftDistance, backLeftDistance, leftFrontDistance, rightFrontDistance;

    //Backend
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry_) {
        telemetry = telemetry_;
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Motors
        collectionMotor = hardwareMap.get(DcMotor.class, names.collectionMotor);
        rotateMotor = hardwareMap.get(DcMotor.class, names.rotateMotor);
        liftMotor = hardwareMap.get(DcMotor.class, names.liftMotor);

        // Servos
        dropServo = hardwareMap.get(Servo.class, names.dropServo);
        extServo = hardwareMap.get(Servo.class, names.extensionServo);

        // Continuous Servos
        leftServo = hardwareMap.get(CRServo.class, names.leftServo);
        rightServo = hardwareMap.get(CRServo.class, names.rightServo);

        //Sensors
        frontDistance = hardwareMap.get(Rev2mDistanceSensor.class, names.frontDistance);
        backDistance = hardwareMap.get(Rev2mDistanceSensor.class, names.backDistance);
        backLeftDistance = hardwareMap.get(Rev2mDistanceSensor.class, names.backLeftDistance);
        frontLeftDistance = hardwareMap.get(Rev2mDistanceSensor.class, names.frontLeftDistance);
        leftDistance = hardwareMap.get(Rev2mDistanceSensor.class, names.leftDistance);
        leftFrontDistance = hardwareMap.get(Rev2mDistanceSensor.class, names.leftFrontDistance);
        rightDistance = hardwareMap.get(Rev2mDistanceSensor.class, names.rightDistance);
        rightFrontDistance = hardwareMap.get(Rev2mDistanceSensor.class, names.rightFrontDistance);

        // Set motor and servo modes and reverse directions
//        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        collectionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //HardwareMaps
        initializeDriveTrain(hardwareMap, telemetry_);
    }

    /* ------------------------------------ ACCESSORS ------------------------------------ */
    public double getFrontDistance() {
        return frontDistance.getDistance(DistanceUnit.INCH);
    }

    public double getLeftDistance() {
        return leftDistance.getDistance(DistanceUnit.INCH);
    }

    public double getLeftFrontDistance() {
        return leftFrontDistance.getDistance(DistanceUnit.INCH);
    }

    public double getRightDistance() {
        return rightDistance.getDistance(DistanceUnit.INCH);
    }

    public double getRightFrontDistance() {
        return rightFrontDistance.getDistance(DistanceUnit.INCH);
    }

    public double getBackDistance() {
        return backDistance.getDistance(DistanceUnit.INCH);
    }

    public double getBackLeftDistance() {
        return backLeftDistance.getDistance(DistanceUnit.INCH);
    }

    public double getFrontLeftDistance() {
        return frontLeftDistance.getDistance(DistanceUnit.INCH);
    }

    public double getExtenderPosition() {
        return extServo.getPosition();
    }

    public double getLiftMotorPosition() {
        return liftMotor.getCurrentPosition();
    }

    public double getRotationMotorPosition() {
        return rotateMotor.getCurrentPosition();
    }

    /* ------------------------------------ SETTERS ------------------------------------ */
    // Motors
//    public void runArmMotor(double power) {
//        armMotor.setPower(power);
//    }
//
//    public void setArmMotor(double power, int position) {
//        armMotor.setPower(power);
//        armMotor.setTargetPosition(position);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }

    public void runLiftMotor(double power) {
        liftMotor.setPower(power);
    }

    public void setLiftMotor(double power, int position) {
        liftMotor.setPower(power);
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runRotateMotor(double power) {
        rotateMotor.setPower(power);
    }

    public void runCollectionMotor(double power) {
        collectionMotor.setPower(power);
    }

    // Servos
    public void setDropServo(double position) {
        dropServo.setPosition(position);
    }

    public void setExtServo(double position) {
        extServo.setPosition(position);
    }


    // Continuous Servos
    public void runLeftServo(double power) {
        leftServo.setPower(power);
    }

    public void runRightServo(double power) {
        rightServo.setPower(power);
    }
}