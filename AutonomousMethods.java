package org.firstinspires.ftc.teamcode.freightFrenzy;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class AutonomousMethods extends LinearOpMode {
    public Attachments myRobot = new Attachments();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    private Orientation angles;
    private ElapsedTime runtime = new ElapsedTime();

    public enum Direction {
        FORWARD,
        BACKWARD,
        RIGHT,
        LEFT
    }

    public boolean opModeStatus() {
        return opModeIsActive();
    }


    // Initializations
    public void initializeAutonomousDrivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        myRobot.initializeDriveTrain(hardwareMap, telemetry);
    }

    public void initializeAutonomousAttachments(HardwareMap hardwareMap, Telemetry telemetry) {
        myRobot.initialize(hardwareMap, telemetry);
    }

    public double autonomousGetAngle() {
        return myRobot.getAngle();
    }

    // TODO: add collection to end of park
    public void park(boolean isRed, boolean parkOnly, boolean neutral) {
        int turnAngle;
        if (isRed) {
            turnAngle = -90;
            if (!parkOnly) {
                turnAngle -= 90;
            }
            encoderTurn(turnAngle + 90, 0.36, 0.12);
            toTargetDistance(0.1, Direction.BACKWARD, 0.1, 3600, 18);
//            toTargetDistance(23, Direction.RIGHT, 1, 7200, 48);
            encoderStrafeDriveInchesRight(36, 0.48);
            if (neutral) {
                encoderTurn(turnAngle + 90, 0.36, 0.12);
                sleep(240);
                toTargetDistance(27, Direction.BACKWARD, 0.6, 1800, 30);
                toTargetDistance(3, Direction.RIGHT, 0.3, 1800, 18);
                encoderTurn(turnAngle - 180, 0.36, 0.12);
                encoderTurn(turnAngle - 90, 0.36, 0.12);
            } else {
                encoderTurn(turnAngle, 0.36, 0.12);
            }
        } else {
            turnAngle = 90;
            if (!parkOnly) {
                turnAngle += 90;
            }
            encoderTurn(turnAngle - 90, 0.36, 0.12);
            toTargetDistance(0.1, Direction.BACKWARD, 0.1, 4800, 18);
//            toTargetDistance(23, Direction.LEFT, 1, 7200, 48);
            encoderStrafeDriveInchesRight(-36, 0.48);
            if (neutral) {
                encoderTurn(turnAngle - 90, 0.36, 0.12);
                sleep(240);
                toTargetDistance(27, Direction.BACKWARD, 0.6, 1800, 30);
                toTargetDistance(3, Direction.LEFT, 0.3, 1800, 18);
                encoderTurn(turnAngle + 180, 0.36, 0.12);
                encoderTurn(turnAngle + 90, 0.36, 0.12);
            } else {
                encoderTurn(turnAngle, 0.36, 0.12);
            }
        }
        teleopResetArm();
    }

    public void parkDelivery(boolean isRed) {
        toTargetDistance(6, Direction.BACKWARD, 0.3, 4800, 30);
//        resetArm();
        sleep(120);
        if (isRed) {
            encoderTurn(-90, 0.3, 0.12);
            sleep(240);
            toTarget(30, 1, Direction.FORWARD, Direction.RIGHT, 36, 9, 0.3, 3600);
        } else {
            encoderTurn(90, 0.3, 0.12);
            sleep(240);
            toTarget(30, 1, Direction.FORWARD, Direction.LEFT, 36, 9, 0.3, 3600);
        }
        teleopResetArm();
    }


    public void drop(double power) {
        myRobot.setDropServo(Constants.dropFrontPosition);
        sleep(360);
        myRobot.runCollectionMotor(Constants.autoEjectPower);
        sleep(1800);
        myRobot.setDropServo(Constants.dropClosePosition);
        myRobot.runCollectionMotor(0);
    }

    public void drop() {
        drop(1);
    }

    public void stopPowers() {
        myRobot.rotateMotor.setPower(0);
        myRobot.liftMotor.setPower(0);
    }

    public void resetDropArm() {
        setExtPosition(Constants.extIn);
        sleep(240);
        setLiftMotor(0, 0.9, false);
        sleep(1560);
    }

    public void resetArmGap() {
//        myRobot.setDropServo(Constants.dropClosePosition);
//        setRotationPosition(0.36, 0);

//        setArmMotorPosition(Constants.armMove, Constants.armSpin - 36);
//        sleep(240);
        setLiftMotor(Constants.armBot, 9, true);
        setExtPosition(Constants.extIn);
    }

    public void teleopResetArm() {
//        setLiftMotorPosition(1, 0);
        setRotationPosition(0.36, 0);
        sleep(120);
        setExtPosition(Constants.extIn);
        setLiftMotor(0, 9, true);
        setLiftMotor(0, 0.6, false);
        sleep(720);
    }

    // PID set limit position todo: uncomment this and the one below
    public void setLiftMotor(int position, double tolerance, boolean usePID) {
        if (usePID) {
//            myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //Undefined constants
            double newPower;
            double currentLiftPosition = myRobot.getLiftMotorPosition();
            double error = (position - currentLiftPosition) / Constants.liftMax;
            //Initial Time
            myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (Math.abs(error) > (tolerance / Constants.liftMax)) {
                //Updating values
                currentLiftPosition = myRobot.getLiftMotorPosition();
                error = (position - currentLiftPosition) / Constants.liftMax;
                telemetry.addData("1", "position: " + currentLiftPosition);
                telemetry.addData("2", "error: " + error * Constants.liftMax);
                //Setting p action
                newPower = Math.max(Math.min(error * Constants.liftkP, 1), -1);
//                Log.d("AHHHHHH liftMotor", "PID newPower: " + newPower);

                //Set real power
                newPower = Math.max(Math.abs(newPower), Constants.liftMin) * Math.signum(newPower);
                if (Math.signum(newPower) == 1) {
                    newPower = newPower * Constants.liftDownRatio;
                }
                telemetry.addData("liftMotor PID newPower", newPower);
                myRobot.runLiftMotor(newPower);

                //Logging
//                Log.d("AHHHHHH liftMotor", "error: " + (error * Constants.liftMax) + ", power: " + newPower + ", current position: " + currentPosition);
                telemetry.update();
            }
            myRobot.runLiftMotor(0);
        } else {
            myRobot.setLiftMotor(tolerance, position);
        }
    }

    public void liftAndExtend(int liftTarget, double extTarget, double extEarly, double tolerance) {
//        myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Undefined constants
        double newPower;
        boolean extended = false;
        double currentLiftPosition = myRobot.getLiftMotorPosition();
        double error = (liftTarget - currentLiftPosition) / Constants.liftMax;
        //Initial Time
        myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(error) > (tolerance / Constants.liftMax)) {
            //Updating values
            currentLiftPosition = myRobot.getLiftMotorPosition();
            error = (liftTarget - currentLiftPosition) / Constants.liftMax;
            telemetry.addData("1", "position: " + currentLiftPosition);
            telemetry.addData("2", "error: " + error * Constants.liftMax);

            //Setting p action
            if ((currentLiftPosition < liftTarget + extEarly) && !extended) {
                myRobot.setExtServo(extTarget);
                extended = true;
            }
            newPower = Math.max(Math.min(error * Constants.liftkP, 1), -1);
//                Log.d("AHHHHHH liftMotor", "PID newPower: " + newPower);

            //Set real power
            newPower = Math.max(Math.abs(newPower), Constants.liftMin) * Math.signum(newPower);
            if (Math.signum(newPower) == 1) {
                newPower = newPower * Constants.liftDownRatio;
            }
            telemetry.addData("liftMotor PID newPower", newPower);
            myRobot.runLiftMotor(newPower);

            //Logging
//                Log.d("AHHHHHH liftMotor", "error: " + (error * Constants.liftMax) + ", power: " + newPower + ", current position: " + currentPosition);
            telemetry.update();
        }
        myRobot.runLiftMotor(0);
    }


    public void setExtPosition(double position) {
        myRobot.setExtServo(position);
    }

    public void setRotationPosition(double speed, int position) {
        myRobot.rotateMotor.setPower(speed);
        myRobot.rotateMotor.setTargetPosition(position);
        myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int duckDetection(boolean isRed) {
        if (!isRed) {
            double rf = myRobot.getRightFrontDistance();
            double r = myRobot.getRightDistance();
            if (r < 12) {
                return Constants.armTopAuto;
            } else if (rf < 12) {
                return Constants.armMed;
            } else {
                return Constants.armBot;
            }
        } else {
            double lf = myRobot.getLeftFrontDistance();
            double l = myRobot.getLeftDistance();
            if (lf < 12) {
                return Constants.armMed;
            } else if (l < 12) {
                return Constants.armBot;
            } else {
                return Constants.armTopAuto;
            }
        }
    }

    public int distanceDetection(boolean isRed) {
        if (isRed) {
            double rf = myRobot.getRightFrontDistance();
            double r = myRobot.getRightDistance();
            if (rf < 12) {
                return Constants.armBot;
            } else if (r < 12) {
                return Constants.armMed;
            } else {
                return Constants.armTopAuto;
            }
        } else {
            double lf = myRobot.getLeftFrontDistance();
            double l = myRobot.getLeftDistance();
            if (lf < 12) {
                return Constants.armTopAuto;
            } else if (l < 12) {
                return Constants.armMed;
            } else {
                return Constants.armBot;
            }
        }
    }

    // Drive stuff

    public void setModeAllDrive(DcMotor.RunMode mode) {
        myRobot.lb.setMode(mode);
        myRobot.lf.setMode(mode);
        myRobot.rb.setMode(mode);
        myRobot.rf.setMode(mode);
    }

    public void runMotors(WheelPowers wps) {
        myRobot.lb.setPower(wps.lbPower);
        myRobot.lf.setPower(wps.lfPower);
        myRobot.rb.setPower(wps.rbPower);
        myRobot.rf.setPower(wps.rfPower);
    }

    public void runMotors(double leftPower, double rightPower) {
        myRobot.lb.setPower(leftPower);
        myRobot.lf.setPower(leftPower);
        myRobot.rb.setPower(rightPower);
        myRobot.rf.setPower(rightPower);
    }

    private void multiSetTargetPosition(double ticks, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setTargetPosition((int) Math.round(ticks));
        }
    }

    private boolean notCloseEnough(int tolerance, DcMotor... motors) {
        for (DcMotor motor : motors) {
            if (Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) > tolerance) {
                return true;
            }
        }
        return false;
    }

    public void encoderStraightDrive(double inches, double power) {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Log.d("test test", "test2 " + (inches * Constants.TICKS_PER_INCH));
        ElapsedTime time = new ElapsedTime();
        multiSetTargetPosition(inches * Constants.TICKS_PER_INCH, myRobot.lb, myRobot.lf, myRobot.rb, myRobot.rf);
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        runMotors(power, power);
//        Log.d("test test", "test");
        while (notCloseEnough(20, myRobot.lf, myRobot.rf, myRobot.lb, myRobot.rb) && /*time.milliseconds()<4000 &&*/ opModeIsActive()) {
            Log.d("test test", "test");
            Log.d("Left Front: ", myRobot.lf.getCurrentPosition() + "beep");
            Log.d("Left Back: ", myRobot.lb.getCurrentPosition() + "beep");
            Log.d("Right Front: ", myRobot.rf.getCurrentPosition() + "beep");
            Log.d("Right Back: ", myRobot.rb.getCurrentPosition() + "beep");
        }
//        Log.d("test test", "test3");
        runMotors(0, 0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Negative = Left, Positive = Right
    public void encoderStrafeDriveInchesRight(double inches, double power) {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.lf.setTargetPosition((int) Math.round(inches * Constants.TICKS_PER_INCH));
        myRobot.lb.setTargetPosition(-(int) Math.round(inches * Constants.TICKS_PER_INCH));
        myRobot.rf.setTargetPosition(-(int) Math.round(inches * Constants.TICKS_PER_INCH));
        myRobot.rb.setTargetPosition((int) Math.round(inches * Constants.TICKS_PER_INCH));
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        ElapsedTime killTimer = new ElapsedTime();
        runMotors(power, power);
        while (notCloseEnough(20, myRobot.lf, myRobot.lb, myRobot.rf, myRobot.rb) && opModeIsActive() /*&& killTimer.seconds()<2*/) {
            Log.d("SkyStone Left Front: ", myRobot.lf.getCurrentPosition() + "");
            Log.d("SkyStone Left Back: ", myRobot.lb.getCurrentPosition() + "");
            Log.d("SkyStone Right Front: ", myRobot.rf.getCurrentPosition() + "");
            Log.d("SkyStone Right Back: ", myRobot.rb.getCurrentPosition() + "");
        }
        runMotors(0, 0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    // IMU Stuff
    public double getHorizontalAngle() {
        angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.firstAngle;
        output = loopAround(output);
        return output;
    }

    private double loopAround(double output) {
        if (output > 180) {
            output -= 360;
        }
        if (output < -180) {
            output += 360;
        }
        return output;
    }

    public double getRoll() {
        angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.secondAngle;
        output = loopAround(output);
        return output;
    }

    public double getVerticalAngle() {
        angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.thirdAngle;
        output = loopAround(output);
        return output;
    }

    //Positive = Clockwise, Negative = Counterclockwise
    public void encoderTurn(double targetAngle, double power, double tolerance) {
        encoderTurnNoStop(targetAngle, power, tolerance);
        runMotors(0, 0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void encoderTurnNoStop(double targetAngle, double power, double tolerance) {
        encoderTurnNoStopPowers(targetAngle, -power, power, tolerance, true);
    }

    void encoderTurnNoStopPowers(double targetAngle, double leftPower, double rightPower, double tolerance, boolean usePID) {
        double kR = Constants.kR;
        double kD = Constants.kD;

        //Undefined constants
        double d;
        double dt;
        double leftProportionalPower;
        double rightProportionalPower;
        //Initial error
        double currentAngle = getHorizontalAngle();
        double error = targetAngle - currentAngle;
        error = loopAround(error);
        double previousError = error;
        //Initial Time
        ElapsedTime clock = new ElapsedTime();
        double t1 = clock.nanoseconds();
        double t2 = t1;
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(error) > tolerance && opModeIsActive()) {
            //Getting Error
            currentAngle = getHorizontalAngle();
            error = loopAround(targetAngle - currentAngle);
            if (usePID) {
                //Getting time difference
                t2 = clock.nanoseconds();
                dt = t2 - t1;

                //Setting d action
                d = (error - previousError) / dt * Math.pow(10, 9);
                //Setting p action
                leftProportionalPower = Math.max(Math.min(error * kR + d * kD, 1), -1) * leftPower;
                rightProportionalPower = Math.max(Math.min(error * kR + d * kD, 1), -1) * rightPower;
                Log.d("Skystone: ", "leftProportionalPower: " + leftProportionalPower + " rightProportionalPower: " + rightProportionalPower);
                Log.d("Skystone: ", "dt: " + dt + "DerivativeAction: " + d * kD);
            } else {
                leftProportionalPower = leftPower * Math.signum(error);
                rightProportionalPower = rightPower * Math.signum(error);
            }

            //Set real power
            double realLeftPower = Math.max(Math.abs(leftPower / 2), Math.abs(leftProportionalPower)) * Math.signum(leftProportionalPower);
            double realRightPower = Math.max(Math.abs(rightPower / 2), Math.abs(rightProportionalPower)) * Math.signum(rightProportionalPower);
            runMotors(realLeftPower, realRightPower);

            //Store old values
            previousError = error;
            if (usePID) {
                t1 = t2;
            }


            //Logging
            Log.d("Skystone: ", "encoderTurn Error: " + error + " leftPower: " + realLeftPower + "rightPower: " + realRightPower + "CurrentAngle: " + currentAngle);
        }
    }


    // Distance sensor stuff
    // TODO: incorporate moving other motors (mainly just arm and lift)
    public void toTargetDistance(double targetDistance, Direction d, double tolerance, double timeLimit,
                                 double distanceCap, double... speeds) {
        ElapsedTime killTimer = new ElapsedTime();
        double kR = Constants.tkR;
        double originalAngle = getHorizontalAngle();
        double maxSpeed; // Highest power given to motors (don't go too far)
        double minSpeed; // Lowest power given to motors (to prevent it getting stuck)

        Rev2mDistanceSensor a;

        // Initializing all the variables
        if (speeds.length == 0) {
            if (d == Direction.FORWARD || d == Direction.BACKWARD) {
                maxSpeed = Constants.setVerticalMaxSpeed;
                minSpeed = Constants.setVerticalMinSpeed;
            } else {
                maxSpeed = Constants.setHorizontalMaxSpeed;
                minSpeed = Constants.setHorizontalMinSpeed;
            }
        } else if (speeds.length == 1) {
            maxSpeed = speeds[0];
            if (d == Direction.FORWARD || d == Direction.BACKWARD) {
                minSpeed = Constants.setVerticalMinSpeed;
            } else {
                minSpeed = Constants.setHorizontalMinSpeed;
            }
        } else if (speeds.length == 2) {
            maxSpeed = speeds[0];
            minSpeed = speeds[1];
        } else {
            throw new IllegalArgumentException();
        }

        if (distanceCap == -1) {
            if (d == Direction.FORWARD || d == Direction.BACKWARD) {
                distanceCap = Constants.setVerticalDisCap;
            } else {
                distanceCap = Constants.setHorizontalDisCap;
            }
        }

        a = getDirectionDistance(d);
        double currentDistance = a.getDistance(DistanceUnit.INCH); // Distance sensor value
        double currentAngle;
        double error = currentDistance - targetDistance; // Error
        double angleError;
        while (opModeIsActive() // While program has not ended
                && (Math.abs(error) > tolerance) // And it is not close enough
                && (killTimer.milliseconds() <= timeLimit)) { // And it has not passed the time limit
            currentDistance = a.getDistance(DistanceUnit.INCH); // Distance sensor value
            currentAngle = getHorizontalAngle(); // imu value
            error = currentDistance - targetDistance; // Error
            angleError = loopAround(currentAngle - originalAngle); // Angle error
            double power = Math.min(error, distanceCap) * maxSpeed / distanceCap; // Cap the power
            double speed = Math.max(Math.abs(power), minSpeed); // Make sure it's above minimum
            power = speed * Math.signum(power); // Makes sure sign is correct
            WheelPowers wps = getDirectionPower(d, power);
            Log.d("AHHHHH starting powers", wps + ", adjust power amount: " + (angleError * kR));
            wps.adjustPowers(angleError * kR, -angleError * kR);
            Log.d("AHHHHH actualPowers", wps.toString());
            runMotors(wps); // Set power to motors
            // Telemetry stuff
            telemetry.addData("currentDistance", currentDistance);
            telemetry.addData("timer", killTimer.toString());
            telemetry.update();
//            Log.d("AHHHHH currentDistance", String.valueOf(currentDistance));
//            Log.d("AHHHHH angle", "angleError: " + angleError + ", og angle: " + originalAngle
//                    + ", current angle" + currentAngle);
            Log.d("AHHHHH timer", killTimer.toString());
        }
        runMotors(0, 0);
    }

    public void toTarget(double vertTargetDistance, double horTargetDistance, Direction vertd,
                         Direction hord, double vertDistanceCap, double horDistanceCap,
                         double tolerance, double timeLimit, double... speeds) {
        ElapsedTime killTimer = new ElapsedTime();
        double kR = Constants.tkR;
        double originalAngle = getHorizontalAngle();
        double maxSpeed; // Highest power given to motors (don't go too far)
        double minSpeed; // Lowest power given to motors (to prevent it getting stuck)

        // Initializing all the variables
        if (speeds.length == 0) {
            maxSpeed = Constants.setDiagonalMaxSpeed;
            minSpeed = Constants.setDiagonalMinSpeed;
        } else if (speeds.length == 1) {
            maxSpeed = speeds[0];
            minSpeed = Constants.setDiagonalMinSpeed;
        } else if (speeds.length == 2) {
            maxSpeed = speeds[0];
            minSpeed = speeds[1];
        } else {
            throw new IllegalArgumentException();
        }

        if (vertDistanceCap == -1) {
            vertDistanceCap = Constants.setVerticalDisCap;
        }
        if (horDistanceCap == -1) {
            horDistanceCap = Constants.setHorizontalDisCap;
        }
//        Log.d("AHHHHH Double Distance", "maxSpeed: " + maxSpeed + ", minSpeed " +
//                minSpeed + "directions: " + vertd + ", " + hord + ", vertDisCap: " + vertDistanceCap
//                + ", horDisCap: " + horDistanceCap + ", vertTarget: " + vertTargetDistance +
//                ", horTarget: " + horTargetDistance);

        Direction d;
        double currentAngle, angleError;
        Rev2mDistanceSensor a, b;
        boolean vNeg, hNeg;

        a = getDirectionDistance(vertd);
        double distance = a.getDistance(DistanceUnit.INCH); // Distance sensor value for vert
        b = getDirectionDistance(hord);
        double distance2 = b.getDistance(DistanceUnit.INCH); // Distance sensor value for hor
        double error = distance - vertTargetDistance; // Error for vert
        vNeg = error < 0;
        double error2 = distance2 - horTargetDistance; // Error for hor
        hNeg = error2 < 0;
//        Log.d("AHHHHHH err+neg", "vError: " + error + ", hError: " + error2 + ", vNeg: " +
//                vNeg + ", hNeg: " + hNeg);

        if (Math.abs(error) <= Math.abs(error2 * Constants.vertHorRatio)) {
            // If vertical error is less than horizontal error * ratio
            d = hord;
//            Log.d("AHHHHH swapping", "vert error is less");
        } else {
            // Swapping values around to make it move based on the right thing
//            Log.d("AHHHHH Before swapping", "directions: " + vertd + ", " + hord
//                    + ", vertDisCap: " + vertDistanceCap
//                    + ", horDisCap: " + horDistanceCap + ", vertTarget: " + vertTargetDistance +
//                    ", horTarget: " + horTargetDistance);
            double temp = vertTargetDistance;
            vertTargetDistance = horTargetDistance;
            horTargetDistance = temp;
            temp = vertDistanceCap;
            vertDistanceCap = horDistanceCap;
            horDistanceCap = temp;
            a = b;
            error = error2;
            d = vertd;
        }

//        Log.d("AHHHHH After swapping?", "directions: " + d +
//                ", " + vertd + ", " + hord + ", vertDisCap: " + vertDistanceCap
//                + ", horDisCap: " + horDistanceCap + ", vertTarget: " + vertTargetDistance +
//                ", horTarget: " + horTargetDistance);

        while (opModeIsActive() // While program has not ended
                && (Math.abs(error) > tolerance) // And it is not close enough
                && (killTimer.milliseconds() <= timeLimit)) { // And it has not passed the time limit
            distance = a.getDistance(DistanceUnit.INCH); // Distance sensor value
            currentAngle = getHorizontalAngle(); // imu value
            error = distance - vertTargetDistance; // Error (using vertTargetDistance but could be hor)
            angleError = loopAround(currentAngle - originalAngle); // Angle error
            double power = Math.min(error, vertDistanceCap) * maxSpeed / vertDistanceCap; // Cap the power
            double speed = Math.max(Math.abs(power), minSpeed); // Make sure it's above minimum
            power = speed * Math.signum(power); // Makes sure sign is correct
            WheelPowers wps = getDirectionPower(vertd, hord, vNeg, hNeg, power);
//            Log.d("AHHHHH powers", wps.toString());
            wps.adjustPowers(angleError * kR, -angleError * kR);
            Log.d("AHHHHH actualPowers", wps.toString());
            runMotors(wps); // Set power to motors
            // Telemetry stuff
//            telemetry.addData("currentDistance", distance);
//            telemetry.addData("timer", killTimer.toString());
//            telemetry.update();
            Log.d("AHHHHH currentDistance", String.valueOf(distance));
            Log.d("AHHHHH angleError", String.valueOf(angleError));
//            Log.d("AHHHHH timer", killTimer.toString());
        }
        runMotors(0, 0);
        Log.d("AHHHHH 1d call", "targetDistance: " + horTargetDistance + ", d: " + d +
                ", distanceCap: " + horDistanceCap);
        toTargetDistance(horTargetDistance, d, tolerance, timeLimit - killTimer.milliseconds(), horDistanceCap);
    }

    public void frontCentering(double distanceCap, double tolerance, double timeLimit,
                               double... speeds) {
        ElapsedTime killTimer = new ElapsedTime();
        double kR = Constants.tkR;
        double originalAngle = getHorizontalAngle();
        double maxSpeed; // Highest power given to motors (don't go too far)
        double minSpeed; // Lowest power given to motors (to prevent it getting stuck)

        // Initializing all the variables
        if (speeds.length == 0) {
            maxSpeed = Constants.setReadjustMaxSpeed;
            minSpeed = Constants.setReadjustMinSpeed;
        } else if (speeds.length == 1) {
            maxSpeed = speeds[0];
            minSpeed = Constants.setReadjustMinSpeed;
        } else if (speeds.length == 2) {
            maxSpeed = speeds[0];
            minSpeed = speeds[1];
        } else {
            throw new IllegalArgumentException();
        }

        if (distanceCap == -1) {
            distanceCap = Constants.setReadjustDisCap;
        }

        double currentAngle, angleError;

        double leftDistance = myRobot.frontLeftDistance.getDistance(DistanceUnit.INCH); // Distance sensor value for front left
        double rightDistance = myRobot.frontDistance.getDistance(DistanceUnit.INCH); // Distance sensor value for front right
        double error = leftDistance - rightDistance; // Error between the 2 distances

        while (opModeIsActive() // While program has not ended
                && (Math.abs(error) > tolerance) // And it is not close enough
                && (killTimer.milliseconds() <= timeLimit)) { // And it has not passed the time limit
            leftDistance = Math.min(distanceCap, myRobot.frontLeftDistance.getDistance(DistanceUnit.INCH)); // Distance sensor value for front left
            rightDistance = Math.min(distanceCap, myRobot.frontDistance.getDistance(DistanceUnit.INCH)); // Distance sensor value for front right
            error = leftDistance - rightDistance; // Error between the 2 distances
            currentAngle = getHorizontalAngle(); // imu value
            angleError = loopAround(currentAngle - originalAngle); // Angle error

            double power = error / distanceCap * maxSpeed; // Cap the power
            double speed = Math.max(Math.abs(power), minSpeed); // Make sure it's above minimum
            power = speed * Math.signum(power); // Makes sure sign is correct
            WheelPowers wps = new WheelPowers(power, -power, -power, power);
//            Log.d("AHHHHH powers", wps.toString());
            wps.adjustPowers(angleError * kR, -angleError * kR);
            Log.d("AHHHHH actualPowers", wps.toString());
            runMotors(wps); // Set power to motors
            // Telemetry stuff
            telemetry.addData("currentDistances", "left: " + leftDistance + ", right: " + rightDistance);
            telemetry.addData("timer", killTimer.toString());
            telemetry.update();
            Log.d("AHHHHH currentDistance", "left: " + leftDistance + ", right: " + rightDistance);
            Log.d("AHHHHH angleError", String.valueOf(angleError));
//            Log.d("AHHHHH timer", killTimer.toString());
        }
        runMotors(0, 0);
    }

    public Rev2mDistanceSensor getDirectionDistance(Direction d) {
        switch (d) {
            case FORWARD:
                return myRobot.frontDistance;
            case BACKWARD:
                return myRobot.backDistance;
            case LEFT:
                return myRobot.leftDistance;
            case RIGHT:
                return myRobot.rightDistance;
            default:
                throw new IllegalArgumentException();
        }
    }

    public WheelPowers getDirectionPower(Direction d, double power) {
        WheelPowers wp;
        switch (d) {
            case FORWARD:
                wp = new WheelPowers(power);
                break;
            case BACKWARD:
                wp = new WheelPowers(-power);
                break;
            case LEFT:
                wp = new WheelPowers(-power, power, power, -power);
                break;
            case RIGHT:
                wp = new WheelPowers(power, -power, -power, power);
                break;
            default:
                throw new IllegalArgumentException();
        }
        return wp;
    }

    public WheelPowers getDirectionPower(Direction vertd, Direction hord, boolean vNeg,
                                         boolean hNeg, double power) {
        WheelPowers wp;
//        Log.d("AHHHHHH dpower call", "vertd: " + vertd + ", hord: " + hord +
//                ", vNeg: " + vNeg + ", hNeg: " + hNeg + ", power: " + power);
        // Swap directions if error is negative
        if (vNeg) {
            switch (vertd) {
                case FORWARD:
                    vertd = Direction.BACKWARD;
                    if (!hNeg) {
                        power = -power;
                    }
                    break;
                case BACKWARD:
                    vertd = Direction.FORWARD;
                    if (!hNeg) {
                        power = -power;
                    }
                    break;
            }
        }
        if (hNeg) {
            switch (hord) {
                case LEFT:
                    hord = Direction.RIGHT;
                    power = -power;
                    break;
                case RIGHT:
                    hord = Direction.LEFT;
                    power = -power;
                    break;
            }
        }
//        Log.d("AHHHHHH dpower swap?", "vertd: " + vertd + ", hord: " + hord +
//                ", vNeg: " + vNeg + ", hNeg: " + hNeg + ", power: " + power);
        if (vertd == Direction.FORWARD) {
            if (hord == Direction.LEFT) {
//                Log.d("AHHHHH", "RIGHT PLACE RIGHT?" + power);
                wp = new WheelPowers(0, power, power, 0);
            } else if (hord == Direction.RIGHT) {
                wp = new WheelPowers(power, 0, 0, power);
            } else {
                throw new IllegalArgumentException();
            }
        } else if (vertd == Direction.BACKWARD) {
            if (hord == Direction.LEFT) {
                wp = new WheelPowers(-power, 0, 0, -power);
            } else if (hord == Direction.RIGHT) {
                wp = new WheelPowers(0, -power, -power, 0);
            } else {
                throw new IllegalArgumentException();
            }
        } else {
            throw new IllegalArgumentException();
        }
        return wp;
    }


    public class WheelPowers {
        public double lfPower;
        public double lbPower;
        public double rfPower;
        public double rbPower;

        public WheelPowers(double power) {
            lfPower = power;
            lbPower = power;
            rfPower = power;
            rbPower = power;
        }

        public WheelPowers(double leftPower, double rightPower) {
            lfPower = leftPower;
            lbPower = leftPower;
            rfPower = rightPower;
            rbPower = rightPower;
        }

        public WheelPowers(double lfp, double lbp, double rfp, double rbp) {
            lfPower = lfp;
            lbPower = lbp;
            rfPower = rfp;
            rbPower = rbp;
        }

        public void adjustPowers(double leftAdjust, double rightAdjust) {
            lfPower += leftAdjust;
            lbPower += leftAdjust;
            rfPower += rightAdjust;
            rbPower += rightAdjust;
        }

        public String toString() {
            return "lf: " + lfPower + ", lb: " + lbPower + ", rf: " + rfPower + ", rb: " + rbPower;
        }
    }
}
