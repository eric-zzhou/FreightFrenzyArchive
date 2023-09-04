package org.firstinspires.ftc.teamcode.freightFrenzy;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Red TeleOp", group = "Iterative Opmode")
//@Disabled
public class RedTeleOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Attachments myRobot = new Attachments();

    private enum rotateD {
        STRAIGHT,
        LEFT90,
        RIGHT90,
        AROUND180,
        RANDOM
    }

    /* ------------------------------------ CONSTANTS ------------------------------------ */
    // Motors
    private double liftPower = 0;
    private int liftTarget = 0;
    private boolean useLiftPower = true;
    private double rotationPower = 0;
    private int rotationTarget = -1;
    private boolean useRotationPower = false;
    private rotateD armDirection = rotateD.STRAIGHT;
    private double collectionPower = 0;
    private double dpadrchill = Constants.buttonDelay;
    private double dpadlchill = Constants.buttonDelay;
    private boolean limits = true;
    private boolean shorten = false;

    private double currentLiftPosition = 0;
    private double currentExtPosition = Constants.extIn;
    private double currentRPosition = 0;

    // Servos
    private double dropPosition = Constants.dropClosePosition;
    private double extPosition = Constants.extIn;

    private int stage = -1;
    private int goalAngle = 0;
    private rotateD goalDirection = rotateD.STRAIGHT;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        myRobot.initialize(hardwareMap, telemetry);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        setRotationPosition(0.36, 0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /* ------------------------------------ Drive ------------------------------------ */
//        if (currentLiftPosition >= Constants.armStop){
        // Position constants
        currentLiftPosition = myRobot.getLiftMotorPosition();
        currentExtPosition = myRobot.getExtenderPosition();
        currentRPosition = myRobot.getRotationMotorPosition();

            // Motors
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double speedMultiplier = 1;
        double rotationMultiplier = 0.72;

            // D-pad
            if (gamepad1.dpad_up) {
                ly = 1;
                lx = 0;
                speedMultiplier = 0.36;
            } else if (gamepad1.dpad_down) {
                ly = -1;
                lx = 0;
                speedMultiplier = 0.72;
            }
            if (gamepad1.dpad_left) {
                lx = -1;
                ly = 0;
            speedMultiplier = 1;
            } else if (gamepad1.dpad_right) {
                lx = 1;
                ly = 0;
            speedMultiplier = 1;
        }

        if (currentLiftPosition < -960/* && stage == -1*/) {
            speedMultiplier = Constants.armDriveLimitRatio / currentLiftPosition;
            rotationMultiplier = .72 * speedMultiplier;
            }
        if (ly > 0.1 && currentLiftPosition < -720) {
            ly *= Constants.forwardLimitRatio;
        }

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.addData("Rotation Multiplier", rotationMultiplier);

            // Math
            double theta = Math.atan2(lx, ly);
            double v_theta = Math.sqrt(lx * lx + ly * ly);
            double v_rotation = gamepad1.right_stick_x;

            // Drive
            myRobot.drive(theta, speedMultiplier * v_theta, rotationMultiplier * v_rotation);
//        } else {
//            myRobot.runMotors(0, 0);
//        }






        /* ------------------------------------ Change ------------------------------------ */
        // Disabling stage thing
        if (gamepad2.b && !gamepad2.start) {
            stage = -1;
            useLiftPower = true;
            useRotationPower = true;
            extPosition = currentExtPosition;
        }

        // Rotating stuff by power
        double lt1 = gamepad1.left_trigger;
        double rt1 = gamepad1.right_trigger;
        if (lt1 > 0.18) {
            useRotationPower = true;
            armDirection = rotateD.RANDOM;
            myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotationPower = -lt1 * Constants.setRotateMultiplier;
        } else if (rt1 > 0.18) {
            useRotationPower = true;
            armDirection = rotateD.RANDOM;
            myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotationPower = rt1 * Constants.setRotateMultiplier;
        } else if (useRotationPower) {
            rotationPower = 0;
        }

        // Rotating stuff by power
        double armRJoystick = gamepad2.left_stick_x;
        if (Math.abs(armRJoystick) > 0.72) {
            useRotationPower = true;
            armDirection = rotateD.RANDOM;
            myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotationPower = armRJoystick * Constants.setRotateMultiplier * 2 / 3;
        }

        // Preset collection positions
        double collectionRJoystick = gamepad2.right_stick_x;
        if (Math.abs(collectionRJoystick) > 0.96) {
            if (collectionRJoystick < 0.96) {
                useLiftPower = false;
                liftTarget = Constants.armStep;
                useRotationPower = false;
                armDirection = rotateD.STRAIGHT;
                shorten = true;
                rotationTarget = 0;
            } else {
                stage = 1;
                goalDirection = rotateD.RIGHT90;
                goalAngle = Constants.armR90;
                dropPosition = Constants.dropClosePosition;
                shorten = true;
                extPosition = Constants.extSide;
            }
        }

        if (stage > 0) {
            switch (stage) {
                case 1:
                    // Raise and turn
                    if (Math.abs(currentRPosition - goalAngle) <= 12) {
                        stage = 2;
                    } else {
                        useLiftPower = false;
                        liftTarget = Constants.armSpinOpt;
                        useRotationPower = false;
                        armDirection = goalDirection;
                        shorten = false;
                        rotationTarget = goalAngle;
                        break;
                    }
                case 2:
                    // Extend (to extSide)
                    // Raise and turn
                    if (currentExtPosition >= Constants.extSide) {
                        stage = 3;
                    } else {
                        extPosition = Constants.extSide;
                        break;
                    }
                case Constants.automationDelay:
                    // Lower down to collect
                    if (Math.abs(currentLiftPosition - 0) >= 36) {
                        useLiftPower = false;
                        liftTarget = 0;
                    } else {
                        stage = -1;
                    }
                    break;
                default:
                    stage++;
            }
        }

        // I need it for the collection auto-rotate as well
        if (gamepad1.x) {
            useLiftPower = false;
            liftTarget = Constants.armSpin;
            useRotationPower = false;
            armDirection = rotateD.LEFT90;
            shorten = false;
            rotationTarget = Constants.armL90;
        } else if (gamepad1.y) {
//            useLiftPower = false;
//            liftTarget = Constants.armSpin;
            useRotationPower = false;
            armDirection = rotateD.STRAIGHT;
//            shorten = false;
            rotationTarget = 0;
        } else if (gamepad1.a && !gamepad1.start) {
            useLiftPower = false;
            liftTarget = Constants.armSpin;
            useRotationPower = false;
            armDirection = rotateD.AROUND180;
            shorten = false;
            if (Math.abs(currentRPosition - Constants.armL180) <
                    Math.abs(currentRPosition - Constants.armR180)) {
                rotationTarget = Constants.armL180;
            } else {
                rotationTarget = Constants.armR180;
            }
        } else if (gamepad1.b) {
            useLiftPower = false;
            liftTarget = Constants.armSpin;
            useRotationPower = false;
            armDirection = rotateD.RIGHT90;
            shorten = false;
            rotationTarget = Constants.armR90;
        }


        // Extension
        double extJoystick = gamepad2.right_stick_y;
        if (Math.abs(extJoystick) > 0.12) {
            extPosition = currentExtPosition + extJoystick * Constants.extRatio;
            extPosition = Math.max(Constants.extIn, Math.min(extPosition, Constants.extOut));
        }

        // Lift set positions
        /*if (gamepad2.dpad_left && dpadlchill == Constants.buttonDelay) {
            useLiftPower = false;
            liftTarget = Constants.armSpinOpt;
            useRotationPower = false;
            if (armDirection.equals(rotateD.STRAIGHT)) {
                shorten = true;
                liftTarget = Constants.armNeutral;
                armDirection = rotateD.AROUND180;
                if (Math.abs(currentRPosition - Constants.armL180) <
                        Math.abs(currentRPosition - Constants.armR180)) {
                    rotationTarget = Constants.armL180;
                } else {
                    rotationTarget = Constants.armR180;
                }
            } else {
                stage = 1;
                goalDirection = rotateD.STRAIGHT;
                goalAngle = 0;
            }
            dpadlchill = 0;
        } else*/ if (gamepad2.dpad_up) {
            useLiftPower = false;
            dropPosition = Constants.dropClosePosition;
            liftTarget = Constants.armTop;
        } else if (gamepad2.dpad_right && dpadrchill == Constants.buttonDelay) {
            useLiftPower = false;
            liftTarget = Constants.armSpinOpt;
            useRotationPower = false;
            if (armDirection.equals(rotateD.LEFT90)) {
                stage = -1;
                armDirection = rotateD.STRAIGHT;
                shorten = true;
                rotationTarget = 0;
                liftTarget = Constants.armNeutral;
            } else {
                stage = 1;
                goalDirection = rotateD.LEFT90;
                goalAngle = Constants.armL90;
            }
            dpadrchill = 0;
        } else if (gamepad2.dpad_down) {
            useLiftPower = false;
            dropPosition = Constants.dropClosePosition;
            liftTarget = Constants.armCollect;
        }

        if (dpadrchill < Constants.buttonDelay) {
            dpadrchill++;
        }
        if (dpadlchill < Constants.buttonDelay) {
            dpadlchill++;
        }

        // Raising lift by power
        double liftJoystick = gamepad2.left_stick_y;
        if (liftJoystick > 0.12) {
            if ((!limits) || currentLiftPosition < -48) {
                useLiftPower = true;
                liftPower = liftJoystick * Constants.liftDownRatio;
            } else {
                useLiftPower = false;
                dropPosition = Constants.dropClosePosition;
                liftTarget = 0;
            }
        } else if (liftJoystick < -0.12) {
            if ((!limits) || currentLiftPosition > (Constants.liftTop + Constants.liftOff)) {
                useLiftPower = true;
                liftPower = liftJoystick * Constants.liftRatio;
            } else {
                useLiftPower = false;
                dropPosition = Constants.dropClosePosition;
                liftTarget = Constants.liftTop;
            }
        } else if (useLiftPower) {
            liftPower = 0;
        }

        // Collection
        double lt2 = gamepad2.left_trigger;
        double rt2 = gamepad2.right_trigger;
        if (rt2 > 0.9) {
            collectionPower = Constants.ejectPower;
        } else if (rt2 > 0.12) {
            collectionPower = Constants.weakEjectPower;
        } else if (lt2 > 0.6) {
            collectionPower = Constants.collectPower;
//            dropPosition = Constants.dropClosePosition;
        } else {
            collectionPower = 0;
        }

        // Duck
        //TODO: maybe add duck automation, might need to change for this side
        if (gamepad2.left_bumper) {
            myRobot.runLeftServo(-Constants.duckSpinPower);
        } else {
            myRobot.runLeftServo(0);
        }
        if (gamepad2.right_bumper) {
            myRobot.runRightServo(-Constants.duckSpinPower);
        } else {
            myRobot.runRightServo(0);
        }

        // Drop
        if (gamepad2.y) {
            dropPosition = Constants.dropClosePosition;
        } else if (gamepad2.x) {
            dropPosition = Constants.dropOpenPosition;
        } else if (gamepad2.a) {
            dropPosition = Constants.dropFrontPosition;
        }

        // Stop powers
        if (gamepad1.left_bumper) {
            useRotationPower = true;
            rotationPower = 0;
            useLiftPower = true;
            liftPower = 0;
        }

        if (gamepad1.right_bumper) {
            limits = !limits;
        }






        /* ------------------------------------ Action ------------------------------------ */
        if (useRotationPower) {
//            telemetry.addData(Double.toString(currentRPosition), Math.signum(rotationPower) * Constants.armRLimit);
            if (Math.signum(rotationPower) == 1) {
                if (currentRPosition > Constants.armRLimit) {
                    rotationPower = 0;
                }
            } else {
                if (currentRPosition < -Constants.armRLimit) {
                    rotationPower = 0;
                }
            }
            myRobot.runRotateMotor(rotationPower);
        } /*else if (rotationTarget != -1 && stage < 1 && Math.abs(rotationTarget) < Constants.armRLimit) {
            setRotationPositionPID(rotationTarget, 9);
        }*/ else if (rotationTarget != -1 && Math.abs(rotationTarget) < Constants.armRLimit) {
            setRotationPositionPID(rotationTarget, 9, shorten);
        }

        if (useLiftPower) {
            myRobot.runLiftMotor(liftPower);
        } else {
            setLiftMotor(liftTarget, 9, true);
        }
        myRobot.setExtServo(extPosition);


        myRobot.runCollectionMotor(collectionPower);

        if (Math.abs(liftPower) > 0.01 || Math.abs(rotationPower) > 0.01) {
            dropPosition = Constants.dropClosePosition;
        }
        myRobot.setDropServo(dropPosition);

//        telemetry.addData("arm power: ", armPower);
//        telemetry.addData("use power: ", useArmPower);
        telemetry.addData("extension position", currentExtPosition);
//        telemetry.addData("drop position", myRobot.dropServo.getPosition());
        telemetry.addData("lift position", currentLiftPosition);
//        telemetry.addData("lift power", useLiftPower);
        telemetry.addData("rotate position", currentRPosition);
        telemetry.addData("limits", limits);
//        telemetry.addData("rotate 180", rotate180);
//        telemetry.addData("use rotate power", useRotationPower);
//        telemetry.addData("rotate power", rotationPower);
        telemetry.update();

        Log.d("AHHHHHH extender", String.valueOf(currentExtPosition));
        Log.d("AHHHHHH rotate", String.valueOf(currentRPosition));
        Log.d("AHHHHHH lift", String.valueOf(currentLiftPosition));
//        Log.d("AHHHHHH collection", String.valueOf(myRobot.collectionMotor.getPower()));
//        Log.d("AHHHHH front: ", "" + myRobot.getFrontDistance());
//        Log.d("AHHHHH right: ", "" + myRobot.getRightDistance());
//        Log.d("AHHHHH left: ", "" + myRobot.getLeftDistance());
//        Log.d("AHHHHH back: ", "" + myRobot.getBackDistance());
        Log.d("AHHHHH imu: ", "" + myRobot.getAngle());
    }

    @Override
    public void stop() {
    }

    void setLiftMotor(int position, double tolerance, boolean usePID) {
        dropPosition = Constants.dropClosePosition;
        if (usePID) {
            //Undefined constants
            double newPower;
            //Initial error
            double error = (position - currentLiftPosition) / Constants.liftMax;
            //Initial Time
            telemetry.addData("1", "error: " + error);
            if (Math.abs(error) > (tolerance / Constants.liftMax)) {
                //Setting p action
                newPower = Math.max(Math.min(error * Constants.liftkP, 1), -1);
//                Log.d("AHHHHHH liftMotor", "PID newPower: " + newPower);
//                telemetry.addData("liftMotor PID newPower", newPower);

                //Set real power
                newPower = Math.max(Math.abs(newPower), Constants.liftMin) * Math.signum(newPower);
                if (Math.signum(newPower) == 1) {
                    newPower = newPower * Constants.liftDownRatio;
                }
                myRobot.runLiftMotor(newPower);

                //Logging
//                Log.d("AHHHHHH liftMotor", "error: " + (error * Constants.liftMax) + ", power: " + newPower + ", current position: " + currentPosition);
//                return false;
            } else {
                useLiftPower = true;
                myRobot.runLiftMotor(0);
//                return true;
            }
        } else {
            myRobot.setLiftMotor(1, position);
//            return true;
        }
    }

    public void setRotationPosition(double speed, int position) {
        if ((currentLiftPosition < Constants.armSpin) ||
                (currentExtPosition > Constants.extSpin) ||
                Math.abs(position - currentRPosition) < 160) {
            myRobot.rotateMotor.setPower(speed);
            myRobot.rotateMotor.setTargetPosition(position);
            myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void setRotationPositionPID(int position, double tolerance, boolean shorter) {
        boolean spin = (currentLiftPosition <= Constants.armSpin);
        if (spin) {
            if (shorter) {
                extPosition = Constants.extIn;
            }
            //Undefined constants
            double newPower;
            //Initial error
            double error = (position - currentRPosition) / Constants.rotationMax;
            //Initial Time
            myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (Math.abs(error) > (tolerance / Constants.rotationMax)) {
                //Setting p action
                newPower = Math.max(Math.min(error * Constants.rotationkP, 1), -1);

                //Set real power
                newPower = Math.max(Math.abs(newPower), Constants.rotationMin) * Math.signum(newPower);
                if (Math.signum(rotationPower) == 1) {
                    if (currentRPosition > Constants.armRLimit) {
                        newPower = 0;
                        useRotationPower = true;
                        rotationPower = 0;
                    }
                } else {
                    if (currentRPosition < -Constants.armRLimit) {
                        newPower = 0;
                        useRotationPower = true;
                        rotationPower = 0;
                    }
                }
                myRobot.runRotateMotor(newPower);

                //Logging
//                Log.d("AHHHHHH rotationMotor", "error: " + (error * Constants.rotationMax) + ", power: " + newPower + ", current position: " + currentRPosition);
//                return false;
            } else {
                rotationTarget = -1;
                setRotationPosition(0.24, position);
//                if (hold) {
//
//                } else {
//                    useRotationPower = true;
//                    myRobot.runRotateMotor(0);
//                }
//                return true;
            }
        }/* else {
            return false;
        }*/
    }
}

