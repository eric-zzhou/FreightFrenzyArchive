package org.firstinspires.ftc.teamcode.freightFrenzy;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue Duck", group = "Linear Opmode")

public class BlueDuckAutonomous extends AutonomousMethods {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private int detectLayer = 0;

    @Override
    public void runOpMode() {
        initializeAutonomousAttachments(hardwareMap, telemetry);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Slides in to drive
        setExtPosition(Constants.extIn);

        // Detection
        Log.d("AHHHHHH detection", "detection started-----------------------------------------------------------------------------");
        toTargetDistance(14, Direction.LEFT, 0.3, 4800, 24);
        sleep(120);
        detectLayer = duckDetection(false);

        // Duck spinning
        Log.d("AHHHHHH duck", "duck spinning started-----------------------------------------------------------------------------");
        toTarget(2.4, 9, Direction.BACKWARD, Direction.LEFT, 24, 9, 0.6, 6000);
        sleep(360);
        toTarget(1, 6, Direction.BACKWARD, Direction.LEFT, 18, 9, 0.6, 6000, 0.6);
        setRotationPosition(0.06, 0);
        myRobot.runRightServo(Constants.duckSpinPower);
        sleep(1500);
        myRobot.runRightServo(Constants.duckAutoSpinPower);
        sleep(900);
        myRobot.runRightServo(0);

        // Dropping at shipping hub
        Log.d("AHHHHHH drop", "dropping started-----------------------------------------------------------------------------");
        toTarget(24, Constants.dropSide + Constants.sideOffset - 1.7, Direction.BACKWARD, Direction.LEFT, 28, 39, 1, 9000);
        setRotationPosition(0.36, 0);

        if (detectLayer == Constants.armTopAuto) {
            setLiftMotor(Constants.armStep, 1, false);
            sleep(120);
            toTargetDistance(Constants.duckDropDis, Direction.BACKWARD, 0.3, 2400, 39);
            myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftAndExtend(detectLayer, Constants.extTop, Constants.extTopEarly, 9);
        } else if (detectLayer == Constants.armMed) {
            setLiftMotor(Constants.armMed/2, 1, false);
            sleep(120);
            toTargetDistance(Constants.duckDropDis, Direction.BACKWARD, 0.3, 2400, 39);
            myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftAndExtend(detectLayer, Constants.extMed, Constants.extMedEarly, 9);
        } else {
            sleep(120);
            toTargetDistance(Constants.duckDropDis, Direction.BACKWARD, 0.3, 2400, 39);
            liftAndExtend(detectLayer, Constants.extBot, Constants.extBotEarly, 9);
        }
        sleep(120);

        drop();
        resetDropArm();

        // Parking
        Log.d("AHHHHHH park", "parking started-----------------------------------------------------------------------------");
        parkDelivery(false);

        AutoTransitioner.transitionOnStop(this, "Blue TeleOp");
    }
}
