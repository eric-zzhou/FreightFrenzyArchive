package org.firstinspires.ftc.teamcode.freightFrenzy;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red Drop", group = "Linear Opmode")

public class RedDropAutonomous extends AutonomousMethods {

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
        detectLayer = distanceDetection(true);

        // Dropping at shipping hub
        Log.d("AHHHHHH drop", "dropping started-----------------------------------------------------------------------------");
        encoderStraightDrive(-1.8, 0.72);
        encoderTurn(0, 0.36, 0.3);
        sleep(360);
        toTargetDistance(Constants.dropSide - 1 + Constants.sideOffset, Direction.LEFT, 0.3, 7200, 54);
        if (detectLayer == Constants.armTopAuto) {
            setLiftMotor(Constants.armStep, 1, false);
            sleep(120);
            myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftAndExtend(detectLayer, Constants.extTop, Constants.extTopEarly, 9);
        } else if (detectLayer == Constants.armMed) {
            setLiftMotor(Constants.armMed/2, 1, false);
            sleep(120);
            myRobot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftAndExtend(detectLayer, Constants.extMed, Constants.extMedEarly, 9);
        } else {
            sleep(120);
            liftAndExtend(detectLayer, Constants.extBot, Constants.extBotEarly, 9);
        }
        sleep(120);

        drop();
        resetDropArm();

        // Parking
        Log.d("AHHHHHH park", "parking started-----------------------------------------------------------------------------");
        toTargetDistance(6 + Constants.sideOffset, Direction.LEFT, 0.3, 2400, 18);
        resetArmGap();
        encoderTurn(-90, 0.3, 0.3);
        park(true, false, true);
        myRobot.runCollectionMotor(Constants.collectPower);
        encoderStraightDrive(9, 0.12);
        myRobot.runCollectionMotor(0);

        AutoTransitioner.transitionOnStop(this, "Red TeleOp");
    }
}
