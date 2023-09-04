package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.freightFrenzy.AutonomousMethods;
import org.firstinspires.ftc.teamcode.freightFrenzy.Constants;

@Autonomous(name="SingleDirectionTest", group="Linear Opmode")
@Disabled
public class SingleDirectionTest extends AutonomousMethods {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    double speed = 1;

    @Override
    public void runOpMode() {
        initializeAutonomousAttachments(hardwareMap, telemetry);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
//        myRobot.runArmMotor(Constants.armRaisePower);
//        sleep((long) Constants.armRaiseTime);
//        myRobot.runArmMotor(Constants.armHoldPower);
        sleep(120);

        // run until the end of the match (driver presses STOP)
//            toTargetDistance(12, Direction.FORWARD, 0.25, 5000, 36);
//        toTargetDistance(12, Direction.RIGHT, 0.25, 10000, 24);
//        toTargetDistance(Constants.testHorTargetDistance, Direction.RIGHT, 0.25, 10000, -1);
//        sleep(960);
//        toTargetDistance(Constants.testHorTargetDistance + 12, Direction.RIGHT, 0.25, 10000, -1);
//        sleep(240);
//        toTargetDistance(Constants.testVertTargetDistance, Direction.FORWARD, 0.25, 10000, -1);
//        sleep(960);
//        toTargetDistance(Constants.testVertTargetDistance + 12, Direction.FORWARD, 0.25, 10000, -1);
//        sleep(240);
//        encoderTurn(180, 0.6, 0.6);
//        sleep(240);
        toTargetDistance(Constants.testHorTargetDistance, Direction.LEFT, 0.25, 10000, -1);
        sleep(960);
        toTargetDistance(Constants.testVertTargetDistance, Direction.BACKWARD, 0.25, 10000, -1);
//        myRobot.runArmMotor(0);
    }
}
