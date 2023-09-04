package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.freightFrenzy.AutonomousMethods;
import org.firstinspires.ftc.teamcode.freightFrenzy.Constants;

@Autonomous(name="DoubleDirectionTest", group="Linear Opmode")
@Disabled
public class DoubleDirectionTest extends AutonomousMethods {
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
        toTarget(Constants.testVertTargetDistance, Constants.testHorTargetDistance, Direction.FORWARD, Direction.LEFT, 18, 12, 0.6, 6000);
//        myRobot.runArmMotor(0);
    }
}
