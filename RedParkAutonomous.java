package org.firstinspires.ftc.teamcode.freightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red Park", group = "Linear Opmode")

public class RedParkAutonomous extends AutonomousMethods {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeAutonomousAttachments(hardwareMap, telemetry);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        resetArmGap();
        sleep(120);
        park(true, true, true);

        AutoTransitioner.transitionOnStop(this, "Red TeleOp");
    }
}
