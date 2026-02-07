package org.firstinspires.ftc.teamcode.TeleOp.Concepts;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.Common.prismled.*;

@TeleOp(name = "TEST: Prism LED Artboards", group = "Test")
@Disabled
public class PrismLedTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        PrismI2c prism = PrismI2c.fromHardwareMap(hardwareMap, "prism");
        ElapsedTime timer = new ElapsedTime();

        telemetry.addLine("Prism LED Test Ready");
        telemetry.addLine("Artboards will cycle every 2 seconds");
        telemetry.update();

        waitForStart();

        timer.reset();

        int slot = 0;

        while (opModeIsActive()) {

            prism.loadArtboard(slot);

            telemetry.addData("Active Artboard Slot", slot);
            telemetry.update();

            // Hold this artboard for 2 seconds
            sleep(2000);

            slot++;
            if (slot > 3) {
                slot = 0;
            }
        }
    }
}
