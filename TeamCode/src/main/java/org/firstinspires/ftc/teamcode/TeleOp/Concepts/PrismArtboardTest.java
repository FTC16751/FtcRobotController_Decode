package org.firstinspires.ftc.teamcode.TeleOp.Concepts;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.Common.prismled.GoBildaPrismDriver;

@TeleOp(name = "TEST: Prism Artboards (goBILDA Driver)", group = "Test")
@Disabled
public class PrismArtboardTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // IMPORTANT: This name must match what YOU name it in the Robot Configuration.
        GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        telemetry.addLine("Prism test ready.");
        telemetry.addLine("Will cycle ARTBOARD_0..ARTBOARD_3 every 2 seconds.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
            telemetry.addData("Artboard", "0");
            telemetry.update();
            sleep(2000);

            prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
            telemetry.addData("Artboard", "1");
            telemetry.update();
            sleep(2000);

            prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);
            telemetry.addData("Artboard", "2");
            telemetry.update();
            sleep(2000);

            prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_3);
            telemetry.addData("Artboard", "3");
            telemetry.update();
            sleep(2000);
        }
    }
}
