package org.firstinspires.ftc.teamcode.pedropathing.constants;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.constants.*;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.*;

public final class LocalizerConstants {
    static {
        PinpointConstants.hardwareMapName = "pinpoint";

        PinpointConstants.distanceUnit = INCH;

        PinpointConstants.useYawScalar = false;
        PinpointConstants.useCustomEncoderResolution = false;

        PinpointConstants.encoderResolution
                = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        PinpointConstants.forwardY = 6.75;
        PinpointConstants.strafeX = 0.625;
    }
}




