// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OIConstants;

/** Add your docs here. */
public class InputUtil {
    public static double trimDriveInput(double joystickInput) {
        double zonedInput = applyDeadzone(joystickInput);
        // Make the lower bound of the stick
        double inputSign = Math.signum(zonedInput);
        double interpolatedInput = MathUtil.interpolate(OIConstants.kBaselinePower, OIConstants.kMaxPower,
                Math.abs(joystickInput));
        return Math.pow(interpolatedInput, OIConstants.kDegreeSmoothing) * inputSign;
    }

    public static double applyDeadzone(double input) {
        return (Math.abs(input) < OIConstants.kJoystickDeadzone) ? 0 : input;
    }
}
