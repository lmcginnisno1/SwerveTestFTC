package org.firstinspires.ftc.teamcode.first.math.kinematics;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import org.firstinspires.ftc.teamcode.first.math.interpolation.Interpolatable;

public interface WheelPositions<T extends WheelPositions<T>> extends Interpolatable<T> {
    /**
     * Returns a copy of this instance.
     *
     * @return A copy.
     */
    T copy();
}
