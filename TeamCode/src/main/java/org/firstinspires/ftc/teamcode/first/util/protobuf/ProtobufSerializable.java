package org.firstinspires.ftc.teamcode.first.util.protobuf;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import org.firstinspires.ftc.teamcode.first.util.WPISerializable;

/**
 * Marker interface to indicate a class is serializable using Protobuf serialization.
 *
 * <p>While this cannot be enforced by the interface, any class implementing this interface should
 * provide a public final static `proto` member variable.
 */
public interface ProtobufSerializable<T> extends WPISerializable<T> {}