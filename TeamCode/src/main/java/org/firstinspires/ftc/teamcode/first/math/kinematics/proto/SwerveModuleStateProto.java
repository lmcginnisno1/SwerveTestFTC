package org.firstinspires.ftc.teamcode.first.math.kinematics.proto;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import org.firstinspires.ftc.teamcode.first.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.first.math.kinematics.SwerveModuleState;
import org.firstinspires.ftc.teamcode.first.math.proto.Kinematics.ProtobufSwerveModuleState;
import org.firstinspires.ftc.teamcode.first.util.protobuf.Protobuf;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class SwerveModuleStateProto
        implements Protobuf<SwerveModuleState, ProtobufSwerveModuleState> {
    @Override
    public Class<SwerveModuleState> getTypeClass() {
        return SwerveModuleState.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufSwerveModuleState.getDescriptor();
    }

    @Override
    public Protobuf<?, ?>[] getNested() {
        return new Protobuf<?, ?>[] {Rotation2d.proto};
    }

    @Override
    public ProtobufSwerveModuleState createMessage() {
        return ProtobufSwerveModuleState.newInstance();
    }

    @Override
    public SwerveModuleState unpack(ProtobufSwerveModuleState msg) {
        return new SwerveModuleState(msg.getSpeed(), Rotation2d.proto.unpack(msg.getAngle()));
    }

    @Override
    public void pack(ProtobufSwerveModuleState msg, SwerveModuleState value) {
        msg.setSpeed(value.speedMetersPerSecond);
        Rotation2d.proto.pack(msg.getMutableAngle(), value.angle);
    }
}
