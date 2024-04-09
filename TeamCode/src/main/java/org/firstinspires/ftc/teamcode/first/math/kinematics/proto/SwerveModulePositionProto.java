package org.firstinspires.ftc.teamcode.first.math.kinematics.proto;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import org.firstinspires.ftc.teamcode.first.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.first.math.kinematics.SwerveModulePosition;
import org.firstinspires.ftc.teamcode.first.math.proto.Kinematics.ProtobufSwerveModulePosition;
import org.firstinspires.ftc.teamcode.first.util.protobuf.Protobuf;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class SwerveModulePositionProto
        implements Protobuf<SwerveModulePosition, ProtobufSwerveModulePosition> {
    @Override
    public Class<SwerveModulePosition> getTypeClass() {
        return SwerveModulePosition.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufSwerveModulePosition.getDescriptor();
    }

    @Override
    public Protobuf<?, ?>[] getNested() {
        return new Protobuf<?, ?>[] {Rotation2d.proto};
    }

    @Override
    public ProtobufSwerveModulePosition createMessage() {
        return ProtobufSwerveModulePosition.newInstance();
    }

    @Override
    public SwerveModulePosition unpack(ProtobufSwerveModulePosition msg) {
        return new SwerveModulePosition(msg.getDistance(), Rotation2d.proto.unpack(msg.getAngle()));
    }

    @Override
    public void pack(ProtobufSwerveModulePosition msg, SwerveModulePosition value) {
        msg.setDistance(value.distanceMeters);
        Rotation2d.proto.pack(msg.getMutableAngle(), value.angle);
    }
}
