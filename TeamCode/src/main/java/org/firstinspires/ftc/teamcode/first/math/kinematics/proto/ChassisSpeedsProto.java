package org.firstinspires.ftc.teamcode.first.math.kinematics.proto;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import org.firstinspires.ftc.teamcode.first.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.first.math.proto.Kinematics.ProtobufChassisSpeeds;
import org.firstinspires.ftc.teamcode.first.util.protobuf.Protobuf;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class ChassisSpeedsProto implements Protobuf<ChassisSpeeds, ProtobufChassisSpeeds> {
    @Override
    public Class<ChassisSpeeds> getTypeClass() {
        return ChassisSpeeds.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufChassisSpeeds.getDescriptor();
    }

    @Override
    public ProtobufChassisSpeeds createMessage() {
        return ProtobufChassisSpeeds.newInstance();
    }

    @Override
    public ChassisSpeeds unpack(ProtobufChassisSpeeds msg) {
        return new ChassisSpeeds(msg.getVx(), msg.getVy(), msg.getOmega());
    }

    @Override
    public void pack(ProtobufChassisSpeeds msg, ChassisSpeeds value) {
        msg.setVx(value.vxMetersPerSecond);
        msg.setVy(value.vyMetersPerSecond);
        msg.setOmega(value.omegaRadiansPerSecond);
    }
}