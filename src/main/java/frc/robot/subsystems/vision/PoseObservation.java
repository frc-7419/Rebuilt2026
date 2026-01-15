package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.util.LimelightHelpers;
import java.nio.ByteBuffer;

public class PoseObservation implements StructSerializable {
    public Pose2d estimatedPose = new Pose2d();
    public double timestampSeconds;
    public double latency;
    public int tagCount;
    public boolean isMegaTag2;

    public int[] fiducialIds;

    public static final Struct<PoseObservation> struct = new PoseObservationStruct();

    private static class PoseObservationStruct implements Struct<PoseObservation> {
        @Override
        public Class<PoseObservation> getTypeClass() {
            return PoseObservation.class;
        }

        @Override
        public String getTypeName() {
            return "PoseObservation";
        }

        @Override
        public String getTypeString() {
            return "struct:PoseObservation";
        }

        @Override
        public int getSize() {
            return Pose2d.struct.getSize() + kSizeDouble * 2 + kSizeInt32 + kSizeBool;
        }

        @Override
        public String getSchema() {
            return "Pose2d estimatedPose;double timestampSeconds;double latency;int32 tagCount;bool isMegaTag2";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] { Pose2d.struct };
        }

        @Override
        public void pack(ByteBuffer buffer, PoseObservation o) {
            Pose2d.struct.pack(buffer, o.estimatedPose);
            buffer.putDouble(o.timestampSeconds);
            buffer.putDouble(o.latency);
            buffer.putInt(o.tagCount);
            buffer.put(o.isMegaTag2 ? (byte) 1 : (byte) 0);
        }

        @Override
        public PoseObservation unpack(ByteBuffer buffer) {
            PoseObservation o = new PoseObservation();
            o.estimatedPose = Pose2d.struct.unpack(buffer);
            o.timestampSeconds = buffer.getDouble();
            o.latency = buffer.getDouble();
            o.tagCount = buffer.getInt();
            o.isMegaTag2 = buffer.get() != 0;
            return o;
        }
    }

    public PoseObservation() {
    }

    public static PoseObservation fromLimelight(LimelightHelpers.PoseEstimate estimate) {
        PoseObservation o = new PoseObservation();

        o.estimatedPose = estimate.pose != null ? estimate.pose : new Pose2d();
        o.isMegaTag2 = estimate.isMegaTag2;

        o.timestampSeconds = estimate.timestampSeconds;
        o.latency = estimate.latency;

        if (estimate.rawFiducials != null) {
            int count = estimate.rawFiducials.length;
            o.tagCount = count;

            if (o.fiducialIds == null || o.fiducialIds.length < count) {
                o.fiducialIds = new int[count];
            }

            for (int i = 0; i < count; i++) {
                o.fiducialIds[i] = estimate.rawFiducials[i].id;
            }
        }

        return o;
    }
}
