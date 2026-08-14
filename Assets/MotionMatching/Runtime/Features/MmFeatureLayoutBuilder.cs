using System.Collections.Generic;
using AnimationTools;

namespace MotionMatching
{
using TrajectoryFeature = MotionMatchingData.TrajectoryFeature;
using PoseFeature = MotionMatchingData.PoseFeature;

/// <summary>
/// Section keys for a motion-matching feature vector. They sit after every built-in pose section,
/// so trajectory floats come first, then pose floats, and the boundary between them is the
/// pose-section start.
/// </summary>
public static class FeatureSections
{
    public const int Trajectory = ChannelSections.Bool + 1;
    public const int Pose = ChannelSections.Bool + 2;
}

/// <summary>
/// One prediction of one trajectory feature. Identity is (feature, prediction) because that pair
/// is what addresses a slot; the bone and axis mask describe how the slot is filled.
/// </summary>
public sealed class TrajectoryFeatureChannel : ChannelDescriptor
{
    public int FeatureIndex { get; }
    public int PredictionIndex { get; }
    public TrajectoryFeature.Type FeatureType { get; }
    public int BoneIndex { get; }
    public bool SimulationBone { get; }
    public bool ZeroX { get; }
    public bool ZeroY { get; }
    public bool ZeroZ { get; }

    public TrajectoryFeatureChannel(int featureIndex, int predictionIndex, TrajectoryFeature.Type featureType,
        int boneIndex, bool simulationBone, bool zeroX, bool zeroY, bool zeroZ)
    {
        FeatureIndex = featureIndex;
        PredictionIndex = predictionIndex;
        FeatureType = featureType;
        BoneIndex = boneIndex;
        SimulationBone = simulationBone;
        ZeroX = zeroX;
        ZeroY = zeroY;
        ZeroZ = zeroZ;
    }

    public override int FloatCount => 3 - (ZeroX ? 1 : 0) - (ZeroY ? 1 : 0) - (ZeroZ ? 1 : 0);
    public override int SectionKey => FeatureSections.Trajectory;

    public override bool Equals(ChannelDescriptor other)
    {
        return other is TrajectoryFeatureChannel channel
               && channel.FeatureIndex == FeatureIndex
               && channel.PredictionIndex == PredictionIndex;
    }

    public override int GetHashCode()
    {
        unchecked
        {
            var hash = typeof(TrajectoryFeatureChannel).GetHashCode();
            hash = hash * 31 + FeatureIndex;
            hash = hash * 31 + PredictionIndex;
            return hash;
        }
    }

    public override int GetContentHash()
    {
        unchecked
        {
            var hash = GetHashCode();
            hash = hash * 31 + (int)FeatureType;
            hash = hash * 31 + BoneIndex;
            hash = hash * 31 + (SimulationBone ? 1 : 0);
            hash = hash * 31 + (ZeroX ? 1 : 0);
            hash = hash * 31 + (ZeroY ? 1 : 0);
            hash = hash * 31 + (ZeroZ ? 1 : 0);
            return hash;
        }
    }
}

/// <summary>One pose feature, always three floats.</summary>
public sealed class PoseFeatureChannel : ChannelDescriptor
{
    public int FeatureIndex { get; }
    public PoseFeature.Type FeatureType { get; }
    public int BoneIndex { get; }

    public PoseFeatureChannel(int featureIndex, PoseFeature.Type featureType, int boneIndex)
    {
        FeatureIndex = featureIndex;
        FeatureType = featureType;
        BoneIndex = boneIndex;
    }

    public override int FloatCount => 3;
    public override int SectionKey => FeatureSections.Pose;

    public override bool Equals(ChannelDescriptor other)
    {
        return other is PoseFeatureChannel channel && channel.FeatureIndex == FeatureIndex;
    }

    public override int GetHashCode()
    {
        unchecked
        {
            return typeof(PoseFeatureChannel).GetHashCode() * 31 + FeatureIndex;
        }
    }

    public override int GetContentHash()
    {
        unchecked
        {
            var hash = GetHashCode();
            hash = hash * 31 + (int)FeatureType;
            hash = hash * 31 + BoneIndex;
            return hash;
        }
    }
}

/// <summary>
/// A built feature layout together with the channels and handles that address it, so callers never
/// recompute feature offsets by hand.
/// </summary>
public sealed class MmFeatureLayout
{
    public PoseLayout Layout { get; }

    /// <summary>Indexed [trajectoryFeatureIndex][predictionIndex].</summary>
    public TrajectoryFeatureChannel[][] TrajectoryChannels { get; }

    public ChannelHandle[][] TrajectoryHandles { get; }
    public PoseFeatureChannel[] PoseChannels { get; }
    public ChannelHandle[] PoseHandles { get; }

    /// <summary>Float offset where the pose features start, i.e. the end of the trajectory block.</summary>
    public int PoseStart { get; }

    public int FloatCount => Layout.FloatCount;

    internal MmFeatureLayout(PoseLayout layout, TrajectoryFeatureChannel[][] trajectoryChannels,
        ChannelHandle[][] trajectoryHandles, PoseFeatureChannel[] poseChannels, ChannelHandle[] poseHandles,
        int poseStart)
    {
        Layout = layout;
        TrajectoryChannels = trajectoryChannels;
        TrajectoryHandles = trajectoryHandles;
        PoseChannels = poseChannels;
        PoseHandles = poseHandles;
        PoseStart = poseStart;
    }
}

/// <summary>
/// Turns a <see cref="MotionMatchingData"/> feature configuration into the layout its feature
/// vectors are stored in.
/// </summary>
public static class MmFeatureLayoutBuilder
{
    public static MmFeatureLayout Build(MotionMatchingData mmData, SkeletonAsset skeleton)
    {
        var channels = new List<ChannelDescriptor>();

        var trajectoryChannels = new TrajectoryFeatureChannel[mmData.trajectoryFeatures.Count][];
        for (var i = 0; i < mmData.trajectoryFeatures.Count; i++)
        {
            var feature = mmData.trajectoryFeatures[i];
            var boneIndex = feature.simulationBone ? 0 : skeleton.FindJointIndexOrZero(feature.bone);
            var perPrediction = new TrajectoryFeatureChannel[feature.framesPrediction.Length];

            for (var p = 0; p < feature.framesPrediction.Length; p++)
            {
                var channel = new TrajectoryFeatureChannel(i, p, feature.featureType, boneIndex,
                    feature.simulationBone, feature.zeroX, feature.zeroY, feature.zeroZ);
                perPrediction[p] = channel;
                channels.Add(channel);
            }

            trajectoryChannels[i] = perPrediction;
        }

        var poseChannels = new PoseFeatureChannel[mmData.poseFeatures.Count];
        for (var i = 0; i < mmData.poseFeatures.Count; i++)
        {
            var feature = mmData.poseFeatures[i];
            var channel = new PoseFeatureChannel(i, feature.featureType,
                skeleton.FindJointIndexOrZero(feature.bone));
            poseChannels[i] = channel;
            channels.Add(channel);
        }

        var layout = PoseLayout.Build(skeleton, channels);

        var trajectoryHandles = new ChannelHandle[trajectoryChannels.Length][];
        for (var i = 0; i < trajectoryChannels.Length; i++)
        {
            trajectoryHandles[i] = new ChannelHandle[trajectoryChannels[i].Length];
            for (var p = 0; p < trajectoryChannels[i].Length; p++)
            {
                trajectoryHandles[i][p] = layout.BindChannel(trajectoryChannels[i][p]);
            }
        }

        var poseHandles = new ChannelHandle[poseChannels.Length];
        for (var i = 0; i < poseChannels.Length; i++)
        {
            poseHandles[i] = layout.BindChannel(poseChannels[i]);
        }

        var poseStart = layout.TryGetSectionRange(FeatureSections.Pose, out var start, out _)
            ? start
            : layout.FloatCount;

        return new MmFeatureLayout(layout, trajectoryChannels, trajectoryHandles, poseChannels, poseHandles,
            poseStart);
    }
}
}
