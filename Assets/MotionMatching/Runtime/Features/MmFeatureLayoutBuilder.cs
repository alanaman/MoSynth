using AnimationTools;

namespace MotionMatching
{
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
/// Turns a <see cref="MotionMatchingData"/> feature configuration into the layout its feature
/// vectors are stored in.
/// </summary>
public static class MmFeatureLayoutBuilder
{
    public static MmFeatureLayout Build(MotionMatchingData mmData, SkeletonAsset skeleton)
    {
        var trajectoryChannels = mmData.trajectoryFeatures.ToArray();
        var poseChannels = mmData.poseFeatures.ToArray();

        var boneIndices = new int[trajectoryChannels.Length + poseChannels.Length];

        for (var i = 0; i < trajectoryChannels.Length; i++)
        {
            var feature = trajectoryChannels[i];
            boneIndices[i] = feature.simulationBone ? 0 : skeleton.FindJointIndexOrZero(feature.bone);
        }

        for (var i = 0; i < poseChannels.Length; i++)
        {
            var feature = poseChannels[i];
            boneIndices[trajectoryChannels.Length + i] = skeleton.FindJointIndexOrZero(feature.bone);
        }

        return new MmFeatureLayout(trajectoryChannels, poseChannels, boneIndices);
    }
}
}
