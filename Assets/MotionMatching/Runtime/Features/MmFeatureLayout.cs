using AnimationTools;

namespace MotionMatching
{
/// <summary>
/// A layout for a motion-matching feature vector, together with the features and handles that
/// address it, so callers never recompute feature offsets by hand.
/// </summary>
public sealed class MmFeatureLayout : StateBufferLayout
{
    /// <summary>The authored trajectory features, in feature-vector order.</summary>
    public TrajectoryFeatureChannel[] TrajectoryChannels { get; }

    /// <summary>One handle per trajectory feature, spanning all of its predictions.</summary>
    public ChannelHandle[] TrajectoryHandles { get; }

    public PoseFeatureChannel[] PoseChannels { get; }
    public ChannelHandle[] PoseHandles { get; }

    /// <summary>
    /// Skeleton joint each feature reads, resolved against the skeleton this layout was built for.
    /// Trajectory features come first, matching <see cref="Features"/>.
    /// </summary>
    public int[] BoneIndices { get; }

    /// <summary>Every feature in feature-vector order: trajectory features, then pose features.</summary>
    public IMatchingFeature[] Features { get; }

    /// <summary>Handles matching <see cref="Features"/> element for element.</summary>
    public ChannelHandle[] Handles { get; }

    /// <summary>Float offset where the pose features start, i.e. the end of the trajectory block.</summary>
    public int PoseStart { get; }

    internal MmFeatureLayout(TrajectoryFeatureChannel[] trajectoryChannels, PoseFeatureChannel[] poseChannels,
        int[] boneIndices)
        : this(ConcatChannels(trajectoryChannels, poseChannels), trajectoryChannels, poseChannels, boneIndices)
    {
    }

    private MmFeatureLayout(ChannelDescriptor[] channels, TrajectoryFeatureChannel[] trajectoryChannels,
        PoseFeatureChannel[] poseChannels, int[] boneIndices)
        : base(channels, HashChannels(17, channels))
    {
        TrajectoryChannels = trajectoryChannels;
        PoseChannels = poseChannels;
        BoneIndices = boneIndices;

        TrajectoryHandles = new ChannelHandle[trajectoryChannels.Length];
        for (var i = 0; i < trajectoryChannels.Length; i++)
        {
            TrajectoryHandles[i] = BindChannel(trajectoryChannels[i]);
        }

        PoseHandles = new ChannelHandle[poseChannels.Length];
        for (var i = 0; i < poseChannels.Length; i++)
        {
            PoseHandles[i] = BindChannel(poseChannels[i]);
        }

        Features = new IMatchingFeature[trajectoryChannels.Length + poseChannels.Length];
        Handles = new ChannelHandle[Features.Length];
        for (var i = 0; i < trajectoryChannels.Length; i++)
        {
            Features[i] = trajectoryChannels[i];
            Handles[i] = TrajectoryHandles[i];
        }

        for (var i = 0; i < poseChannels.Length; i++)
        {
            Features[trajectoryChannels.Length + i] = poseChannels[i];
            Handles[trajectoryChannels.Length + i] = PoseHandles[i];
        }

        PoseStart = TryGetSectionRange(FeatureSections.Pose, out var start, out _) ? start : FloatCount;
    }

    private static ChannelDescriptor[] ConcatChannels(TrajectoryFeatureChannel[] trajectoryChannels,
        PoseFeatureChannel[] poseChannels)
    {
        var channels = new ChannelDescriptor[trajectoryChannels.Length + poseChannels.Length];
        for (var i = 0; i < trajectoryChannels.Length; i++)
        {
            channels[i] = trajectoryChannels[i];
        }

        for (var i = 0; i < poseChannels.Length; i++)
        {
            channels[trajectoryChannels.Length + i] = poseChannels[i];
        }

        return channels;
    }
}
}