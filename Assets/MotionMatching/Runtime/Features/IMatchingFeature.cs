using AnimationTools;

namespace MotionMatching
{
/// <summary>
/// One feature of a Motion Matching query vector. Implementations are <see cref="ChannelDescriptor"/>s,
/// so their width and placement come from the layout; this adds the part that is specific to
/// matching, namely filling the feature in from a pose.
/// </summary>
public interface IMatchingFeature
{
    /// <summary>Authored name, used to address the feature from control inputs and the databases.</summary>
    string Name { get; }

    /// <summary>Writes this feature's floats for one pose into <paramref name="frame"/>.</summary>
    void Extract(in FeatureExtractionContext context, int boneIndex, ChannelHandle handle, StateBuffer frame);
}
}
