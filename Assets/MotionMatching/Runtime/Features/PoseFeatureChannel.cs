using System;
using AnimationTools;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Serialization;

namespace MotionMatching
{
/// <summary>
/// A joint of the current pose, expressed in the character frame. Always three floats.
/// </summary>
/// <remarks>
/// This is both the authored definition and the layout channel. <see cref="PoseLayout"/> keys its
/// offset table on the descriptor, so rebuild the layout after editing a feature rather than
/// mutating one a built layout already holds.
/// </remarks>
[Serializable]
public sealed class PoseFeatureChannel : ChannelDescriptor, IMatchingFeature
{
    public enum Type
    {
        Position,
        Velocity
    }

    [FormerlySerializedAs("Name")] public string name;
    [FormerlySerializedAs("FeatureType")] public Type featureType;
    [FormerlySerializedAs("Bone")] public HumanBodyBones bone;

    public string Name => name;

    public override int FloatCount => 3;

    public override int SectionKey => FeatureSections.Pose;

    public void Extract(in FeatureExtractionContext context, int boneIndex, ChannelHandle handle, StateBuffer frame)
    {
        var value = float3.zero;
        switch (featureType)
        {
            case Type.Position:
                value = context.JointPositionLocal(context.CurrentPose, boneIndex);
                break;
            case Type.Velocity:
                value = context.JointVelocityLocal(boneIndex);
                break;
            default:
                Debug.Assert(false, "Unknown PoseFeatureChannel.Type: " + featureType);
                break;
        }

        frame.SetFloat3(handle, value);
    }

    public override bool Equals(ChannelDescriptor other)
    {
        return other is PoseFeatureChannel channel
               && channel.featureType == featureType
               && channel.bone == bone;
    }

    public override int GetHashCode()
    {
        unchecked
        {
            var hash = typeof(PoseFeatureChannel).GetHashCode();
            hash = hash * 31 + (int)featureType;
            hash = hash * 31 + (int)bone;
            return hash;
        }
    }

    public override int GetContentHash() => GetHashCode();
}
}
