using System.Collections.Generic;
using AnimationTools;
using UnityEngine;

namespace AnimationTools
{
/// <summary>
/// Builds the single pose layout the motion-matching runtime shares: a full pose
/// (parent-local position + rotation per bone, element index == bone index), per-bone
/// velocities/angular velocities with bone 0 carrying root motion in root-local space,
/// and two foot-contact Bool channels. <see cref="PoseSet"/> and
/// MotionSynthesisComponent both build through here over the same
/// <see cref="SkeletonAsset"/>, so <see cref="PoseLayout.Build"/>'s cache hands them the
/// same instance and frames can be copied between them (<see cref="PoseBuffer.CopyFrom"/>
/// requires equal layout hashes).
/// </summary>
    public static class PoseLayoutBuilder
{
    public readonly struct ContactHandles
    {
        public readonly ChannelHandle Left;
        public readonly ChannelHandle Right;

        internal ContactHandles(ChannelHandle left, ChannelHandle right)
        {
            Left = left;
            Right = right;
        }
    }

    /// <summary>
    /// The full-pose channel set without the contact bools, for consumers that append their
    /// own extra channels before calling <see cref="PoseLayout.Build"/> themselves.
    /// </summary>
    public static List<ChannelDescriptor> BuildFullPoseChannels(SkeletonAsset asset)
    {
        var boneCount = asset.BoneCount;
        var channels = new List<ChannelDescriptor>(boneCount * 4 + 2);

        for (var i = 0; i < boneCount; i++)
        {
            var boneId = asset.GetBone(i).id;
            channels.Add(new PositionChannel(boneId));
            channels.Add(new RotationChannel(boneId));
        }

        for (var i = 0; i < boneCount; i++)
        {
            var boneId = asset.GetBone(i).id;
            var isRoot = i == 0;
            channels.Add(new VelocityChannel(boneId,
                isRoot ? ChannelSpace.RootLocal : ChannelSpace.ParentLocal,
                isRoot ? ChannelUsage.RootMotion : ChannelUsage.Default));
        }

        for (var i = 0; i < boneCount; i++)
        {
            var boneId = asset.GetBone(i).id;
            var isRoot = i == 0;
            channels.Add(new AngularVelocityChannel(boneId,
                isRoot ? ChannelSpace.RootLocal : ChannelSpace.ParentLocal,
                isRoot ? ChannelUsage.RootMotion : ChannelUsage.Default));
        }

        return channels;
    }

    /// <summary>
    /// The authoritative motion-matching pose layout: full-pose channels plus the two
    /// foot-contact Bool channels, with their handles bound.
    /// </summary>
    public static PoseLayout Build(SkeletonAsset asset, out ContactHandles contacts)
    {
        var channels = BuildFullPoseChannels(asset);

        var leftContactBone = PickContactBoneIndex(asset, HumanBodyBones.LeftToes, HumanBodyBones.LeftFoot, 0);
        var rightContactBone = PickContactBoneIndex(asset, HumanBodyBones.RightToes, HumanBodyBones.RightFoot,
            asset.BoneCount - 1);
        // The layout rejects duplicate channel identities, so a skeleton missing one side must
        // still end up with two distinct host bones.
        if (rightContactBone == leftContactBone)
        {
            rightContactBone = leftContactBone == 0 ? asset.BoneCount - 1 : 0;
        }

        channels.Add(new BoolChannel(asset.GetBone(leftContactBone).id, ChannelUsage.Contact));
        channels.Add(new BoolChannel(asset.GetBone(rightContactBone).id, ChannelUsage.Contact));

        var layout = PoseLayout.Build(asset, channels);
        contacts = new ContactHandles(
            layout.BindChannel(new BoolChannel(asset.GetBone(leftContactBone).id, ChannelUsage.Contact)),
            layout.BindChannel(new BoolChannel(asset.GetBone(rightContactBone).id, ChannelUsage.Contact)));
        return layout;
    }

    private static int PickContactBoneIndex(SkeletonAsset asset, HumanBodyBones toes, HumanBodyBones foot,
        int fallbackIndex)
    {
        if (asset.TryFindByHumanBone(toes, out var index)) return index;
        if (asset.TryFindByHumanBone(foot, out index)) return index;
        return fallbackIndex;
    }
}
}
