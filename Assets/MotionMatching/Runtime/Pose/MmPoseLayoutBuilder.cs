using System.Collections.Generic;
using AnimationTools;
using UnityEngine;

namespace MotionMatching
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
public static class MmPoseLayoutBuilder
{
    public readonly struct ContactHandles
    {
        public readonly BoolHandle Left;
        public readonly BoolHandle Right;

        internal ContactHandles(BoolHandle left, BoolHandle right)
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
            channels.Add(new ChannelDescriptor
            {
                boneId = boneId, type = ChannelType.Position, space = ChannelSpace.ParentLocal,
                representation = RotationRepresentation.Quaternion, usage = ChannelUsage.Default
            });
            channels.Add(new ChannelDescriptor
            {
                boneId = boneId, type = ChannelType.Rotation, space = ChannelSpace.ParentLocal,
                representation = RotationRepresentation.Quaternion, usage = ChannelUsage.Default
            });
        }

        for (var i = 0; i < boneCount; i++)
        {
            var boneId = asset.GetBone(i).id;
            var isRoot = i == 0;
            channels.Add(new ChannelDescriptor
            {
                boneId = boneId, type = ChannelType.Velocity,
                space = isRoot ? ChannelSpace.RootLocal : ChannelSpace.ParentLocal,
                representation = RotationRepresentation.RotationVector,
                usage = isRoot ? ChannelUsage.RootMotion : ChannelUsage.Default
            });
        }

        for (var i = 0; i < boneCount; i++)
        {
            var boneId = asset.GetBone(i).id;
            var isRoot = i == 0;
            channels.Add(new ChannelDescriptor
            {
                boneId = boneId, type = ChannelType.AngularVelocity,
                space = isRoot ? ChannelSpace.RootLocal : ChannelSpace.ParentLocal,
                representation = RotationRepresentation.RotationVector,
                usage = isRoot ? ChannelUsage.RootMotion : ChannelUsage.Default
            });
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
        // The layout rejects duplicate (bone, type, usage) triples, so a skeleton missing one
        // side must still end up with two distinct host bones.
        if (rightContactBone == leftContactBone)
        {
            rightContactBone = leftContactBone == 0 ? asset.BoneCount - 1 : 0;
        }

        channels.Add(new ChannelDescriptor
        {
            boneId = asset.GetBone(leftContactBone).id, type = ChannelType.Bool, usage = ChannelUsage.Contact
        });
        channels.Add(new ChannelDescriptor
        {
            boneId = asset.GetBone(rightContactBone).id, type = ChannelType.Bool, usage = ChannelUsage.Contact
        });

        var layout = PoseLayout.Build(asset, channels);
        contacts = new ContactHandles(
            layout.BindBool(BoneRef(asset, leftContactBone), ChannelUsage.Contact),
            layout.BindBool(BoneRef(asset, rightContactBone), ChannelUsage.Contact));
        return layout;
    }

    private static BoneReference BoneRef(SkeletonAsset asset, int boneIndex)
    {
        var bone = asset.GetBone(boneIndex);
        return new BoneReference(bone.id, bone.name);
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
