using System;
using System.Collections.Generic;
using AnimationTools;
using Unity.Mathematics;

namespace MotionMatching
{
/// <summary>
/// Translates between the legacy <see cref="PoseVector"/> representation and an
/// AnimationTools <see cref="PoseBuffer"/> built over a <see cref="SkeletonAsset"/>, via a
/// <see cref="SkeletonMap"/>. The buffer's layout is bone-aligned (element index == asset
/// bone index) in every section, which is required by <see cref="PoseFK"/>.
/// </summary>
public sealed class PoseVectorAdapter
{
    private readonly SkeletonAsset asset;

    public PoseLayout Layout { get; }
    public SkeletonMap Map { get; }

    public PoseVectorAdapter(SkeletonAsset asset, SkeletonMap map,
        IReadOnlyList<ChannelDescriptor> extraChannels = null)
    {
        if (asset == null) throw new ArgumentException("asset must not be null.", nameof(asset));
        if (map == null) throw new ArgumentException("map must not be null.", nameof(map));
        if (map.AssetToMm == null || map.AssetToMm.Length != asset.BoneCount)
            throw new ArgumentException(
                $"map.AssetToMm length ({map.AssetToMm?.Length ?? -1}) must match asset.BoneCount ({asset.BoneCount}).",
                nameof(map));
        if (map.MmToAsset == null)
            throw new ArgumentException("map.MmToAsset must not be null.", nameof(map));

        for (var i = 0; i < map.MmToAsset.Length; i++)
        {
            var assetIndex = map.MmToAsset[i];
            if (assetIndex < 0 || assetIndex >= asset.BoneCount)
                throw new ArgumentException(
                    $"map.MmToAsset[{i}] = {assetIndex} is out of range for asset bone count {asset.BoneCount}.",
                    nameof(map));
        }

        this.asset = asset;
        Map = map;
        Layout = PoseLayout.Build(asset, BuildChannels(asset, extraChannels));
    }

    private static List<ChannelDescriptor> BuildChannels(SkeletonAsset asset, IReadOnlyList<ChannelDescriptor> extraChannels)
    {
        var boneCount = asset.BoneCount;
        var channels = new List<ChannelDescriptor>(boneCount * 4 + (extraChannels?.Count ?? 0));

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

        if (extraChannels != null) channels.AddRange(extraChannels);

        return channels;
    }

    /// <summary>
    /// Copies position/rotation/velocity/angular-velocity from <paramref name="source"/> into
    /// <paramref name="destination"/>, per-bone via <see cref="SkeletonMap.AssetToMm"/>. Asset
    /// bones with no MM counterpart are initialized from the asset's rest pose instead of
    /// being left stale. Contact bools are not touched; consumers own their own bool channels.
    /// </summary>
    public void Write(in PoseVector source, PoseBuffer destination)
    {
        var positions = destination.Positions;
        var rotations = destination.Rotations;
        var velocities = destination.Velocities;
        var angularVelocities = destination.AngularVelocities;

        var boneCount = asset.BoneCount;
        for (var i = 0; i < boneCount; i++)
        {
            var mmIndex = Map.AssetToMm[i];
            if (mmIndex < 0)
            {
                var bone = asset.GetBone(i);
                positions[i] = bone.restLocalPosition;
                rotations[i] = bone.restLocalRotation;
                velocities[i] = float3.zero;
                angularVelocities[i] = float3.zero;
                continue;
            }

            positions[i] = source.jointLocalPositions[mmIndex];
            rotations[i] = source.jointLocalRotations[mmIndex];
            velocities[i] = source.jointLocalVelocities[mmIndex];
            angularVelocities[i] = source.jointLocalAngularVelocities[mmIndex];
        }
    }

    /// <summary>
    /// Inverse of <see cref="Write"/>: copies position/rotation/velocity/angular-velocity from
    /// <paramref name="source"/> into <paramref name="destination"/> for every asset bone that
    /// has an MM counterpart (via <see cref="SkeletonMap.AssetToMm"/>); bones without one are
    /// skipped. Leaves <paramref name="destination"/>'s contact bools untouched.
    /// </summary>
    public void Read(PoseBuffer source, ref PoseVector destination)
    {
        var positions = source.Positions;
        var rotations = source.Rotations;
        var velocities = source.Velocities;
        var angularVelocities = source.AngularVelocities;

        var boneCount = asset.BoneCount;
        for (var i = 0; i < boneCount; i++)
        {
            var mmIndex = Map.AssetToMm[i];
            if (mmIndex < 0) continue;

            destination.jointLocalPositions[mmIndex] = positions[i];
            destination.jointLocalRotations[mmIndex] = rotations[i];
            destination.jointLocalVelocities[mmIndex] = velocities[i];
            destination.jointLocalAngularVelocities[mmIndex] = angularVelocities[i];
        }
    }
}
}
