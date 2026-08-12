using System;
using System.Collections.Generic;

namespace AnimationTools
{
/// <summary>
/// Unmanaged, Burst-embeddable section offsets/counts for a <see cref="PoseLayout"/>.
/// Starts are in floats, counts are in elements (e.g. one Position element is 3 floats).
/// </summary>
public struct PoseLayoutData
{
    public int PositionStart, PositionCount;
    public int RotationStart, RotationCount;
    /// <summary>4 for Quaternion, 3 for RotationVector/EulerDegrees.</summary>
    public byte RotationStride;
    public int ScaleStart, ScaleCount;
    public int VelocityStart, VelocityCount;
    public int AngularVelocityStart, AngularVelocityCount;
    /// <summary>Bools are stored as one float each (0f/1f).</summary>
    public int BoolStart, BoolCount;
    public int FloatCount;
    public int LayoutHash;
}

/// <summary>
/// An immutable, self-describing map from named/typed pose channels to positions within a
/// flat float buffer (see <see cref="PoseBuffer"/>). This layout IS the pose file's
/// self-description: a serialized pose asset is expected to store enough of it to
/// reconstruct a <see cref="PoseLayout"/> without external context.
/// </summary>
/// <remarks>
/// Planned file format for later serialization work (not implemented by this layer):
/// <c>magic 'MSPS' | int version (PoseFormatVersion) | skeleton table {id, name,
/// parentIndex, restLocalPosition, restLocalRotation, humanBone} | channel table |
/// LayoutHash | frame count | raw float frames</c>.
/// <para/>
/// Section order within the buffer is fixed: positions, rotations, scales, velocities,
/// angular velocities, bools. Strides: Position/Scale/Velocity/AngularVelocity are 3
/// floats; Rotation is 4 floats (Quaternion) or 3 (RotationVector/EulerDegrees); Bool is
/// 1 float. Within a section, elements keep the declaration order of their channel
/// descriptors as passed to <see cref="Build"/>.
/// </remarks>
public sealed class PoseLayout
{
    public const int PoseFormatVersion = 1;

    private static readonly Dictionary<int, PoseLayout> Cache = new();

    private readonly ChannelDescriptor[] positionChannels;
    private readonly ChannelDescriptor[] rotationChannels;
    private readonly ChannelDescriptor[] scaleChannels;
    private readonly ChannelDescriptor[] velocityChannels;
    private readonly ChannelDescriptor[] angularVelocityChannels;
    private readonly ChannelDescriptor[] boolChannels;

    public SkeletonAsset Skeleton { get; }
    public IReadOnlyList<ChannelDescriptor> Channels { get; }
    public PoseLayoutData Data { get; }
    public int FloatCount => Data.FloatCount;
    public int LayoutHash => Data.LayoutHash;

    /// <summary>The shared representation of every Rotation channel in this layout.</summary>
    public RotationRepresentation RotationFormat { get; }

    private PoseLayout(SkeletonAsset skeleton, ChannelDescriptor[] channels, PoseLayoutData data,
        RotationRepresentation rotationFormat, ChannelDescriptor[] positionChannels,
        ChannelDescriptor[] rotationChannels, ChannelDescriptor[] scaleChannels,
        ChannelDescriptor[] velocityChannels, ChannelDescriptor[] angularVelocityChannels,
        ChannelDescriptor[] boolChannels)
    {
        Skeleton = skeleton;
        Channels = channels;
        Data = data;
        RotationFormat = rotationFormat;
        this.positionChannels = positionChannels;
        this.rotationChannels = rotationChannels;
        this.scaleChannels = scaleChannels;
        this.velocityChannels = velocityChannels;
        this.angularVelocityChannels = angularVelocityChannels;
        this.boolChannels = boolChannels;
    }

    /// <summary>
    /// Builds (or returns the cached) layout for a channel set. Throws
    /// <see cref="ArgumentException"/> if any boneId is not present in
    /// <paramref name="skeleton"/>, if a (boneId, type, usage) triple repeats, if Rotation
    /// channels don't share one representation, or if an AngularVelocity channel isn't
    /// RotationVector.
    /// </summary>
    public static PoseLayout Build(SkeletonAsset skeleton, IReadOnlyList<ChannelDescriptor> channels)
    {
        if (skeleton == null) throw new ArgumentException("skeleton must not be null.", nameof(skeleton));
        if (channels == null) throw new ArgumentException("channels must not be null.", nameof(channels));

        var seenTriples = new HashSet<(int boneId, ChannelType type, ChannelUsage usage)>();
        RotationRepresentation? rotationFormat = null;

        for (var i = 0; i < channels.Count; i++)
        {
            var channel = channels[i];

            if (skeleton.IndexOfId(channel.boneId) < 0)
                throw new ArgumentException($"Channel {i} references bone id {channel.boneId}, which is not present in skeleton \"{skeleton.name}\".", nameof(channels));

            var triple = (channel.boneId, channel.type, channel.usage);
            if (!seenTriples.Add(triple))
                throw new ArgumentException($"Duplicate channel for bone id {channel.boneId}, type {channel.type}, usage {channel.usage}.", nameof(channels));

            if (channel.type == ChannelType.Rotation)
            {
                if (rotationFormat == null) rotationFormat = channel.representation;
                else if (rotationFormat != channel.representation)
                    throw new ArgumentException($"Mixed rotation representations in one layout: {rotationFormat} and {channel.representation}. All Rotation channels must share one representation.", nameof(channels));
            }
            else if (channel.type == ChannelType.AngularVelocity && channel.representation != RotationRepresentation.RotationVector)
            {
                throw new ArgumentException($"AngularVelocity channel for bone id {channel.boneId} has representation {channel.representation}; only RotationVector is valid.", nameof(channels));
            }
        }

        var hash = ComputeHash(skeleton, channels);
        var cacheKey = CombineKey(skeleton.GetEntityId().GetHashCode(), hash);
        if (Cache.TryGetValue(cacheKey, out var cached)) return cached;

        var positions = new List<ChannelDescriptor>();
        var rotations = new List<ChannelDescriptor>();
        var scales = new List<ChannelDescriptor>();
        var velocities = new List<ChannelDescriptor>();
        var angularVelocities = new List<ChannelDescriptor>();
        var bools = new List<ChannelDescriptor>();
        var channelsCopy = new ChannelDescriptor[channels.Count];

        for (var i = 0; i < channels.Count; i++)
        {
            var channel = channels[i];
            channelsCopy[i] = channel;
            switch (channel.type)
            {
                case ChannelType.Position: positions.Add(channel); break;
                case ChannelType.Rotation: rotations.Add(channel); break;
                case ChannelType.Scale: scales.Add(channel); break;
                case ChannelType.Velocity: velocities.Add(channel); break;
                case ChannelType.AngularVelocity: angularVelocities.Add(channel); break;
                case ChannelType.Bool: bools.Add(channel); break;
                default: throw new ArgumentOutOfRangeException(nameof(channels), channel.type, "Unknown channel type.");
            }
        }

        var resolvedRotationFormat = rotationFormat ?? RotationRepresentation.Quaternion;
        var rotationStride = (byte)(resolvedRotationFormat == RotationRepresentation.Quaternion ? 4 : 3);

        var cursor = 0;
        var data = new PoseLayoutData();

        data.PositionStart = cursor; data.PositionCount = positions.Count; cursor += data.PositionCount * 3;
        data.RotationStart = cursor; data.RotationCount = rotations.Count; data.RotationStride = rotationStride; cursor += data.RotationCount * rotationStride;
        data.ScaleStart = cursor; data.ScaleCount = scales.Count; cursor += data.ScaleCount * 3;
        data.VelocityStart = cursor; data.VelocityCount = velocities.Count; cursor += data.VelocityCount * 3;
        data.AngularVelocityStart = cursor; data.AngularVelocityCount = angularVelocities.Count; cursor += data.AngularVelocityCount * 3;
        data.BoolStart = cursor; data.BoolCount = bools.Count; cursor += data.BoolCount;
        data.FloatCount = cursor;
        data.LayoutHash = hash;

        var layout = new PoseLayout(skeleton, channelsCopy, data, resolvedRotationFormat,
            positions.ToArray(), rotations.ToArray(), scales.ToArray(), velocities.ToArray(),
            angularVelocities.ToArray(), bools.ToArray());

        Cache[cacheKey] = layout;
        return layout;
    }

    /// <summary>
    /// One ParentLocal Position + one ParentLocal Quaternion Rotation channel per bone, in
    /// skeleton DFS order, so element index equals bone index. Optionally adds per-bone
    /// ParentLocal RotationVector Velocity/AngularVelocity channels, likewise bone-index
    /// aligned. All channels use <see cref="ChannelUsage.Default"/>.
    /// </summary>
    public static PoseLayout CreateFullPose(SkeletonAsset skeleton, bool includeVelocities, bool includeAngularVelocities)
    {
        if (skeleton == null) throw new ArgumentException("skeleton must not be null.", nameof(skeleton));

        var boneCount = skeleton.BoneCount;
        var channels = new List<ChannelDescriptor>(boneCount * 2);

        for (var i = 0; i < boneCount; i++)
        {
            var boneId = skeleton.GetBone(i).id;
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

        if (includeVelocities)
        {
            for (var i = 0; i < boneCount; i++)
            {
                channels.Add(new ChannelDescriptor
                {
                    boneId = skeleton.GetBone(i).id, type = ChannelType.Velocity, space = ChannelSpace.ParentLocal,
                    representation = RotationRepresentation.RotationVector, usage = ChannelUsage.Default
                });
            }
        }

        if (includeAngularVelocities)
        {
            for (var i = 0; i < boneCount; i++)
            {
                channels.Add(new ChannelDescriptor
                {
                    boneId = skeleton.GetBone(i).id, type = ChannelType.AngularVelocity, space = ChannelSpace.ParentLocal,
                    representation = RotationRepresentation.RotationVector, usage = ChannelUsage.Default
                });
            }
        }

        return Build(skeleton, channels);
    }

    public bool TryBindPosition(BoneReference bone, out PositionHandle handle, ChannelUsage usage = ChannelUsage.Default)
    {
        var index = FindElementIndex(positionChannels, bone, usage);
        handle = index >= 0 ? new PositionHandle(index) : PositionHandle.Invalid;
        return index >= 0;
    }

    public PositionHandle BindPosition(BoneReference bone, ChannelUsage usage = ChannelUsage.Default)
    {
        if (!TryBindPosition(bone, out var handle, usage))
            throw new ArgumentException($"No Position channel with usage {usage} bound to bone id {bone.BoneId}.");
        return handle;
    }

    public bool TryBindRotation(BoneReference bone, out RotationHandle handle, ChannelUsage usage = ChannelUsage.Default)
    {
        var index = FindElementIndex(rotationChannels, bone, usage);
        handle = index >= 0 ? new RotationHandle(index) : RotationHandle.Invalid;
        return index >= 0;
    }

    public RotationHandle BindRotation(BoneReference bone, ChannelUsage usage = ChannelUsage.Default)
    {
        if (!TryBindRotation(bone, out var handle, usage))
            throw new ArgumentException($"No Rotation channel with usage {usage} bound to bone id {bone.BoneId}.");
        return handle;
    }

    public bool TryBindScale(BoneReference bone, out ScaleHandle handle, ChannelUsage usage = ChannelUsage.Default)
    {
        var index = FindElementIndex(scaleChannels, bone, usage);
        handle = index >= 0 ? new ScaleHandle(index) : ScaleHandle.Invalid;
        return index >= 0;
    }

    public ScaleHandle BindScale(BoneReference bone, ChannelUsage usage = ChannelUsage.Default)
    {
        if (!TryBindScale(bone, out var handle, usage))
            throw new ArgumentException($"No Scale channel with usage {usage} bound to bone id {bone.BoneId}.");
        return handle;
    }

    public bool TryBindBool(BoneReference bone, out BoolHandle handle, ChannelUsage usage = ChannelUsage.Default)
    {
        var index = FindElementIndex(boolChannels, bone, usage);
        handle = index >= 0 ? new BoolHandle(index) : BoolHandle.Invalid;
        return index >= 0;
    }

    public BoolHandle BindBool(BoneReference bone, ChannelUsage usage = ChannelUsage.Default)
    {
        if (!TryBindBool(bone, out var handle, usage))
            throw new ArgumentException($"No Bool channel with usage {usage} bound to bone id {bone.BoneId}.");
        return handle;
    }

    public bool TryBindVelocity(BoneReference bone, out VelocityHandle handle, ChannelUsage usage = ChannelUsage.Default)
    {
        var index = FindElementIndex(velocityChannels, bone, usage);
        handle = index >= 0 ? new VelocityHandle(index) : VelocityHandle.Invalid;
        return index >= 0;
    }

    public VelocityHandle BindVelocity(BoneReference bone, ChannelUsage usage = ChannelUsage.Default)
    {
        if (!TryBindVelocity(bone, out var handle, usage))
            throw new ArgumentException($"No Velocity channel with usage {usage} bound to bone id {bone.BoneId}.");
        return handle;
    }

    public bool TryBindAngularVelocity(BoneReference bone, out AngularVelocityHandle handle, ChannelUsage usage = ChannelUsage.Default)
    {
        var index = FindElementIndex(angularVelocityChannels, bone, usage);
        handle = index >= 0 ? new AngularVelocityHandle(index) : AngularVelocityHandle.Invalid;
        return index >= 0;
    }

    public AngularVelocityHandle BindAngularVelocity(BoneReference bone, ChannelUsage usage = ChannelUsage.Default)
    {
        if (!TryBindAngularVelocity(bone, out var handle, usage))
            throw new ArgumentException($"No AngularVelocity channel with usage {usage} bound to bone id {bone.BoneId}.");
        return handle;
    }

    /// <summary>Reverse lookup for tooling/debugging: the descriptor behind a raw section element index.</summary>
    public ChannelDescriptor GetChannel(ChannelType type, int elementIndex)
    {
        return GetSection(type)[elementIndex];
    }

    private ChannelDescriptor[] GetSection(ChannelType type)
    {
        switch (type)
        {
            case ChannelType.Position: return positionChannels;
            case ChannelType.Rotation: return rotationChannels;
            case ChannelType.Scale: return scaleChannels;
            case ChannelType.Velocity: return velocityChannels;
            case ChannelType.AngularVelocity: return angularVelocityChannels;
            case ChannelType.Bool: return boolChannels;
            default: throw new ArgumentOutOfRangeException(nameof(type), type, "Unknown channel type.");
        }
    }

    private static int FindElementIndex(ChannelDescriptor[] descriptors, BoneReference bone, ChannelUsage usage)
    {
        if (!bone.IsSet) return -1;

        for (var i = 0; i < descriptors.Length; i++)
        {
            if (descriptors[i].boneId == bone.BoneId && descriptors[i].usage == usage) return i;
        }

        return -1;
    }

    private static int ComputeHash(SkeletonAsset skeleton, IReadOnlyList<ChannelDescriptor> channels)
    {
        unchecked
        {
            var hash = 17;
            hash = hash * 31 + skeleton.Version;

            for (var i = 0; i < skeleton.BoneCount; i++)
            {
                hash = hash * 31 + skeleton.GetBone(i).id;
            }

            for (var i = 0; i < channels.Count; i++)
            {
                hash = hash * 31 + channels[i].GetHashCode();
            }

            return hash;
        }
    }

    private static int CombineKey(int skeletonInstanceId, int layoutHash)
    {
        unchecked
        {
            return skeletonInstanceId * 397 ^ layoutHash;
        }
    }
}
}
