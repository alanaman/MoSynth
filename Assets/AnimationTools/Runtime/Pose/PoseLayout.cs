using System;
using System.Collections.Generic;

namespace AnimationTools
{
/// <summary>
/// Unmanaged, Burst-embeddable section offsets/counts for a <see cref="PoseLayout"/>.
/// Starts are in floats, counts are in elements (e.g. one Position element is 3 floats).
/// </summary>
/// <remarks>
/// The named sections describe only the six built-in channel kinds. A layout may hold further
/// sections after them (see <see cref="ChannelSections"/>); <see cref="ExtraStart"/> and
/// <see cref="ExtraCount"/> cover that tail in floats, so nothing here assumes the named sections
/// span the whole buffer.
/// </remarks>
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
    /// <summary>Floats belonging to sections beyond the six built-in kinds.</summary>
    public int ExtraStart, ExtraCount;
    public int FloatCount;
    public int LayoutHash;
}

/// <summary>
/// An immutable, self-describing map from typed pose channels to positions within a flat float
/// buffer (see <see cref="PoseBuffer"/>). This layout IS the pose file's self-description: a
/// serialized pose asset is expected to store enough of it to reconstruct a
/// <see cref="PoseLayout"/> without external context.
/// </summary>
/// <remarks>
/// Sections are laid out in ascending <see cref="ChannelDescriptor.SectionKey"/> order, and within
/// a section channels keep the declaration order they were passed to <see cref="Build"/> in. Each
/// channel occupies its own <see cref="ChannelDescriptor.FloatCount"/>, so a section is only
/// uniform-stride when its channels are.
/// <para/>
/// <c>Bind*</c> allocates a probe descriptor per call and is meant for setup, not per-frame use.
/// </remarks>
public sealed class PoseLayout
{
    public const int PoseFormatVersion = 1;

    private static readonly Dictionary<int, PoseLayout> Cache = new();

    private readonly struct ChannelSlot
    {
        public readonly int Offset;
        public readonly int FloatCount;

        public ChannelSlot(int offset, int floatCount)
        {
            Offset = offset;
            FloatCount = floatCount;
        }
    }

    private readonly struct SectionRange
    {
        public readonly int Start;
        public readonly int FloatCount;

        public SectionRange(int start, int floatCount)
        {
            Start = start;
            FloatCount = floatCount;
        }
    }

    private readonly Dictionary<ChannelDescriptor, ChannelSlot> _offsets;
    private readonly Dictionary<int, ChannelDescriptor[]> _sections;
    private readonly Dictionary<int, SectionRange> _sectionRanges;

    public SkeletonAsset Skeleton { get; }
    public IReadOnlyList<ChannelDescriptor> Channels { get; }
    public PoseLayoutData Data { get; }
    public int FloatCount => Data.FloatCount;
    public int LayoutHash => Data.LayoutHash;

    /// <summary>The shared representation of every Rotation channel in this layout.</summary>
    public RotationRepresentation RotationFormat { get; }

    private PoseLayout(SkeletonAsset skeleton, ChannelDescriptor[] channels, PoseLayoutData data,
        RotationRepresentation rotationFormat, Dictionary<ChannelDescriptor, ChannelSlot> offsets,
        Dictionary<int, ChannelDescriptor[]> sections, Dictionary<int, SectionRange> sectionRanges)
    {
        Skeleton = skeleton;
        Channels = channels;
        Data = data;
        RotationFormat = rotationFormat;
        _offsets = offsets;
        _sections = sections;
        _sectionRanges = sectionRanges;
    }

    /// <summary>
    /// Builds (or returns the cached) layout for a channel set. Throws
    /// <see cref="ArgumentException"/> if any boneId is not present in
    /// <paramref name="skeleton"/>, if a channel identity repeats, if Rotation channels don't
    /// share one representation, or if a non-built-in channel claims a built-in section key.
    /// </summary>
    public static PoseLayout Build(SkeletonAsset skeleton, IReadOnlyList<ChannelDescriptor> channels)
    {
        if (skeleton == null) throw new ArgumentException("skeleton must not be null.", nameof(skeleton));
        if (channels == null) throw new ArgumentException("channels must not be null.", nameof(channels));

        var seen = new HashSet<ChannelDescriptor>();
        RotationRepresentation? rotationFormat = null;

        for (var i = 0; i < channels.Count; i++)
        {
            var channel = channels[i];
            if (channel == null) throw new ArgumentException($"Channel {i} is null.", nameof(channels));

            if (channel is BoneChannelDescriptor boneChannel && skeleton.IndexOfId(boneChannel.BoneId) < 0)
                throw new ArgumentException($"Channel {i} references bone id {boneChannel.BoneId}, which is not present in skeleton \"{skeleton.name}\".", nameof(channels));

            if (!seen.Add(channel))
                throw new ArgumentException($"Duplicate channel {Describe(channel)}.", nameof(channels));

            if (channel.SectionKey < 0)
                throw new ArgumentException($"Channel {Describe(channel)} has negative section key {channel.SectionKey}.", nameof(channels));

            if (channel.SectionKey <= ChannelSections.Bool && !IsBuiltIn(channel))
                throw new ArgumentException($"Channel {Describe(channel)} claims built-in section key {channel.SectionKey}; keys up to {ChannelSections.Bool} are reserved.", nameof(channels));

            if (channel is RotationChannel rotation)
            {
                if (rotationFormat == null) rotationFormat = rotation.Representation;
                else if (rotationFormat != rotation.Representation)
                    throw new ArgumentException($"Mixed rotation representations in one layout: {rotationFormat} and {rotation.Representation}. All Rotation channels must share one representation.", nameof(channels));
            }
        }

        var hash = ComputeHash(skeleton, channels);
        var cacheKey = CombineKey(skeleton.GetEntityId().GetHashCode(), hash);
        if (Cache.TryGetValue(cacheKey, out var cached)) return cached;

        var grouped = new SortedDictionary<int, List<ChannelDescriptor>>();
        var channelsCopy = new ChannelDescriptor[channels.Count];

        for (var i = 0; i < channels.Count; i++)
        {
            var channel = channels[i];
            channelsCopy[i] = channel;
            if (!grouped.TryGetValue(channel.SectionKey, out var section))
            {
                section = new List<ChannelDescriptor>();
                grouped[channel.SectionKey] = section;
            }

            section.Add(channel);
        }

        var offsets = new Dictionary<ChannelDescriptor, ChannelSlot>(channels.Count);
        var sections = new Dictionary<int, ChannelDescriptor[]>(grouped.Count);
        var sectionRanges = new Dictionary<int, SectionRange>(grouped.Count);
        var cursor = 0;

        foreach (var group in grouped)
        {
            var start = cursor;
            foreach (var channel in group.Value)
            {
                offsets[channel] = new ChannelSlot(cursor, channel.FloatCount);
                cursor += channel.FloatCount;
            }

            sections[group.Key] = group.Value.ToArray();
            sectionRanges[group.Key] = new SectionRange(start, cursor - start);
        }

        var resolvedRotationFormat = rotationFormat ?? RotationRepresentation.Quaternion;

        var data = new PoseLayoutData
        {
            PositionStart = SectionStart(sectionRanges, ChannelSections.Position),
            PositionCount = SectionElementCount(sections, ChannelSections.Position),
            RotationStart = SectionStart(sectionRanges, ChannelSections.Rotation),
            RotationCount = SectionElementCount(sections, ChannelSections.Rotation),
            RotationStride = (byte)(resolvedRotationFormat == RotationRepresentation.Quaternion ? 4 : 3),
            ScaleStart = SectionStart(sectionRanges, ChannelSections.Scale),
            ScaleCount = SectionElementCount(sections, ChannelSections.Scale),
            VelocityStart = SectionStart(sectionRanges, ChannelSections.Velocity),
            VelocityCount = SectionElementCount(sections, ChannelSections.Velocity),
            AngularVelocityStart = SectionStart(sectionRanges, ChannelSections.AngularVelocity),
            AngularVelocityCount = SectionElementCount(sections, ChannelSections.AngularVelocity),
            BoolStart = SectionStart(sectionRanges, ChannelSections.Bool),
            BoolCount = SectionElementCount(sections, ChannelSections.Bool),
            FloatCount = cursor,
            LayoutHash = hash
        };

        // Built-in section keys are the lowest, so anything else forms a contiguous tail.
        var extraStart = cursor;
        foreach (var range in sectionRanges)
        {
            if (range.Key > ChannelSections.Bool && range.Value.Start < extraStart) extraStart = range.Value.Start;
        }

        data.ExtraStart = extraStart;
        data.ExtraCount = cursor - extraStart;

        var layout = new PoseLayout(skeleton, channelsCopy, data, resolvedRotationFormat, offsets, sections,
            sectionRanges);

        Cache[cacheKey] = layout;
        return layout;
    }

    /// <summary>
    /// One ParentLocal Position + one ParentLocal Quaternion Rotation channel per bone, in
    /// skeleton DFS order, so element index equals bone index. Optionally adds per-bone
    /// ParentLocal Velocity/AngularVelocity channels, likewise bone-index aligned. All channels
    /// use <see cref="ChannelUsage.Default"/>.
    /// </summary>
    public static PoseLayout CreateFullPose(SkeletonAsset skeleton, bool includeVelocities, bool includeAngularVelocities)
    {
        if (skeleton == null) throw new ArgumentException("skeleton must not be null.", nameof(skeleton));

        var boneCount = skeleton.BoneCount;
        var channels = new List<ChannelDescriptor>(boneCount * 2);

        for (var i = 0; i < boneCount; i++)
        {
            var boneId = skeleton.GetBone(i).id;
            channels.Add(new PositionChannel(boneId));
            channels.Add(new RotationChannel(boneId));
        }

        if (includeVelocities)
        {
            for (var i = 0; i < boneCount; i++)
            {
                channels.Add(new VelocityChannel(skeleton.GetBone(i).id));
            }
        }

        if (includeAngularVelocities)
        {
            for (var i = 0; i < boneCount; i++)
            {
                channels.Add(new AngularVelocityChannel(skeleton.GetBone(i).id));
            }
        }

        return Build(skeleton, channels);
    }

    public bool TryBindPosition(BoneReference bone, out PositionHandle handle, ChannelUsage usage = ChannelUsage.Default)
    {
        if (bone.IsSet && _offsets.TryGetValue(new PositionChannel(bone.BoneId, usage: usage), out var slot))
        {
            handle = new PositionHandle(slot.Offset, Data.LayoutHash);
            return true;
        }

        handle = PositionHandle.Invalid;
        return false;
    }

    public PositionHandle BindPosition(BoneReference bone, ChannelUsage usage = ChannelUsage.Default)
    {
        if (!TryBindPosition(bone, out var handle, usage))
            throw new ArgumentException($"No Position channel with usage {usage} bound to bone id {bone.BoneId}.");
        return handle;
    }

    public bool TryBindRotation(BoneReference bone, out RotationHandle handle, ChannelUsage usage = ChannelUsage.Default)
    {
        if (bone.IsSet && _offsets.TryGetValue(new RotationChannel(bone.BoneId, usage: usage), out var slot))
        {
            handle = new RotationHandle(slot.Offset, Data.LayoutHash);
            return true;
        }

        handle = RotationHandle.Invalid;
        return false;
    }

    public RotationHandle BindRotation(BoneReference bone, ChannelUsage usage = ChannelUsage.Default)
    {
        if (!TryBindRotation(bone, out var handle, usage))
            throw new ArgumentException($"No Rotation channel with usage {usage} bound to bone id {bone.BoneId}.");
        return handle;
    }

    public bool TryBindScale(BoneReference bone, out ScaleHandle handle, ChannelUsage usage = ChannelUsage.Default)
    {
        if (bone.IsSet && _offsets.TryGetValue(new ScaleChannel(bone.BoneId, usage: usage), out var slot))
        {
            handle = new ScaleHandle(slot.Offset, Data.LayoutHash);
            return true;
        }

        handle = ScaleHandle.Invalid;
        return false;
    }

    public ScaleHandle BindScale(BoneReference bone, ChannelUsage usage = ChannelUsage.Default)
    {
        if (!TryBindScale(bone, out var handle, usage))
            throw new ArgumentException($"No Scale channel with usage {usage} bound to bone id {bone.BoneId}.");
        return handle;
    }

    public bool TryBindBool(BoneReference bone, out BoolHandle handle, ChannelUsage usage = ChannelUsage.Default)
    {
        if (bone.IsSet && _offsets.TryGetValue(new BoolChannel(bone.BoneId, usage), out var slot))
        {
            handle = new BoolHandle(slot.Offset, Data.LayoutHash);
            return true;
        }

        handle = BoolHandle.Invalid;
        return false;
    }

    public BoolHandle BindBool(BoneReference bone, ChannelUsage usage = ChannelUsage.Default)
    {
        if (!TryBindBool(bone, out var handle, usage))
            throw new ArgumentException($"No Bool channel with usage {usage} bound to bone id {bone.BoneId}.");
        return handle;
    }

    public bool TryBindVelocity(BoneReference bone, out VelocityHandle handle, ChannelUsage usage = ChannelUsage.Default)
    {
        if (bone.IsSet && _offsets.TryGetValue(new VelocityChannel(bone.BoneId, usage: usage), out var slot))
        {
            handle = new VelocityHandle(slot.Offset, Data.LayoutHash);
            return true;
        }

        handle = VelocityHandle.Invalid;
        return false;
    }

    public VelocityHandle BindVelocity(BoneReference bone, ChannelUsage usage = ChannelUsage.Default)
    {
        if (!TryBindVelocity(bone, out var handle, usage))
            throw new ArgumentException($"No Velocity channel with usage {usage} bound to bone id {bone.BoneId}.");
        return handle;
    }

    public bool TryBindAngularVelocity(BoneReference bone, out AngularVelocityHandle handle, ChannelUsage usage = ChannelUsage.Default)
    {
        if (bone.IsSet && _offsets.TryGetValue(new AngularVelocityChannel(bone.BoneId, usage: usage), out var slot))
        {
            handle = new AngularVelocityHandle(slot.Offset, Data.LayoutHash);
            return true;
        }

        handle = AngularVelocityHandle.Invalid;
        return false;
    }

    public AngularVelocityHandle BindAngularVelocity(BoneReference bone, ChannelUsage usage = ChannelUsage.Default)
    {
        if (!TryBindAngularVelocity(bone, out var handle, usage))
            throw new ArgumentException($"No AngularVelocity channel with usage {usage} bound to bone id {bone.BoneId}.");
        return handle;
    }

    /// <summary>
    /// Binds any channel by identity, including kinds without a named section. The width comes
    /// from the stored channel, so a probe that differs in descriptive metadata still binds
    /// correctly.
    /// </summary>
    public bool TryBindChannel(ChannelDescriptor probe, out ChannelHandle handle)
    {
        if (probe != null && _offsets.TryGetValue(probe, out var slot))
        {
            handle = new ChannelHandle(slot.Offset, slot.FloatCount, Data.LayoutHash);
            return true;
        }

        handle = ChannelHandle.Invalid;
        return false;
    }

    public ChannelHandle BindChannel(ChannelDescriptor probe)
    {
        if (!TryBindChannel(probe, out var handle))
            throw new ArgumentException($"No channel {Describe(probe)} in this layout.");
        return handle;
    }

    /// <summary>The float range a whole section occupies. False when the section is absent.</summary>
    public bool TryGetSectionRange(int sectionKey, out int startFloat, out int floatCount)
    {
        if (_sectionRanges.TryGetValue(sectionKey, out var range))
        {
            startFloat = range.Start;
            floatCount = range.FloatCount;
            return true;
        }

        startFloat = 0;
        floatCount = 0;
        return false;
    }

    /// <summary>Reverse lookup for tooling/debugging: the descriptor behind a section element index.</summary>
    public ChannelDescriptor GetChannel(int sectionKey, int elementIndex)
    {
        if (!_sections.TryGetValue(sectionKey, out var section))
            throw new ArgumentOutOfRangeException(nameof(sectionKey), sectionKey, "No such section in this layout.");
        return section[elementIndex];
    }

    /// <summary>Number of channels in a section, or 0 when the section is absent.</summary>
    public int GetSectionElementCount(int sectionKey)
    {
        return _sections.TryGetValue(sectionKey, out var section) ? section.Length : 0;
    }

    private static bool IsBuiltIn(ChannelDescriptor channel)
    {
        return channel is PositionChannel or RotationChannel or ScaleChannel or VelocityChannel
            or AngularVelocityChannel or BoolChannel;
    }

    private static string Describe(ChannelDescriptor channel)
    {
        if (channel == null) return "<null>";
        return channel is BoneChannelDescriptor bone
            ? $"{channel.GetType().Name}(bone {bone.BoneId}, usage {bone.Usage})"
            : channel.GetType().Name;
    }

    private static int SectionStart(Dictionary<int, SectionRange> ranges, int sectionKey)
    {
        return ranges.TryGetValue(sectionKey, out var range) ? range.Start : 0;
    }

    private static int SectionElementCount(Dictionary<int, ChannelDescriptor[]> sections, int sectionKey)
    {
        return sections.TryGetValue(sectionKey, out var section) ? section.Length : 0;
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
                hash = hash * 31 + channels[i].GetContentHash();
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
