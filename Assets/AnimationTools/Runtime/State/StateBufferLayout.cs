using System;
using System.Collections.Generic;

namespace AnimationTools
{
/// <summary>
/// Unmanaged, Burst-embeddable identity of a <see cref="StateBufferLayout"/>: the buffer width in
/// floats and the layout hash every <see cref="ChannelHandle"/> is checked against.
/// </summary>
public struct StateBufferLayoutData
{
    public int FloatCount;
    public int LayoutHash;
}

/// <summary>
/// An immutable map from <see cref="ChannelDescriptor"/>s to positions within a flat float buffer
/// (see <see cref="StateBuffer"/>). This is the general per-frame layout; skeleton-aware pose
/// layouts (<see cref="PoseLayout"/>) and domain-specific layouts extend it.
/// </summary>
/// <remarks>
/// Sections are laid out in ascending <see cref="ChannelDescriptor.SectionKey"/> order, and within
/// a section channels keep the declaration order they were passed to the constructor in. Each
/// channel occupies its own <see cref="ChannelDescriptor.FloatCount"/>, so a section is only
/// uniform-stride when its channels are.
/// <para/>
/// A handle is procured by presenting the <see cref="ChannelDescriptor"/> that identifies the
/// channel (<see cref="BindChannel"/>). Binding allocates a probe descriptor per call and is meant
/// for setup, not per-frame use.
/// </remarks>
public class StateBufferLayout
{
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

    private readonly Dictionary<ChannelDescriptor, ChannelHandle> _handles;
    private readonly Dictionary<int, ChannelDescriptor[]> _sections;
    private readonly Dictionary<int, SectionRange> _sectionRanges;

    public IReadOnlyList<ChannelDescriptor> Channels { get; }
    public int FloatCount { get; }
    public int LayoutHash { get; }

    public StateBufferLayoutData Data => new StateBufferLayoutData
    {
        FloatCount = FloatCount,
        LayoutHash = LayoutHash
    };

    /// <summary>
    /// Validates the channel set (no nulls, no duplicate identities, no negative section keys) and
    /// assigns offsets. <paramref name="layoutHash"/> is supplied by the subclass because what a
    /// layout's identity covers is subclass-specific (see <see cref="HashChannels"/> for the
    /// channel-content part).
    /// </summary>
    protected StateBufferLayout(IReadOnlyList<ChannelDescriptor> channels, int layoutHash)
    {
        if (channels == null) throw new ArgumentException("channels must not be null.", nameof(channels));

        var seen = new HashSet<ChannelDescriptor>();
        for (var i = 0; i < channels.Count; i++)
        {
            var channel = channels[i];
            if (channel == null) throw new ArgumentException($"Channel {i} is null.", nameof(channels));

            if (!seen.Add(channel))
                throw new ArgumentException($"Duplicate channel {Describe(channel)}.", nameof(channels));

            if (channel.SectionKey < 0)
                throw new ArgumentException($"Channel {Describe(channel)} has negative section key {channel.SectionKey}.", nameof(channels));
        }

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

        _handles = new Dictionary<ChannelDescriptor, ChannelHandle>(channels.Count);
        _sections = new Dictionary<int, ChannelDescriptor[]>(grouped.Count);
        _sectionRanges = new Dictionary<int, SectionRange>(grouped.Count);
        var cursor = 0;

        foreach (var group in grouped)
        {
            var start = cursor;
            foreach (var channel in group.Value)
            {
                _handles[channel] = new ChannelHandle(cursor, channel.FloatCount, layoutHash);
                cursor += channel.FloatCount;
            }

            _sections[group.Key] = group.Value.ToArray();
            _sectionRanges[group.Key] = new SectionRange(start, cursor - start);
        }

        Channels = channelsCopy;
        FloatCount = cursor;
        LayoutHash = layoutHash;
    }

    /// <summary>
    /// Binds any channel by identity. The width comes from the stored channel, so a probe that
    /// differs in descriptive metadata still binds correctly.
    /// </summary>
    public bool TryBindChannel(ChannelDescriptor probe, out ChannelHandle handle)
    {
        if (probe != null && _handles.TryGetValue(probe, out handle)) return true;

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

    /// <summary>Folds every channel's content hash into <paramref name="seed"/>.</summary>
    protected static int HashChannels(int seed, IReadOnlyList<ChannelDescriptor> channels)
    {
        unchecked
        {
            var hash = seed;
            for (var i = 0; i < channels.Count; i++)
            {
                hash = hash * 31 + channels[i].GetContentHash();
            }

            return hash;
        }
    }

    protected static string Describe(ChannelDescriptor channel)
    {
        if (channel == null) return "<null>";
        return channel is BoneChannelDescriptor bone
            ? $"{channel.GetType().Name}(bone {bone.BoneId}, usage {bone.Usage})"
            : channel.GetType().Name;
    }
}
}
