using System;

namespace AnimationTools
{
/// <summary>
/// Describes one channel of a <see cref="PoseLayout"/>: how wide it is and which section it
/// belongs to. Subclass this to put a new kind of data in a <see cref="PoseBuffer"/>.
/// </summary>
/// <remarks>
/// Two hashes, deliberately. <see cref="GetHashCode"/> answers "which channel is this?" and keys
/// the layout's offset lookup, so it covers only identity. <see cref="GetContentHash"/> also folds
/// in descriptive metadata such as space and rotation representation, and feeds
/// <see cref="PoseLayout.LayoutHash"/>. Two layouts differing only in a channel's space therefore
/// address identically but are still not interchangeable.
/// </remarks>
public abstract class ChannelDescriptor : IEquatable<ChannelDescriptor>
{
    /// <summary>Width of one occurrence of this channel, in floats.</summary>
    public abstract int FloatCount { get; }

    /// <summary>
    /// Groups channels into buffer sections, laid out in ascending key order with declaration
    /// order preserved inside a section. Built-in kinds use <see cref="ChannelSections"/>;
    /// everything else must use a key greater than <see cref="ChannelSections.Bool"/>.
    /// </summary>
    public abstract int SectionKey { get; }

    public abstract bool Equals(ChannelDescriptor other);

    public override bool Equals(object obj) => Equals(obj as ChannelDescriptor);

    public abstract override int GetHashCode();

    public abstract int GetContentHash();
}

/// <summary>
/// A channel addressed by a skeleton bone. Identity is (concrete type, bone, usage) — descriptive
/// fields on subclasses are excluded, so binding never has to restate them.
/// </summary>
public abstract class BoneChannelDescriptor : ChannelDescriptor
{
    public int BoneId { get; }
    public ChannelUsage Usage { get; }

    protected BoneChannelDescriptor(int boneId, ChannelUsage usage)
    {
        BoneId = boneId;
        Usage = usage;
    }

    public sealed override bool Equals(ChannelDescriptor other)
    {
        return other is BoneChannelDescriptor bone
               && other.GetType() == GetType()
               && bone.BoneId == BoneId
               && bone.Usage == Usage;
    }

    public sealed override int GetHashCode()
    {
        unchecked
        {
            var hash = GetType().GetHashCode();
            hash = hash * 31 + BoneId;
            hash = hash * 31 + (int)Usage;
            return hash;
        }
    }

    public override int GetContentHash() => GetHashCode();
}

/// <summary>A bone channel whose values are expressed in a particular reference frame.</summary>
public abstract class SpacedBoneChannelDescriptor : BoneChannelDescriptor
{
    public ChannelSpace Space { get; }

    protected SpacedBoneChannelDescriptor(int boneId, ChannelSpace space, ChannelUsage usage)
        : base(boneId, usage)
    {
        Space = space;
    }

    public override int GetContentHash()
    {
        unchecked
        {
            return base.GetContentHash() * 31 + (int)Space;
        }
    }
}
}
