using System;

namespace AnimationTools
{
/// <summary>
/// Describes one channel of pose data: which bone it belongs to, what kind of value it
/// carries, in what space, encoded how, and disambiguated by which usage. A
/// <see cref="PoseLayout"/> is built from an ordered list of these.
/// </summary>
[Serializable]
public struct ChannelDescriptor : IEquatable<ChannelDescriptor>
{
    public int boneId;
    public ChannelType type;
    public ChannelSpace space;
    public RotationRepresentation representation;
    public ChannelUsage usage;

    public bool Equals(ChannelDescriptor other)
    {
        return boneId == other.boneId
            && type == other.type
            && space == other.space
            && representation == other.representation
            && usage == other.usage;
    }

    public override bool Equals(object obj) => obj is ChannelDescriptor other && Equals(other);

    public override int GetHashCode()
    {
        unchecked
        {
            var hash = boneId;
            hash = hash * 31 + (int)type;
            hash = hash * 31 + (int)space;
            hash = hash * 31 + (int)representation;
            hash = hash * 31 + (int)usage;
            return hash;
        }
    }
}
}
