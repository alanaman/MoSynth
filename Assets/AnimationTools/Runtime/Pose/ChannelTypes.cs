namespace AnimationTools
{
/// <summary>
/// Section keys for the built-in channel kinds. A <see cref="PoseLayout"/> lays sections out in
/// ascending key order, so these values fix the order of a pose inside a <see cref="PoseBuffer"/>.
/// </summary>
/// <remarks>
/// Deliberately constants rather than an enum's ordinals: section order is a buffer-layout
/// contract, and deriving it from a declaration order would let a cosmetic reorder silently move
/// pose data. Channel kinds outside this set must use a key greater than <see cref="Bool"/>, which
/// keeps them in a contiguous tail after the built-in sections.
/// </remarks>
public static class ChannelSections
{
    public const int Position = 0;
    public const int Rotation = 1;
    public const int Scale = 2;
    public const int Velocity = 3;
    public const int AngularVelocity = 4;
    public const int Bool = 5;
}

/// <summary>Reference frame a channel's values are expressed in.</summary>
public enum ChannelSpace : byte
{
    ParentLocal,
    RootLocal,
    Character,
    World
}

/// <summary>
/// On-disk/in-buffer encoding of a rotation-valued channel.
/// <see cref="RotationRepresentation.RotationVector"/> is axis * radians; angular velocity
/// channels always use this representation (axis * radians/second) since a quaternion has
/// no meaningful "rate" form.
/// </summary>
public enum RotationRepresentation : byte
{
    Quaternion,
    RotationVector,
    EulerDegrees
}

/// <summary>
/// Disambiguates multiple channels of the same kind on the same bone. In particular, root motion
/// is an explicit, discoverable channel declaration (<see cref="RootMotion"/>) rather than an
/// implicit "index 0" convention.
/// </summary>
public enum ChannelUsage : byte
{
    Default,
    RootMotion,
    Contact
}
}
