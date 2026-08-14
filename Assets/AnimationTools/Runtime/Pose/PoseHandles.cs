namespace AnimationTools
{
// Structurally identical handle types kept distinct on purpose: the type system stops a
// PositionHandle from being used where a VelocityHandle is expected, even though both are just an
// absolute float offset into a PoseBuffer.
//
// Index is an offset into PoseBuffer.Data, not a section-local element index, so reading through a
// handle costs no layout arithmetic. That also means a handle bound against one layout would
// address arbitrary floats in a buffer built from another, so each handle carries the LayoutHash
// it was bound against and every accessor asserts the match. LayoutHash is the right equivalence
// class rather than an approximation: it covers the skeleton version, every bone id and every
// channel's content, so equal hashes mean identical offsets. It is also what PoseBuffer.CopyFrom
// asserts on, so handle validity and copy compatibility agree by construction.

public readonly struct PositionHandle
{
    internal readonly int Index;
    internal readonly int LayoutHash;
    internal PositionHandle(int index, int layoutHash) { Index = index; LayoutHash = layoutHash; }
    public bool IsValid => Index >= 0;
    public static PositionHandle Invalid => new(-1, 0);
}

public readonly struct RotationHandle
{
    internal readonly int Index;
    internal readonly int LayoutHash;
    internal RotationHandle(int index, int layoutHash) { Index = index; LayoutHash = layoutHash; }
    public bool IsValid => Index >= 0;
    public static RotationHandle Invalid => new(-1, 0);
}

public readonly struct ScaleHandle
{
    internal readonly int Index;
    internal readonly int LayoutHash;
    internal ScaleHandle(int index, int layoutHash) { Index = index; LayoutHash = layoutHash; }
    public bool IsValid => Index >= 0;
    public static ScaleHandle Invalid => new(-1, 0);
}

public readonly struct BoolHandle
{
    internal readonly int Index;
    internal readonly int LayoutHash;
    internal BoolHandle(int index, int layoutHash) { Index = index; LayoutHash = layoutHash; }
    public bool IsValid => Index >= 0;
    public static BoolHandle Invalid => new(-1, 0);
}

public readonly struct VelocityHandle
{
    internal readonly int Index;
    internal readonly int LayoutHash;
    internal VelocityHandle(int index, int layoutHash) { Index = index; LayoutHash = layoutHash; }
    public bool IsValid => Index >= 0;
    public static VelocityHandle Invalid => new(-1, 0);
}

public readonly struct AngularVelocityHandle
{
    internal readonly int Index;
    internal readonly int LayoutHash;
    internal AngularVelocityHandle(int index, int layoutHash) { Index = index; LayoutHash = layoutHash; }
    public bool IsValid => Index >= 0;
    public static AngularVelocityHandle Invalid => new(-1, 0);
}

/// <summary>
/// A handle to a channel of arbitrary width, for kinds outside the six with a named section.
/// </summary>
public readonly struct ChannelHandle
{
    internal readonly int Index;
    internal readonly int Count;
    internal readonly int LayoutHash;

    internal ChannelHandle(int index, int count, int layoutHash)
    {
        Index = index;
        Count = count;
        LayoutHash = layoutHash;
    }

    /// <summary>Width of this channel in floats.</summary>
    public int FloatCount => Count;

    /// <summary>
    /// Offset of this channel from the start of a buffer, in floats. Exposed because per-dimension
    /// side tables (normalisation statistics, per-float weights) have to be indexed positionally;
    /// reading buffer data should still go through <see cref="PoseBuffer"/>, which checks the
    /// handle against the buffer's layout.
    /// </summary>
    public int FloatOffset => Index;

    public bool IsValid => Index >= 0;
    public static ChannelHandle Invalid => new(-1, 0, 0);
}
}
