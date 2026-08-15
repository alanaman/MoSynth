namespace AnimationTools
{
// Index is an offset into StateBuffer.Data, not a section-local element index, so reading through
// a handle costs no layout arithmetic. That also means a handle bound against one layout would
// address arbitrary floats in a buffer built from another, so each handle carries the LayoutHash
// it was bound against and every accessor asserts the match. LayoutHash is the right equivalence
// class rather than an approximation: it covers every channel's content (and, for pose layouts,
// the skeleton), so equal hashes mean identical offsets. It is also what StateBuffer.CopyFrom
// asserts on, so handle validity and copy compatibility agree by construction.

/// <summary>
/// A handle to one channel of a <see cref="StateBuffer"/>, procured by presenting the channel's
/// <see cref="ChannelDescriptor"/> to <see cref="StateBufferLayout.BindChannel"/>.
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
    /// reading buffer data should still go through <see cref="StateBuffer"/>, which checks the
    /// handle against the buffer's layout.
    /// </summary>
    public int FloatOffset => Index;

    public bool IsValid => Index >= 0;
    public static ChannelHandle Invalid => new(-1, 0, 0);
}
}
