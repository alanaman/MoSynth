using Unity.Collections;

namespace AnimationTools
{
/// <summary>
/// A <see cref="StateSequence"/> whose layout is a <see cref="PoseLayout"/>, handing out
/// <see cref="PoseBuffer"/> frame views.
/// </summary>
/// <remarks>
/// <see cref="GetFrame"/> and <see cref="Layout"/> hide (not override) the base members, so a
/// <see cref="StateSequence"/>-typed reference to a pose sequence yields plain
/// <see cref="StateBuffer"/> frames. Ownership rules are the base class's: never Dispose a frame
/// view, and only Dispose sequences whose storage was allocated here.
/// </remarks>
public sealed class PoseSequence : StateSequence
{
    public new PoseLayout Layout => (PoseLayout)base.Layout;

    /// <summary>Wraps an existing array; does not copy. See <see cref="StateSequence"/> for the throws.</summary>
    public PoseSequence(PoseLayout layout, NativeArray<float> data)
        : base(layout, data)
    {
    }

    public static PoseSequence Allocate(PoseLayout layout, int frameCount, Allocator allocator)
    {
        return new PoseSequence(layout, AllocateData(layout, frameCount, allocator));
    }

    public static PoseSequence FromArray(PoseLayout layout, float[] frameData, Allocator allocator)
    {
        return new PoseSequence(layout, CopyData(layout, frameData, allocator));
    }

    /// <summary>
    /// View over one frame — the caller must NEVER Dispose the returned buffer (it is a
    /// <see cref="NativeArray{T}.GetSubArray"/> view; disposing it throws).
    /// </summary>
    public new PoseBuffer GetFrame(int frameIndex)
    {
        return new PoseBuffer { State = base.GetFrame(frameIndex), Layout = Layout.Data };
    }
}
}
