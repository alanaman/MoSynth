using System;
using Unity.Collections;

namespace AnimationTools
{
/// <summary>
/// A contiguous run of <see cref="FrameCount"/> poses sharing one <see cref="PoseLayout"/>,
/// stored back to back in a single flat float array.
/// </summary>
/// <remarks>
/// <see cref="GetFrame"/> hands out a <see cref="PoseBuffer"/> that is a view into
/// <see cref="Data"/> (a <see cref="NativeArray{T}.GetSubArray"/> slice) — it aliases this
/// sequence's storage rather than copying it. The caller must NEVER call
/// <see cref="PoseBuffer.Dispose"/> on that view; disposing a sub-array throws.
/// <para/>
/// Ownership of <see cref="Data"/> follows the usual Native* rule: whoever allocated it is
/// responsible for disposing it. <see cref="Allocate"/> and <see cref="FromArray"/> allocate
/// fresh storage that the caller owns and must eventually pass to <see cref="Dispose"/>. A
/// <see cref="PoseSequence"/> built over someone else's <see cref="NativeArray{T}"/> (e.g. one
/// backed by <see cref="Allocator.Domain"/>) must never be disposed here — disposal is the
/// original allocator's responsibility, and calling <see cref="Dispose"/> on a Domain-backed
/// sequence will throw.
/// </remarks>
public sealed class PoseSequence
{
    public PoseLayout Layout { get; }
    public NativeArray<float> Data { get; }
    public int FrameCount { get; }

    /// <summary>
    /// Wraps an existing array; does not copy. Throws <see cref="ArgumentNullException"/> if
    /// <paramref name="layout"/> is null, or <see cref="ArgumentException"/> if
    /// <paramref name="data"/>'s length isn't a positive multiple of
    /// <see cref="PoseLayout.FloatCount"/>.
    /// </summary>
    public PoseSequence(PoseLayout layout, NativeArray<float> data)
    {
        if (layout == null) throw new ArgumentNullException(nameof(layout));
        if (data.Length <= 0 || data.Length % layout.FloatCount != 0)
            throw new ArgumentException($"data.Length ({data.Length}) must be a positive multiple of layout.FloatCount ({layout.FloatCount}).", nameof(data));

        Layout = layout;
        Data = data;
        FrameCount = data.Length / layout.FloatCount;
    }

    public static PoseSequence Allocate(PoseLayout layout, int frameCount, Allocator allocator)
    {
        if (layout == null) throw new ArgumentNullException(nameof(layout));
        if (frameCount <= 0) throw new ArgumentException("frameCount must be positive.", nameof(frameCount));

        var data = new NativeArray<float>(frameCount * layout.FloatCount, allocator, NativeArrayOptions.ClearMemory);
        return new PoseSequence(layout, data);
    }

    public static PoseSequence FromArray(PoseLayout layout, float[] frameData, Allocator allocator)
    {
        if (layout == null) throw new ArgumentNullException(nameof(layout));
        if (frameData == null) throw new ArgumentNullException(nameof(frameData));

        var data = new NativeArray<float>(frameData, allocator);
        return new PoseSequence(layout, data);
    }

    /// <summary>
    /// View over one frame — the caller must NEVER Dispose the returned buffer (it is a
    /// <see cref="NativeArray{T}.GetSubArray"/> view; disposing it throws).
    /// </summary>
    public PoseBuffer GetFrame(int frameIndex)
    {
        if (frameIndex < 0 || frameIndex >= FrameCount)
            throw new ArgumentOutOfRangeException(nameof(frameIndex), frameIndex, $"frameIndex must be in [0, {FrameCount}).");

        return new PoseBuffer { Data = Data.GetSubArray(frameIndex * Layout.FloatCount, Layout.FloatCount), Layout = Layout.Data };
    }

    public void CopyTo(float[] destination)
    {
        if (destination == null) throw new ArgumentNullException(nameof(destination));
        if (destination.Length != Data.Length)
            throw new ArgumentException($"destination.Length ({destination.Length}) must equal Data.Length ({Data.Length}).", nameof(destination));

        Data.CopyTo(destination);
    }

    /// <summary>
    /// Disposes <see cref="Data"/>. Only call this for sequences built by <see cref="Allocate"/>
    /// or <see cref="FromArray"/>; never call it on a sequence wrapping a Domain-backed (or
    /// otherwise externally owned) array — that storage outlives this sequence and disposing it
    /// here would throw.
    /// </summary>
    public void Dispose()
    {
        if (Data.IsCreated) Data.Dispose();
    }
}
}
