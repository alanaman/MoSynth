using System;

namespace AnimationTools
{
/// <summary>
/// Section key for channels recorded by <see cref="MotionRecorder"/>. A single value because
/// every recorder channel — time, bone transforms, the full pose, custom ones — lives in one
/// contiguous tail section; recorder channels don't need per-kind ordering the way a
/// <see cref="PoseLayout"/>'s built-in sections do.
/// </summary>
public static class RecorderSections
{
    public const int Recorded = ChannelSections.Bool + 1;
}

/// <summary>
/// Everything a <see cref="RecorderChannel"/> needs to produce one sample: the synthesizer it
/// was bound against, the pose for this tick, and timing/frame bookkeeping.
/// </summary>
public readonly struct RecorderSampleContext
{
    public readonly MotionSynthesisComponent Synthesizer;

    /// <summary>The last post-stage pose applied to the skeleton. Read-only view.</summary>
    public readonly PoseBuffer Pose;

    /// <summary>Tick dt or Time.deltaTime, depending on the recorder's trigger.</summary>
    public readonly float DeltaTime;

    /// <summary>Seconds elapsed since recording started.</summary>
    public readonly float Time;

    public readonly int FrameIndex;

    public RecorderSampleContext(MotionSynthesisComponent synthesizer, PoseBuffer pose, float deltaTime,
        float time, int frameIndex)
    {
        Synthesizer = synthesizer;
        Pose = pose;
        DeltaTime = deltaTime;
        Time = time;
        FrameIndex = frameIndex;
    }
}

/// <summary>
/// One column group of a <see cref="MotionRecorder"/>'s output: how many floats it writes per
/// sample, and how to write them. Unlike <see cref="BoneChannelDescriptor"/>, recorder channels
/// are inspector-serialized (<see cref="SubclassSelectorAttribute"/>) so they need default
/// constructors and mutable fields, and so subclass identity is (concrete type, name, config)
/// rather than (type, bone id, usage).
/// </summary>
[Serializable]
public abstract class RecorderChannel : ChannelDescriptor
{
    public string name;

    public override int SectionKey => RecorderSections.Recorded;

    /// <summary>
    /// Resolves references (bones, control input, pose layout) once at record start, before the
    /// <see cref="RecordingLayout"/> is built — <see cref="ChannelDescriptor.FloatCount"/> may
    /// depend on state this sets up.
    /// </summary>
    public virtual void Bind(MotionSynthesisComponent synthesizer)
    {
    }

    /// <summary>Writes this channel's <see cref="ChannelDescriptor.FloatCount"/> floats for the current sample into frame.</summary>
    public abstract void Sample(in RecorderSampleContext context, ChannelHandle handle, StateBuffer frame);

    /// <summary>Fills manifest metadata beyond name/type/offset/width.</summary>
    public virtual void PopulateManifest(RecordingManifestChannel entry)
    {
    }
}
}
