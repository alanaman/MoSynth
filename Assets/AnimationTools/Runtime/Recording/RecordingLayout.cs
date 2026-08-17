using System.Collections.Generic;

namespace AnimationTools
{
/// <summary>
/// The <see cref="StateBufferLayout"/> for one <see cref="MotionRecorder"/> session: one
/// <see cref="RecorderChannel"/> per column group, laid out in the single
/// <see cref="RecorderSections.Recorded"/> section in declaration order.
/// </summary>
public sealed class RecordingLayout : StateBufferLayout
{
    private const int HashSeed = 8117;

    public RecordingLayout(IReadOnlyList<ChannelDescriptor> channels)
        : base(channels, HashChannels(HashSeed, channels))
    {
    }
}
}
