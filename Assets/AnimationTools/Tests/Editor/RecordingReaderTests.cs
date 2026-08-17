using System;
using NUnit.Framework;

namespace AnimationTools.Tests
{
public class RecordingReaderTests
{
    private static RecordingManifest BuildManifest()
    {
        return new RecordingManifest
        {
            frameFloatCount = 5,
            channels = new[]
            {
                new RecordingManifestChannel { name = "a", floatOffset = 0, floatCount = 2 },
                new RecordingManifestChannel { name = "b", floatOffset = 2, floatCount = 3 }
            }
        };
    }

    [Test]
    public void GetChannel_ReturnsMatchingEntry()
    {
        var reader = new RecordingReader(BuildManifest(), new float[10]);

        var channel = reader.GetChannel("b");

        Assert.AreEqual("b", channel.name);
        Assert.AreEqual(2, channel.floatOffset);
        Assert.AreEqual(3, channel.floatCount);
    }

    [Test]
    public void GetChannel_MissingName_Throws()
    {
        var reader = new RecordingReader(BuildManifest(), new float[10]);

        Assert.Throws<ArgumentException>(() => reader.GetChannel("missing"));
    }

    [Test]
    public void TryGetChannel_MissingName_ReturnsFalse()
    {
        var reader = new RecordingReader(BuildManifest(), new float[10]);

        var found = reader.TryGetChannel("missing", out var channel);

        Assert.IsFalse(found);
        Assert.IsNull(channel);
    }

    [Test]
    public void GetFloat_And_GetFloat3_ReturnExpectedValues()
    {
        var data = new float[10]
        {
            0f, 1f, 2f, 3f, 4f, // frame 0: a = [0, 1], b = [2, 3, 4]
            5f, 6f, 7f, 8f, 9f  // frame 1: a = [5, 6], b = [7, 8, 9]
        };
        var reader = new RecordingReader(BuildManifest(), data);
        var a = reader.GetChannel("a");
        var b = reader.GetChannel("b");

        Assert.AreEqual(0f, reader.GetFloat(0, a, 0));
        Assert.AreEqual(1f, reader.GetFloat(0, a, 1));
        Assert.AreEqual(5f, reader.GetFloat(1, a, 0));
        Assert.AreEqual(6f, reader.GetFloat(1, a, 1));

        var b0 = reader.GetFloat3(0, b);
        Assert.AreEqual(2f, b0.x);
        Assert.AreEqual(3f, b0.y);
        Assert.AreEqual(4f, b0.z);

        var b1 = reader.GetFloat3(1, b);
        Assert.AreEqual(7f, b1.x);
        Assert.AreEqual(8f, b1.y);
        Assert.AreEqual(9f, b1.z);
    }

    [Test]
    public void FrameCount_TruncatesPartialTrailingFrame()
    {
        var manifest = BuildManifest();
        manifest.frameCount = 3;
        var data = new float[12]; // 2.4 frames of 5 floats each

        var reader = new RecordingReader(manifest, data);

        Assert.AreEqual(2, reader.FrameCount);
    }
}
}
