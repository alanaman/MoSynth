using System;
using System.Collections.Generic;
using NUnit.Framework;

namespace AnimationTools.Tests
{
public class RecordingLayoutTests
{
    [Test]
    public void Build_AssignsSequentialOffsetsInAuthoredOrder()
    {
        var time = new TimeChannel { name = "time" };
        var hips = new BoneWorldPositionChannel { name = "hips" };
        var root = new BoneWorldPositionChannel { name = "root", simulationBone = true };

        var layout = new RecordingLayout(new List<ChannelDescriptor> { time, hips, root });

        var timeHandle = layout.BindChannel(time);
        var hipsHandle = layout.BindChannel(hips);
        var rootHandle = layout.BindChannel(root);

        Assert.AreEqual(0, timeHandle.FloatOffset);
        Assert.AreEqual(2, hipsHandle.FloatOffset);
        Assert.AreEqual(5, rootHandle.FloatOffset);
        Assert.AreEqual(8, layout.FloatCount);
    }

    [Test]
    public void Build_SameConfigDifferentNames_Coexist()
    {
        var a = new BoneWorldPositionChannel { name = "a" };
        var b = new BoneWorldPositionChannel { name = "b" };

        var layout = new RecordingLayout(new List<ChannelDescriptor> { a, b });

        var handleA = layout.BindChannel(a);
        var handleB = layout.BindChannel(b);

        Assert.AreNotEqual(handleA.FloatOffset, handleB.FloatOffset);
    }

    [Test]
    public void Build_DuplicateIdentity_Throws()
    {
        var channels = new List<ChannelDescriptor>
        {
            new TimeChannel { name = "t" },
            new TimeChannel { name = "t" }
        };

        Assert.Throws<ArgumentException>(() => new RecordingLayout(channels));
    }

    [Test]
    public void Channels_UseRecordedSection()
    {
        Assert.AreEqual(RecorderSections.Recorded, new TimeChannel().SectionKey);
        Assert.Greater(RecorderSections.Recorded, ChannelSections.Bool);
    }
}
}
