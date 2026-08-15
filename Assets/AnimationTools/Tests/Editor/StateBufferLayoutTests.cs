using System;
using System.Collections.Generic;
using AnimationTools;
using NUnit.Framework;

namespace AnimationTools.Tests
{
public class StateBufferLayoutTests
{
    private const int SectionA = 0;
    private const int SectionB = 5;

    private sealed class TestChannel : ChannelDescriptor
    {
        private readonly int _id;
        private readonly int _sectionKey;

        public TestChannel(int id, int floatCount = 1, int sectionKey = SectionA)
        {
            _id = id;
            FloatCount = floatCount;
            _sectionKey = sectionKey;
        }

        public override int FloatCount { get; }
        public override int SectionKey => _sectionKey;

        public override bool Equals(ChannelDescriptor other) => other is TestChannel t && t._id == _id;
        public override int GetHashCode() => _id;
        public override int GetContentHash() => _id;
    }

    private sealed class TestLayout : StateBufferLayout
    {
        public TestLayout(IReadOnlyList<ChannelDescriptor> channels)
            : base(channels, HashChannels(17, channels))
        {
        }
    }

    private static List<ChannelDescriptor> BuildTwoSectionChannels()
    {
        return new List<ChannelDescriptor>
        {
            new TestChannel(1, 3, SectionA),
            new TestChannel(2, 2, SectionA),
            new TestChannel(3, 4, SectionB)
        };
    }

    [Test]
    public void Build_AssignsSequentialOffsetsWithinAndAcrossSections()
    {
        var layout = new TestLayout(BuildTwoSectionChannels());

        var first = layout.BindChannel(new TestChannel(1));
        var second = layout.BindChannel(new TestChannel(2));
        var third = layout.BindChannel(new TestChannel(3, sectionKey: SectionB));

        Assert.AreEqual(0, first.FloatOffset);
        Assert.AreEqual(3, first.FloatCount);
        Assert.AreEqual(3, second.FloatOffset);
        Assert.AreEqual(2, second.FloatCount);
        Assert.AreEqual(5, third.FloatOffset);
        Assert.AreEqual(4, third.FloatCount);
        Assert.AreEqual(9, layout.FloatCount);
    }

    [Test]
    public void TryBindChannel_UndeclaredChannel_ReturnsFalseWithInvalidHandle()
    {
        var layout = new TestLayout(BuildTwoSectionChannels());

        var found = layout.TryBindChannel(new TestChannel(99), out var handle);

        Assert.IsFalse(found);
        Assert.IsFalse(handle.IsValid);
    }

    [Test]
    public void BindChannel_UndeclaredChannel_Throws()
    {
        var layout = new TestLayout(BuildTwoSectionChannels());

        Assert.Throws<ArgumentException>(() => layout.BindChannel(new TestChannel(99)));
    }

    [Test]
    public void TryGetSectionRange_DeclaredSectionsReturnRanges_AbsentSectionReturnsFalse()
    {
        var layout = new TestLayout(BuildTwoSectionChannels());

        Assert.IsTrue(layout.TryGetSectionRange(SectionA, out var start, out var floatCount));
        Assert.AreEqual(0, start);
        Assert.AreEqual(5, floatCount);

        Assert.IsTrue(layout.TryGetSectionRange(SectionB, out start, out floatCount));
        Assert.AreEqual(5, start);
        Assert.AreEqual(4, floatCount);

        Assert.IsFalse(layout.TryGetSectionRange(SectionB + 1, out _, out _));
    }

    [Test]
    public void GetSectionElementCount_MatchesDeclaredChannelCountAndZeroWhenAbsent()
    {
        var layout = new TestLayout(BuildTwoSectionChannels());

        Assert.AreEqual(2, layout.GetSectionElementCount(SectionA));
        Assert.AreEqual(1, layout.GetSectionElementCount(SectionB));
        Assert.AreEqual(0, layout.GetSectionElementCount(SectionB + 1));
    }

    [Test]
    public void Constructor_DuplicateChannelIdentity_Throws()
    {
        var channels = new List<ChannelDescriptor> { new TestChannel(1), new TestChannel(1) };

        Assert.Throws<ArgumentException>(() => new TestLayout(channels));
    }
}
}
