using System.Collections.Generic;
using System.IO;
using AnimationTools;
using NUnit.Framework;
using UnityEngine;

namespace MotionMatching.Tests
{
public class PoseSerializerRoundTripTests
{
    private const float FrameTime = 1f / 30f;

    [Test]
    public void SerializeThenDeserialize_RoundTripsPosesClipsAndTags()
    {
        var skeleton = MmTestData.BuildSkeletonAsset();
        var poseSet = new PoseSet();
        poseSet.SetSkeleton(skeleton);

        const int firstCount = 4;
        const int secondCount = 3;

        var firstFrames = poseSet.BeginClip(firstCount, FrameTime);
        for (var i = 0; i < firstCount; i++)
        {
            var frame = firstFrames[i];
            MmTestData.FillRandomPose(frame, skeleton, (uint)(i + 1));
            frame.SetBool(poseSet.LeftFootContactHandle, i % 2 == 0);
            frame.SetBool(poseSet.RightFootContactHandle, i % 3 == 0);
        }

        var tag = new AnnotatedAnimationClip.Tag
        {
            name = "Jump",
            start = new[] { 0 },
            end = new[] { 2 }
        };
        poseSet.EndClip(new List<AnnotatedAnimationClip.Tag> { tag });

        var secondFrames = poseSet.BeginClip(secondCount, FrameTime);
        for (var i = 0; i < secondCount; i++)
        {
            var frame = secondFrames[i];
            MmTestData.FillRandomPose(frame, skeleton, (uint)(i + 1) + 500);
            frame.SetBool(poseSet.LeftFootContactHandle, i % 2 == 1);
            frame.SetBool(poseSet.RightFootContactHandle, i % 2 == 0);
        }

        poseSet.EndClip(new List<AnnotatedAnimationClip.Tag>());
        poseSet.ConvertTagsToNativeArrays();

        var tempDir = Path.Combine(Application.temporaryCachePath,
            "PoseSerializerRoundTripTests_" + Path.GetRandomFileName());
        const string fileName = "RoundTrip";
        PoseSet loaded = null;

        try
        {
            new PoseSerializer().Serialize(poseSet, tempDir, fileName);

            var serializer = new PoseSerializer();
            Assert.IsTrue(serializer.Deserialize(tempDir, fileName, out loaded));

            Assert.AreEqual(poseSet.NumberPoses, loaded.NumberPoses);
            Assert.AreEqual(poseSet.NumberClips, loaded.NumberClips);
            Assert.AreEqual(poseSet.FrameTime, loaded.FrameTime);
            Assert.AreEqual(skeleton.BoneCount, loaded.SkeletonAsset.BoneCount);

            for (var i = 0; i < poseSet.NumberClips; i++)
            {
                var expectedClip = poseSet.GetAnimationClip(i);
                var actualClip = loaded.GetAnimationClip(i);
                Assert.AreEqual(expectedClip.Start, actualClip.Start);
                Assert.AreEqual(expectedClip.End, actualClip.End);
                Assert.AreEqual(expectedClip.FrameTime, actualClip.FrameTime);
            }

            for (var i = 0; i < poseSet.NumberPoses; i++)
            {
                var expected = poseSet.GetPoseBuffer(i);
                var actual = loaded.GetPoseBuffer(i);

                var expectedPositions = expected.Positions;
                var actualPositions = actual.Positions;
                var expectedRotations = expected.Rotations;
                var actualRotations = actual.Rotations;
                var expectedVelocities = expected.Velocities;
                var actualVelocities = actual.Velocities;
                var expectedAngularVelocities = expected.AngularVelocities;
                var actualAngularVelocities = actual.AngularVelocities;

                for (var j = 0; j < skeleton.BoneCount; j++)
                {
                    Assert.AreEqual(expectedPositions[j], actualPositions[j]);
                    Assert.AreEqual(expectedRotations[j].value, actualRotations[j].value);
                    Assert.AreEqual(expectedVelocities[j], actualVelocities[j]);
                    Assert.AreEqual(expectedAngularVelocities[j], actualAngularVelocities[j]);
                }

                Assert.AreEqual(expected.GetBool(poseSet.LeftFootContactHandle), actual.GetBool(loaded.LeftFootContactHandle));
                Assert.AreEqual(expected.GetBool(poseSet.RightFootContactHandle), actual.GetBool(loaded.RightFootContactHandle));
            }

            Assert.AreEqual(poseSet.NumberTags, loaded.NumberTags);
            var expectedTag = poseSet.GetTag("Jump");
            var actualTag = loaded.GetTag("Jump");
            Assert.AreEqual(expectedTag.NumberRanges, actualTag.NumberRanges);
            for (var r = 0; r < expectedTag.NumberRanges; r++)
            {
                expectedTag.GetRange(r, out var expectedStart, out var expectedEnd);
                actualTag.GetRange(r, out var actualStart, out var actualEnd);
                Assert.AreEqual(expectedStart, actualStart);
                Assert.AreEqual(expectedEnd, actualEnd);
            }
        }
        finally
        {
            if (Directory.Exists(tempDir)) Directory.Delete(tempDir, true);
            loaded?.Dispose();
            poseSet.Dispose();
        }
    }
}
}
