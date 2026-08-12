using AnimationTools;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching.Tests
{
/// <summary>
/// Parity tests between <see cref="PoseFK"/> (AnimationTools) and the legacy
/// <see cref="Skeleton"/> FK methods it was ported from, so the two stay numerically
/// equivalent for as long as both representations coexist.
/// </summary>
public class PoseFkParityTests
{
    private const float Tolerance = 1e-4f;

    private Skeleton mmSkeleton;
    private SkeletonAsset asset;
    private SkeletonMap map;
    private PoseVectorAdapter adapter;
    private SkeletonData skeletonData;

    [SetUp]
    public void SetUp()
    {
        mmSkeleton = MmTestData.BuildMmSkeleton();
        asset = MmTestData.BuildSkeletonAsset(mmSkeleton);
        map = SkeletonMap.Build(mmSkeleton, asset);
        Assert.IsNotNull(map, "SkeletonMap.Build should match every synthetic MM joint by name.");
        Assert.IsTrue(map.IsIdentity, "A SkeletonAsset built directly from the MM skeleton should map identity.");
        adapter = new PoseVectorAdapter(asset, map);
        skeletonData = asset.GetSkeletonData();
    }

    [TearDown]
    public void TearDown()
    {
        if (asset != null) UnityEngine.Object.DestroyImmediate(asset);
        asset = null;
    }

    [TestCase(1u)]
    [TestCase(2u)]
    [TestCase(42u)]
    public void SyntheticRandomPose_MatchesLegacyFK(uint seed)
    {
        var pose = MmTestData.BuildRandomPose(mmSkeleton, seed);
        AssertParity(mmSkeleton, pose, adapter, skeletonData);
    }

    [TestCase(0)]
    [TestCase(100)]
    [TestCase(1000)]
    public void DemoPose_MatchesLegacyFK(int poseIndex)
    {
        if (!MmTestData.TryLoadDemoPose(out var poseSet, out var demoSkeleton))
        {
            Assert.Ignore("Demo MotionMatchingData is not available in this environment.");
            return;
        }

        var demoAsset = MmTestData.BuildSkeletonAsset(demoSkeleton);
        try
        {
            var demoMap = SkeletonMap.Build(demoSkeleton, demoAsset);
            Assert.IsNotNull(demoMap, "SkeletonMap.Build should match every demo MM joint by name.");
            var demoAdapter = new PoseVectorAdapter(demoAsset, demoMap);
            var demoSkeletonData = demoAsset.GetSkeletonData();

            var clampedIndex = poseIndex < poseSet.NumberPoses ? poseIndex : poseSet.NumberPoses - 1;
            poseSet.GetPose(clampedIndex, out var pose);

            AssertParity(demoSkeleton, pose, demoAdapter, demoSkeletonData);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(demoAsset);
        }
    }

    private static void AssertParity(Skeleton mm, PoseVector pose, PoseVectorAdapter adapter, SkeletonData skeletonData)
    {
        var buffer = PoseBuffer.Allocate(adapter.Layout, Allocator.Temp);
        try
        {
            adapter.Write(pose, buffer);

            var boneCount = skeletonData.BoneCount;
            var outPositions = new NativeArray<float3>(boneCount, Allocator.Temp);
            var outRotations = new NativeArray<quaternion>(boneCount, Allocator.Temp);
            try
            {
                PoseFK.LocalToCharacter(buffer, skeletonData, outPositions, outRotations);

                for (var i = 0; i < boneCount; i++)
                {
                    var joint = mm.Joints[i];

                    var expectedPosition = mm.GetWorldSpacePosition(joint, pose);
                    var expectedRotation = mm.GetWorldSpaceRotation(joint, pose);
                    var expectedVelocity = mm.GetWorldSpaceVelocity(joint, pose);

                    var actualPosition = PoseFK.CharacterPosition(buffer, skeletonData, i);
                    var actualRotation = PoseFK.CharacterRotation(buffer, skeletonData, i);
                    var actualVelocity = PoseFK.CharacterVelocity(buffer, skeletonData, i);

                    AssertApprox(expectedPosition, actualPosition);
                    AssertApproxRotation(expectedRotation, actualRotation);
                    AssertApprox(expectedVelocity, actualVelocity);

                    // The batched walk must agree with the per-bone chain walk for every bone.
                    AssertApprox(actualPosition, outPositions[i]);
                    AssertApproxRotation(actualRotation, outRotations[i]);
                }
            }
            finally
            {
                outPositions.Dispose();
                outRotations.Dispose();
            }
        }
        finally
        {
            buffer.Dispose();
        }
    }

    private static void AssertApprox(float3 expected, float3 actual)
    {
        Assert.AreEqual(expected.x, actual.x, Tolerance);
        Assert.AreEqual(expected.y, actual.y, Tolerance);
        Assert.AreEqual(expected.z, actual.z, Tolerance);
    }

    private static void AssertApproxRotation(quaternion expected, quaternion actual)
    {
        // q and -q represent the same rotation; compare via |dot| instead of aligning hemispheres.
        Assert.AreEqual(1f, math.abs(math.dot(expected.value, actual.value)), Tolerance);
    }
}
}
