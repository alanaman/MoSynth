using System.Collections.Generic;
using AnimationTools;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

namespace MotionMatching.Tests
{
/// <summary>
/// Shared fixtures for the MotionMatching-coupled edit-mode test suite: a small synthetic
/// MM skeleton/pose pair for fast deterministic tests, plus an opt-in loader for the real
/// demo pose database so a handful of tests can also run against production data.
/// </summary>
static class MmTestData
{
    /// <summary>SimulationBone(0) -&gt; Hips(1) -&gt; {Spine(2), LeftFoot(3) -&gt; LeftToe(4)}.</summary>
    public static SkeletonAsset BuildSkeletonAsset()
    {
        var bones = new List<Bone>
        {
            new()
            {
                id = 1, name = "SimulationBone", parentIndex = -1, restLocalPosition = float3.zero,
                restLocalRotation = quaternion.identity, humanBone = HumanBodyBones.LastBone
            },
            new()
            {
                id = 2, name = "Hips", parentIndex = 0, restLocalPosition = new float3(0f, 1f, 0f),
                restLocalRotation = quaternion.identity, humanBone = HumanBodyBones.Hips
            },
            new()
            {
                id = 3, name = "Spine", parentIndex = 1, restLocalPosition = new float3(0f, 0.2f, 0f),
                restLocalRotation = quaternion.identity, humanBone = HumanBodyBones.Spine
            },
            new()
            {
                id = 4, name = "LeftFoot", parentIndex = 1, restLocalPosition = new float3(0.2f, -0.9f, 0f),
                restLocalRotation = quaternion.identity, humanBone = HumanBodyBones.LeftFoot
            },
            new()
            {
                id = 5, name = "LeftToe", parentIndex = 3, restLocalPosition = new float3(0f, -0.1f, 0.15f),
                restLocalRotation = quaternion.identity, humanBone = HumanBodyBones.LeftToes
            }
        };

        return PoseSet.CreateRuntimeSkeletonAsset(bones);
    }

    /// <summary>
    /// Fills <paramref name="pose"/> with a deterministic pseudo-random pose over
    /// <paramref name="asset"/>. Positions are only meaningful for the root (0) and hips (1),
    /// matching <see cref="PoseExtractor"/>'s convention -- every other joint holds its rest
    /// offset. Rotations are uniformly distributed normalized quaternions; velocities/angular
    /// velocities are uniform in [-2, 2]. Contacts are left untouched (Allocate zero-initializes
    /// them, i.e. false).
    /// </summary>
    public static void FillRandomPose(PoseBuffer pose, SkeletonAsset asset, uint seed)
    {
        var random = new Unity.Mathematics.Random(seed == 0 ? 1u : seed);
        var boneCount = asset.BoneCount;
        var positions = pose.Positions;
        var rotations = pose.Rotations;
        var velocities = pose.Velocities;
        var angularVelocities = pose.AngularVelocities;

        for (var i = 0; i < boneCount; i++)
        {
            positions[i] = i <= 1 ? random.NextFloat3(-2f, 2f) : asset.GetBone(i).restLocalPosition;
            rotations[i] = random.NextQuaternionRotation();
            velocities[i] = random.NextFloat3(-2f, 2f);
            angularVelocities[i] = random.NextFloat3(-2f, 2f);
        }
    }

    /// <summary>
    /// Loads the checked-in demo pose database. Returns false (rather than throwing) on any
    /// failure so demo-guarded tests can <c>Assert.Ignore</c> when it isn't available.
    /// </summary>
    public static bool TryLoadDemoPose(out PoseSet poseSet, out SkeletonAsset skeleton)
    {
        poseSet = null;
        skeleton = null;

        var data = AssetDatabase.LoadAssetAtPath<MotionMatchingData>(
            "Assets/Animation/MotionMatching/MotionMatchingData.asset");
        if (data == null) return false;

        try
        {
            poseSet = data.GetOrImportPoseSet();
        }
        catch
        {
            return false;
        }

        if (poseSet == null || poseSet.NumberPoses == 0) return false;

        skeleton = poseSet.SkeletonAsset;
        return skeleton != null;
    }
}
}
