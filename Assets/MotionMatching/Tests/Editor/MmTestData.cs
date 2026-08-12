using System;
using System.Collections.Generic;
using AnimationTools;
using AnimationTools.Editor;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
// Types below (Skeleton, PoseVector, PoseSet, MotionMatchingData) live in the enclosing
// MotionMatching namespace and resolve unqualified; no `using MotionMatching;` needed.

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
    public static Skeleton BuildMmSkeleton()
    {
        var skeleton = new Skeleton();
        skeleton.AddJoint(new Skeleton.Joint("SimulationBone", 0, -1, float3.zero, HumanBodyBones.LastBone));
        skeleton.AddJoint(new Skeleton.Joint("Hips", 1, 0, new float3(0f, 1f, 0f), HumanBodyBones.Hips));
        skeleton.AddJoint(new Skeleton.Joint("Spine", 2, 1, new float3(0f, 0.2f, 0f), HumanBodyBones.Spine));
        skeleton.AddJoint(new Skeleton.Joint("LeftFoot", 3, 1, new float3(0.2f, -0.9f, 0f), HumanBodyBones.LeftFoot));
        skeleton.AddJoint(new Skeleton.Joint("LeftToe", 4, 3, new float3(0f, -0.1f, 0.15f), HumanBodyBones.LeftToes));
        return skeleton;
    }

    /// <summary>Converts an MM skeleton into a fresh, in-memory <see cref="SkeletonAsset"/>.</summary>
    public static SkeletonAsset BuildSkeletonAsset(Skeleton mm)
    {
        var asset = ScriptableObject.CreateInstance<SkeletonAsset>();
        SkeletonAssetAuthoring.BuildOrMerge(asset, ConvertToBoneSources(mm));
        return asset;
    }

    // Mirrors MotionMatching.SkeletonAssetFromMmData.ConvertToBoneSources, which lives in the
    // editor-only MotionMatching.Editor assembly that this test asmdef doesn't reference.
    private static List<SkeletonAssetAuthoring.BoneSource> ConvertToBoneSources(Skeleton mmSkeleton)
    {
        var joints = mmSkeleton.Joints;
        var source = new List<SkeletonAssetAuthoring.BoneSource>(joints.Count);

        for (var i = 0; i < joints.Count; i++)
        {
            var joint = joints[i];

            var parentIndex = joint.parentIndex;
            if (parentIndex == i || parentIndex < 0)
            {
                if (i != 0)
                {
                    throw new InvalidOperationException(
                        $"MM joint \"{joint.name}\" at index {i} has no valid parent " +
                        $"(parentIndex {joint.parentIndex}) but is not the root.");
                }

                parentIndex = -1;
            }

            source.Add(new SkeletonAssetAuthoring.BoneSource
            {
                name = joint.name,
                parentIndex = parentIndex,
                restLocalPosition = joint.localOffset,
                restLocalRotation = quaternion.identity,
                humanBone = joint.type
            });
        }

        return source;
    }

    /// <summary>
    /// A deterministic pseudo-random pose over <paramref name="mm"/>. Positions are only
    /// meaningful for the root (0) and hips (1), matching <see cref="PoseExtractor"/>'s
    /// convention -- every other joint holds its rest offset. Rotations are uniformly
    /// distributed normalized quaternions; velocities/angular velocities are uniform in
    /// [-2, 2]. Both contacts are false.
    /// </summary>
    public static PoseVector BuildRandomPose(Skeleton mm, uint seed)
    {
        var random = new Unity.Mathematics.Random(seed == 0 ? 1u : seed);
        var jointCount = mm.Joints.Count;
        var pose = new PoseVector(jointCount);

        for (var i = 0; i < jointCount; i++)
        {
            pose.jointLocalPositions[i] = i <= 1 ? random.NextFloat3(-2f, 2f) : mm.Joints[i].localOffset;
            pose.jointLocalRotations[i] = RandomRotation(ref random);
            pose.jointLocalVelocities[i] = random.NextFloat3(-2f, 2f);
            pose.jointLocalAngularVelocities[i] = random.NextFloat3(-2f, 2f);
        }

        pose.leftFootContact = false;
        pose.rightFootContact = false;
        return pose;
    }

    private static quaternion RandomRotation(ref Unity.Mathematics.Random random)
    {
        return random.NextQuaternionRotation();
    }

    /// <summary>
    /// Loads the checked-in demo pose database. Returns false (rather than throwing) on any
    /// failure so demo-guarded tests can <c>Assert.Ignore</c> when it isn't available.
    /// </summary>
    public static bool TryLoadDemoPose(out PoseSet poseSet, out Skeleton skeleton)
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

        skeleton = poseSet.Skeleton;
        return skeleton != null;
    }
}
}
