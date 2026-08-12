using System.Collections.Generic;
using AnimationTools;
using Unity.Mathematics;
using UnityEngine;

namespace AnimationTools.Tests
{
/// <summary>
/// Factories for small <see cref="SkeletonAsset"/> instances shared across the AnimationTools
/// edit-mode test suite. Callers own the returned asset and must Object.DestroyImmediate it,
/// typically from their own [TearDown].
/// </summary>
static class TestSkeletons
{
    public const int RootId = 1;
    public const int SpineId = 2;
    public const int HeadId = 3;
    public const int LeftHandId = 3;
    public const int RightHandId = 4;

    /// <summary>root(1) -&gt; spine(2) -&gt; head(3), each offset (0,1,0) from its parent.</summary>
    public static SkeletonAsset CreateChain3()
    {
        var skeleton = ScriptableObject.CreateInstance<SkeletonAsset>();

        var bones = new List<Bone>
        {
            new()
            {
                id = RootId, name = "root", parentIndex = -1,
                restLocalPosition = float3.zero, restLocalRotation = quaternion.identity,
                humanBone = HumanBodyBones.LastBone
            },
            new()
            {
                id = SpineId, name = "spine", parentIndex = 0,
                restLocalPosition = new float3(0f, 1f, 0f), restLocalRotation = quaternion.identity,
                humanBone = HumanBodyBones.LastBone
            },
            new()
            {
                id = HeadId, name = "head", parentIndex = 1,
                restLocalPosition = new float3(0f, 1f, 0f), restLocalRotation = quaternion.identity,
                humanBone = HumanBodyBones.LastBone
            }
        };

        skeleton.SetBones(bones, 4);
        return skeleton;
    }

    /// <summary>root(1) -&gt; spine(2), spine branches into leftHand(3) and rightHand(4).</summary>
    public static SkeletonAsset CreateBranch4()
    {
        var skeleton = ScriptableObject.CreateInstance<SkeletonAsset>();

        var bones = new List<Bone>
        {
            new()
            {
                id = RootId, name = "root", parentIndex = -1,
                restLocalPosition = float3.zero, restLocalRotation = quaternion.identity,
                humanBone = HumanBodyBones.LastBone
            },
            new()
            {
                id = SpineId, name = "spine", parentIndex = 0,
                restLocalPosition = new float3(0f, 1f, 0f), restLocalRotation = quaternion.identity,
                humanBone = HumanBodyBones.LastBone
            },
            new()
            {
                id = LeftHandId, name = "leftHand", parentIndex = 1,
                restLocalPosition = new float3(-1f, 0f, 0f), restLocalRotation = quaternion.identity,
                humanBone = HumanBodyBones.LastBone
            },
            new()
            {
                id = RightHandId, name = "rightHand", parentIndex = 1,
                restLocalPosition = new float3(1f, 0f, 0f), restLocalRotation = quaternion.identity,
                humanBone = HumanBodyBones.LastBone
            }
        };

        skeleton.SetBones(bones, 5);
        return skeleton;
    }
}
}
