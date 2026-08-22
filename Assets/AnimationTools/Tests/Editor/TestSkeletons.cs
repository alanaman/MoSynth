using System.Collections.Generic;
using AnimationTools;
using Unity.Mathematics;

namespace AnimationTools.Tests
{
/// <summary>
/// Factories for small <see cref="Skeleton"/> instances shared across the AnimationTools
/// edit-mode test suite. Bone ids follow <see cref="Skeleton.GetBoneId"/>'s index + 1
/// convention, so the constants below double as both index and id helpers for the fixtures.
/// </summary>
static class TestSkeletons
{
    public const int RootId = 1;
    public const int SpineId = 2;
    public const int HeadId = 3;
    public const int LeftHandId = 3;
    public const int RightHandId = 4;

    /// <summary>root(1) -&gt; spine(2) -&gt; head(3), each offset (0,1,0) from its parent.</summary>
    public static Skeleton CreateChain3()
    {
        var bones = new List<SkeletonBoneData>
        {
            new()
            {
                name = "root", parentIndex = -1,
                restLocalPosition = float3.zero, restLocalRotation = quaternion.identity
            },
            new()
            {
                name = "spine", parentIndex = 0,
                restLocalPosition = new float3(0f, 1f, 0f), restLocalRotation = quaternion.identity
            },
            new()
            {
                name = "head", parentIndex = 1,
                restLocalPosition = new float3(0f, 1f, 0f), restLocalRotation = quaternion.identity
            }
        };

        return new Skeleton(bones, "Chain3");
    }

    /// <summary>root(1) -&gt; spine(2), spine branches into leftHand(3) and rightHand(4).</summary>
    public static Skeleton CreateBranch4()
    {
        var bones = new List<SkeletonBoneData>
        {
            new()
            {
                name = "root", parentIndex = -1,
                restLocalPosition = float3.zero, restLocalRotation = quaternion.identity
            },
            new()
            {
                name = "spine", parentIndex = 0,
                restLocalPosition = new float3(0f, 1f, 0f), restLocalRotation = quaternion.identity
            },
            new()
            {
                name = "leftHand", parentIndex = 1,
                restLocalPosition = new float3(-1f, 0f, 0f), restLocalRotation = quaternion.identity
            },
            new()
            {
                name = "rightHand", parentIndex = 1,
                restLocalPosition = new float3(1f, 0f, 0f), restLocalRotation = quaternion.identity
            }
        };

        return new Skeleton(bones, "Branch4");
    }
}
}
