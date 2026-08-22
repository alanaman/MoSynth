using AnimationTools;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
/// <summary>
/// Forward kinematics over a full-pose <see cref="PoseBuffer"/> (element index == bone index)
/// and a <see cref="SkeletonData"/> parent hierarchy (see
/// <see cref="Skeleton.GetSkeletonData"/>). Index 0 is the root (SimulationBone);
/// "world" here is the immediate parent transform of the root.
/// </summary>
/// <remarks>
/// Keeps the Matrix4x4-based arithmetic the shipped feature databases were generated with;
/// regenerated databases must stay byte-identical. TODO: consolidate with
/// <see cref="PoseFK"/> once byte parity with shipped databases is no longer required.
/// </remarks>
public static class PoseBufferFK
{
    /// <summary>
    /// Returns the rotation of the joint in world space after applying FK using the pose
    /// </summary>
    public static quaternion WorldRotation(in SkeletonData skeleton, PoseBuffer pose, int jointIndex)
    {
        var rotations = pose.Rotations;
        var worldRot = quaternion.identity;
        var index = jointIndex;
        while (index != 0) // while not root
        {
            worldRot = math.mul(rotations[index], worldRot);
            index = skeleton.ParentIndices[index];
        }

        worldRot = math.mul(rotations[0], worldRot); // root
        return worldRot;
    }

    /// <summary>
    /// Returns the position of the joint in world space after applying FK using the pose
    /// </summary>
    public static float3 WorldPosition(in SkeletonData skeleton, PoseBuffer pose, int jointIndex)
    {
        var positions = pose.Positions;
        var rotations = pose.Rotations;
        var localToWorld = Matrix4x4.identity;
        var index = jointIndex;
        while (index != 0) // while not root
        {
            localToWorld = Matrix4x4.TRS(positions[index], rotations[index],
                new float3(1.0f, 1.0f, 1.0f)) * localToWorld;
            index = skeleton.ParentIndices[index];
        }

        localToWorld = Matrix4x4.TRS(positions[0], rotations[0], new float3(1.0f, 1.0f, 1.0f)) *
                       localToWorld; // root
        return localToWorld.MultiplyPoint3x4(Vector3.zero);
    }

    /// <summary>
    /// Returns the rotation of the joint in the root space (SimulationBone) after applying FK;
    /// the root's own rotation is excluded.
    /// </summary>
    public static quaternion RootSpaceRotation(in SkeletonData skeleton, PoseBuffer pose, int jointIndex)
    {
        var rotations = pose.Rotations;
        var rot = quaternion.identity;
        var index = jointIndex;
        while (index != 0) // while not root
        {
            rot = math.mul(rotations[index], rot);
            index = skeleton.ParentIndices[index];
        }

        return rot;
    }
    
    
}
}
