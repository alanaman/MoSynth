using AnimationTools;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
/// <summary>
/// Forward kinematics over a <see cref="PoseVector"/> and a <see cref="SkeletonData"/>
/// parent hierarchy (see <see cref="SkeletonAsset.GetSkeletonData"/>). Index 0 is the root
/// (SimulationBone); "world" here is the immediate parent transform of the root.
/// </summary>
public static class PoseVectorFK
{
    /// <summary>
    /// Returns the rotation of the joint in world space after applying FK using the pose
    /// </summary>
    public static quaternion WorldRotation(in SkeletonData skeleton, PoseVector pose, int jointIndex)
    {
        var worldRot = quaternion.identity;
        var index = jointIndex;
        while (index != 0) // while not root
        {
            worldRot = math.mul(pose.jointLocalRotations[index], worldRot);
            index = skeleton.ParentIndices[index];
        }

        worldRot = math.mul(pose.jointLocalRotations[0], worldRot); // root
        return worldRot;
    }

    /// <summary>
    /// Returns the position of the joint in world space after applying FK using the pose
    /// </summary>
    public static float3 WorldPosition(in SkeletonData skeleton, PoseVector pose, int jointIndex)
    {
        var localToWorld = Matrix4x4.identity;
        var index = jointIndex;
        while (index != 0) // while not root
        {
            localToWorld = Matrix4x4.TRS(pose.jointLocalPositions[index], pose.jointLocalRotations[index],
                new float3(1.0f, 1.0f, 1.0f)) * localToWorld;
            index = skeleton.ParentIndices[index];
        }

        localToWorld = Matrix4x4.TRS(pose.jointLocalPositions[0], pose.jointLocalRotations[0], new float3(1.0f, 1.0f, 1.0f)) *
                       localToWorld; // root
        return localToWorld.MultiplyPoint3x4(Vector3.zero);
    }

    /// <summary>
    /// Returns the rotation of the joint in the root space (SimulationBone) after applying FK;
    /// the root's own rotation is excluded.
    /// </summary>
    public static quaternion RootSpaceRotation(in SkeletonData skeleton, PoseVector pose, int jointIndex)
    {
        var rot = quaternion.identity;
        var index = jointIndex;
        while (index != 0) // while not root
        {
            rot = math.mul(pose.jointLocalRotations[index], rot);
            index = skeleton.ParentIndices[index];
        }

        return rot;
    }
}
}
