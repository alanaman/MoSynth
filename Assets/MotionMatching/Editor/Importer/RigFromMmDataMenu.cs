using AnimationTools;
using UnityEditor;
using UnityEngine;

namespace MotionMatching.Editor
{
/// <summary>
/// Migration tool: builds a scene rig matching a <see cref="MotionMatchingData"/>'s generated
/// database, including the SimulationBone root the database extraction prepends. Use it to get a
/// rig to assign to feature/contact <see cref="BoneTransform"/> fields when no imported rig asset
/// already provides one.
/// </summary>
public static class RigFromMmDataMenu
{
    [MenuItem("MotionMatching/Create Rig From MotionMatchingData")]
    private static void CreateRigFromMotionMatchingData()
    {
        var data = (MotionMatchingData)Selection.activeObject;

        if (!PoseSerializer.TryDeserializeSkeleton(data.GetAssetPath(), data.name, out var skeleton))
        {
            Debug.LogError(
                $"[RigFromMmDataMenu] No .mmskeleton found for \"{data.name}\" at {data.GetAssetPath()}. " +
                "Generate Databases on the MotionMatchingData first.");
            return;
        }

        var rigRoot = SkeletonRigBuilder.CreateHierarchy(skeleton);
        Undo.RegisterCreatedObjectUndo(rigRoot.gameObject, "Create Rig From MotionMatchingData");
        Selection.activeGameObject = rigRoot.gameObject;
    }

    [MenuItem("MotionMatching/Create Rig From MotionMatchingData", true)]
    private static bool ValidateCreateRigFromMotionMatchingData()
    {
        return Selection.activeObject is MotionMatchingData;
    }
}
}
