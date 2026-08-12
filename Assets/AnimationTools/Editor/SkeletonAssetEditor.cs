using UnityEditor;
using UnityEngine;

namespace AnimationTools.Editor
{
/// <summary>Read-only inspector: lists the bone hierarchy. Editing happens through authoring tools.</summary>
[CustomEditor(typeof(SkeletonAsset))]
public class SkeletonAssetEditor : UnityEditor.Editor
{
    private const float DepthIndent = 12f;

    public override void OnInspectorGUI()
    {
        var skeleton = (SkeletonAsset)target;

        EditorGUILayout.LabelField($"{skeleton.BoneCount} bone(s)", $"version {skeleton.Version}");
        EditorGUILayout.Space();

        for (var i = 0; i < skeleton.BoneCount; i++)
        {
            DrawBoneRow(skeleton, i);
        }
    }

    private static void DrawBoneRow(SkeletonAsset skeleton, int index)
    {
        var bone = skeleton.GetBone(index);
        var depth = DepthOf(skeleton, index);

        using (new EditorGUILayout.HorizontalScope())
        {
            GUILayout.Space(depth * DepthIndent);
            EditorGUILayout.LabelField(bone.name);
            EditorGUILayout.LabelField($"id:{bone.id}", GUILayout.Width(50f));

            if (bone.humanBone != HumanBodyBones.LastBone)
            {
                EditorGUILayout.LabelField(bone.humanBone.ToString(), EditorStyles.miniLabel, GUILayout.Width(110f));
            }
        }
    }

    /// <summary>Bounded by bone count so a malformed parent chain cannot hang the inspector.</summary>
    private static int DepthOf(SkeletonAsset skeleton, int index)
    {
        var depth = 0;
        var parent = skeleton.GetBone(index).parentIndex;
        var guard = 0;
        while (parent >= 0 && guard++ < skeleton.BoneCount)
        {
            depth++;
            parent = skeleton.GetBone(parent).parentIndex;
        }

        return depth;
    }
}
}
