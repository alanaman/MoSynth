using System.Collections;
using System.Collections.Generic;
using System.Diagnostics.Contracts;
using UnityEngine;
using static UnityEngine.GraphicsBuffer;
#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
#endif

namespace MotionMatching
{
/// <summary>
/// Defines an animation used by MotionMatchingData.
/// Single representation of an animation clip (or mocap file).
/// It also stores metadata such as tags.
/// </summary>
[CreateAssetMenu(fileName = "AnimationData", menuName = "MotionMatching/AnimationData")]
public class AnimationData : ScriptableObject
{
    public BvhAnimation Animation;
    public List<Tag> Tags = new();

    [Pure]
    public BvhAnimation GetAnimation()
    {
        return Animation;
    }

    public void SetAnimation(BvhAnimation animation)
    {
        Animation = animation;
    }

    public List<Tag> GetTags()
    {
        return Tags;
    }

    public void AddTag(string name)
    {
        Tag newTag = new Tag
        {
            Name = name
        };
        Tags.Add(newTag);
#if UNITY_EDITOR
        SaveEditor();
#endif
    }

    public void RemoveTag(int index)
    {
        for (int i = index + 1; i < Tags.Count; ++i)
        {
            Tags[i - 1] = Tags[i];
        }

        Tags.RemoveAt(Tags.Count - 1);
#if UNITY_EDITOR
        SaveEditor();
#endif
    }

    public void UpdateMecanimInformation(MotionMatchingData mmData)
    {
        Animation.UpdateMecanimInformation(mmData);
    }

    [System.Serializable]
    public struct Tag
    {
        public string Name;
        public int[] Start; // Each element with index i, where, 0 <= i <= Start.Length == End.Length
        public int[] End; // represents a range. That is, for an arbitrary i -> [Start[i], End[i]]
    }

#if UNITY_EDITOR
    public void SaveEditor()
    {
        EditorUtility.SetDirty(this);
    }
#endif
}

#if UNITY_EDITOR
[CustomEditor(typeof(AnimationData))]
public class AnimationDataEditor : UnityEditor.Editor
{
    private bool TagsFoldout;

    public override void OnInspectorGUI()
    {
        AnimationData data = (AnimationData)target;

        // BVH
        data.Animation = (BvhAnimation)EditorGUILayout.ObjectField(data.Animation, typeof(BvhAnimation), false);
        // Tags
        TagsFoldout = EditorGUILayout.BeginFoldoutHeaderGroup(TagsFoldout, "Tags");
        if (TagsFoldout)
        {
            EditorGUI.indentLevel++;
            if (data.Tags == null || data.Tags.Count == 0)
            {
                EditorGUILayout.HelpBox("To include tags, access the 'MotionMatching/Animation Viewer' window.",
                    MessageType.Info);
            }

            GUI.enabled = false;
            for (int tagIndex = 0; tagIndex < (data.Tags == null ? 0 : data.Tags.Count); ++tagIndex)
            {
                AnimationData.Tag tag = data.Tags[tagIndex];
                tag.Name = EditorGUILayout.TextField(tag.Name);
                for (int rangeIndex = 0; rangeIndex < (tag.Start == null ? 0 : tag.Start.Length); ++rangeIndex)
                {
                    EditorGUILayout.BeginHorizontal();
                    EditorGUILayout.LabelField(tag.Start[rangeIndex].ToString());
                    EditorGUILayout.LabelField(tag.End[rangeIndex].ToString());
                    EditorGUILayout.EndHorizontal();
                }
            }

            GUI.enabled = true;
            EditorGUI.indentLevel--;
        }

        EditorGUILayout.EndFoldoutHeaderGroup();
        // Save
        if (GUI.changed)
        {
            EditorUtility.SetDirty(target);
            EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        }
    }
}
#endif
}
