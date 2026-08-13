using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;
using AnimationTools;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.SceneManagement;
#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
#endif

namespace MotionMatching
{
/// <summary>
/// Stores annotation using tags and other information for a raw animation clip.
/// To be used by Motion Synthesis Systems.
/// </summary>
[CreateAssetMenu(fileName = "AnimationData", menuName = "MotionMatching/AnimationData")]
public class AnnotatedAnimationClip : ScriptableObject
{
    [SerializeField]
    private BvhAnimation animation;
    public List<Tag> tags = new();

    [Min(0)] [Tooltip("Start frame of the animation clip. 0 indexing, inclusive.")]
    public int startFrame;

    [Min(0)] [Tooltip("End frame of the animation clip. 0 indexing, exclusive.")]
    public int endFrame;

    public SkeletonAsset Skeleton => animation.Skeleton;

    /// Number of frames in the [startFrame, endFrame) slice, clamped defensively — a serialized
    /// endFrame can exceed the animation's frame count until OnValidate re-runs.
    public int FrameCount => animation == null ? 0 : Math.Max(0, Math.Min(endFrame, animation.FrameCount) - startFrame);

    public float FrameTime => animation.FrameTime;

    [Pure]
    public BvhAnimation GetRawAnimationClip()
    {
        return animation;
    }

    /// Frame view offset by startFrame. Never Dispose the returned buffer.
    public PoseBuffer GetFrame(int frameIndex) => animation.PoseSequence.GetFrame(startFrame + frameIndex);

    public List<Tag> GetTags()
    {
        return tags;
    }

    public void AddTag(string tagName)
    {
        Tag newTag = new Tag
        {
            name = tagName
        };
        tags.Add(newTag);
#if UNITY_EDITOR
        SaveEditor();
#endif
    }

    public void RemoveTag(int index)
    {
        for (int i = index + 1; i < tags.Count; ++i)
        {
            tags[i - 1] = tags[i];
        }

        tags.RemoveAt(tags.Count - 1);
#if UNITY_EDITOR
        SaveEditor();
#endif
    }

    public void UpdateMecanimInformation(IPoseSetSource source)
    {
        animation.UpdateMecanimInformation(source);
    }

    private void OnValidate()
    {
        if (animation == null)
            return;

        if (startFrame >= animation.FrameCount)
            startFrame = animation.FrameCount;

        if (endFrame >= animation.FrameCount)
            endFrame = animation.FrameCount;
    }

    /// <summary>
    /// Character-space (accumulated parent-chain) rotation of a bone at a sliced frame index.
    /// </summary>
    public quaternion GetWorldRotation(int boneIndex, int frameIndex)
        => PoseFK.CharacterRotation(GetFrame(frameIndex), animation.Skeleton.GetSkeletonData(), boneIndex);

    [Serializable]
    public struct Tag
    {
        public string name;
        public int[] start; // Each element with index i, where, 0 <= i <= Start.Length == End.Length
        public int[] end; // represents a range. That is, for an arbitrary i -> [Start[i], End[i]]
    }

#if UNITY_EDITOR
    public void SaveEditor()
    {
        EditorUtility.SetDirty(this);
    }
#endif
}

#if UNITY_EDITOR
[CustomEditor(typeof(AnnotatedAnimationClip))]
public class AnimationDataEditor : UnityEditor.Editor
{
    private bool _tagsFoldout;

    private SerializedProperty _animationProp;
    private SerializedProperty _startFrameProp;
    private SerializedProperty _endFrameProp;

    private void OnEnable()
    {
        // Link the serialized properties
        _animationProp = serializedObject.FindProperty("animation");
        _startFrameProp = serializedObject.FindProperty("startFrame");
        _endFrameProp = serializedObject.FindProperty("endFrame");
    }

    public override void OnInspectorGUI()
    {
        AnnotatedAnimationClip clip = (AnnotatedAnimationClip)target;

        serializedObject.Update();

        // BVH & Frames (Using PropertyField automatically triggers OnValidate when changed)
        EditorGUILayout.PropertyField(_animationProp);
        EditorGUILayout.PropertyField(_startFrameProp);
        EditorGUILayout.PropertyField(_endFrameProp);

        // Apply changes to the serialized object (This is what actually calls OnValidate)
        serializedObject.ApplyModifiedProperties();

        // Tags
        _tagsFoldout = EditorGUILayout.BeginFoldoutHeaderGroup(_tagsFoldout, "Tags");
        if (_tagsFoldout)
        {
            EditorGUI.indentLevel++;
            if (clip.tags == null || clip.tags.Count == 0)
            {
                EditorGUILayout.HelpBox("To include tags, access the 'MotionMatching/Animation Viewer' window.",
                    MessageType.Info);
            }

            GUI.enabled = false;
            for (int tagIndex = 0; tagIndex < (clip.tags == null ? 0 : clip.tags.Count); ++tagIndex)
            {
                AnnotatedAnimationClip.Tag tag = clip.tags[tagIndex];
                tag.name = EditorGUILayout.TextField(tag.name);
                for (int rangeIndex = 0; rangeIndex < (tag.start == null ? 0 : tag.start.Length); ++rangeIndex)
                {
                    EditorGUILayout.BeginHorizontal();
                    EditorGUILayout.LabelField(tag.start[rangeIndex].ToString());
                    EditorGUILayout.LabelField(tag.end[rangeIndex].ToString());
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
            EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());
        }
    }
}
#endif
}