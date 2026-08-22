using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace AnimationTools.Editor
{
    [CustomEditor(typeof(AnnotatedAnimationClip))]
    public class AnimationDataEditor : SkeletonAnimationEditor
    {
        private bool _tagsFoldout;

        private SerializedProperty _clipProp;
        private SerializedProperty _rigProp;
        private SerializedProperty _rootBoneProp;
        private SerializedProperty _startFrameProp;
        private SerializedProperty _endFrameProp;

        protected override void OnEnable()
        {
            base.OnEnable();

            _clipProp = serializedObject.FindProperty("clip");
            _rigProp = serializedObject.FindProperty("rig");
            _rootBoneProp = serializedObject.FindProperty("rootBone");
            _startFrameProp = serializedObject.FindProperty("startFrame");
            _endFrameProp = serializedObject.FindProperty("endFrame");
        }

        public override void OnInspectorGUI()
        {
            AnnotatedAnimationClip clip = (AnnotatedAnimationClip)target;

            serializedObject.Update();

            EditorGUILayout.PropertyField(_clipProp);
            EditorGUILayout.PropertyField(_rigProp);
            EditorGUILayout.PropertyField(_rootBoneProp);
            EditorGUILayout.PropertyField(_startFrameProp);
            EditorGUILayout.PropertyField(_endFrameProp);

            serializedObject.ApplyModifiedProperties();

            var raw = (SkeletonAnimation)target;
            var resolvedRootBone = raw.RootBone;
            if (resolvedRootBone == null)
            {
                EditorGUILayout.HelpBox(
                    "No root bone. Pick one above, or leave it empty to auto-resolve to the rig's " +
                    "single child or a bone whose name ends with 'Hips'.",
                    MessageType.Warning);
            }

            using (new EditorGUI.DisabledScope(true))
            {
                EditorGUILayout.LabelField($"Raw frames: {raw.FrameCount}   Frame time: {raw.FrameTime:F4}s");
                var skeleton = raw.Skeleton;
                EditorGUILayout.LabelField(
                    $"Root bone: {(resolvedRootBone != null ? resolvedRootBone.name : "<unresolved>")}" +
                    $"   Bones: {(skeleton != null ? skeleton.BoneCount.ToString() : "-")}");
            }

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
}
