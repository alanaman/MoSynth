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
            
            // Save
            if (GUI.changed)
            {
                EditorUtility.SetDirty(target);
                EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());
            }
        }
    }
}
