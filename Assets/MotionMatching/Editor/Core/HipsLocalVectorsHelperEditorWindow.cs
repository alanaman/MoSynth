using AnimationTools;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;

namespace MotionMatching
{
public class HipsLocalVectorsHelperEditorWindow : EditorWindow
{
    public static HipsLocalVectorsHelperEditorWindow Window;
    private SceneGUI ReturnButton;
    private MotionMatchingData MMData;
    private Transform[] Skeleton;
    private AnnotatedAnimationClip _animationClip;

    public static void ShowWindow(MotionMatchingData mmData)
    {
        Window = GetWindow<HipsLocalVectorsHelperEditorWindow>();
        Window.titleContent = new GUIContent("Auto-Set Hips Local Vectors");
        // Open new scene
        EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo();
        var currentScene = EditorSceneManager.GetActiveScene().path;
        var scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);

        Window.ReturnButton = new SceneGUI(currentScene);
        SceneView.duringSceneGui += Window.UpdateGUI;

        Window.MMData = mmData;
        Window._animationClip = mmData.tPoseAnimationClip;

        Window.ImportBVH();
    }

    private void OnGUI()
    {
        EditorGUILayout.LabelField(
            "Rotate the skeleton to align its forward direction with world Z (Blue) and its up direction with world Y (Green).  Click 'Set Hips Local Vectors' to apply.",
            EditorStyles.wordWrappedLabel);
        EditorGUILayout.Space();
        if (GUILayout.Button("Set Hips Local Vectors"))
        {
            Undo.RecordObject(MMData, "Set Hips Local Vectors");
            MMData.hipsForwardLocalVector = Skeleton[0].InverseTransformDirection(Vector3.forward);
            MMData.hipsUpLocalVector = Skeleton[0].InverseTransformDirection(Vector3.up);
            EditorUtility.SetDirty(MMData);

            Debug.Log("Hips Local Forward Vector is set to " + MMData.hipsForwardLocalVector);
            Debug.Log("Hips Local Up Vector is set to " + MMData.hipsUpLocalVector);

            Close();
        }
    }

    private void ImportBVH()
    {
        RemoveSkeleton();
        var skeletonAsset = _animationClip.Skeleton;
        var rawAnimation = _animationClip.GetRawAnimationClip();
        if (skeletonAsset == null || rawAnimation.FrameCount == 0) return;

        var rawFrame0 = rawAnimation.GetFrame(0);
        var rawRotations = rawFrame0.Rotations;

        // Create skeleton
        Skeleton = new Transform[skeletonAsset.BoneCount];
        for (var j = 0; j < Skeleton.Length; j++)
        {
            // Joints
            var bone = skeletonAsset.GetBone(j);
            var t = (new GameObject()).transform;
            t.name = bone.name;
            if (j > 0)
            {
                t.SetParent(Skeleton[bone.parentIndex], false);
            }

            t.SetLocalPositionAndRotation(bone.restLocalPosition, rawRotations[j]);
            Skeleton[j] = t;
            // Visual
            var visual = (new GameObject()).transform;
            visual.name = "Visual";
            visual.SetParent(t, false);
            visual.localScale = new Vector3(0.1f, 0.1f, 0.1f);
            visual.localPosition = Vector3.zero;
            visual.localRotation = Quaternion.identity;
            // Sphere
            var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere).transform;
            sphere.name = "Sphere";
            sphere.SetParent(visual, false);
            sphere.localScale = Vector3.one;
            sphere.localPosition = Vector3.zero;
            sphere.localRotation = Quaternion.identity;
            if (j > 0)
            {
                // Capsule
                var capsule = GameObject.CreatePrimitive(PrimitiveType.Capsule).transform;
                capsule.name = "Capsule";
                capsule.SetParent(Skeleton[bone.parentIndex].Find("Visual"), false);
                var distance = Vector3.Distance(t.position, t.parent.position) * (1.0f / visual.localScale.y) * 0.5f;
                capsule.localScale = new Vector3(0.5f, distance, 0.5f);
                var up = (t.position - t.parent.position).normalized;
                capsule.localPosition = t.parent.InverseTransformDirection(up) * distance;
                capsule.localRotation = Quaternion.Inverse(t.parent.rotation) *
                                        Quaternion.LookRotation(new Vector3(-up.y, up.x, 0.0f), up);
            }
        }
    }

    private void RemoveSkeleton()
    {
        if (Skeleton != null)
        {
            for (var i = 0; i < Skeleton.Length; i++)
            {
                if (Skeleton[i] != null)
                {
                    DestroyImmediate(Skeleton[i].gameObject);
                    Skeleton[i] = null;
                }
            }

            Skeleton = null;
        }
    }

    private void UpdateGUI(SceneView sceneView)
    {
        ReturnButton.RenderSceneGUI(sceneView);
    }

    private void OnDestroy()
    {
        ReturnButton.ReturnScene();
        SceneView.duringSceneGui -= UpdateGUI;
    }
}
}