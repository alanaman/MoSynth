using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using System.Collections.Generic;
using Unity.Mathematics;
using System;
using AnimationTools;

namespace MotionMatching
{
[CustomEditor(typeof(MotionMatchingData))]
public class MotionMatchingDataEditor : UnityEditor.Editor
{
    private bool SkeletonToMecanimFoldout;
    private bool TrajectoryFeaturesSelectorFoldout;
    private bool PoseFeaturesSelectorFoldout;

    private SerializedProperty _animationClipsProperty;
    private SerializedProperty _tPoseAnimationClipProperty;
    

    public void GenerateDatabases(MotionMatchingData mmData)
    {
        PROFILE.BEGIN_SAMPLE_PROFILING("Pose Extract");
        mmData.ImportPoseSet();
        PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Pose Extract");

        PROFILE.BEGIN_SAMPLE_PROFILING("Pose Serialize");
        PoseSerializer poseSerializer = new();
        poseSerializer.Serialize(mmData.GetOrImportPoseSet(), mmData.GetAssetPath(), mmData.name);
        PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Pose Serialize");

        mmData.ComputeJointsLocalForward();

        PROFILE.BEGIN_SAMPLE_PROFILING("Feature Extract");
        mmData.ImportFeatureSet();
        PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Feature Extract");

        PROFILE.BEGIN_SAMPLE_PROFILING("Feature Serialize");
        FeatureSerializer featureSerializer = new();
        featureSerializer.Serialize(mmData.FeatureSet, mmData, mmData.GetAssetPath(), mmData.name);
        PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Feature Serialize");

        AssetDatabase.Refresh();
    }

    private void OnEnable()
    {
        _animationClipsProperty = serializedObject.FindProperty("animationClips");
        _tPoseAnimationClipProperty = serializedObject.FindProperty("tPoseAnimationClip");
    }

    public override void OnInspectorGUI()
    {
        MotionMatchingData data = (MotionMatchingData)target;

        bool generateButtonError = false;

        // BVH
        EditorGUILayout.LabelField("Animations", EditorStyles.boldLabel);
        EditorGUILayout.PropertyField(_animationClipsProperty);

        // BVH TPose
        EditorGUILayout.Separator();
        EditorGUILayout.PropertyField(_tPoseAnimationClipProperty);
        
        
        // Hips Local Vectors --------
        EditorGUILayout.Separator();
        EditorGUILayout.BeginHorizontal();
        EditorGUILayout.LabelField("Hips Local Vectors", EditorStyles.boldLabel);
        if (GUILayout.Button("Auto-Set Hips Vectors", GUILayout.Width(150)))
        {
            HipsLocalVectorsHelperEditorWindow.ShowWindow(data);
        }

        EditorGUILayout.EndHorizontal();
        EditorGUI.indentLevel++;
        // DefaultHipsForward
        data.hipsForwardLocalVector = EditorGUILayout.Vector3Field(
            new GUIContent("Forward Vector", "Local vector (axis) pointing in the forward direction of the hips"),
            data.hipsForwardLocalVector);
        if (math.abs(math.length(data.hipsForwardLocalVector) - 1.0f) > 1E-6f)
        {
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.HelpBox("Hips Forward Local Vector should be normalized", MessageType.Error);
            if (GUILayout.Button("Fix")) data.hipsForwardLocalVector = math.normalize(data.hipsForwardLocalVector);
            EditorGUILayout.EndHorizontal();
            generateButtonError = true;
        }

        // HipsUpLocalVector
        data.hipsUpLocalVector = EditorGUILayout.Vector3Field(
            new GUIContent("Up Vector", "Local vector (axis) pointing in the up direction of the hips"),
            data.hipsUpLocalVector);
        if (math.abs(math.length(data.hipsUpLocalVector) - 1.0f) > 1E-6f)
        {
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.HelpBox("Hips Up Local Vector should be normalized", MessageType.Error);
            if (GUILayout.Button("Fix")) data.hipsUpLocalVector = math.normalize(data.hipsUpLocalVector);
            EditorGUILayout.EndHorizontal();
            generateButtonError = true;
        }

        EditorGUI.indentLevel--;

        // SmoothSimulationBone
        //data.SmoothSimulationBone = EditorGUILayout.Toggle(new GUIContent("Smooth Simulation Bone", "Smooth the simulation bone (articial root added during pose extraction) using Savitzky-Golay filter"),
        //                                                   data.SmoothSimulationBone);

        // ContactVelocityThreshold
        EditorGUILayout.Separator();
        data.contactVelocityThreshold = EditorGUILayout.FloatField(
            new GUIContent("Contact Velocity Threshold",
                "Minimum velocity of the foot to be considered in movement and not in contact with the ground"),
            data.contactVelocityThreshold);

        // SkeletonToMecanim
        EditorGUILayout.Separator();
        if (data.tPoseAnimationClip == null)
        {
            EditorGUILayout.HelpBox("Animation with T-Pose not set", MessageType.Warning);
            return;
        }

        if (GUILayout.Button("Read Skeleton from BVH"))
        {
            // TODO: Check if SkeletonToMecanim should be reset
            data.animationChannelToMecanim.Clear();
            var skeleton = data.tPoseAnimationClip.Skeleton;
            for (var i = 0; i < skeleton.BoneCount; i++)
            {
                var jointName = skeleton.GetBone(i).name;
                HumanBodyBones bone;
                try
                {
                    bone = (HumanBodyBones)Enum.Parse(typeof(HumanBodyBones), jointName);
                }
                catch (Exception)
                {
                    bone = HumanBodyBones.LastBone;
                }

                data.animationChannelToMecanim.Add(new JointToMecanim(jointName, bone));
            }
        }

        // Display SkeletonToMecanim
        SkeletonToMecanimFoldout =
            EditorGUILayout.BeginFoldoutHeaderGroup(SkeletonToMecanimFoldout, "Skeleton to Mecanim");
        if (SkeletonToMecanimFoldout)
        {
            EditorGUI.indentLevel++;
            for (int i = 0; i < data.animationChannelToMecanim.Count; i++)
            {
                JointToMecanim jtm = data.animationChannelToMecanim[i];
                EditorGUILayout.BeginHorizontal();
                GUI.contentColor = jtm.mecanimBone == HumanBodyBones.LastBone
                    ? new Color(1.0f, 0.6f, 0.6f)
                    : Color.white;
                HumanBodyBones newHumanBodyBone = (HumanBodyBones)EditorGUILayout.EnumPopup(jtm.name, jtm.mecanimBone);
                GUI.contentColor = Color.white;
                jtm.mecanimBone = newHumanBodyBone;
                data.animationChannelToMecanim[i] = jtm;
                EditorGUILayout.EndHorizontal();
            }

            EditorGUI.indentLevel--;
        }

        EditorGUILayout.EndFoldoutHeaderGroup();

        // Trajectory Features ------------------------------------------------------------------------------------
        TrajectoryFeaturesSelectorFoldout =
            EditorGUILayout.BeginFoldoutHeaderGroup(TrajectoryFeaturesSelectorFoldout, "Trajectory Features");
        if (TrajectoryFeaturesSelectorFoldout)
        {
            EditorGUI.indentLevel++;
            bool hasAMainPositionFeature = false;
            for (int i = 0; i < data.trajectoryFeatures.Count; i++)
            {
                TrajectoryFeatureChannel trajectoryFeature = data.trajectoryFeatures[i];
                // Header
                EditorGUILayout.BeginVertical(GUI.skin.box);
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField((i + 1).ToString());
                GUILayout.FlexibleSpace();
                if (GUILayout.Button("x"))
                {
                    data.trajectoryFeatures.RemoveAt(i--);
                }

                EditorGUILayout.EndHorizontal();
                // Name
                trajectoryFeature.name = EditorGUILayout.TextField("Name", trajectoryFeature.name);
                // Feature Type
                trajectoryFeature.featureType =
                    (TrajectoryFeatureChannel.Type)EditorGUILayout.EnumPopup("Type",
                        trajectoryFeature.featureType);
                if (trajectoryFeature.featureType == TrajectoryFeatureChannel.Type.Position)
                {
                    trajectoryFeature.isMainPositionFeature = EditorGUILayout.Toggle("Main Position Feature",
                        trajectoryFeature.isMainPositionFeature);
                    if (trajectoryFeature.isMainPositionFeature)
                    {
                        if (hasAMainPositionFeature)
                        {
                            EditorGUILayout.HelpBox("Only one main position feature is allowed", MessageType.Error);
                            generateButtonError = true;
                        }

                        hasAMainPositionFeature = true;
                    }
                }

                generateButtonError = generateButtonError || TrajectoryFramesLayout(trajectoryFeature);
                generateButtonError = generateButtonError || TrajectoryTypeOptionsLayout(trajectoryFeature);
                EditorGUILayout.EndVertical();
            }

            if (GUILayout.Button("Add Trajectory Feature"))
            {
                data.trajectoryFeatures.Add(new TrajectoryFeatureChannel());
            }

            EditorGUI.indentLevel--;
        }

        EditorGUILayout.EndFoldoutHeaderGroup();

        // Pose Features ------------------------------------------------------------------------------------
        PoseFeaturesSelectorFoldout =
            EditorGUILayout.BeginFoldoutHeaderGroup(PoseFeaturesSelectorFoldout, "Pose Features");
        if (PoseFeaturesSelectorFoldout)
        {
            EditorGUI.indentLevel++;
            for (int i = 0; i < data.poseFeatures.Count; i++)
            {
                PoseFeatureChannel poseFeature = data.poseFeatures[i];
                // Header
                EditorGUILayout.BeginVertical(GUI.skin.box);
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField((i + 1).ToString());
                GUILayout.FlexibleSpace();
                if (GUILayout.Button("x"))
                {
                    data.poseFeatures.RemoveAt(i--);
                }

                EditorGUILayout.EndHorizontal();
                //  Properties
                poseFeature.name = EditorGUILayout.TextField("Name", poseFeature.name);
                poseFeature.featureType =
                    (PoseFeatureChannel.Type)EditorGUILayout.EnumPopup("Type", poseFeature.featureType);
                poseFeature.bone = (HumanBodyBones)EditorGUILayout.EnumPopup(poseFeature.bone);
                EditorGUILayout.EndVertical();
            }

            if (GUILayout.Button("Add Pose Feature"))
            {
                data.poseFeatures.Add(new PoseFeatureChannel());
            }

            EditorGUI.indentLevel--;
        }

        EditorGUILayout.EndFoldoutHeaderGroup();

        // Generate Databases
        EditorGUILayout.Separator();
        if (generateButtonError)
        {
            EditorGUILayout.HelpBox(
                "Oops! Looks like there are errors in the feature definition. Please resolve them before generating the databases.",
                MessageType.Error);
            GUI.enabled = false;
        }

        if (GUILayout.Button("Generate Databases", GUILayout.Height(30)))
        {
            GenerateDatabases(data);
        }

        GUI.enabled = true;

        // Error Check
        if (data.JointsLocalForwardError)
        {
            EditorGUILayout.HelpBox("Internal error detected. Please regenerate databases.", MessageType.Error);
        }

        // Save
        if (GUI.changed)
        {
            EditorUtility.SetDirty(target);
            EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        }
    }

    private bool TrajectoryFramesLayout(TrajectoryFeatureChannel trajectoryFeature)
    {
        bool generateButtonError = false;
        EditorGUILayout.LabelField("Frames Prediction");
        EditorGUILayout.BeginHorizontal();
        for (int j = 0; j < trajectoryFeature.predictionFrames.Length; j++)
        {
            trajectoryFeature.predictionFrames[j] = EditorGUILayout.IntField(trajectoryFeature.predictionFrames[j]);
        }

        if (GUILayout.Button("Add"))
        {
            int[] newFrames = new int[trajectoryFeature.predictionFrames.Length + 1];
            for (int j = 0; j < trajectoryFeature.predictionFrames.Length; j++)
                newFrames[j] = trajectoryFeature.predictionFrames[j];
            trajectoryFeature.predictionFrames = newFrames;
        }

        if (trajectoryFeature.predictionFrames.Length > 0 && GUILayout.Button("Remove"))
        {
            int[] newFrames = new int[trajectoryFeature.predictionFrames.Length - 1];
            for (int j = 0; j < trajectoryFeature.predictionFrames.Length - 1; j++)
                newFrames[j] = trajectoryFeature.predictionFrames[j];
            trajectoryFeature.predictionFrames = newFrames;
        }

        EditorGUILayout.EndHorizontal();
        return generateButtonError;
    }

    private bool TrajectoryTypeOptionsLayout(TrajectoryFeatureChannel trajectoryFeature)
    {
        bool generateButtonError = false;
        if (trajectoryFeature.featureType == TrajectoryFeatureChannel.Type.Position ||
            trajectoryFeature.featureType == TrajectoryFeatureChannel.Type.Direction)
        {
            // Bone
            trajectoryFeature.simulationBone =
                EditorGUILayout.Toggle("Simulation Bone", trajectoryFeature.simulationBone);
            if (!trajectoryFeature.simulationBone)
            {
                trajectoryFeature.bone = (HumanBodyBones)EditorGUILayout.EnumPopup("Bone", trajectoryFeature.bone);
                GUI.enabled = !trajectoryFeature.zeroY || !trajectoryFeature.zeroZ;
                trajectoryFeature.zeroX = EditorGUILayout.Toggle("Zero X", trajectoryFeature.zeroX);
                GUI.enabled = !trajectoryFeature.zeroX || !trajectoryFeature.zeroZ;
                trajectoryFeature.zeroY = EditorGUILayout.Toggle("Zero Y", trajectoryFeature.zeroY);
                GUI.enabled = !trajectoryFeature.zeroX || !trajectoryFeature.zeroY;
                trajectoryFeature.zeroZ = EditorGUILayout.Toggle("Zero Z", trajectoryFeature.zeroZ);
                GUI.enabled = true;
            }
            else
            {
                trajectoryFeature.zeroX = false;
                trajectoryFeature.zeroY = true; // project simulation bone to the ground
                trajectoryFeature.zeroZ = false;
            }
        }

        return generateButtonError;
    }
}
}