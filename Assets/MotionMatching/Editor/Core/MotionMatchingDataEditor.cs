using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using System.Collections.Generic;
using Unity.Mathematics;
using System;

namespace MotionMatching
{
using Joint = Skeleton.Joint;

[CustomEditor(typeof(MotionMatchingData))]
public class MotionMatchingDataEditor : UnityEditor.Editor
{
    private bool SkeletonToMecanimFoldout;
    private bool TrajectoryFeaturesSelectorFoldout;
    private bool PoseFeaturesSelectorFoldout;
    private bool EnvironmentFeaturesSelectorFoldout;

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

                data.animationChannelToMecanim.Add(new MotionMatchingData.JointToMecanim(jointName, bone));
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
                MotionMatchingData.JointToMecanim jtm = data.animationChannelToMecanim[i];
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
                MotionMatchingData.TrajectoryFeature trajectoryFeature = data.trajectoryFeatures[i];
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
                    (MotionMatchingData.TrajectoryFeature.Type)EditorGUILayout.EnumPopup("Type",
                        trajectoryFeature.featureType);
                if (trajectoryFeature.featureType == MotionMatchingData.TrajectoryFeature.Type.Position)
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
                data.trajectoryFeatures.Add(new MotionMatchingData.TrajectoryFeature());
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
                MotionMatchingData.PoseFeature poseFeature = data.poseFeatures[i];
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
                    (MotionMatchingData.PoseFeature.Type)EditorGUILayout.EnumPopup("Type", poseFeature.featureType);
                poseFeature.bone = (HumanBodyBones)EditorGUILayout.EnumPopup(poseFeature.bone);
                EditorGUILayout.EndVertical();
            }

            if (GUILayout.Button("Add Pose Feature"))
            {
                data.poseFeatures.Add(new MotionMatchingData.PoseFeature());
            }

            EditorGUI.indentLevel--;
        }

        EditorGUILayout.EndFoldoutHeaderGroup();

        // Environment Features ------------------------------------------------------------------------------------
        EnvironmentFeaturesSelectorFoldout =
            EditorGUILayout.BeginFoldoutHeaderGroup(EnvironmentFeaturesSelectorFoldout, "Environment Features");
        if (EnvironmentFeaturesSelectorFoldout)
        {
            EditorGUI.indentLevel++;
            for (int i = 0; i < data.environmentFeatures.Count; i++)
            {
                MotionMatchingData.TrajectoryFeature environmentFeature = data.environmentFeatures[i];
                // Header
                EditorGUILayout.BeginVertical(GUI.skin.box);
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField((i + 1).ToString());
                GUILayout.FlexibleSpace();
                if (GUILayout.Button("x"))
                {
                    data.environmentFeatures.RemoveAt(i--);
                }

                EditorGUILayout.EndHorizontal();
                // Name
                environmentFeature.name = EditorGUILayout.TextField("Name", environmentFeature.name);
                // Feature Type
                environmentFeature.featureType =
                    (MotionMatchingData.TrajectoryFeature.Type)EditorGUILayout.EnumPopup("Type",
                        environmentFeature.featureType);
                environmentFeature.isMainPositionFeature = false;
                generateButtonError = generateButtonError || TrajectoryFramesLayout(environmentFeature);
                generateButtonError = generateButtonError || TrajectoryTypeOptionsLayout(environmentFeature);
                EditorGUILayout.EndVertical();
            }

            if (GUILayout.Button("Add Environment Feature"))
            {
                data.environmentFeatures.Add(new MotionMatchingData.TrajectoryFeature());
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

    private bool TrajectoryFramesLayout(MotionMatchingData.TrajectoryFeature trajectoryFeature)
    {
        bool generateButtonError = false;
        EditorGUILayout.LabelField("Frames Prediction");
        EditorGUILayout.BeginHorizontal();
        for (int j = 0; j < trajectoryFeature.framesPrediction.Length; j++)
        {
            trajectoryFeature.framesPrediction[j] = EditorGUILayout.IntField(trajectoryFeature.framesPrediction[j]);
        }

        if (GUILayout.Button("Add"))
        {
            int[] newFrames = new int[trajectoryFeature.framesPrediction.Length + 1];
            for (int j = 0; j < trajectoryFeature.framesPrediction.Length; j++)
                newFrames[j] = trajectoryFeature.framesPrediction[j];
            trajectoryFeature.framesPrediction = newFrames;
        }

        if (trajectoryFeature.framesPrediction.Length > 0 && GUILayout.Button("Remove"))
        {
            int[] newFrames = new int[trajectoryFeature.framesPrediction.Length - 1];
            for (int j = 0; j < trajectoryFeature.framesPrediction.Length - 1; j++)
                newFrames[j] = trajectoryFeature.framesPrediction[j];
            trajectoryFeature.framesPrediction = newFrames;
        }

        EditorGUILayout.EndHorizontal();
        return generateButtonError;
    }

    private bool TrajectoryTypeOptionsLayout(MotionMatchingData.TrajectoryFeature trajectoryFeature)
    {
        bool generateButtonError = false;
        if (trajectoryFeature.featureType == MotionMatchingData.TrajectoryFeature.Type.Position ||
            trajectoryFeature.featureType == MotionMatchingData.TrajectoryFeature.Type.Direction)
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
        else
        {
            if (trajectoryFeature.featureType == MotionMatchingData.TrajectoryFeature.Type.Custom1D)
            {
                trajectoryFeature.featureExtractor = EditorGUILayout.ObjectField(
                    new GUIContent("Feature1DExtractor",
                        "ScriptableObject inheriting from the 'Feature1DExtractor' class"),
                    trajectoryFeature.featureExtractor, typeof(Feature1DExtractor), false) as ScriptableObject;
                if (trajectoryFeature.featureExtractor == null)
                {
                    EditorGUILayout.HelpBox(
                        "Please enter an instance of a ScriptableObject inheriting from the 'Feature1DExtractor' class",
                        MessageType.Error);
                    generateButtonError = true;
                }
            }
            else if (trajectoryFeature.featureType == MotionMatchingData.TrajectoryFeature.Type.Custom2D)
            {
                trajectoryFeature.featureExtractor = EditorGUILayout.ObjectField(
                    new GUIContent("Feature2DExtractor",
                        "ScriptableObject inheriting from the 'Feature2DExtractor' class"),
                    trajectoryFeature.featureExtractor, typeof(Feature2DExtractor), false) as ScriptableObject;
                if (trajectoryFeature.featureExtractor == null)
                {
                    EditorGUILayout.HelpBox(
                        "Please enter an instance of a ScriptableObject inheriting from the 'Feature2DExtractor' class",
                        MessageType.Error);
                    generateButtonError = true;
                }
            }
            else if (trajectoryFeature.featureType == MotionMatchingData.TrajectoryFeature.Type.Custom3D)
            {
                trajectoryFeature.featureExtractor = EditorGUILayout.ObjectField(
                    new GUIContent("Feature3DExtractor",
                        "ScriptableObject inheriting from the 'Feature3DExtractor' class"),
                    trajectoryFeature.featureExtractor, typeof(Feature3DExtractor), false) as ScriptableObject;
                if (trajectoryFeature.featureExtractor == null)
                {
                    EditorGUILayout.HelpBox(
                        "Please enter an instance of a ScriptableObject inheriting from the 'Feature3DExtractor' class",
                        MessageType.Error);
                    generateButtonError = true;
                }
            }
            else if (trajectoryFeature.featureType == MotionMatchingData.TrajectoryFeature.Type.Custom4D)
            {
                trajectoryFeature.featureExtractor = EditorGUILayout.ObjectField(
                    new GUIContent("Feature4DExtractor",
                        "ScriptableObject inheriting from the 'Feature4DExtractor' class"),
                    trajectoryFeature.featureExtractor, typeof(Feature4DExtractor), false) as ScriptableObject;
                if (trajectoryFeature.featureExtractor == null)
                {
                    EditorGUILayout.HelpBox(
                        "Please enter an instance of a ScriptableObject inheriting from the 'Feature4DExtractor' class",
                        MessageType.Error);
                    generateButtonError = true;
                }
            }
        }

        return generateButtonError;
    }
}
}