using UnityEngine;
using UnityEditor;
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

    private bool _generateButtonError;

    private SerializedProperty _animationClipsProperty;
    private SerializedProperty _tPoseAnimationClipProperty;
    private SerializedProperty _hipsForwardLocalVectorProperty;
    private SerializedProperty _hipsUpLocalVectorProperty;
    private SerializedProperty _contactVelocityThresholdProperty;
    private SerializedProperty _animationChannelToMecanimProperty;
    private SerializedProperty _trajectoryFeaturesProperty;
    private SerializedProperty _poseFeaturesProperty;

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
        _hipsForwardLocalVectorProperty = serializedObject.FindProperty("hipsForwardLocalVector");
        _hipsUpLocalVectorProperty = serializedObject.FindProperty("hipsUpLocalVector");
        _contactVelocityThresholdProperty = serializedObject.FindProperty("contactVelocityThreshold");
        _animationChannelToMecanimProperty = serializedObject.FindProperty("animationChannelToMecanim");
        _trajectoryFeaturesProperty = serializedObject.FindProperty("trajectoryFeatures");
        _poseFeaturesProperty = serializedObject.FindProperty("poseFeatures");
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        var data = (MotionMatchingData)target;
        _generateButtonError = false;

        DrawAnimations();
        _generateButtonError |= DrawHipsVectors(data);

        // SmoothSimulationBone
        //data.SmoothSimulationBone = EditorGUILayout.Toggle(new GUIContent("Smooth Simulation Bone", "Smooth the simulation bone (articial root added during pose extraction) using Savitzky-Golay filter"),
        //                                                   data.SmoothSimulationBone);

        DrawContactThreshold();
        DrawSkeletonToMecanim(data);
        DrawTrajectoryFeatures();
        DrawPoseFeatures();
        DrawGenerateButton(data);

        serializedObject.ApplyModifiedProperties();
    }

    // BVH
    private void DrawAnimations()
    {
        EditorGUILayout.LabelField("Animations", EditorStyles.boldLabel);
        EditorGUILayout.PropertyField(_animationClipsProperty);

        // BVH TPose
        EditorGUILayout.Separator();
        EditorGUILayout.PropertyField(_tPoseAnimationClipProperty);
    }

    // Hips Local Vectors --------
    private bool DrawHipsVectors(MotionMatchingData data)
    {
        var error = false;

        EditorGUILayout.Separator();
        EditorGUILayout.BeginHorizontal();
        EditorGUILayout.LabelField("Hips Local Vectors", EditorStyles.boldLabel);
        if (GUILayout.Button("Auto-Set Hips Vectors", GUILayout.Width(150)))
        {
            // ShowWindow opens a new scene, which rebuilds the inspector and disposes its
            // SerializedObject mid-draw; defer it until after this GUI pass has completed
            EditorApplication.delayCall += () => HipsLocalVectorsHelperEditorWindow.ShowWindow(data);
        }

        EditorGUILayout.EndHorizontal();
        EditorGUI.indentLevel++;

        // DefaultHipsForward
        EditorGUILayout.PropertyField(_hipsForwardLocalVectorProperty,
            new GUIContent("Forward Vector", "Local vector (axis) pointing in the forward direction of the hips"));
        var hipsForward = ReadFloat3(_hipsForwardLocalVectorProperty);
        if (math.abs(math.length(hipsForward) - 1.0f) > 1E-6f)
        {
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.HelpBox("Hips Forward Local Vector should be normalized", MessageType.Error);
            if (GUILayout.Button("Fix")) WriteFloat3(_hipsForwardLocalVectorProperty, math.normalize(hipsForward));
            EditorGUILayout.EndHorizontal();
            error = true;
        }

        // HipsUpLocalVector
        EditorGUILayout.PropertyField(_hipsUpLocalVectorProperty,
            new GUIContent("Up Vector", "Local vector (axis) pointing in the up direction of the hips"));
        var hipsUp = ReadFloat3(_hipsUpLocalVectorProperty);
        if (math.abs(math.length(hipsUp) - 1.0f) > 1E-6f)
        {
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.HelpBox("Hips Up Local Vector should be normalized", MessageType.Error);
            if (GUILayout.Button("Fix")) WriteFloat3(_hipsUpLocalVectorProperty, math.normalize(hipsUp));
            EditorGUILayout.EndHorizontal();
            error = true;
        }

        EditorGUI.indentLevel--;

        return error;
    }

    // ContactVelocityThreshold
    private void DrawContactThreshold()
    {
        EditorGUILayout.Separator();
        EditorGUILayout.PropertyField(_contactVelocityThresholdProperty,
            new GUIContent("Contact Velocity Threshold",
                "Minimum velocity of the foot to be considered in movement and not in contact with the ground"));
    }

    // SkeletonToMecanim
    private void DrawSkeletonToMecanim(MotionMatchingData data)
    {
        EditorGUILayout.Separator();
        if (data.tPoseAnimationClip == null)
        {
            EditorGUILayout.HelpBox("Animation with T-Pose not set", MessageType.Warning);
        }

        GUI.enabled = data.tPoseAnimationClip != null;
        if (GUILayout.Button("Read Skeleton from BVH"))
        {
            // TODO: Check if SkeletonToMecanim should be reset
            var skeleton = data.tPoseAnimationClip.Skeleton;
            _animationChannelToMecanimProperty.arraySize = skeleton.BoneCount;
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

                var element = _animationChannelToMecanimProperty.GetArrayElementAtIndex(i);
                element.FindPropertyRelative("name").stringValue = jointName;
                element.FindPropertyRelative("mecanimBone").intValue = (int)bone;
            }
        }

        GUI.enabled = true;

        // Display SkeletonToMecanim
        SkeletonToMecanimFoldout =
            EditorGUILayout.BeginFoldoutHeaderGroup(SkeletonToMecanimFoldout, "Skeleton to Mecanim");
        if (SkeletonToMecanimFoldout)
        {
            EditorGUI.indentLevel++;
            for (var i = 0; i < _animationChannelToMecanimProperty.arraySize; i++)
            {
                var element = _animationChannelToMecanimProperty.GetArrayElementAtIndex(i);
                var nameProp = element.FindPropertyRelative("name");
                var mecanimProp = element.FindPropertyRelative("mecanimBone");
                EditorGUILayout.BeginHorizontal();
                GUI.contentColor = (HumanBodyBones)mecanimProp.intValue == HumanBodyBones.LastBone
                    ? new Color(1.0f, 0.6f, 0.6f)
                    : Color.white;
                var newHumanBodyBone =
                    (HumanBodyBones)EditorGUILayout.EnumPopup(nameProp.stringValue, (HumanBodyBones)mecanimProp.intValue);
                GUI.contentColor = Color.white;
                mecanimProp.intValue = (int)newHumanBodyBone;
                EditorGUILayout.EndHorizontal();
            }

            EditorGUI.indentLevel--;
        }

        EditorGUILayout.EndFoldoutHeaderGroup();
    }

    // Trajectory Features ------------------------------------------------------------------------------------
    private void DrawTrajectoryFeatures()
    {
        TrajectoryFeaturesSelectorFoldout =
            EditorGUILayout.BeginFoldoutHeaderGroup(TrajectoryFeaturesSelectorFoldout, "Trajectory Features");
        if (TrajectoryFeaturesSelectorFoldout)
        {
            EditorGUI.indentLevel++;
            var hasAMainPositionFeature = false;
            // Deleting mid-loop would change the control count between the layout and repaint passes
            var removeIndex = -1;
            for (var i = 0; i < _trajectoryFeaturesProperty.arraySize; i++)
            {
                var trajectoryFeature = _trajectoryFeaturesProperty.GetArrayElementAtIndex(i);
                // Header
                EditorGUILayout.BeginVertical(GUI.skin.box);
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField((i + 1).ToString());
                GUILayout.FlexibleSpace();
                if (GUILayout.Button("x"))
                {
                    removeIndex = i;
                }

                EditorGUILayout.EndHorizontal();
                // Name
                var nameProp = trajectoryFeature.FindPropertyRelative("name");
                nameProp.stringValue = EditorGUILayout.TextField("Name", nameProp.stringValue);
                // Feature Type
                var featureTypeProp = trajectoryFeature.FindPropertyRelative("featureType");
                featureTypeProp.intValue = (int)(TrajectoryFeatureChannel.Type)EditorGUILayout.EnumPopup("Type",
                    (TrajectoryFeatureChannel.Type)featureTypeProp.intValue);
                if ((TrajectoryFeatureChannel.Type)featureTypeProp.intValue == TrajectoryFeatureChannel.Type.Position)
                {
                    var isMainPositionFeatureProp = trajectoryFeature.FindPropertyRelative("isMainPositionFeature");
                    isMainPositionFeatureProp.boolValue =
                        EditorGUILayout.Toggle("Main Position Feature", isMainPositionFeatureProp.boolValue);
                    if (isMainPositionFeatureProp.boolValue)
                    {
                        if (hasAMainPositionFeature)
                        {
                            EditorGUILayout.HelpBox("Only one main position feature is allowed", MessageType.Error);
                            _generateButtonError = true;
                        }

                        hasAMainPositionFeature = true;
                    }
                }

                _generateButtonError = _generateButtonError || TrajectoryFramesLayout(trajectoryFeature);
                _generateButtonError = _generateButtonError || TrajectoryTypeOptionsLayout(trajectoryFeature);

                EditorGUILayout.EndVertical();
            }

            if (removeIndex >= 0)
            {
                _trajectoryFeaturesProperty.DeleteArrayElementAtIndex(removeIndex);
            }

            if (GUILayout.Button("Add Trajectory Feature"))
            {
                var insertIndex = _trajectoryFeaturesProperty.arraySize;
                _trajectoryFeaturesProperty.arraySize++;
                ResetTrajectoryFeature(_trajectoryFeaturesProperty.GetArrayElementAtIndex(insertIndex));
            }

            EditorGUI.indentLevel--;
        }

        EditorGUILayout.EndFoldoutHeaderGroup();
    }

    // Pose Features ------------------------------------------------------------------------------------
    private void DrawPoseFeatures()
    {
        PoseFeaturesSelectorFoldout =
            EditorGUILayout.BeginFoldoutHeaderGroup(PoseFeaturesSelectorFoldout, "Pose Features");
        if (PoseFeaturesSelectorFoldout)
        {
            EditorGUI.indentLevel++;
            // Deleting mid-loop would change the control count between the layout and repaint passes
            var removeIndex = -1;
            for (var i = 0; i < _poseFeaturesProperty.arraySize; i++)
            {
                var poseFeature = _poseFeaturesProperty.GetArrayElementAtIndex(i);
                // Header
                EditorGUILayout.BeginVertical(GUI.skin.box);
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField((i + 1).ToString());
                GUILayout.FlexibleSpace();
                if (GUILayout.Button("x"))
                {
                    removeIndex = i;
                }

                EditorGUILayout.EndHorizontal();
                //  Properties
                var nameProp = poseFeature.FindPropertyRelative("name");
                nameProp.stringValue = EditorGUILayout.TextField("Name", nameProp.stringValue);
                var featureTypeProp = poseFeature.FindPropertyRelative("featureType");
                featureTypeProp.intValue = (int)(PoseFeatureChannel.Type)EditorGUILayout.EnumPopup("Type",
                    (PoseFeatureChannel.Type)featureTypeProp.intValue);
                var boneProp = poseFeature.FindPropertyRelative("bone");
                boneProp.intValue = (int)(HumanBodyBones)EditorGUILayout.EnumPopup((HumanBodyBones)boneProp.intValue);

                EditorGUILayout.EndVertical();
            }

            if (removeIndex >= 0)
            {
                _poseFeaturesProperty.DeleteArrayElementAtIndex(removeIndex);
            }

            if (GUILayout.Button("Add Pose Feature"))
            {
                var insertIndex = _poseFeaturesProperty.arraySize;
                _poseFeaturesProperty.arraySize++;
                ResetPoseFeature(_poseFeaturesProperty.GetArrayElementAtIndex(insertIndex));
            }

            EditorGUI.indentLevel--;
        }

        EditorGUILayout.EndFoldoutHeaderGroup();
    }

    // Generate Databases
    private void DrawGenerateButton(MotionMatchingData data)
    {
        EditorGUILayout.Separator();
        if (_generateButtonError)
        {
            EditorGUILayout.HelpBox(
                "Oops! Looks like there are errors in the feature definition. Please resolve them before generating the databases.",
                MessageType.Error);
        }

        GUI.enabled = !_generateButtonError && data.tPoseAnimationClip != null;
        if (GUILayout.Button("Generate Databases", GUILayout.Height(30)))
        {
            serializedObject.ApplyModifiedProperties();
            GenerateDatabases(data);
            EditorUtility.SetDirty(data);
            serializedObject.Update();
        }

        GUI.enabled = true;

        // Error Check
        if (data.JointsLocalForwardError)
        {
            EditorGUILayout.HelpBox("Internal error detected. Please regenerate databases.", MessageType.Error);
        }
    }

    private static float3 ReadFloat3(SerializedProperty property) => new(
        property.FindPropertyRelative("x").floatValue,
        property.FindPropertyRelative("y").floatValue,
        property.FindPropertyRelative("z").floatValue);

    private static void WriteFloat3(SerializedProperty property, float3 value)
    {
        property.FindPropertyRelative("x").floatValue = value.x;
        property.FindPropertyRelative("y").floatValue = value.y;
        property.FindPropertyRelative("z").floatValue = value.z;
    }

    private static void ResetTrajectoryFeature(SerializedProperty element)
    {
        element.FindPropertyRelative("name").stringValue = "";
        element.FindPropertyRelative("featureType").intValue = 0;
        element.FindPropertyRelative("simulationBone").boolValue = false;
        element.FindPropertyRelative("bone").intValue = 0;
        element.FindPropertyRelative("zeroX").boolValue = false;
        element.FindPropertyRelative("zeroY").boolValue = false;
        element.FindPropertyRelative("zeroZ").boolValue = false;
        element.FindPropertyRelative("isMainPositionFeature").boolValue = false;
        element.FindPropertyRelative("predictionFrames").arraySize = 0;
    }

    private static void ResetPoseFeature(SerializedProperty element)
    {
        element.FindPropertyRelative("name").stringValue = "";
        element.FindPropertyRelative("featureType").intValue = 0;
        element.FindPropertyRelative("bone").intValue = 0;
    }

    private bool TrajectoryFramesLayout(SerializedProperty trajectoryFeature)
    {
        var generateButtonError = false;
        var predictionFrames = trajectoryFeature.FindPropertyRelative("predictionFrames");
        EditorGUILayout.LabelField("Frames Prediction");
        EditorGUILayout.BeginHorizontal();
        for (var j = 0; j < predictionFrames.arraySize; j++)
        {
            var element = predictionFrames.GetArrayElementAtIndex(j);
            element.intValue = EditorGUILayout.IntField(element.intValue);
        }

        if (GUILayout.Button("Add"))
        {
            var addIndex = predictionFrames.arraySize;
            predictionFrames.arraySize++;
            // Growing an array copies the previous element rather than default-initialising it
            predictionFrames.GetArrayElementAtIndex(addIndex).intValue = 0;
        }

        if (predictionFrames.arraySize > 0 && GUILayout.Button("Remove"))
        {
            predictionFrames.arraySize--;
        }

        EditorGUILayout.EndHorizontal();
        return generateButtonError;
    }

    private bool TrajectoryTypeOptionsLayout(SerializedProperty trajectoryFeature)
    {
        var generateButtonError = false;
        var featureType = (TrajectoryFeatureChannel.Type)trajectoryFeature.FindPropertyRelative("featureType").intValue;
        if (featureType == TrajectoryFeatureChannel.Type.Position ||
            featureType == TrajectoryFeatureChannel.Type.Direction)
        {
            // Bone
            var simulationBoneProp = trajectoryFeature.FindPropertyRelative("simulationBone");
            var boneProp = trajectoryFeature.FindPropertyRelative("bone");
            var zeroXProp = trajectoryFeature.FindPropertyRelative("zeroX");
            var zeroYProp = trajectoryFeature.FindPropertyRelative("zeroY");
            var zeroZProp = trajectoryFeature.FindPropertyRelative("zeroZ");

            simulationBoneProp.boolValue = EditorGUILayout.Toggle("Simulation Bone", simulationBoneProp.boolValue);
            if (!simulationBoneProp.boolValue)
            {
                boneProp.intValue =
                    (int)(HumanBodyBones)EditorGUILayout.EnumPopup("Bone", (HumanBodyBones)boneProp.intValue);
                GUI.enabled = !zeroYProp.boolValue || !zeroZProp.boolValue;
                zeroXProp.boolValue = EditorGUILayout.Toggle("Zero X", zeroXProp.boolValue);
                GUI.enabled = !zeroXProp.boolValue || !zeroZProp.boolValue;
                zeroYProp.boolValue = EditorGUILayout.Toggle("Zero Y", zeroYProp.boolValue);
                GUI.enabled = !zeroXProp.boolValue || !zeroYProp.boolValue;
                zeroZProp.boolValue = EditorGUILayout.Toggle("Zero Z", zeroZProp.boolValue);
                GUI.enabled = true;
            }
            else
            {
                zeroXProp.boolValue = false;
                zeroYProp.boolValue = true; // project simulation bone to the ground
                zeroZProp.boolValue = false;
            }
        }

        return generateButtonError;
    }
}
}
