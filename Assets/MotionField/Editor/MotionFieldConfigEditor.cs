using System;
using System.IO;
using MotionMatching;
using Python.Runtime;
using UnityEditor;
using UnityEngine;

namespace MotionField.Editor
{
/// <summary>
/// Inspector for <see cref="MotionFieldConfig"/>: extract the pose database, then train the value
/// function over it.
///
/// Training runs in-process through PythonNET, synchronously on the main thread. It takes roughly
/// 20 s on a GPU for a 7.8k-state database, which is short enough that a frozen editor beats the
/// alternative: numpy and scipy hold the GIL for hundreds of milliseconds at a stretch, so a
/// background worker would stall play mode anyway, cannot be aborted (Thread.Abort is gone), and
/// hard-crashes the editor if a domain reload lands while it holds the GIL.
/// </summary>
[CustomEditor(typeof(MotionFieldConfig))]
public class MotionFieldConfigEditor : UnityEditor.Editor
{
    private MotionMatchingData _importSource;
    private bool _showImport;

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        var config = (MotionFieldConfig)target;

        EditorGUILayout.Space();
        DrawImportSection(config);

        EditorGUILayout.Space();
        DrawDatabaseSection(config);

        EditorGUILayout.Space();
        DrawTrainingSection(config);

        EditorGUILayout.Space();
        DrawVisualizationSection(config);

        if (GUI.changed)
        {
            EditorUtility.SetDirty(config);
        }
    }

    private void DrawImportSection(MotionFieldConfig config)
    {
        _showImport = EditorGUILayout.Foldout(_showImport, "Import Settings From MotionMatchingData", true);
        if (!_showImport) return;

        EditorGUILayout.HelpBox(
            "One-time copy of the clips, hips forward vector, contact threshold and Mecanim map. " +
            "It does not create a link -- later edits to the source asset will not follow.",
            MessageType.Info);

        _importSource = (MotionMatchingData)EditorGUILayout.ObjectField(
            "Source", _importSource, typeof(MotionMatchingData), false);

        using (new EditorGUI.DisabledScope(_importSource == null))
        {
            if (GUILayout.Button("Copy Settings"))
            {
                Undo.RecordObject(config, "Import MotionField settings");
                config.animationClips = new System.Collections.Generic.List<AnnotatedAnimationClip>(
                    _importSource.animationClips);
                config.hipsForwardLocalVector = _importSource.hipsForwardLocalVector;
                config.contactVelocityThreshold = _importSource.contactVelocityThreshold;
                config.animationChannelToMecanim =
                    new System.Collections.Generic.List<MotionMatchingData.JointToMecanim>(
                        _importSource.animationChannelToMecanim);
                EditorUtility.SetDirty(config);
                Debug.Log($"[MotionField] Copied {config.animationClips.Count} clip(s) and " +
                          $"{config.animationChannelToMecanim.Count} bone mapping(s) from " +
                          $"'{_importSource.name}'.");
            }
        }
    }

    private void DrawDatabaseSection(MotionFieldConfig config)
    {
        EditorGUILayout.LabelField("Pose Database", EditorStyles.boldLabel);
        EditorGUILayout.LabelField("Output", ProjectRelative(config.GetAssetPath()));

        bool hasClips = config.animationClips != null && config.animationClips.Count > 0;
        if (!hasClips)
        {
            EditorGUILayout.HelpBox("Assign at least one animation clip.", MessageType.Warning);
        }
        else if (config.animationChannelToMecanim == null || config.animationChannelToMecanim.Count == 0)
        {
            EditorGUILayout.HelpBox(
                "No Mecanim bone mapping. Pose extraction locates the toes through it for foot " +
                "contact detection and will fail without it.", MessageType.Warning);
        }

        using (new EditorGUI.DisabledScope(!hasClips))
        {
            if (GUILayout.Button("Generate Pose Database", GUILayout.Height(24)))
            {
                GeneratePoseDatabase(config);
            }
        }
    }

    private void DrawTrainingSection(MotionFieldConfig config)
    {
        EditorGUILayout.LabelField("Value Function", EditorStyles.boldLabel);

        string valuePath = config.GetValueFunctionPath();
        bool databaseExists = File.Exists(Path.Combine(config.GetAssetPath(), config.name + ".mmpose"));
        bool trained = File.Exists(valuePath);

        EditorGUILayout.LabelField("Trained file",
            trained
                ? $"{ProjectRelative(valuePath)}  ({new FileInfo(valuePath).Length / 1024} KB)"
                : "not trained yet");

        if (!databaseExists)
        {
            EditorGUILayout.HelpBox("Generate the pose database first.", MessageType.Warning);
        }

        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            EditorGUILayout.HelpBox(
                "Training is disabled in play mode. It would hold the Python GIL for seconds at a " +
                "time and stall the running MotionFieldStage.", MessageType.Info);
        }

        using (new EditorGUI.DisabledScope(!databaseExists || EditorApplication.isPlayingOrWillChangePlaymode))
        {
            if (GUILayout.Button("Train Motion Field", GUILayout.Height(30)))
            {
                TrainMotionField(config, force: false);
            }

            if (GUILayout.Button("Retrain (ignore transition cache)"))
            {
                TrainMotionField(config, force: true);
            }
        }
    }

    private void DrawVisualizationSection(MotionFieldConfig config)
    {
        EditorGUILayout.LabelField("Debug Visualization", EditorStyles.boldLabel);

        string embeddingPath = config.GetEmbeddingPath();
        bool databaseExists = File.Exists(Path.Combine(config.GetAssetPath(), config.name + ".mmpose"));
        bool embedded = File.Exists(embeddingPath);

        EditorGUILayout.LabelField("Embedding",
            embedded
                ? $"{ProjectRelative(embeddingPath)}  ({new FileInfo(embeddingPath).Length / 1024} KB)"
                : "not computed yet");

        if (!databaseExists)
        {
            EditorGUILayout.HelpBox("Generate the pose database first.", MessageType.Warning);
        }

        EditorGUILayout.HelpBox(
            "Projects the motion field to 3D so MotionFieldVisualizer can draw it. Independent of " +
            "training -- the value function does not need it, and deleting it only turns the " +
            "visualizer off. Expect ~20 s: the first UMAP import pays a one-off numba warm-up.",
            MessageType.None);

        using (new EditorGUI.DisabledScope(!databaseExists || EditorApplication.isPlayingOrWillChangePlaymode))
        {
            if (GUILayout.Button("Compute UMAP Embedding", GUILayout.Height(30)))
            {
                ComputeEmbedding(config);
            }
        }
    }

    private static void GeneratePoseDatabase(MotionFieldConfig config)
    {
        try
        {
            EditorUtility.DisplayProgressBar("Motion Field", "Extracting poses...", 0.3f);
            config.ImportPoseSet();

            EditorUtility.DisplayProgressBar("Motion Field", "Writing database...", 0.7f);
            new PoseSerializer().Serialize(config.GetOrImportPoseSet(), config.GetAssetPath(), config.name);

            Debug.Log($"[MotionField] Wrote {config.GetOrImportPoseSet().NumberPoses} poses to " +
                      $"{ProjectRelative(config.GetAssetPath())}.");
        }
        catch (Exception e)
        {
            Debug.LogException(e);
        }
        finally
        {
            EditorUtility.ClearProgressBar();
            config.InvalidatePoseSet();
            AssetDatabase.Refresh();
        }
    }

    private static void TrainMotionField(MotionFieldConfig config, bool force)
    {
        // The interpreter runs in this process. A domain reload while it is mid-call takes the
        // editor down with it, so hold reloads off for the duration.
        EditorApplication.LockReloadAssemblies();
        try
        {
            PythonRuntime.EnsureInitialized(config.pythonDllPath, config.pythonVenvPath);

            // Marshalled into Python as a callable. Training is synchronous on the main thread, so
            // this fires on the main thread too and may touch the editor UI directly.
            Action<string, double> report = (stage, fraction) =>
                EditorUtility.DisplayProgressBar("Training Motion Field", stage, (float)fraction);

            using (Py.GIL())
            {
                PyObject trainer = PythonRuntime.Import("motion_field_trainer", reload: true);

                using var args = new PyTuple(new[]
                {
                    config.GetAssetPath().ToPython(),
                    config.name.ToPython(),
                    config.GetValueFunctionPath().ToPython(),
                });

                using var kwargs = new PyDict();
                kwargs["cache_path"] = config.GetTablesCachePath().ToPython();
                kwargs["k_neighbors"] = config.kNeighbors.ToPython();
                kwargs["theta_count"] = config.thetaCount.ToPython();
                kwargs["epochs"] = config.epochs.ToPython();
                kwargs["gamma"] = ((double)config.gamma).ToPython();
                kwargs["tug_ratio"] = ((double)config.tugRatio).ToPython();
                kwargs["pos_weight"] = ((double)config.posWeight).ToPython();
                kwargs["vel_weight"] = ((double)config.velWeight).ToPython();
                kwargs["device"] = config.DeviceName.ToPython();
                kwargs["knn_chunk"] = config.knnChunk.ToPython();
                kwargs["state_chunk"] = config.stateChunk.ToPython();
                kwargs["force"] = force.ToPython();
                kwargs["progress"] = report.ToPython();

                using PyObject summary = trainer.InvokeMethod("train", args, kwargs);
                Debug.Log($"[MotionField] {summary}");
            }

            GC.KeepAlive(report);
        }
        catch (Exception e)
        {
            Debug.LogException(e);
        }
        finally
        {
            EditorUtility.ClearProgressBar();
            EditorApplication.UnlockReloadAssemblies();
            AssetDatabase.Refresh();
        }
    }

    /// <summary>
    /// Fit the UMAP projection of the database. Same execution model as
    /// <see cref="TrainMotionField"/>: in-process, synchronous, reloads locked out.
    /// </summary>
    private static void ComputeEmbedding(MotionFieldConfig config)
    {
        EditorApplication.LockReloadAssemblies();
        try
        {
            PythonRuntime.EnsureInitialized(config.pythonDllPath, config.pythonVenvPath);

            Action<string, double> report = (stage, fraction) =>
                EditorUtility.DisplayProgressBar("Embedding Motion Field", stage, (float)fraction);

            using (Py.GIL())
            {
                PyObject embedding = PythonRuntime.Import("motion_field_embedding", reload: true);

                using var args = new PyTuple(new[]
                {
                    config.GetAssetPath().ToPython(),
                    config.name.ToPython(),
                    config.GetEmbeddingPath().ToPython(),
                });

                using var kwargs = new PyDict();
                kwargs["n_components"] = config.umapComponents.ToPython();
                kwargs["n_neighbors"] = config.umapNeighbors.ToPython();
                kwargs["min_dist"] = ((double)config.umapMinDist).ToPython();
                kwargs["device"] = config.DeviceName.ToPython();
                kwargs["seed"] = config.umapSeed.ToPython();
                kwargs["knn_chunk"] = config.knnChunk.ToPython();
                kwargs["pos_weight"] = ((double)config.posWeight).ToPython();
                kwargs["vel_weight"] = ((double)config.velWeight).ToPython();
                kwargs["progress"] = report.ToPython();

                using PyObject summary = embedding.InvokeMethod("compute_embedding", args, kwargs);
                Debug.Log($"[MotionField] {summary}");
            }

            GC.KeepAlive(report);
        }
        catch (Exception e)
        {
            Debug.LogException(e);
        }
        finally
        {
            EditorUtility.ClearProgressBar();
            EditorApplication.UnlockReloadAssemblies();
            AssetDatabase.Refresh();
        }
    }

    private static string ProjectRelative(string absolute)
    {
        string root = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
        string full = Path.GetFullPath(absolute);
        return full.StartsWith(root, StringComparison.OrdinalIgnoreCase)
            ? full.Substring(root.Length).TrimStart(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar)
            : full;
    }
}
}