using System;
using System.IO;
using Python.Runtime;
using UnityEngine;
using MotionMatching;

namespace MotionField
{
[Serializable]
public class MotionFieldStage : MoSynthStage, IDisposable
{
    private bool _isInitialized = false;
    private PyModule _scope;

    private dynamic _actionPredictor;
    private dynamic _motionField;
    private dynamic _skeleton;
    private dynamic _currentX;
    private dynamic _currentV;
    private dynamic _currentContacts;

    public bool play = true;

    public enum Display
    {
        Current,
        Near,
        Blended
    }
    [SerializeField] public Display display = Display.Current;
    [SerializeField] [Range(0, 15)] public int nearIdx = 0;

    public override void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        if (_isInitialized) return;

        // Initialize the Python Engine
        if (!PythonEngine.IsInitialized)
        {
            Runtime.PythonDLL = @"C:\Users\amana\AppData\Local\Python\pythoncore-3.13-64\python313.dll";
            PythonEngine.Initialize();
        }

        using (Py.GIL())
        {
            // Environment Setup
            string venvPath = @"D:\iitbpg\MoSynth\AnimationTech\.anim_env";
            string sitePackages = Path.Combine(venvPath, "Lib", "site-packages");

            dynamic site = Py.Import("site");
            site.addsitedir(sitePackages);

            // Script Directory Setup
            dynamic sys = Py.Import("sys");
            var scriptsFolder = Path.Combine(Application.dataPath, "../Python");
            sys.path.append(scriptsFolder);

            _scope = Py.CreateScope();
            _actionPredictor = Py.Import("action_predictor");
            dynamic motionFieldModule = Py.Import("MotionField");

            // Force reload modules so Python script changes apply without restarting Unity
            dynamic importlib = Py.Import("importlib");
            importlib.reload(_actionPredictor);
            importlib.reload(motionFieldModule);

            // Load animations and initialize motion field state
            string dataPath =
                Path.GetFullPath(Path.Combine(Application.streamingAssetsPath, "MMDatabases", "MotionMatchingData"));
            var animData = _actionPredictor.load_animations(dataPath);
            _skeleton = animData[0];
            dynamic poseX = animData[1];
            dynamic poseV = animData[2];
            dynamic poseY = animData[3];
            dynamic poseContacts = animData[4];
            dynamic frameTime = animData[5];

            _motionField = motionFieldModule.MotionField(poseX, poseV, poseY, _skeleton, frameTime);

            int lastIndex = 700;
            _currentX = poseX[lastIndex].copy();
            _currentV = poseV[lastIndex].copy();
            _currentContacts = poseContacts[lastIndex];
        }

        _isInitialized = true;
    }

    public override bool Apply(PoseVector pose, float deltaTime)
    {
        if (!_isInitialized) return false;

        using (Py.GIL())
        {
            // Call get_next_pose to update the current state
            var nextPoseTuple = _motionField.get_pose(_currentX, _currentV, deltaTime);
            _currentX = nextPoseTuple[0];
            _currentV = nextPoseTuple[1];

            // Get unmanaged memory pointers and contact states
            dynamic poseArrays = _actionPredictor.get_pose_arrays(_skeleton, _currentX, _currentV, _currentContacts);

            float[] posArray = (float[])poseArrays[0];
            float[] quatArray = (float[])poseArrays[1];
            float[] lvArray = (float[])poseArrays[2];
            float[] lavArray = (float[])poseArrays[3];

            pose.leftFootContact = (bool)poseArrays[4];
            pose.rightFootContact = (bool)poseArrays[5];

            int numJoints = pose.jointLocalPositions.Length;

            for (int i = 0; i < numJoints; i++)
            {
                pose.jointLocalPositions[i] = new Unity.Mathematics.float3(
                    posArray[i * 3], posArray[i * 3 + 1], posArray[i * 3 + 2]);

                pose.jointLocalRotations[i] = new Unity.Mathematics.quaternion(
                    quatArray[i * 4], quatArray[i * 4 + 1], quatArray[i * 4 + 2], quatArray[i * 4 + 3]);

                pose.jointLocalVelocities[i] = new Unity.Mathematics.float3(
                    lvArray[i * 3], lvArray[i * 3 + 1], lvArray[i * 3 + 2]);

                pose.jointLocalAngularVelocities[i] = new Unity.Mathematics.float3(
                    lavArray[i * 3], lavArray[i * 3 + 1], lavArray[i * 3 + 2]);
            }

            return true;
        }
    }

    public void Dispose()
    {
        if (_isInitialized)
        {
            using (Py.GIL())
            {
                _actionPredictor = null;
                _motionField = null;
                _skeleton = null;
                _currentX = null;
                _currentV = null;
                _currentContacts = null;
                _scope?.Dispose();
            }

            _isInitialized = false;
        }
    }

    public override void OnDestroy()
    {
        Dispose();
    }
}
}