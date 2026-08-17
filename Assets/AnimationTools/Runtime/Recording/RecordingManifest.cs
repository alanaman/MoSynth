using System;
using System.IO;
using UnityEngine;

namespace AnimationTools
{
/// <summary>
/// Sidecar JSON written next to a <see cref="MotionRecorder"/>'s binary data file. Describes the
/// data file well enough to load it without any AnimationTools code, e.g. from Python:
/// <code>
/// import json, numpy as np
/// m = json.load(open("recording.json"))
/// data = np.fromfile("recording.bin", dtype="&lt;f4").reshape(-1, m["frameFloatCount"])
/// ch = {c["name"]: c for c in m["channels"]}
/// hips = data[:, ch["hips"]["floatOffset"] : ch["hips"]["floatOffset"] + 3]
/// </code>
/// </summary>
[Serializable]
public class RecordingManifest
{
    public int formatVersion = 1;
    public string recordingName;

    /// <summary>ISO-8601.</summary>
    public string createdUtc;

    /// <summary>Sidecar-relative filename of the binary data file.</summary>
    public string dataFile;

    public string dtype = "float32";
    public string endianness = "little";
    public string trigger;
    public float synthesisFrameRate;
    public int frameFloatCount;
    public int frameCount;
    public RecordingManifestChannel[] channels;

    public static void Write(RecordingManifest manifest, string path)
    {
        File.WriteAllText(path, JsonUtility.ToJson(manifest, true));
    }
}

[Serializable]
public class RecordingManifestChannel
{
    public string name;
    public string type;
    public int floatOffset;
    public int floatCount;
    public string[] components;
    public string bone;
    public string space;
    public RecordingManifestPoseLayout poseLayout;
}

[Serializable]
public class RecordingManifestPoseLayout
{
    public string[] boneNames;
    public string rotationFormat;
    public int positionStart, positionCount;
    public int rotationStart, rotationCount, rotationStride;
    public int velocityStart, velocityCount;
    public int angularVelocityStart, angularVelocityCount;
    public int boolStart, boolCount;
}
}
