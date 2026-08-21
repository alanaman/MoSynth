using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using AnimationTools;
using Unity.Mathematics;
using UnityEditor.AssetImporters;
using UnityEngine;

namespace MotionMatching.Editor
{
    /// <summary>
    /// Imports a BVH file and stores the animation data in Unity format (BVHAnimation).
    /// </summary>
    [ScriptedImporter(2, "bvh")]
    public class BvhImporter : ScriptedImporter
    {
        public float unitScale = 0.01f;
        public bool onlyFirstFrame;
        public override void OnImportAsset(AssetImportContext ctx)
        {
            var (bvhAsset, skeletonAsset) = Import(ctx, unitScale, onlyFirstFrame);
            ctx.AddObjectToAsset("main obj", bvhAsset); // keep the "main obj" id — existing references depend on it
            ctx.AddObjectToAsset("skeleton", skeletonAsset);
            ctx.SetMainObject(bvhAsset);
        }

        private static (SkeletonAnimation bvhAsset, SkeletonAsset skeletonAsset) Import(AssetImportContext ctx, float scale = 0.01f, bool onlyFirstFrame = false)
        {
            var channelAxisOrders = new List<AxisOrder>();

            var parentIndexStack = new Stack<int>();
            var whitespace = new char[] { ' ', '\t', '\r', '\n' };

            var words = File.ReadAllText(ctx.assetPath).Split(whitespace, System.StringSplitOptions.RemoveEmptyEntries);
            // string[] words = Regex.Split(bvh.text, "[\\s+|\\r*\\n+]+");
            var w = 0;
            // ROOT
            if (words[w++] != "HIERARCHY") Debug.LogError("[BVHImporter] HIERARCHY not found");
            if (words[w++] != "ROOT") Debug.LogError("[BVHImporter] ROOT not found");
            var rootName = words[w++];
            ReadLeftBracket(words, ref w);
            ReadOffset(words, ref w); // consumed but discarded: root position always comes from motion channel 0, never HIERARCHY
            ReadChannels(channelAxisOrders, words, ref w, true);
            var bones = new List<Bone>
            {
                new Bone
                {
                    id = 1,
                    name = rootName,
                    parentIndex = -1,
                    restLocalPosition = float3.zero,
                    restLocalRotation = quaternion.identity,
                    humanBone = HumanBodyBones.LastBone
                }
            };
            // JOINTS
            var brackets = 1;
            var parent = 0;
            var jointIndex = 1;
            var it = 100000;
            if (ReadRightBracket(words, ref w)) brackets -= 1;
            while (brackets > 0 && --it > 0)
            {
                if (words[w++] != "JOINT") Debug.LogError("[BVHImporter] JOINT not found");
                var jointName = words[w++];
                var boneIndex = jointIndex++;
                ReadLeftBracket(words, ref w);
                parentIndexStack.Push(parent);
                var boneParentIndex = parent;
                parent = boneIndex;
                brackets += 1;
                var offset = ReadOffset(words, ref w) * scale;
                ReadChannels(channelAxisOrders, words, ref w);
                bones.Add(new Bone
                {
                    id = boneIndex + 1,
                    name = jointName,
                    parentIndex = boneParentIndex,
                    restLocalPosition = (float3)offset,
                    restLocalRotation = quaternion.identity,
                    humanBone = HumanBodyBones.LastBone
                });
                if (words[w] == "End")
                {
                    w += 1;
                    if (words[w++] != "Site") Debug.LogError("[BVHImporter] End Site not found");
                    ReadLeftBracket(words, ref w);
                    ReadOffset(words, ref w);
                    if (!ReadRightBracket(words, ref w)) Debug.LogError("[BVHImporter] End Site right bracket not found");
                }
                while (words[w] == "End")
                {
                    w += 1;
                    if (words[w++] != "Site") Debug.LogError("[BVHImporter] End Site not found");
                    ReadLeftBracket(words, ref w);
                    ReadOffset(words, ref w);
                    if (!ReadRightBracket(words, ref w)) Debug.LogError("[BVHImporter] End Site right bracket not found");
                }
                while (ReadRightBracket(words, ref w))
                {
                    brackets -= 1;
                    if (parentIndexStack.Count > 0) parent = parentIndexStack.Pop();
                }
                while (words[w] == "End")
                {
                    w += 1;
                    if (words[w++] != "Site") Debug.LogError("[BVHImporter] End Site not found");
                    ReadLeftBracket(words, ref w);
                    ReadOffset(words, ref w);
                    if (!ReadRightBracket(words, ref w)) Debug.LogError("[BVHImporter] End Site right bracket not found");
                }
                while (ReadRightBracket(words, ref w))
                {
                    brackets -= 1;
                    if (parentIndexStack.Count > 0) parent = parentIndexStack.Pop();
                }
            }
            if (it <= 0) Debug.LogError("[BVHImporter] Infinite loop detected, Left and Right brackets does not match");

            // MOTION
            if (words[w++] != "MOTION") Debug.LogError("[BVHImporter] MOTION not found");
            if (words[w++] != "Frames:") Debug.LogError("[BVHImporter] Frames: not found");
            var numberFrames = int.Parse(words[w++]);
            if (words[w++] != "Frame") Debug.LogError("[BVHImporter] Frame not found");
            if (words[w++] != "Time:") Debug.LogError("[BVHImporter] Time: not found");
            var frameTime = float.Parse(words[w++], CultureInfo.InvariantCulture);

            var skeletonAsset = ScriptableObject.CreateInstance<SkeletonAsset>();
            skeletonAsset.name = Path.GetFileNameWithoutExtension(ctx.assetPath) + "_Skeleton";
            // Ids restart at 1 on every reimport: OnImportAsset cannot reliably read the previous
            // sub-asset to merge, and no BoneReference targets BVH skeletons. Ids are deterministic
            // (index + 1) for an unchanged file regardless.
            skeletonAsset.SetBones(bones, bones.Count + 1);

            var layout = PoseLayout.CreateFullPose(skeletonAsset, false, false);
            var d = layout.Data;

            var numberChannels = channelAxisOrders.Count;
            var framesStored = onlyFirstFrame ? Mathf.Min(1, numberFrames) : numberFrames;
            var frameData = new float[framesStored * layout.FloatCount];

            // One-frame template with every bone's rest position; slot 0 (root) is overwritten
            // per frame below, identity rotations elsewhere are fine since every rotation slot
            // is written per frame too.
            var template = new float[layout.FloatCount];
            for (var b = 0; b < bones.Count; b++)
            {
                var restPosition = bones[b].restLocalPosition;
                template[d.PositionStart + b * 3 + 0] = restPosition.x;
                template[d.PositionStart + b * 3 + 1] = restPosition.y;
                template[d.PositionStart + b * 3 + 2] = restPosition.z;
            }

            for (var f = 0; f < framesStored; f++)
            {
                var frameBase = f * layout.FloatCount;
                Array.Copy(template, 0, frameData, frameBase, layout.FloatCount);
                for (var j = 0; j < numberChannels; ++j)
                {
                    var v1 = float.Parse(words[w++], CultureInfo.InvariantCulture);
                    var v2 = float.Parse(words[w++], CultureInfo.InvariantCulture);
                    var v3 = float.Parse(words[w++], CultureInfo.InvariantCulture);
                    var axisOrder = channelAxisOrders[j];
                    if (j == 0)
                    {
                        var rootMotion = BvhToUnityTranslation(v1, v2, v3, axisOrder) * scale;
                        var positionBase = frameBase + d.PositionStart;
                        frameData[positionBase + 0] = rootMotion.x;
                        frameData[positionBase + 1] = rootMotion.y;
                        frameData[positionBase + 2] = rootMotion.z;
                    }
                    else
                    {
                        var q = BvhToUnityRotation(v1, v2, v3, axisOrder);
                        var rotationBase = frameBase + d.RotationStart + (j - 1) * 4;
                        frameData[rotationBase + 0] = q.x;
                        frameData[rotationBase + 1] = q.y;
                        frameData[rotationBase + 2] = q.z;
                        frameData[rotationBase + 3] = q.w;
                    }
                }
            }

            var bvhAsset = ScriptableObject.CreateInstance<SkeletonAnimation>();
            bvhAsset.Initialize(skeletonAsset, frameTime, framesStored, frameData);
            return (bvhAsset, skeletonAsset);
        }

        private static void ReadLeftBracket(string[] words, ref int w)
        {
            if (words[w++] != "{") Debug.LogError("[BVHImporter] { not found");
        }

        private static bool ReadRightBracket(string[] words, ref int w)
        {
            var isRightBracket = words[w] == "}";
            if (isRightBracket)
            {
                w += 1;
            }
            return isRightBracket;
        }

        private static Vector3 ReadOffset(string[] words, ref int w)
        {
            var offset = Vector3.zero;
            if (words[w++] != "OFFSET") Debug.LogError("[BVHImporter] OFFSET not found");
            offset.x = float.Parse(words[w++], CultureInfo.InvariantCulture);
            offset.y = float.Parse(words[w++], CultureInfo.InvariantCulture);
            offset.z = -float.Parse(words[w++], CultureInfo.InvariantCulture); // Unity is left-handed and BVH is right-handed (Z is opposite sign)
            return offset;
        }

        private static void ReadChannels(List<AxisOrder> channels, string[] words, ref int w, bool root = false)
        {
            if (words[w++] != "CHANNELS")
            {
                Debug.LogError("[BVHImporter] CHANNELS keyword not found");
                return;
            }

            var numChannels = int.Parse(words[w++]);

            if (root)
            {
                // The root **must** provide 3 translation + 3 rotation = 6 channels.
                if (numChannels != 6)
                {
                    Debug.LogError("[BVHImporter] The root joint must have exactly 6 channels");
                }

                // Remember in which order the three position axes appear (XYZ / XZY �)
                channels.Add(ReadChannelPosition(words, ref w));
            }
            else
            {
                // Non-root joints may come with 3 **or** 6 channels depending on the exporter.
                // When 6, the first three are XYZ-position that we can safely ignore.
                if (numChannels == 6)
                {
                    // Consume and discard the position axis order
                    ReadChannelPosition(words, ref w);
                }
                else if (numChannels != 3)
                {
                    Debug.LogError($"[BVHImporter] Unexpected channel count ({numChannels}) at joint � expected 3 or 6");
                }
                // If there are only 3 channels � nothing to skip, cursor is already after the count.
            }
            // Rotation channels are always 3, so we can safely read them.
            channels.Add(ReadChannelRotation(words, ref w));
        }

        private static AxisOrder ReadChannelPosition(string[] words, ref int w)
        {
            var order1 = words[w++];
            var order2 = words[w++];
            var order3 = words[w++];
            if (order1 != "Xposition" && order1 != "Yposition" && order1 != "Zposition") Debug.LogError("[BVHImporter] root position channels must be Xposition, Yposition or Zposition");
            if (order2 != "Xposition" && order2 != "Yposition" && order2 != "Zposition") Debug.LogError("[BVHImporter] root position channels must be Xposition, Yposition or Zposition");
            if (order3 != "Xposition" && order3 != "Yposition" && order3 != "Zposition") Debug.LogError("[BVHImporter] root position channels must be Xposition, Yposition or Zposition");
            if (order1 == "Xposition")
            {
                if (order2 == "Yposition")
                {
                    if (order3 == "Zposition") return AxisOrder.XYZ;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xposition, Yposition and Zposition");
                }
                else
                {
                    if (order3 == "Yposition") return AxisOrder.XZY;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xposition, Yposition and Zposition");
                }
            }
            else if (order1 == "Yposition")
            {
                if (order2 == "Xposition")
                {
                    if (order3 == "Zposition") return AxisOrder.YXZ;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xposition, Yposition and Zposition");
                }
                else
                {
                    if (order3 == "Xposition") return AxisOrder.YZX;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xposition, Yposition and Zposition");
                }
            }
            else
            {
                if (order2 == "Xposition")
                {
                    if (order3 == "Yposition") return AxisOrder.ZXY;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xposition, Yposition and Zposition");
                }
                else
                {
                    if (order3 == "Xposition") return AxisOrder.ZYX;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xposition, Yposition and Zposition");
                }
            }
            return AxisOrder.None;
        }

        private static AxisOrder ReadChannelRotation(string[] words, ref int w)
        {
            var order1 = words[w++];
            var order2 = words[w++];
            var order3 = words[w++];
            if (order1 != "Xrotation" && order1 != "Yrotation" && order1 != "Zrotation") Debug.LogError("[BVHImporter] root or joint rotation channels must be Xrotation, Yrotation or Zrotation");
            if (order2 != "Xrotation" && order2 != "Yrotation" && order2 != "Zrotation") Debug.LogError("[BVHImporter] root or joint rotation channels must be Xrotation, Yrotation or Zrotation");
            if (order3 != "Xrotation" && order3 != "Yrotation" && order3 != "Zrotation") Debug.LogError("[BVHImporter] root or joint rotation channels must be Xrotation, Yrotation or Zrotation");
            if (order1 == "Xrotation")
            {
                if (order2 == "Yrotation")
                {
                    if (order3 == "Zrotation") return AxisOrder.XYZ;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xrotation, Yrotation and Zrotation");
                }
                else
                {
                    if (order3 == "Yrotation") return AxisOrder.XZY;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xrotation, Yrotation and Zrotation");
                }
            }
            else if (order1 == "Yrotation")
            {
                if (order2 == "Xrotation")
                {
                    if (order3 == "Zrotation") return AxisOrder.YXZ;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xrotation, Yrotation and Zrotation");
                }
                else
                {
                    if (order3 == "Xrotation") return AxisOrder.YZX;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xrotation, Yrotation and Zrotation");
                }
            }
            else
            {
                if (order2 == "Xrotation")
                {
                    if (order3 == "Yrotation") return AxisOrder.ZXY;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xrotation, Yrotation and Zrotation");
                }
                else
                {
                    if (order3 == "Xrotation") return AxisOrder.ZYX;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xrotation, Yrotation and Zrotation");
                }
            }
            return AxisOrder.None;
        }

        private static Quaternion BvhToUnityRotation(float v1, float v2, float v3, AxisOrder rotationOrder)
        {
            if (rotationOrder == AxisOrder.None) Debug.LogError("[BVHImporter] rotationOrder is None. There was an error while reading the channels");

            switch (rotationOrder)
            {
                // Why some are negative? Because Unity is left-handed and BVH is right-handed. See: https://stackoverflow.com/questions/31191752/right-handed-euler-angles-xyz-to-left-handed-euler-angles-xyz
                case AxisOrder.XYZ: return Quaternion.AngleAxis(-v1, Vector3.right) * Quaternion.AngleAxis(-v2, Vector3.up) * Quaternion.AngleAxis(v3, Vector3.forward); // XYZ
                case AxisOrder.XZY: return Quaternion.AngleAxis(-v1, Vector3.right) * Quaternion.AngleAxis(v2, Vector3.forward) * Quaternion.AngleAxis(-v3, Vector3.up); // XZY
                case AxisOrder.YXZ: return Quaternion.AngleAxis(-v1, Vector3.up) * Quaternion.AngleAxis(-v2, Vector3.right) * Quaternion.AngleAxis(v3, Vector3.forward); // YXZ
                case AxisOrder.YZX: return Quaternion.AngleAxis(-v1, Vector3.up) * Quaternion.AngleAxis(v2, Vector3.forward) * Quaternion.AngleAxis(-v3, Vector3.right); // YZX
                case AxisOrder.ZXY: return Quaternion.AngleAxis(v1, Vector3.forward) * Quaternion.AngleAxis(-v2, Vector3.right) * Quaternion.AngleAxis(-v3, Vector3.up); // ZXY
                case AxisOrder.ZYX: return Quaternion.AngleAxis(v1, Vector3.forward) * Quaternion.AngleAxis(-v2, Vector3.up) * Quaternion.AngleAxis(-v3, Vector3.right); // ZYX
            }

            return Quaternion.identity;
        }

        private static Vector3 BvhToUnityTranslation(float v1, float v2, float v3, AxisOrder translationOrder)
        {
            if (translationOrder == AxisOrder.None) Debug.LogError("[BVHImporter] translationOrder is None. There was an error while reading the channels");

            // BVH's z+ axis is Unity's left (z-) (Unity is left-handed BVH is right-handed)
            switch (translationOrder)
            {
                case AxisOrder.XYZ: return new Vector3(v1, v2, -v3); // XYZ
                case AxisOrder.XZY: return new Vector3(v1, v3, -v2); // XZY
                case AxisOrder.YXZ: return new Vector3(v2, v1, -v3); // YXZ
                case AxisOrder.YZX: return new Vector3(v3, v1, -v2); // YZX
                case AxisOrder.ZXY: return new Vector3(v2, v3, -v1); // ZXY
                case AxisOrder.ZYX: return new Vector3(v3, v2, -v1); // ZYX
            }
            return Vector3.zero;
        }

        private enum AxisOrder
        {
            XYZ, XZY, YXZ, YZX, ZXY, ZYX, None
        }

    }
}
