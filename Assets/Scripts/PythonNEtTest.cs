using System;
using System.IO;
using Python.Runtime;
using UnityEditor;
using UnityEngine;

public class PythonNEtTest : MonoBehaviour
{
    void Start()
    {
        // 1. Point to your local Python installation DLL

        // 2. Initialize the Python Engine
        if (!PythonEngine.IsInitialized)
        {
            Runtime.PythonDLL = @"C:\Users\amana\AppData\Local\Python\pythoncore-3.13-64\python313.dll";
            PythonEngine.Initialize();
        }

        // 3. Acquire the Global Interpreter Lock (GIL)
        using (Py.GIL())
        {
            #region Environment Setup

            string venvPath = @"D:\iitbpg\MoSynth\AnimationTech\.anim_env";
            string sitePackages = Path.Combine(venvPath, "Lib", "site-packages");

            dynamic site = Py.Import("site");
            site.addsitedir(sitePackages);

            #endregion

            #region Script Directory Setup

            dynamic sys = Py.Import("sys");

            var scriptsFolder = Path.Combine(Application.dataPath, "../Python");
            sys.path.append(scriptsFolder);
            
            #endregion

            // Create a dedicated scope for your inline code
            using (var scope = Py.CreateScope())
            {
                dynamic np = Py.Import("numpy");
                dynamic array = np.array(new int[] { 1, 2, 3 });
                Debug.Log($"Successfully loaded package from venv! Array: {array}");
                // Execute the string code within this specific scope

                dynamic action_predictor = Py.Import("action_predictor");
                // Retrieve the function from the scope
                // dynamic multiply = scope.Get("get_multiplier");

                // Call the function
                dynamic result = action_predictor.get_multiplier();

                // Use Debug.Log for Unity
                Debug.Log($"Python multiply result: {result}"); // Output: 42
            }


            // --- Example B: Importing an external Python module (e.g., math) ---
            dynamic math = Py.Import("math");
            dynamic sqrtResult = math.sqrt(144);
            Debug.Log($"Python math.sqrt result: {sqrtResult}");
        }
    }
}