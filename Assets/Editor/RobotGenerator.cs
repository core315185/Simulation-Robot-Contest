using System.IO;
using UnityEditor;
using UnityEngine;

namespace SimulationRobotContest.EditorTools
{
    /// <summary>
    /// メニュー「Tools > Create Robot」を押すだけで、ロボット掃除機＋見守りカメラの
    /// GameObject階層一式（Base / Camera Pole / Camera / Wheel Left / Wheel Right）と、
    /// Rigidbody・Collider・PhysicMaterial・質量・重心・Collision Detectionの設定、
    /// および各スクリプトコンポーネントの自動配線までを行うエディタ拡張。
    /// </summary>
    public static class RobotGenerator
    {
        #region Constants（マジックナンバー回避のため名前付き定数として集約）

        // ---- 課題必須条件に基づく寸法・物性値 ----
        private const float RobotDiameter = 0.30f;      // ロボット直径 [m]
        private const float RobotRadius = RobotDiameter / 2f;
        private const float CameraHeight = 1.00f;        // カメラ高さ（床面から）[m]
        private const float RobotMass = 10.0f;           // 全質量 [kg]
        private const float CenterOfMassHeight = 0.50f;  // 重心高さ（床面から）[m]

        // ---- 差動駆動パラメータ（要調整可） ----
        private const float WheelTrack = 0.22f;
        private const float WheelRadius = 0.03f;
        private const float WheelWidth = 0.02f;

        // ---- ロボット本体（円柱として表現）の高さ ----
        private const float BaseBodyHeight = 0.15f;

        // ---- 摩擦（物理マテリアル） ----
        private const float WheelDynamicFriction = 0.02f;
        private const float WheelStaticFriction = 0.02f;

        // ---- 名前 ----
        private const string RobotRootName = "Robot";
        private const string BaseName = "Base";
        private const string CameraPoleName = "Camera Pole";
        private const string CameraName = "Camera";
        private const string WheelLeftName = "Wheel Left";
        private const string WheelRightName = "Wheel Right";

        private const string PhysicMaterialFolder = "Assets/Physics";
        private const string PhysicMaterialAssetPath = "Assets/Physics/RobotWheelFriction.physicMaterial";

        private const string TagWall = "Wall";

        #endregion

        #region Menu Entry Point

        [MenuItem("Tools/Create Robot")]
        public static void CreateRobot()
        {
            EnsureTagExists(TagWall);

            RemoveExistingRobot();

            GameObject root = new GameObject(RobotRootName);
            Undo.RegisterCreatedObjectUndo(root, "Create Robot");
            root.transform.position = Vector3.zero;

            Rigidbody rb = ConfigureRigidbody(root);
            ConfigureRootCollider(root);

            GameObject baseVisual = CreateBaseVisual(root.transform);
            GameObject wheelLeft = CreateWheelVisual(root.transform, WheelLeftName, isLeft: true);
            GameObject wheelRight = CreateWheelVisual(root.transform, WheelRightName, isLeft: false);
            GameObject cameraPole = CreateCameraPole(root.transform);
            GameObject cameraObject = CreateCamera(cameraPole.transform);

            // ---- スクリプトコンポーネントの追加 ----
            RobotController robotController = root.AddComponent<RobotController>();
            WaypointFollower waypointFollower = root.AddComponent<WaypointFollower>();
            DataLogger dataLogger = root.AddComponent<DataLogger>();
            GraphExporter graphExporter = root.AddComponent<GraphExporter>();
            CameraController cameraController = cameraObject.AddComponent<CameraController>();

            // ---- 依存関係の配線 ----
            robotController.SetDependencies(waypointFollower, cameraController, dataLogger, graphExporter);
            robotController.SetKinematicsParameters(WheelTrack, WheelRadius);
            robotController.SetVisualWheels(wheelLeft.transform, wheelRight.transform);
            cameraController.SetTargetRigidbody(rb);

            Selection.activeGameObject = root;

            Debug.Log("[RobotGenerator] ロボットの生成が完了しました。"
                + " スタート地点（Course/Waypoints/WP_0）への配置とPIDゲイン調整を行ってください。");
        }

        #endregion

        #region Cleanup

        private static void RemoveExistingRobot()
        {
            GameObject existing = GameObject.Find(RobotRootName);
            if (existing != null)
            {
                Undo.DestroyObjectImmediate(existing);
            }
        }

        #endregion

        #region Rigidbody / Collider

        private static Rigidbody ConfigureRigidbody(GameObject root)
        {
            Rigidbody rb = root.AddComponent<Rigidbody>();
            rb.mass = RobotMass;
            rb.centerOfMass = new Vector3(0f, CenterOfMassHeight, 0f);
            rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            return rb;
        }

        private static void ConfigureRootCollider(GameObject root)
        {
            CapsuleCollider collider = root.AddComponent<CapsuleCollider>();
            collider.radius = RobotRadius;
            collider.height = Mathf.Max(CameraHeight, BaseBodyHeight);
            collider.direction = 1; // Y軸方向
            collider.center = new Vector3(0f, collider.height / 2f, 0f);

            PhysicsMaterial material = LoadOrCreateWheelPhysicMaterial();
            if (material != null)
            {
                collider.material = material;
            }
        }

        private static PhysicsMaterial LoadOrCreateWheelPhysicMaterial()
        {
            PhysicsMaterial existing = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(PhysicMaterialAssetPath);
            if (existing != null)
            {
                return existing;
            }

            if (!AssetDatabase.IsValidFolder(PhysicMaterialFolder))
            {
                AssetDatabase.CreateFolder("Assets", "Physics");
            }

            PhysicsMaterial material = new PhysicsMaterial("RobotWheelFriction")
            {
                dynamicFriction = WheelDynamicFriction,
                staticFriction = WheelStaticFriction,
                // 床(Floor)側のマテリアルの摩擦値に引っ張られないよう、
                // 必ず低い方（＝このマテリアルの値）を採用させる
                frictionCombine = PhysicsMaterialCombine.Minimum,
                bounciness = 0f,
                bounceCombine = PhysicsMaterialCombine.Minimum
            };

            AssetDatabase.CreateAsset(material, PhysicMaterialAssetPath);
            AssetDatabase.SaveAssets();

            return material;
        }

        #endregion

        #region Visual Parts

        private static GameObject CreateBaseVisual(Transform parent)
        {
            GameObject baseVisual = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            baseVisual.name = BaseName;
            baseVisual.transform.SetParent(parent, false);
            baseVisual.transform.localPosition = new Vector3(0f, BaseBodyHeight / 2f, 0f);
            baseVisual.transform.localScale = new Vector3(RobotDiameter, BaseBodyHeight / 2f, RobotDiameter);

            // 見た目のみのオブジェクトなので、物理判定に二重で影響しないようColliderを取り除く
            Object.DestroyImmediate(baseVisual.GetComponent<Collider>());

            Undo.RegisterCreatedObjectUndo(baseVisual, "Create Robot Base Visual");
            return baseVisual;
        }

        private static GameObject CreateWheelVisual(Transform parent, string name, bool isLeft)
        {
            GameObject wheel = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            wheel.name = name;
            wheel.transform.SetParent(parent, false);

            float xOffset = (isLeft ? -1f : 1f) * (WheelTrack / 2f);
            wheel.transform.localPosition = new Vector3(xOffset, WheelRadius, 0f);

            // Unityのデフォルト円柱はY軸方向が高さになるため、車軸をX軸方向に向けるよう90度回転させる
            wheel.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);
            wheel.transform.localScale = new Vector3(WheelWidth, WheelRadius, WheelWidth);

            // 見た目のみのオブジェクトなので、物理判定に二重で影響しないようColliderを取り除く
            Object.DestroyImmediate(wheel.GetComponent<Collider>());

            Undo.RegisterCreatedObjectUndo(wheel, $"Create {name}");
            return wheel;
        }

        private static GameObject CreateCameraPole(Transform parent)
        {
            GameObject pole = new GameObject(CameraPoleName);
            pole.transform.SetParent(parent, false);
            pole.transform.localPosition = new Vector3(0f, BaseBodyHeight, 0f);

            // ポール自体は細い円柱メッシュとして視覚化する（コスメティック、Colliderなし）
            GameObject poleVisual = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            poleVisual.name = "Pole Visual";
            poleVisual.transform.SetParent(pole.transform, false);
            float poleHeight = CameraHeight - BaseBodyHeight;
            poleVisual.transform.localPosition = new Vector3(0f, poleHeight / 2f, 0f);
            poleVisual.transform.localScale = new Vector3(0.02f, poleHeight / 2f, 0.02f);
            Object.DestroyImmediate(poleVisual.GetComponent<Collider>());

            Undo.RegisterCreatedObjectUndo(pole, "Create Camera Pole");
            return pole;
        }

        private static GameObject CreateCamera(Transform poleTransform)
        {
            GameObject cameraObject = new GameObject(CameraName);
            cameraObject.transform.SetParent(poleTransform, false);

            float poleHeight = CameraHeight - BaseBodyHeight;
            cameraObject.transform.localPosition = new Vector3(0f, poleHeight, 0f);

            Camera camera = cameraObject.AddComponent<Camera>();
            camera.fieldOfView = 60f;
            camera.nearClipPlane = 0.05f;

            // 「見守りカメラ」として、進行方向やや下向きにコースを見渡す設定にしておく
            cameraObject.transform.localRotation = Quaternion.Euler(20f, 0f, 0f);

            Undo.RegisterCreatedObjectUndo(cameraObject, "Create Camera");
            return cameraObject;
        }

        #endregion

        #region Tag Management

        private static void EnsureTagExists(string tagName)
        {
            if (string.IsNullOrEmpty(tagName))
            {
                return;
            }

            Object[] tagManagerAssets = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset");
            if (tagManagerAssets == null || tagManagerAssets.Length == 0)
            {
                Debug.LogWarning("[RobotGenerator] TagManager.assetの読み込みに失敗しました。タグは手動で追加してください。");
                return;
            }

            SerializedObject tagManager = new SerializedObject(tagManagerAssets[0]);
            SerializedProperty tagsProperty = tagManager.FindProperty("tags");

            if (tagsProperty == null)
            {
                Debug.LogWarning("[RobotGenerator] TagManagerのtagsプロパティが見つかりませんでした。");
                return;
            }

            for (int i = 0; i < tagsProperty.arraySize; i++)
            {
                if (tagsProperty.GetArrayElementAtIndex(i).stringValue == tagName)
                {
                    return;
                }
            }

            int insertIndex = tagsProperty.arraySize;
            tagsProperty.InsertArrayElementAtIndex(insertIndex);
            tagsProperty.GetArrayElementAtIndex(insertIndex).stringValue = tagName;
            tagManager.ApplyModifiedProperties();
        }

        #endregion
    }
}
