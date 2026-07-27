using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace SimulationRobotContest.EditorTools
{
    /// <summary>
    /// メニュー「Tools > Create Course」を押すだけで、S字クランクコース一式
    /// （Floor / Outer Walls / Inner Walls / Start / Goal / Waypoints）を
    /// シーンに自動生成するエディタ拡張。
    ///
    /// 【壁生成方式について（修正版）】
    /// 以前のバージョンは「中心線からのオフセット＋延長によるミータ処理」で壁を生成していたが、
    /// 直角コーナーで隙間や壁の浮きが発生し、正しいS字形状にならない不具合があった。
    /// 本バージョンでは、コース全体（2.4m×3.0m）がちょうど通路幅0.6m四方のマスで
    /// 「4列×5行」のグリッドに分割できることを利用し、
    ///   1. 各マスを「通行可能(open)」「不可(closed)」で定義する
    ///   2. 通行可能マスの各辺について、隣接マスが不可（またはコース外）なら
    ///      その辺の位置に軸並行な壁ボックスを1枚置く
    /// という決定的なアルゴリズムで壁を生成する。壁は常にX軸またはZ軸に平行なため
    /// 回転計算が不要になり、コーナーの隙間・浮きが原理的に発生しない。
    ///
    /// 課題文の「添付画像と同じような形状であればよい。完全一致は不要」という条件に基づき、
    /// 通路幅0.6m・全体寸法2.4m×3.0mという必須寸法は厳守しつつ、
    /// スタート(左下)→ゴール(右上)への明確なS字（Z字）クランクとして再設計している。
    /// </summary>
    public static class CourseGenerator
    {
        #region Constants（マジックナンバー回避のため名前付き定数として集約）

        // ---- 課題必須条件に基づく寸法 ----
        private const float CorridorWidth = 0.6f;    // 通路幅（＝グリッドのマスサイズ）[m]
        private const int GridCols = 4;              // 全体幅2.4m ÷ 0.6m = 4列
        private const int GridRows = 5;               // 全体高さ3.0m ÷ 0.6m = 5行
        private const float CourseWidth = GridCols * CorridorWidth;  // 2.4m
        private const float CourseHeight = GridRows * CorridorWidth; // 3.0m

        // ---- 生成パラメータ（図面に数値記載が無いため妥当な値を仮定。要調整） ----
        private const float WallThickness = 0.03f;   // 壁の厚み [m]
        private const float WallHeight = 1.3f;       // 壁の高さ [m]（カメラ高さ1.00mより高く設定）
        private const float FloorThickness = 0.05f;  // 床の厚み [m]

        // ---- オブジェクト名・タグ ----
        private const string RootName = "Course";
        private const string FloorName = "Floor";
        private const string OuterWallsGroupName = "Outer Walls";
        private const string InnerWallsGroupName = "Inner Walls";
        private const string WaypointsGroupName = "Waypoints";
        private const string StartLineName = "Start";
        private const string GoalLineName = "Goal";

        private const string TagWall = "Wall";
        private const string TagStartLine = "StartLine";
        private const string TagGoalLine = "GoalLine";
        private const string TagFloor = "Floor";
        private const string TagCourseBounds = "CourseBounds";

        #endregion

        #region Path Grid Definition

        /// <summary>
        /// 通行可能マスの定義（[行, 列]。行0が最下段=スタート側、行4が最上段=ゴール側）。
        /// 列0が左端、列3が右端。
        ///
        /// レイアウト（S字/Z字クランク）:
        ///   行4 (上段) : 全列 open  … 左→右へ進み、右端(列3)がゴール
        ///   行3        : 列0のみ open … 行2から行4へ抜ける「左側の縦通路」
        ///   行2 (中段) : 全列 open  … 右→左へ進む
        ///   行1        : 列3のみ open … 行0から行2へ抜ける「右側の縦通路」
        ///   行0 (下段) : 全列 open  … 左→右へ進み、左端(列0)がスタート
        /// </summary>
        private static readonly bool[,] OpenCells =
        {
            { true, true, true, true },     // 行0（下段、スタート側）
            { false, false, false, true },  // 行1（右側のみ縦通路）
            { true, true, true, true },     // 行2（中段）
            { true, false, false, false },  // 行3（左側のみ縦通路）
            { true, true, true, true },     // 行4（上段、ゴール側）
        };

        #endregion

        #region Menu Entry Point

        [MenuItem("Tools/Create Course")]
        public static void CreateCourse()
        {
            EnsureTagExists(TagWall);
            EnsureTagExists(TagStartLine);
            EnsureTagExists(TagGoalLine);
            EnsureTagExists(TagFloor);
            EnsureTagExists(TagCourseBounds);

            RemoveExistingCourse();

            GameObject root = new GameObject(RootName);
            Undo.RegisterCreatedObjectUndo(root, "Create Course");

            List<Vector3> centerline = BuildDefaultCenterline();

            CreateFloor(root.transform);
            CreateGridWalls(root.transform);
            CreateStartLine(root.transform, centerline);
            CreateGoalLine(root.transform, centerline);
            CreateWaypoints(root.transform, centerline);
            CreateOutOfBoundsVolume(root.transform);

            Selection.activeGameObject = root;

            Debug.Log("[CourseGenerator] コースの生成が完了しました（グリッドベースのS字クランク）。");
        }

        #endregion

        #region Cleanup

        private static void RemoveExistingCourse()
        {
            GameObject existing = GameObject.Find(RootName);
            if (existing != null)
            {
                Undo.DestroyObjectImmediate(existing);
            }
        }

        #endregion

        #region Centerline Definition（Waypoints/Start/Goal用。OpenCellsのパスと対応させている）

        /// <summary>
        /// S字クランクの中心線ウェイポイント（XZ平面）。OpenCellsで定義した経路の
        /// 各マス中心を、直進区間はまとめて通る形で並べている。
        /// </summary>
        private static List<Vector3> BuildDefaultCenterline()
        {
            float half = CorridorWidth / 2f;

            return new List<Vector3>
            {
                CellCenter(0, 0, half), // WP0: スタート（行0・列0）
                CellCenter(0, 3, half), // WP1: 行0を右端まで
                CellCenter(2, 3, half), // WP2: 行1の縦通路を抜けて行2の右端
                CellCenter(2, 0, half), // WP3: 行2を左端まで
                CellCenter(4, 0, half), // WP4: 行3の縦通路を抜けて行4の左端
                CellCenter(4, 3, half), // WP5: ゴール（行4・列3）
            };
        }

        private static Vector3 CellCenter(int row, int col, float half)
        {
            float x = (col * CorridorWidth) + half;
            float z = (row * CorridorWidth) + half;
            return new Vector3(x, 0f, z);
        }

        #endregion

        #region Floor

        private static void CreateFloor(Transform parent)
        {
            GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.name = FloorName;
            floor.tag = TagFloor;
            floor.transform.SetParent(parent, false);
            floor.transform.localScale = new Vector3(CourseWidth, FloorThickness, CourseHeight);
            floor.transform.localPosition = new Vector3(CourseWidth / 2f, -FloorThickness / 2f, CourseHeight / 2f);

            Undo.RegisterCreatedObjectUndo(floor, "Create Course Floor");
        }

        #endregion

        #region Grid-Based Wall Generation（Outer / Inner 共通ロジック）

        private static void CreateGridWalls(Transform parent)
        {
            GameObject outerGroup = new GameObject(OuterWallsGroupName);
            outerGroup.transform.SetParent(parent, false);
            Undo.RegisterCreatedObjectUndo(outerGroup, "Create Outer Walls Group");

            GameObject innerGroup = new GameObject(InnerWallsGroupName);
            innerGroup.transform.SetParent(parent, false);
            Undo.RegisterCreatedObjectUndo(innerGroup, "Create Inner Walls Group");

            int wallIndex = 0;

            for (int row = 0; row < GridRows; row++)
            {
                for (int col = 0; col < GridCols; col++)
                {
                    if (!OpenCells[row, col])
                    {
                        continue; // 通行不可マスは壁生成の起点にしない
                    }

                    float xMin = col * CorridorWidth;
                    float xMax = xMin + CorridorWidth;
                    float zMin = row * CorridorWidth;
                    float zMax = zMin + CorridorWidth;

                    // ---- 右辺（列+1方向） ----
                    if (!IsOpen(row, col + 1))
                    {
                        CreateVerticalEdgeWall(outerGroup.transform, innerGroup.transform, ref wallIndex,
                            x: xMax, zMin: zMin, zMax: zMax);
                    }

                    // ---- 左辺（列-1方向） ----
                    if (!IsOpen(row, col - 1))
                    {
                        CreateVerticalEdgeWall(outerGroup.transform, innerGroup.transform, ref wallIndex,
                            x: xMin, zMin: zMin, zMax: zMax);
                    }

                    // ---- 上辺（行+1方向） ----
                    if (!IsOpen(row + 1, col))
                    {
                        CreateHorizontalEdgeWall(outerGroup.transform, innerGroup.transform, ref wallIndex,
                            z: zMax, xMin: xMin, xMax: xMax);
                    }

                    // ---- 下辺（行-1方向） ----
                    if (!IsOpen(row - 1, col))
                    {
                        CreateHorizontalEdgeWall(outerGroup.transform, innerGroup.transform, ref wallIndex,
                            z: zMin, xMin: xMin, xMax: xMax);
                    }
                }
            }
        }

        private static bool IsOpen(int row, int col)
        {
            if (row < 0 || row >= GridRows || col < 0 || col >= GridCols)
            {
                return false; // コース外は「不可」として扱う
            }
            return OpenCells[row, col];
        }

        /// <summary>X座標一定（Z方向に伸びる）壁を1枚生成し、境界か内部かでOuter/Innerに振り分ける</summary>
        private static void CreateVerticalEdgeWall(Transform outerParent, Transform innerParent, ref int wallIndex,
            float x, float zMin, float zMax)
        {
            bool isBoundary = Mathf.Approximately(x, 0f) || Mathf.Approximately(x, CourseWidth);
            Transform parent = isBoundary ? outerParent : innerParent;
            string prefix = isBoundary ? "Wall_Outer_V" : "Wall_Inner_V";

            Vector3 center = new Vector3(x, WallHeight / 2f, (zMin + zMax) / 2f);
            Vector3 size = new Vector3(WallThickness, WallHeight, (zMax - zMin) + WallThickness);

            CreateWallBox(parent, $"{prefix}_{wallIndex++}", center, size, Quaternion.identity);
        }

        /// <summary>Z座標一定（X方向に伸びる）壁を1枚生成し、境界か内部かでOuter/Innerに振り分ける</summary>
        private static void CreateHorizontalEdgeWall(Transform outerParent, Transform innerParent, ref int wallIndex,
            float z, float xMin, float xMax)
        {
            bool isBoundary = Mathf.Approximately(z, 0f) || Mathf.Approximately(z, CourseHeight);
            Transform parent = isBoundary ? outerParent : innerParent;
            string prefix = isBoundary ? "Wall_Outer_H" : "Wall_Inner_H";

            Vector3 center = new Vector3((xMin + xMax) / 2f, WallHeight / 2f, z);
            Vector3 size = new Vector3((xMax - xMin) + WallThickness, WallHeight, WallThickness);

            CreateWallBox(parent, $"{prefix}_{wallIndex++}", center, size, Quaternion.identity);
        }

        #endregion

        #region Start / Goal Lines

        private static void CreateStartLine(Transform parent, List<Vector3> centerline)
        {
            if (centerline.Count < 2)
            {
                Debug.LogWarning("[CourseGenerator] centerlineの点数が不足しているため、Startラインを生成できません。");
                return;
            }

            Vector3 direction = (centerline[1] - centerline[0]).normalized;
            CreateLineTrigger(parent, StartLineName, TagStartLine, centerline[0], direction);
        }

        private static void CreateGoalLine(Transform parent, List<Vector3> centerline)
        {
            if (centerline.Count < 2)
            {
                Debug.LogWarning("[CourseGenerator] centerlineの点数が不足しているため、Goalラインを生成できません。");
                return;
            }

            int last = centerline.Count - 1;
            Vector3 direction = (centerline[last] - centerline[last - 1]).normalized;
            CreateLineTrigger(parent, GoalLineName, TagGoalLine, centerline[last], direction);
        }

        private static void CreateLineTrigger(Transform parent, string name, string tag, Vector3 position, Vector3 travelDirection)
        {
            GameObject line = new GameObject(name);
            line.tag = tag;
            line.transform.SetParent(parent, false);
            line.transform.localPosition = position + Vector3.up * (WallHeight / 2f);
            line.transform.localRotation = Quaternion.LookRotation(travelDirection, Vector3.up);

            BoxCollider collider = line.AddComponent<BoxCollider>();
            collider.isTrigger = true;
            collider.size = new Vector3(CorridorWidth, WallHeight, 0.02f);

            var marker = line.AddComponent<CourseLineMarker>();
            marker.TravelDirection = travelDirection;

            Undo.RegisterCreatedObjectUndo(line, $"Create {name}");
        }

        #endregion

        #region Waypoints

        private static void CreateWaypoints(Transform parent, List<Vector3> centerline)
        {
            GameObject group = new GameObject(WaypointsGroupName);
            group.transform.SetParent(parent, false);
            Undo.RegisterCreatedObjectUndo(group, "Create Waypoints Group");

            for (int i = 0; i < centerline.Count; i++)
            {
                GameObject wp = new GameObject($"WP_{i}");
                wp.transform.SetParent(group.transform, false);
                wp.transform.localPosition = centerline[i];
                Undo.RegisterCreatedObjectUndo(wp, $"Create WP_{i}");
            }
        }

        #endregion

        #region Out Of Bounds Volume

        private static void CreateOutOfBoundsVolume(Transform parent)
        {
            GameObject bounds = new GameObject("OutOfBoundsVolume");
            bounds.tag = TagCourseBounds;
            bounds.transform.SetParent(parent, false);

            BoxCollider collider = bounds.AddComponent<BoxCollider>();
            collider.isTrigger = true;
            collider.center = new Vector3(CourseWidth / 2f, WallHeight / 2f, CourseHeight / 2f);
            collider.size = new Vector3(CourseWidth, WallHeight * 4f, CourseHeight);

            Undo.RegisterCreatedObjectUndo(bounds, "Create Out Of Bounds Volume");
        }

        #endregion

        #region Wall Helper

        private static void CreateWallBox(Transform parent, string name, Vector3 center, Vector3 size, Quaternion rotation)
        {
            GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
            wall.name = name;
            wall.tag = TagWall;
            wall.transform.SetParent(parent, false);
            wall.transform.localPosition = center;
            wall.transform.localRotation = rotation;
            wall.transform.localScale = size;

            Undo.RegisterCreatedObjectUndo(wall, $"Create {name}");
        }

        #endregion

        #region Tag Management

        /// <summary>
        /// 指定タグがプロジェクトに存在しなければ、TagManagerアセットへ直接追加する。
        /// これにより「Tools > Create Course」を押すだけで、事前のタグ手動作成が不要になる。
        /// </summary>
        private static void EnsureTagExists(string tagName)
        {
            if (string.IsNullOrEmpty(tagName))
            {
                return;
            }

            Object[] tagManagerAssets = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset");
            if (tagManagerAssets == null || tagManagerAssets.Length == 0)
            {
                Debug.LogWarning("[CourseGenerator] TagManager.assetの読み込みに失敗しました。タグは手動で追加してください。");
                return;
            }

            SerializedObject tagManager = new SerializedObject(tagManagerAssets[0]);
            SerializedProperty tagsProperty = tagManager.FindProperty("tags");

            if (tagsProperty == null)
            {
                Debug.LogWarning("[CourseGenerator] TagManagerのtagsプロパティが見つかりませんでした。");
                return;
            }

            for (int i = 0; i < tagsProperty.arraySize; i++)
            {
                if (tagsProperty.GetArrayElementAtIndex(i).stringValue == tagName)
                {
                    return; // 既に存在する
                }
            }

            int insertIndex = tagsProperty.arraySize;
            tagsProperty.InsertArrayElementAtIndex(insertIndex);
            tagsProperty.GetArrayElementAtIndex(insertIndex).stringValue = tagName;
            tagManager.ApplyModifiedProperties();

            Debug.Log($"[CourseGenerator] タグ \"{tagName}\" をプロジェクトに追加しました。");
        }

        #endregion
    }
}
