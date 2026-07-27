using System.Collections.Generic;
using UnityEngine;

namespace SimulationRobotContest
{
    /// <summary>
    /// シーン開始時に以下を行う起動スクリプト（Composition Root）。
    ///  ・課題必須条件「Fixed Timestepを明示し、シミュレーション中は一定とする」を保証
    ///  ・CourseGeneratorが生成した "Course/Waypoints" の子要素をWaypointFollowerへ注入
    ///  ・RobotController / CameraController / DataLogger / GraphExporter 間の依存関係を
    ///    タグ・名前検索で自動的に解決し、Inspectorでの手動配線漏れを防ぐ
    ///
    /// シーン内に空のGameObjectを1つ作り、本スクリプトだけをアタッチすればよい。
    /// </summary>
    public class SceneInitializer : MonoBehaviour
    {
        #region Inspector Fields

        [Header("物理設定（課題必須条件）")]
        [Tooltip("物理演算のFixed Timestep [s]（課題必須条件: 0.01）")]
        [SerializeField]
        private float fixedTimestep = 0.01f;

        [Header("コースオブジェクト名")]
        [SerializeField] private string courseRootName = "Course";
        [SerializeField] private string waypointsGroupName = "Waypoints";

        [Header("自動配線を行うかどうか")]
        [Tooltip("falseにした場合、各コンポーネントの参照はInspectorで手動設定されている前提になる")]
        [SerializeField]
        private bool autoWireDependencies = true;

        #endregion

        #region Unity Lifecycle

        private void Awake()
        {
            ApplyFixedTimestep();
        }

        private void Start()
        {
            if (autoWireDependencies)
            {
                WireWaypoints();
                WireRobotDependencies();
            }
        }

        #endregion

        #region Physics Setup

        private void ApplyFixedTimestep()
        {
            Time.fixedDeltaTime = fixedTimestep;
            Time.maximumDeltaTime = fixedTimestep * 4f;

            Debug.Log($"[SceneInitializer] Fixed Timestep = {Time.fixedDeltaTime} s に設定しました。");
        }

        #endregion

        #region Dependency Wiring

        private void WireWaypoints()
        {
            GameObject courseRoot = GameObject.Find(courseRootName);
            if (courseRoot == null)
            {
                Debug.LogWarning($"[SceneInitializer] \"{courseRootName}\" が見つかりません。"
                    + " Tools > Create Course を先に実行してください。");
                return;
            }

            Transform waypointsGroup = courseRoot.transform.Find(waypointsGroupName);
            if (waypointsGroup == null)
            {
                Debug.LogWarning($"[SceneInitializer] \"{courseRootName}/{waypointsGroupName}\" が見つかりません。");
                return;
            }

            var waypointList = new List<Transform>();
            for (int i = 0; i < waypointsGroup.childCount; i++)
            {
                waypointList.Add(waypointsGroup.GetChild(i));
            }

            WaypointFollower waypointFollower = FindFirstObjectByType<WaypointFollower>();
            if (waypointFollower == null)
            {
                Debug.LogWarning("[SceneInitializer] シーン内にWaypointFollowerが見つかりません。"
                    + " Tools > Create Robot を先に実行してください。");
                return;
            }

            waypointFollower.Initialize(waypointList);
            Debug.Log($"[SceneInitializer] ウェイポイントを{waypointList.Count}点、WaypointFollowerへ注入しました。");

            PlaceRobotAtStartWaypoint(waypointList);
        }

        /// <summary>
        /// ロボットをスタート地点（先頭のウェイポイント）へ自動で配置する。
        ///
        /// RobotGeneratorはロボットをワールド原点(0,0,0)に生成するが、これはコース外壁の
        /// 角（x=0とz=0の壁の交点）にほぼ一致する。手動でスタート地点へ動かし忘れたまま
        /// Playすると、ロボットが壁にめり込んだ状態で物理演算が始まり、めり込み解消のための
        /// 強い反発力・トルクが発生して「その場でぐるぐる回転する」不具合の原因になる。
        /// これを防ぐため、Play開始時に必ずスタート地点へ確実に配置する。
        /// </summary>
        private void PlaceRobotAtStartWaypoint(List<Transform> waypointList)
        {
            if (waypointList == null || waypointList.Count == 0)
            {
                return;
            }

            RobotController robotController = FindFirstObjectByType<RobotController>();
            if (robotController == null)
            {
                return;
            }

            Rigidbody robotRigidbody = robotController.GetComponent<Rigidbody>();
            if (robotRigidbody == null)
            {
                return;
            }

            Vector3 startPosition = waypointList[0].position;
            Quaternion startRotation = Quaternion.identity;

            if (waypointList.Count > 1)
            {
                Vector3 directionToNext = waypointList[1].position - waypointList[0].position;
                directionToNext.y = 0f;
                if (directionToNext.sqrMagnitude > 1e-6f)
                {
                    startRotation = Quaternion.LookRotation(directionToNext.normalized, Vector3.up);
                }
            }

            // Transform直接操作ではなく、Rigidbodyの初期化用APIを用いて配置する
            // （シミュレーション開始前のセットアップであり、走行中の移動には一切使用しない）。
            robotRigidbody.position = startPosition;
            robotRigidbody.rotation = startRotation;
            robotRigidbody.linearVelocity = Vector3.zero;
            robotRigidbody.angularVelocity = Vector3.zero;

            Debug.Log($"[SceneInitializer] ロボットをスタート地点 {startPosition} へ自動配置しました。");
        }

        private void WireRobotDependencies()
        {
            RobotController robotController = FindFirstObjectByType<RobotController>();
            if (robotController == null)
            {
                Debug.LogWarning("[SceneInitializer] シーン内にRobotControllerが見つかりません。");
                return;
            }

            WaypointFollower waypointFollower = FindFirstObjectByType<WaypointFollower>();
            CameraController cameraController = FindFirstObjectByType<CameraController>();
            DataLogger dataLogger = robotController.GetComponent<DataLogger>();
            GraphExporter graphExporter = robotController.GetComponent<GraphExporter>();

            robotController.SetDependencies(waypointFollower, cameraController, dataLogger, graphExporter);

            if (cameraController != null)
            {
                Rigidbody robotRigidbody = robotController.GetComponent<Rigidbody>();
                cameraController.SetTargetRigidbody(robotRigidbody);
            }

            Debug.Log("[SceneInitializer] RobotControllerの依存コンポーネントを自動配線しました。");
        }

        #endregion
    }
}
