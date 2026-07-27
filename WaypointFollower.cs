using System.Collections.Generic;
using UnityEngine;

namespace SimulationRobotContest
{
    /// <summary>
    /// コース中心線上のウェイポイント列を追従するための目標速度・目標角速度を計算する。
    ///
    /// 本クラスはRigidbodyやPhysicsに一切触れず、「現在位置・現在向き」を入力として
    /// 「目標前進速度・目標角速度」を出力する純粋な計算コンポーネントとして設計している
    /// （SOLID: SRP。物理操作の責務はRobotControllerに委譲する）。
    /// </summary>
    public class WaypointFollower : MonoBehaviour
    {
        #region Inspector Fields

        [Header("経路")]
        [Tooltip("CourseGeneratorが生成したWaypointsの子Transformを順番に登録する")]
        [SerializeField]
        private List<Transform> waypoints = new List<Transform>();

        [Header("速度プロファイル")]
        [Tooltip("直線区間での目標巡航速度 [m/s]")]
        [SerializeField]
        private float cruiseSpeed = 0.20f;

        [Tooltip("急カーブでも維持する最低速度 [m/s]")]
        [SerializeField]
        private float minimumSpeed = 0.10f;

        [Tooltip("ウェイポイント到達とみなす距離 [m]")]
        [SerializeField]
        private float waypointReachedRadius = 0.06f;

        [Header("旋回制御")]
        [Tooltip("進行方向と目標方向の角度誤差[rad]に対する角速度指令の比例ゲイン")]
        [SerializeField]
        private float headingGain = 4.0f;

        [Tooltip("旋回指令の角速度上限 [rad/s]。方位のずれがどんなに大きくても"
            + "この値を超える急旋回は指令しない（車体が壁を振り回すように接触するのを防ぐ）")]
        [SerializeField]
        private float maxAngularSpeed = 2.0f;

        [Tooltip("先読みしたカーブのきつさに応じて速度を落とす感度")]
        [SerializeField]
        private float curvatureSlowdownGain = 6.0f;

        #endregion

        #region Private State

        private int _currentWaypointIndex;

        #endregion

        #region Public Properties

        /// <summary>最後のウェイポイントに到達したかどうか</summary>
        public bool HasReachedFinalWaypoint { get; private set; }

        /// <summary>直近に計算した方位誤差 [度]（デバッグ・診断用）</summary>
        public float LastHeadingErrorDegrees { get; private set; }

        /// <summary>現在目標としているウェイポイントのインデックス</summary>
        public int CurrentWaypointIndex => _currentWaypointIndex;

        /// <summary>登録されているウェイポイント数</summary>
        public int WaypointCount => waypoints?.Count ?? 0;

        #endregion

        #region Public Methods

        /// <summary>
        /// 外部（SceneInitializer等）からウェイポイント列を注入する。
        /// CourseGeneratorが生成したWaypointsコンテナの子要素をそのまま渡すことを想定している。
        /// </summary>
        public void Initialize(List<Transform> waypointList)
        {
            waypoints = waypointList ?? new List<Transform>();
            _currentWaypointIndex = 0;
            HasReachedFinalWaypoint = false;
        }

        /// <summary>
        /// 追従状態をリセットする（レース再スタート時に呼ぶ）。
        /// </summary>
        public void ResetFollower()
        {
            _currentWaypointIndex = 0;
            HasReachedFinalWaypoint = false;
        }

        /// <summary>
        /// 現在位置・現在の前進方向から、目標前進速度と目標角速度を計算する。
        /// </summary>
        /// <param name="currentPosition">ロボット重心のワールド座標</param>
        /// <param name="currentForward">ロボットの前進方向（ワールド座標、正規化不要）</param>
        /// <param name="desiredLinearSpeed">出力: 目標前進速度 [m/s]</param>
        /// <param name="desiredAngularSpeed">出力: 目標ヨー角速度 [rad/s]</param>
        public void ComputeDesiredVelocity(Vector3 currentPosition, Vector3 currentForward,
            out float desiredLinearSpeed, out float desiredAngularSpeed)
        {
            desiredLinearSpeed = 0.0f;
            desiredAngularSpeed = 0.0f;

            if (waypoints == null || waypoints.Count == 0 || HasReachedFinalWaypoint)
            {
                return;
            }

            Vector3 flatPosition = new Vector3(currentPosition.x, 0f, currentPosition.z);

            AdvanceWaypointIfReached(flatPosition);

            if (_currentWaypointIndex >= waypoints.Count)
            {
                HasReachedFinalWaypoint = true;
                return;
            }

            Transform targetWaypoint = waypoints[_currentWaypointIndex];
            if (targetWaypoint == null)
            {
                Debug.LogWarning($"[WaypointFollower] インデックス{_currentWaypointIndex}のWaypointがnullです。スキップします。");
                _currentWaypointIndex++;
                return;
            }

            Vector3 targetPosition = new Vector3(targetWaypoint.position.x, 0f, targetWaypoint.position.z);
            Vector3 toTarget = targetPosition - flatPosition;
            float distanceToTarget = toTarget.magnitude;
            Vector3 directionToTarget = distanceToTarget > 1e-4f ? toTarget / distanceToTarget : currentForward.normalized;

            float headingError = SignedAngleOnXZPlane(currentForward, directionToTarget);
            LastHeadingErrorDegrees = headingError * Mathf.Rad2Deg;


            float curvature = EstimateUpcomingCurvature(directionToTarget);

            float speedTarget = cruiseSpeed / (1.0f + curvatureSlowdownGain * curvature);
            speedTarget = Mathf.Max(speedTarget, minimumSpeed);

            // 最終ウェイポイント手前ではオーバーシュート防止のため減速する
            if (_currentWaypointIndex == waypoints.Count - 1)
            {
                speedTarget = Mathf.Min(speedTarget, Mathf.Max(minimumSpeed, distanceToTarget * 2.0f));
            }

            // 方位のずれが大きい場合（およそ15度=約0.26rad以上）は前進速度をゼロにし、
            // その場旋回（超信地旋回に近い動き）で向きを合わせることを優先する。
            // 「進みながら曲がる」方式だと、旋回中もわずかな並進速度が残るため、
            // 車体（半径0.15m）が通路の内側の壁角を振り回すように接触するリスクが高い。
            // 向きが合ってから前進を再開することで、旋回中の並進をほぼゼロに抑え、
            // コーナーでの接触を構造的に防ぐ。
            const float stopAndTurnThresholdRadians = 0.26f; // 約15度
            if (Mathf.Abs(headingError) > stopAndTurnThresholdRadians)
            {
                desiredLinearSpeed = 0f;
            }
            else
            {
                float headingAlignmentFactor = Mathf.Clamp01(1.0f - Mathf.Abs(headingError) / (Mathf.PI * 0.5f));
                desiredLinearSpeed = speedTarget * Mathf.Max(headingAlignmentFactor, 0.05f);
            }
            desiredAngularSpeed = Mathf.Clamp(headingGain * headingError, -maxAngularSpeed, maxAngularSpeed);
        }

        #endregion

        #region Private Methods

        private void AdvanceWaypointIfReached(Vector3 flatPosition)
        {
            while (_currentWaypointIndex < waypoints.Count)
            {
                Transform wp = waypoints[_currentWaypointIndex];
                if (wp == null)
                {
                    _currentWaypointIndex++;
                    continue;
                }

                Vector3 wpFlat = new Vector3(wp.position.x, 0f, wp.position.z);
                if (Vector3.Distance(flatPosition, wpFlat) < waypointReachedRadius)
                {
                    _currentWaypointIndex++;
                }
                else
                {
                    break;
                }
            }
        }

        private float EstimateUpcomingCurvature(Vector3 currentDirection)
        {
            int nextIndex = _currentWaypointIndex + 1;
            if (nextIndex >= waypoints.Count || waypoints[nextIndex] == null || waypoints[_currentWaypointIndex] == null)
            {
                return 0.0f;
            }

            Vector3 currentWp = waypoints[_currentWaypointIndex].position;
            Vector3 nextWp = waypoints[nextIndex].position;
            Vector3 nextDirection = new Vector3(nextWp.x - currentWp.x, 0f, nextWp.z - currentWp.z).normalized;

            float turnAngle = Mathf.Abs(SignedAngleOnXZPlane(currentDirection, nextDirection));
            return turnAngle;
        }

        private static float SignedAngleOnXZPlane(Vector3 from, Vector3 to)
        {
            from.y = 0f;
            to.y = 0f;
            float angleDegrees = Vector3.SignedAngle(from, to, Vector3.up);
            return angleDegrees * Mathf.Deg2Rad;
        }

        #endregion

        #region Gizmos

        private void OnDrawGizmosSelected()
        {
            if (waypoints == null) return;

            Gizmos.color = Color.cyan;
            for (int i = 0; i < waypoints.Count; i++)
            {
                if (waypoints[i] == null) continue;
                Gizmos.DrawSphere(waypoints[i].position, 0.04f);
                if (i > 0 && waypoints[i - 1] != null)
                {
                    Gizmos.DrawLine(waypoints[i - 1].position, waypoints[i].position);
                }
            }
        }

        #endregion
    }
}
