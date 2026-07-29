using System.Collections.Generic;
using UnityEngine;

namespace SimulationRobotContest
{
    /// <summary>
    /// コース上の最適軌道（アウト・イン・アウト）を追従し、
    /// 物理ベースの制動距離計算およびゴール直前のフルスロットル制御を行うコンポーネント。
    /// </summary>
    public class WaypointFollower : MonoBehaviour
    {
        #region Inspector Fields

        [Header("経路（アウト・イン・アウト最適化座標）")]
        [Tooltip("コース幅と車体半径(0.15m)を考慮した最短・緩カーブのウェイポイント列")]
        [SerializeField]
        private List<Transform> waypoints = new List<Transform>();

        [Header("速度プロファイル（物理限界パラメータ）")]
        [Tooltip("直線区間での目標巡航最高速度 [m/s]")]
        [SerializeField]
        private float cruiseSpeed = 1.80f;

        [Tooltip("コーナー（曲がり角）通過時の目標速度 [m/s]")]
        [SerializeField]
        private float cornerSpeed = 0.45f;

        [Tooltip("急カーブでも維持する最低速度 [m/s]")]
        [SerializeField]
        private float minimumSpeed = 0.25f;

        [Tooltip("許容する最大減速度 [m/s²] (1.00m/s²制限に対する攻めの安全マージン)")]
        [SerializeField]
        private float maxDeceleration = 0.94f;

        [Tooltip("ウェイポイント到達とみなす距離 [m]")]
        [SerializeField]
        private float waypointReachedRadius = 0.22f;

        [Header("旋回制御")]
        [Tooltip("進行方向と目標方向の角度誤差[rad]に対する角速度指令の比例ゲイン")]
        [SerializeField]
        private float headingGain = 6.0f;

        [Tooltip("旋回指令の角速度上限 [rad/s]")]
        [SerializeField]
        private float maxAngularSpeed = 4.2f;

        [Tooltip("完全に停止して超信地旋回を行う角度閾値 [rad]（約30度）")]
        [SerializeField]
        private float stopAndTurnThresholdRadians = 0.52f;

        #endregion

        #region Private State

        private int _currentWaypointIndex;

        // ハードコード用フォールバック座標（Inverse-Sコース用のアウト・イン・アウト軌道）
        private readonly List<Vector3> _defaultOptimizedWaypoints = new List<Vector3>()
        {
            new Vector3(-0.90f, 0f, -2.10f), // 1. スタートライン位置
            new Vector3(-0.90f, 0f, -1.05f), // 2. 第1コーナー侵入（アウト：外側）
            new Vector3( 0.72f, 0f, -0.75f), // 3. 第1コーナー頂点（イン：内角ギリギリ）
            new Vector3( 0.90f, 0f,  0.10f), // 4. 中間ストレート（アウト：外側切り返し）
            new Vector3(-0.72f, 0f,  0.45f), // 5. 第2コーナー頂点（イン：内角ギリギリ）
            new Vector3(-0.90f, 0f,  1.50f), // 6. ゴール直線侵入
            new Vector3(-0.90f, 0f,  2.20f)  // 7. ゴール完全通過位置
        };

        #endregion

        #region Public Properties

        /// <summary>最後のウェイポイントに到達したかどうか</summary>
        public bool HasReachedFinalWaypoint { get; private set; }

        /// <summary>直近に計算した方位誤差 [度]（デバッグ・診断用）</summary>
        public float LastHeadingErrorDegrees { get; private set; }

        /// <summary>現在目標としているウェイポイントのインデックス</summary>
        public int CurrentWaypointIndex => _currentWaypointIndex;

        /// <summary>登録されているウェイポイント数</summary>
        public int WaypointCount => (waypoints != null && waypoints.Count > 0) ? waypoints.Count : _defaultOptimizedWaypoints.Count;

        #endregion

        #region Public Methods

        public void Initialize(List<Transform> waypointList)
        {
            waypoints = waypointList ?? new List<Transform>();
            _currentWaypointIndex = 0;
            HasReachedFinalWaypoint = false;
        }

        public void ResetFollower()
        {
            _currentWaypointIndex = 0;
            HasReachedFinalWaypoint = false;
        }

        public void ComputeDesiredVelocity(Vector3 currentPosition, Vector3 currentForward,
            out float desiredLinearSpeed, out float desiredAngularSpeed)
        {
            desiredLinearSpeed = 0.0f;
            desiredAngularSpeed = 0.0f;

            if (HasReachedFinalWaypoint) return;

            Vector3 flatPosition = new Vector3(currentPosition.x, 0f, currentPosition.z);

            AdvanceWaypointIfReached(flatPosition);

            int totalWpCount = WaypointCount;
            if (_currentWaypointIndex >= totalWpCount)
            {
                HasReachedFinalWaypoint = true;
                return;
            }

            Vector3 targetPosition = GetWaypointPosition(_currentWaypointIndex);
            Vector3 toTarget = targetPosition - flatPosition;
            float distanceToTarget = toTarget.magnitude;
            Vector3 directionToTarget = distanceToTarget > 1e-4f ? toTarget / distanceToTarget : currentForward.normalized;

            float headingError = SignedAngleOnXZPlane(currentForward, directionToTarget);
            LastHeadingErrorDegrees = headingError * Mathf.Rad2Deg;

            // --- 1. 速度制御（物理制動＋ラストスパート） ---
            float speedTarget;

            // ★ラストスパート（ゴール手前の最終区間は全開加速）
            if (_currentWaypointIndex >= totalWpCount - 1)
            {
                speedTarget = 2.50f; // フルスロットル
            }
            else
            {
                speedTarget = CalculatePhysicsBasedTargetSpeed(directionToTarget, distanceToTarget);
            }

            // --- 2. 角度誤差に応じた並進・旋回速度の設定 ---
            float absHeadingError = Mathf.Abs(headingError);

            if (absHeadingError > stopAndTurnThresholdRadians)
            {
                // 方位ズレが非常に大きい場合のみ、完全停止して旋回を優先
                desiredLinearSpeed = 0f;
            }
            else
            {
                // 滑らかな旋回と減速（連続走行の維持）
                float headingAlignmentFactor = Mathf.Clamp01(1.0f - absHeadingError / stopAndTurnThresholdRadians);
                desiredLinearSpeed = speedTarget * Mathf.Max(headingAlignmentFactor, 0.25f);
            }

            desiredAngularSpeed = Mathf.Clamp(headingGain * headingError, -maxAngularSpeed, maxAngularSpeed);
        }

        #endregion

        #region Private Methods

        private void AdvanceWaypointIfReached(Vector3 flatPosition)
        {
            int totalWpCount = WaypointCount;
            while (_currentWaypointIndex < totalWpCount)
            {
                Vector3 wpFlat = GetWaypointPosition(_currentWaypointIndex);
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

        /// <summary>
        /// 物理制動距離 d = (v² - v_target²) / (2 * a) に基づく減速制御
        /// </summary>
        private float CalculatePhysicsBasedTargetSpeed(Vector3 currentDirection, float distanceToNextWp)
        {
            float upcomingTurnAngle = EstimateUpcomingTurnAngle(currentDirection);

            // 緩やかなカーブ（35度未満）は巡航速度のまま通過
            if (upcomingTurnAngle < 0.61f) // 約35度
            {
                return cruiseSpeed;
            }

            // 必要制動距離の算出
            float requiredBrakeDistance = (cruiseSpeed * cruiseSpeed - cornerSpeed * cornerSpeed) / (2.0f * maxDeceleration);

            // ブレーキポイント手前は巡航速度を全開維持
            if (distanceToNextWp > requiredBrakeDistance)
            {
                return cruiseSpeed;
            }

            // ブレーキ区間内の物理的限界到達速度
            float allowedSpeed = Mathf.Sqrt(cornerSpeed * cornerSpeed + 2.0f * maxDeceleration * distanceToNextWp);

            return Mathf.Clamp(allowedSpeed, minimumSpeed, cruiseSpeed);
        }

        private float EstimateUpcomingTurnAngle(Vector3 currentDirection)
        {
            int nextIndex = _currentWaypointIndex + 1;
            int totalWpCount = WaypointCount;

            if (nextIndex >= totalWpCount) return 0.0f;

            Vector3 currentWp = GetWaypointPosition(_currentWaypointIndex);
            Vector3 nextWp = GetWaypointPosition(nextIndex);
            Vector3 nextDirection = new Vector3(nextWp.x - currentWp.x, 0f, nextWp.z - currentWp.z).normalized;

            return Mathf.Abs(SignedAngleOnXZPlane(currentDirection, nextDirection));
        }

        private Vector3 GetWaypointPosition(int index)
        {
            if (waypoints != null && index < waypoints.Count && waypoints[index] != null)
            {
                return new Vector3(waypoints[index].position.x, 0f, waypoints[index].position.z);
            }

            if (index < _defaultOptimizedWaypoints.Count)
            {
                return _defaultOptimizedWaypoints[index];
            }

            return Vector3.zero;
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
            Gizmos.color = Color.cyan;
            int total = WaypointCount;

            for (int i = 0; i < total; i++)
            {
                Vector3 pos = GetWaypointPosition(i);
                Gizmos.DrawSphere(pos, 0.05f);
                if (i > 0)
                {
                    Gizmos.DrawLine(GetWaypointPosition(i - 1), pos);
                }
            }
        }

        #endregion
    }
}