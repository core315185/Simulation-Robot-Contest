using UnityEngine;

namespace SimulationRobotContest
{
    /// <summary>
    /// ロボット掃除機本体を統括する中心クラス。
    ///
    /// 責務:
    ///  ・WaypointFollower（または手動操作API）からの目標速度・角速度を受け取り、
    ///    DifferentialDriveで左右車輪の目標速度に変換
    ///  ・左右車輪ごとのPIDControllerで力を計算し、合成した並進力・旋回トルクを
    ///    Rigidbody.AddForce/AddTorqueとしてのみ適用（Transform直接操作は一切行わない）
    ///  ・カメラ先端水平加速度が1.00 m/s^2を超えないよう、合成水平力を
    ///    F_max = mass × maxHorizontalAccel にクランプ
    ///  ・スタート/ゴール判定、壁接触・コース外・転倒による失格判定、タイマー計測
    ///
    /// 設計判断（力の作用点について）:
    ///  車輪接地点（床面, y=0）に直接AddForceAtPositionする方式は、
    ///  重心（y=0.50m）との高低差からピッチ方向の余計なトルクを生み、
    ///  「カメラ先端加速度1.00 m/s^2以下」の機構的な保証を不安定にする。
    ///  そのため本実装では、差動二輪の力学（左右力の和＝並進力、左右力の差×半トラック＝旋回トルク）
    ///  を重心に対する一般化力として適用する、平面移動ロボットの標準的な簡略化モデルを採用する。
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public class RobotController : MonoBehaviour
    {
        #region Race State Enum

        public enum RaceState
        {
            NotStarted,
            Running,
            Finished,
            Disqualified
        }

        #endregion

        #region Inspector Fields - Physical Parameters

        [Header("車体物理パラメータ（課題必須条件）")]
        [Tooltip("ロボット全質量 [kg]（課題必須条件: 10kg）")]
        [SerializeField]
        private float robotMass = 10.0f;

        [Tooltip("床面からの重心高さ [m]（課題必須条件: 0.50m）")]
        [SerializeField]
        private float centerOfMassHeight = 0.50f;

        [Header("差動駆動パラメータ")]
        [Tooltip("左右駆動輪の間隔 [m]。ロボット直径0.30m未満にすること")]
        [SerializeField]
        private float wheelTrack = 0.22f;

        [Tooltip("駆動輪の半径 [m]（視覚的な車輪回転表現にのみ使用）")]
        [SerializeField]
        private float wheelRadius = 0.03f;

        [Tooltip("ロボットの外形半径 [m]（コース外判定・ゴール完全通過判定用。直径0.30mなら0.15）")]
        [SerializeField]
        private float robotRadius = 0.15f;

        [Header("加速度制約（課題必須条件）")]
        [Tooltip("カメラ先端に許される水平合成加速度の上限 [m/s^2]")]
        [SerializeField]
        private float maxHorizontalAccel = 1.00f;

        [Tooltip("旋回トルクの数値的安全上限 [N・m]（課題上はトルク無制限だが、発散防止のため設定）")]
        [SerializeField]
        private float maxYawTorque = 1000.0f;

        #endregion

        #region Inspector Fields - Control Gains

        [Header("左車輪 速度PID")]
        [SerializeField]
        private PIDController leftWheelPID = new PIDController
        {
            ProportionalGain = 15f,
            IntegralGain = 3f,
            DerivativeGain = 0.5f,
            IntegralClampValue = 5f,
            OutputClampValue = 15f
        };

        [Header("右車輪 速度PID")]
        [SerializeField]
        private PIDController rightWheelPID = new PIDController
        {
            ProportionalGain = 15f,
            IntegralGain = 3f,
            DerivativeGain = 0.5f,
            IntegralClampValue = 5f,
            OutputClampValue = 15f
        };

        [Header("横滑り抑制（車輪と床面の摩擦相当）")]
        [Tooltip("横方向速度を打ち消す力の比例ゲイン")]
        [SerializeField]
        private float lateralGripGain = 40.0f;

        #endregion

        #region Inspector Fields - Failure Detection

        [Header("失格判定")]
        [Tooltip("転倒とみなす傾き角度のしきい値 [度]")]
        [SerializeField]
        private float tipOverAngleThresholdDegrees = 25.0f;

        #endregion

        #region Inspector Fields - Dependencies

        [Header("依存コンポーネント（SceneInitializerまたはRobotGeneratorが自動配線）")]
        [SerializeField] private WaypointFollower waypointFollower;
        [SerializeField] private CameraController cameraController;
        [SerializeField] private DataLogger dataLogger;
        [SerializeField] private GraphExporter graphExporter;

        [Header("車輪ビジュアル（コスメティック回転のみ、物理には無関係）")]
        [SerializeField] private Transform wheelLeftVisual;
        [SerializeField] private Transform wheelRightVisual;

        [Header("自動走行")]
        [Tooltip("trueならWaypointFollowerによる自動走行、falseなら手動操作API待ち")]
        [SerializeField]
        private bool autoDriveEnabled = true;

        [Header("デバッグ")]
        [Tooltip("trueにすると、走行中の目標値・実測値を一定間隔でConsoleへ出力する（動作確認用）")]
        [SerializeField]
        private bool enableDebugLogging = false;

        [Tooltip("デバッグログの出力間隔 [s]")]
        [SerializeField]
        private float debugLogIntervalSeconds = 0.5f;

        #endregion

        #region Private State

        private Rigidbody _rigidbody;
        private DifferentialDrive _differentialDrive;

        /// <summary>
        /// DifferentialDriveのインスタンスを取得する。何らかの理由でAwake()完了前に
        /// 参照された場合でも、ここで確実に生成してnullを返さないようにする防御的アクセサ
        /// （スクリプト再コンパイル直後のPlay開始タイミングなどでの NullReferenceException を防ぐ）。
        /// </summary>
        private DifferentialDrive Drive
        {
            get
            {
                if (_differentialDrive == null)
                {
                    _differentialDrive = new DifferentialDrive(wheelTrack, wheelRadius);
                }
                return _differentialDrive;
            }
        }
        private Collider _courseBoundsCollider;
        private CourseLineMarker _goalLineMarker;
        private Transform _goalLineTransform;
        private float _debugLogTimer;
        private float _lastDesiredLinearSpeed;
        private float _lastDesiredAngularSpeed;

        private float _manualDesiredLinearSpeed;
        private float _manualDesiredAngularSpeed;
        private bool _manualOverrideActive;

        private float _raceStartTime;
        private string _disqualificationReason = string.Empty;

        #endregion

        #region Public Properties

        public RaceState CurrentState { get; private set; } = RaceState.NotStarted;
        public float ElapsedTime { get; private set; }
        public float CurrentForwardSpeed { get; private set; }
        public float CurrentAngularSpeed { get; private set; }
        public string DisqualificationReason => _disqualificationReason;

        #endregion

        #region Unity Lifecycle

        private void Awake()
        {
            _rigidbody = GetComponent<Rigidbody>();
            InitializePhysicalProperties();

            _differentialDrive = new DifferentialDrive(wheelTrack, wheelRadius);

            if (dataLogger == null) dataLogger = GetComponent<DataLogger>();
            if (graphExporter == null) graphExporter = GetComponent<GraphExporter>();
            if (waypointFollower == null) waypointFollower = GetComponent<WaypointFollower>();
        }

        private void Start()
        {
            GameObject boundsObject = GameObject.FindGameObjectWithTag("CourseBounds");
            if (boundsObject != null)
            {
                _courseBoundsCollider = boundsObject.GetComponent<Collider>();
            }
            else
            {
                Debug.LogWarning("[RobotController] \"CourseBounds\" タグのオブジェクトが見つかりません。コース外判定は無効化されます。");
            }

            GameObject goalObject = GameObject.FindGameObjectWithTag("GoalLine");
            if (goalObject != null)
            {
                _goalLineMarker = goalObject.GetComponent<CourseLineMarker>();
                _goalLineTransform = goalObject.transform;
            }
            else
            {
                Debug.LogWarning("[RobotController] \"GoalLine\" タグのオブジェクトが見つかりません。ゴール判定は無効化されます。");
            }
        }

        private void FixedUpdate()
        {
            float dt = Time.fixedDeltaTime;

            UpdateMotionMeasurements();

            if (CurrentState == RaceState.Running)
            {
                ApplyDriveControl(dt);
                UpdateWheelVisuals(dt);

                ElapsedTime = Time.time - _raceStartTime;

                CheckTipOver();
                CheckOutOfBounds();
                CheckGoalReached();

                if (CurrentState == RaceState.Running && dataLogger != null && cameraController != null)
                {
                    dataLogger.LogSample(
                        ElapsedTime,
                        _rigidbody.position,
                        CurrentForwardSpeed,
                        CurrentAngularSpeed,
                        cameraController.CurrentHorizontalAcceleration,
                        cameraController.CurrentHorizontalJerk);
                }
            }
            else
            {
                // 走行していない間は力を加えない（静止状態を維持）
                UpdateWheelVisuals(dt);
            }
        }

        #endregion

        #region Initialization Helpers

        private void InitializePhysicalProperties()
        {
            _rigidbody.mass = robotMass;
            _rigidbody.centerOfMass = new Vector3(0f, centerOfMassHeight, 0f);

            // 課題必須条件: Collision Detection は Continuous Dynamic
            _rigidbody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            _rigidbody.interpolation = RigidbodyInterpolation.Interpolate;

            // ロボットの当たり判定は直径0.30m・高さ1.00mの縦に細長いカプセルであり、
            // 慣性モーメントが小さいため、床との接地摩擦の反力だけで容易に転倒してしまう。
            // 実機の掃除機ロボットは接地面が低く、通常走行でピッチ・ロール方向に傾くことは
            // 想定されないため、その2軸の回転を物理的に拘束する（ヨー回転と水平移動は自由）。
            // これにより「駆動力自体は重心に作用させ、直接の転倒トルクを生まない」という
            // 制御設計の意図が、接地反力によるピッチ不安定性で崩れるのを防ぐ。
            _rigidbody.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        }

        #endregion

        #region Motion Measurement

        private void UpdateMotionMeasurements()
        {
            Vector3 velocity = _rigidbody.linearVelocity;
            Vector3 horizontalVelocity = new Vector3(velocity.x, 0f, velocity.z);

            CurrentForwardSpeed = Vector3.Dot(horizontalVelocity, transform.forward);
            CurrentAngularSpeed = _rigidbody.angularVelocity.y;
        }

        #endregion

        #region Drive Control

        private void ApplyDriveControl(float dt)
        {
            GetDesiredVelocity(out float desiredLinearSpeed, out float desiredAngularSpeed);
            _lastDesiredLinearSpeed = desiredLinearSpeed;
            _lastDesiredAngularSpeed = desiredAngularSpeed;

            Drive.ComputeWheelSpeeds(desiredLinearSpeed, desiredAngularSpeed,
                out float desiredLeftWheelSpeed, out float desiredRightWheelSpeed);

            Drive.ComputeWheelSpeeds(CurrentForwardSpeed, CurrentAngularSpeed,
                out float actualLeftWheelSpeed, out float actualRightWheelSpeed);

            float leftForce = leftWheelPID.ComputeOutput(desiredLeftWheelSpeed - actualLeftWheelSpeed, dt);
            float rightForce = rightWheelPID.ComputeOutput(desiredRightWheelSpeed - actualRightWheelSpeed, dt);

            float netForwardForce = leftForce + rightForce;
            float netYawTorque = (rightForce - leftForce) * (wheelTrack * 0.5f);

            // ---- 横滑り抑制力（摩擦相当） ----
            Vector3 velocity = _rigidbody.linearVelocity;
            Vector3 horizontalVelocity = new Vector3(velocity.x, 0f, velocity.z);
            float lateralSpeed = Vector3.Dot(horizontalVelocity, transform.right);
            float lateralForce = -lateralSpeed * lateralGripGain;

            // ---- 合成水平力を F_max = m * a_max にクランプ ----
            Vector3 totalHorizontalForce = (transform.forward * netForwardForce) + (transform.right * lateralForce);
            float maxForce = _rigidbody.mass * maxHorizontalAccel;
            if (totalHorizontalForce.magnitude > maxForce)
            {
                totalHorizontalForce = totalHorizontalForce.normalized * maxForce;
            }

            _rigidbody.AddForce(totalHorizontalForce, ForceMode.Force);

            float clampedTorque = Mathf.Clamp(netYawTorque, -maxYawTorque, maxYawTorque);
            _rigidbody.AddTorque(Vector3.up * clampedTorque, ForceMode.Force);

            LogDebugStateIfNeeded(dt);
        }

        private void LogDebugStateIfNeeded(float dt)
        {
            if (!enableDebugLogging)
            {
                return;
            }

            _debugLogTimer += dt;
            if (_debugLogTimer < debugLogIntervalSeconds)
            {
                return;
            }
            _debugLogTimer = 0f;

            float headingErrorDegrees = waypointFollower != null ? waypointFollower.LastHeadingErrorDegrees : 0f;
            int waypointIndex = waypointFollower != null ? waypointFollower.CurrentWaypointIndex : -1;

            Debug.Log(
                $"[RobotController][DEBUG] t={ElapsedTime:F2}s "
                + $"WP={waypointIndex} headingErr={headingErrorDegrees:F1}deg "
                + $"desiredV={_lastDesiredLinearSpeed:F2} actualV={CurrentForwardSpeed:F2} "
                + $"desiredOmega={_lastDesiredAngularSpeed:F2} actualOmega={CurrentAngularSpeed:F2} "
                + $"pos=({_rigidbody.position.x:F2},{_rigidbody.position.z:F2})");
        }

        private void GetDesiredVelocity(out float desiredLinearSpeed, out float desiredAngularSpeed)
        {

            if (_manualOverrideActive)
            {
                desiredLinearSpeed = _manualDesiredLinearSpeed;
                desiredAngularSpeed = _manualDesiredAngularSpeed;
                return;
            }

            if (autoDriveEnabled && waypointFollower != null)
            {
                waypointFollower.ComputeDesiredVelocity(_rigidbody.position, transform.forward,
                    out desiredLinearSpeed, out desiredAngularSpeed);
                return;
            }

            desiredLinearSpeed = 0f;
            desiredAngularSpeed = 0f;
        }

        private void UpdateWheelVisuals(float dt)
        {
            if (wheelLeftVisual == null && wheelRightVisual == null)
            {
                return;
            }

            Drive.ComputeWheelSpeeds(CurrentForwardSpeed, CurrentAngularSpeed,
                out float actualLeftWheelSpeed, out float actualRightWheelSpeed);

            if (wheelLeftVisual != null)
            {
                float leftAngularSpeed = Drive.LinearSpeedToWheelAngularSpeed(actualLeftWheelSpeed);
                wheelLeftVisual.Rotate(Vector3.right, leftAngularSpeed * Mathf.Rad2Deg * dt, Space.Self);
            }

            if (wheelRightVisual != null)
            {
                float rightAngularSpeed = Drive.LinearSpeedToWheelAngularSpeed(actualRightWheelSpeed);
                wheelRightVisual.Rotate(Vector3.right, rightAngularSpeed * Mathf.Rad2Deg * dt, Space.Self);
            }
        }

        #endregion

        #region Manual Motion Primitives（課題必須条件の各動作を明示的にデモ実行するためのAPI）

        /// <summary>手動操作モードへ切り替える（WaypointFollowerを無視する）</summary>
        public void EnableManualControl()
        {
            _manualOverrideActive = true;
        }

        /// <summary>自動走行モードへ戻す</summary>
        public void DisableManualControl()
        {
            _manualOverrideActive = false;
        }

        public void MoveForward(float speed)
        {
            EnableManualControl();
            _manualDesiredLinearSpeed = Mathf.Abs(speed);
            _manualDesiredAngularSpeed = 0f;
        }

        public void MoveBackward(float speed)
        {
            EnableManualControl();
            _manualDesiredLinearSpeed = -Mathf.Abs(speed);
            _manualDesiredAngularSpeed = 0f;
        }

        public void StopMotion()
        {
            EnableManualControl();
            _manualDesiredLinearSpeed = 0f;
            _manualDesiredAngularSpeed = 0f;
        }

        /// <summary>信地旋回（片方の車輪を軸に旋回）</summary>
        public void PivotTurn(float wheelSpeed, bool turnLeft)
        {
            EnableManualControl();
            Drive.ComputePivotTurnSpeeds(wheelSpeed, turnLeft,
                out float leftWheelSpeed, out float rightWheelSpeed);
            Drive.ComputeBodyVelocity(leftWheelSpeed, rightWheelSpeed,
                out _manualDesiredLinearSpeed, out _manualDesiredAngularSpeed);
        }

        /// <summary>超信地旋回（その場でのゼロ半径旋回）</summary>
        public void SpinInPlace(float angularSpeed)
        {
            EnableManualControl();
            _manualDesiredLinearSpeed = 0f;
            _manualDesiredAngularSpeed = angularSpeed;
        }

        #endregion

        #region Race Lifecycle

        private void BeginRace()
        {
            CurrentState = RaceState.Running;
            _raceStartTime = Time.time;
            ElapsedTime = 0f;
            _disqualificationReason = string.Empty;

            leftWheelPID.ResetState();
            rightWheelPID.ResetState();
            cameraController?.ResetMeasurement();
            dataLogger?.BeginLogging();

            Debug.Log("[RobotController] 計測を開始しました。");
        }

        private void FinishRaceSuccess()
        {
            if (CurrentState != RaceState.Running)
            {
                return;
            }

            CurrentState = RaceState.Finished;
            StopMotion();
            FinalizeLogging();

            Debug.Log($"[RobotController] ゴール到達。走破時間: {ElapsedTime:F3} s");
        }

        private void Disqualify(string reason)
        {
            if (CurrentState != RaceState.Running)
            {
                return;
            }

            CurrentState = RaceState.Disqualified;
            _disqualificationReason = reason;
            StopMotion();
            FinalizeLogging();

            Debug.Log($"[RobotController] 失格: {reason}");
        }

        private void FinalizeLogging()
        {
            if (dataLogger == null)
            {
                return;
            }

            bool success = CurrentState == RaceState.Finished;
            dataLogger.EndLogging();

            if (graphExporter != null && !string.IsNullOrEmpty(dataLogger.LastCsvPath))
            {
                graphExporter.ExportGraphs(dataLogger.LastCsvPath);
            }
        }

        #endregion

        #region Failure Detection

        /// <summary>
        /// 転倒判定。InitializePhysicalPropertiesでピッチ・ロール回転を拘束しているため、
        /// 通常の走行では実質的に発生しない（コード上は課題必須条件を満たす形で残してある）。
        /// </summary>
        private void CheckTipOver()
        {
            float angle = Vector3.Angle(transform.up, Vector3.up);
            if (angle > tipOverAngleThresholdDegrees)
            {
                Disqualify("転倒");
            }
        }

        private void CheckOutOfBounds()
        {
            if (_courseBoundsCollider == null)
            {
                return;
            }

            Bounds bounds = _courseBoundsCollider.bounds;
            Vector3 position = _rigidbody.position;

            bool insideXZ =
                position.x >= bounds.min.x && position.x <= bounds.max.x &&
                position.z >= bounds.min.z && position.z <= bounds.max.z;

            if (!insideXZ)
            {
                Disqualify("コース外への移動");
            }
        }

        #endregion

        #region Collision / Trigger Handling

        private void OnCollisionEnter(Collision collision)
        {
            if (collision.collider == null)
            {
                return;
            }

            if (CurrentState == RaceState.Running && collision.collider.CompareTag("Wall"))
            {
                Disqualify("壁への接触");
            }
        }

        private void OnTriggerEnter(Collider other)
        {
            if (other == null)
            {
                return;
            }

            if (CurrentState == RaceState.NotStarted && other.CompareTag("StartLine"))
            {
                BeginRace();
            }
        }

        /// <summary>
        /// ゴール判定。以前はOnTriggerStay/OnTriggerExit（トリガーの当たり判定が
        /// 実際に重なっている間だけ発火するイベント）に頼っていたが、ゴールラインの
        /// 当たり判定が薄い板状であるため、ロボットが通過する一瞬のタイミング次第で
        /// イベントの発火漏れが起こりうる不安定な設計だった。
        /// 代わりに、毎FixedUpdateで直接「ロボットがラインの進行方向側へ
        /// ロボット半径ぶん完全に抜けているか」を計算するため、イベントの発火漏れに
        /// 起因する判定漏れが原理的に発生しない。
        /// </summary>
        private void CheckGoalReached()
        {
            if (_goalLineMarker == null || _goalLineTransform == null)
            {
                return;
            }

            Vector3 direction = _goalLineMarker.TravelDirection.normalized;
            Vector3 linePosition = _goalLineTransform.position;

            Vector3 toRobot = _rigidbody.position - linePosition;
            Vector3 toRobotFlat = new Vector3(toRobot.x, 0f, toRobot.z);

            float signedDistance = Vector3.Dot(toRobotFlat, direction);

            // 進行方向に対して直交する横方向のずれも確認する。
            // これを見ずに前後方向の座標だけで判定すると、コース上の全く別の場所
            // （例えば別の角）でも進行方向の座標が偶然一致してしまい、
            // 誤ってゴール扱いになってしまう（実際に発生した不具合）。
            Vector3 lateralAxis = Vector3.Cross(Vector3.up, direction);
            float lateralOffset = Vector3.Dot(toRobotFlat, lateralAxis);
            bool withinLane = Mathf.Abs(lateralOffset) <= _goalLineMarker.LateralHalfWidth;

            // ロボット全体（半径ぶん含む）がラインの進行方向側へ完全に抜けたら成功
            if (withinLane && signedDistance > robotRadius)
            {
                FinishRaceSuccess();
            }
        }

        #endregion

        #region Dependency Injection API（RobotGenerator等から配線するための公開メソッド）

        public void SetDependencies(WaypointFollower newWaypointFollower, CameraController newCameraController,
            DataLogger newDataLogger, GraphExporter newGraphExporter)
        {
            waypointFollower = newWaypointFollower;
            cameraController = newCameraController;
            dataLogger = newDataLogger;
            graphExporter = newGraphExporter;
        }

        public void SetKinematicsParameters(float newWheelTrack, float newWheelRadius)
        {
            wheelTrack = newWheelTrack;
            wheelRadius = newWheelRadius;
            _differentialDrive = new DifferentialDrive(wheelTrack, wheelRadius);
        }

        public void SetVisualWheels(Transform leftWheel, Transform rightWheel)
        {
            wheelLeftVisual = leftWheel;
            wheelRightVisual = rightWheel;
        }

        #endregion
    }
}
