using UnityEngine;

namespace SimulationRobotContest
{
    /// <summary>
    /// カメラ先端（Camera Pole上部、床面から1.00m）における水平方向の合成加速度を
    /// Rigidbody.GetPointVelocity を用いて実測するコンポーネント。
    ///
    /// 設計意図:
    ///  RobotControllerはCOM（重心）の並進加速度を根拠に力をクランプして
    ///  「1.00 m/s^2以下」を機構的に保証しているが、それが実際にカメラ先端でも
    ///  成立しているかを独立に検証するため、あえて別コンポーネントとして
    ///  実測ロジックを分離している（SOLID: SRP、および検証の独立性の確保）。
    ///  GetPointVelocityは剛体の任意の点の瞬間速度を
    ///  v_point = v_com + ω × r （rはCOMから対象点へのベクトル）として計算するため、
    ///  万一ロボットが（本来禁止されている）ピッチ・ロール方向に傾いた場合でも
    ///  正しくカメラ先端の実速度・実加速度を反映する。
    /// </summary>
    public class CameraController : MonoBehaviour
    {
        #region Inspector Fields

        [Header("参照")]
        [Tooltip("カメラが取り付けられているロボット本体のRigidbody")]
        [SerializeField]
        private Rigidbody targetRigidbody;

        [Header("見守りカメラ（任意）")]
        [Tooltip("見守り用のUnity Camera（未設定でも加速度計測は動作する）")]
        [SerializeField]
        private Camera monitoringCamera;

        #endregion

        #region Private State

        private Vector3 _previousHorizontalVelocity;
        private float _previousHorizontalAccelMagnitude;
        private bool _hasPreviousSample;

        #endregion

        #region Public Properties

        /// <summary>カメラ先端における水平合成加速度の実測値 [m/s^2]</summary>
        public float CurrentHorizontalAcceleration { get; private set; }

        /// <summary>カメラ先端における水平方向ジャークの実測値 [m/s^3]</summary>
        public float CurrentHorizontalJerk { get; private set; }

        /// <summary>このセッション内で観測された最大水平合成加速度 [m/s^2]</summary>
        public float MaxObservedHorizontalAcceleration { get; private set; }

        #endregion

        #region Unity Lifecycle

        private void Reset()
        {
            monitoringCamera = GetComponentInChildren<Camera>();
        }

        private void FixedUpdate()
        {
            if (targetRigidbody == null)
            {
                return;
            }

            float dt = Time.fixedDeltaTime;
            if (dt <= 0f)
            {
                return;
            }

            Vector3 pointVelocity = targetRigidbody.GetPointVelocity(transform.position);
            Vector3 horizontalVelocity = new Vector3(pointVelocity.x, 0f, pointVelocity.z);

            if (_hasPreviousSample)
            {
                Vector3 accelVector = (horizontalVelocity - _previousHorizontalVelocity) / dt;
                float accelMagnitude = accelVector.magnitude;

                CurrentHorizontalAcceleration = accelMagnitude;
                CurrentHorizontalJerk = Mathf.Abs(accelMagnitude - _previousHorizontalAccelMagnitude) / dt;

                if (accelMagnitude > MaxObservedHorizontalAcceleration)
                {
                    MaxObservedHorizontalAcceleration = accelMagnitude;
                }

                _previousHorizontalAccelMagnitude = accelMagnitude;
            }
            else
            {
                _hasPreviousSample = true;
            }

            _previousHorizontalVelocity = horizontalVelocity;
        }

        #endregion

        #region Public Methods

        /// <summary>計測状態をリセットする（レース再スタート時に呼ぶ）</summary>
        public void ResetMeasurement()
        {
            _hasPreviousSample = false;
            CurrentHorizontalAcceleration = 0f;
            CurrentHorizontalJerk = 0f;
            MaxObservedHorizontalAcceleration = 0f;
        }

        /// <summary>
        /// 参照するRigidbodyを外部から設定する（RobotGeneratorやSceneInitializerから利用）。
        /// </summary>
        public void SetTargetRigidbody(Rigidbody rigidbody)
        {
            targetRigidbody = rigidbody;
        }

        #endregion
    }
}
