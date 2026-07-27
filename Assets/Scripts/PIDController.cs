using UnityEngine;

namespace SimulationRobotContest
{
    /// <summary>
    /// 汎用PID制御器。
    /// 速度制御・角速度制御・車輪速度制御など、あらゆる1入力1出力のフィードバック制御に
    /// 再利用できるよう、Unity非依存（MonoBehaviourに依存しない）な設計にしている（SOLID: SRP/DIP）。
    /// [System.Serializable] を付与しているため、他のMonoBehaviourの[SerializeField]として
    /// 埋め込むことでInspector上からゲインを調整できる。
    /// </summary>
    [System.Serializable]
    public class PIDController
    {
        #region Inspector Fields

        [Header("PIDゲイン")]
        [Tooltip("比例ゲイン")]
        [SerializeField]
        private float proportionalGain = 1.0f;

        [Tooltip("積分ゲイン")]
        [SerializeField]
        private float integralGain = 0.0f;

        [Tooltip("微分ゲイン")]
        [SerializeField]
        private float derivativeGain = 0.0f;

        [Header("出力制限")]
        [Tooltip("積分項の絶対値上限（ワインドアップ防止）")]
        [SerializeField]
        private float integralClampValue = 10.0f;

        [Tooltip("出力の絶対値上限。0以下を指定した場合は無制限として扱う")]
        [SerializeField]
        private float outputClampValue = 0.0f;

        #endregion

        #region Private State

        private float _integralSum;
        private float _previousError;
        private bool _hasPreviousError;

        #endregion

        #region Public Properties

        public float ProportionalGain
        {
            get => proportionalGain;
            set => proportionalGain = value;
        }

        public float IntegralGain
        {
            get => integralGain;
            set => integralGain = value;
        }

        public float DerivativeGain
        {
            get => derivativeGain;
            set => derivativeGain = value;
        }

        public float IntegralClampValue
        {
            get => integralClampValue;
            set => integralClampValue = Mathf.Max(0f, value);
        }

        public float OutputClampValue
        {
            get => outputClampValue;
            set => outputClampValue = value;
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// 内部状態（積分項・前回誤差）をリセットする。
        /// レース再スタート時や、目標値が不連続に切り替わった直後に呼ぶこと。
        /// </summary>
        public void ResetState()
        {
            _integralSum = 0.0f;
            _previousError = 0.0f;
            _hasPreviousError = false;
        }

        /// <summary>
        /// PID制御出力を計算する。
        /// </summary>
        /// <param name="error">誤差（目標値 - 現在値）</param>
        /// <param name="deltaTime">前回呼び出しからの経過時間 [s]（FixedUpdateではTime.fixedDeltaTimeを渡す）</param>
        /// <returns>制御出力（力・トルク等、呼び出し側が意味を定義する）</returns>
        public float ComputeOutput(float error, float deltaTime)
        {
            // deltaTimeが不正な場合は演算しない（ゼロ除算・発散防止の例外的ガード）
            if (deltaTime <= 0.0f)
            {
                return 0.0f;
            }

            _integralSum += error * deltaTime;

            if (integralClampValue > 0.0f)
            {
                _integralSum = Mathf.Clamp(_integralSum, -integralClampValue, integralClampValue);
            }

            float derivative = 0.0f;
            if (_hasPreviousError)
            {
                derivative = (error - _previousError) / deltaTime;
            }

            _previousError = error;
            _hasPreviousError = true;

            float output =
                (proportionalGain * error) +
                (integralGain * _integralSum) +
                (derivativeGain * derivative);

            if (outputClampValue > 0.0f)
            {
                output = Mathf.Clamp(output, -outputClampValue, outputClampValue);
            }

            return output;
        }

        #endregion
    }
}
