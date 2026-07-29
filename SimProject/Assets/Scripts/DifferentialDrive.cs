using System;
using UnityEngine;

namespace SimulationRobotContest
{
    /// <summary>
    /// 左右2輪差動駆動ロボットの運動学（キネマティクス）を扱うクラス。
    ///
    /// 責務をここに閉じ込めることで（SOLID: SRP）、RobotControllerは
    /// 「PID制御」「Rigidbody操作」に専念でき、差動駆動の幾何学的な変換ロジックは
    /// 単体でテスト・再利用可能になる。
    ///
    /// 座標系: ロボット前方をローカルZ+、右方向をローカルX+とする。
    /// v: 車体重心の前進方向速度 [m/s]
    /// omega: 車体のヨー角速度 [rad/s]（反時計回りを正）
    /// vL, vR: 左右車輪の接地点における周速度（線速度） [m/s]
    /// </summary>
    public class DifferentialDrive
    {
        #region Fields

        private readonly float _wheelTrack;   // 左右車輪間距離 [m]
        private readonly float _wheelRadius;  // 車輪半径 [m]（視覚的な回転表現に使用）

        #endregion

        #region Properties

        public float WheelTrack => _wheelTrack;
        public float WheelRadius => _wheelRadius;

        #endregion

        #region Constructor

        /// <summary>
        /// 差動駆動モデルを生成する。
        /// </summary>
        /// <param name="wheelTrack">左右車輪間距離 [m]。ロボット直径0.30m未満にすること</param>
        /// <param name="wheelRadius">車輪半径 [m]。視覚的な回転速度の算出にのみ使用する</param>
        /// <exception cref="ArgumentOutOfRangeException">パラメータが物理的に不正な場合</exception>
        public DifferentialDrive(float wheelTrack, float wheelRadius)
        {
            if (wheelTrack <= 0.0f)
            {
                throw new ArgumentOutOfRangeException(nameof(wheelTrack), "wheelTrackは正の値である必要があります。");
            }
            if (wheelRadius <= 0.0f)
            {
                throw new ArgumentOutOfRangeException(nameof(wheelRadius), "wheelRadiusは正の値である必要があります。");
            }

            _wheelTrack = wheelTrack;
            _wheelRadius = wheelRadius;
        }

        #endregion

        #region Forward Kinematics (wheel speeds -> body velocity)

        /// <summary>
        /// 左右車輪の線速度から、車体重心の前進速度とヨー角速度を求める（順運動学）。
        /// </summary>
        public void ComputeBodyVelocity(float leftWheelSpeed, float rightWheelSpeed,
            out float linearSpeed, out float angularSpeed)
        {
            linearSpeed = (leftWheelSpeed + rightWheelSpeed) * 0.5f;
            angularSpeed = (rightWheelSpeed - leftWheelSpeed) / _wheelTrack;
        }

        #endregion

        #region Inverse Kinematics (body velocity -> wheel speeds)

        /// <summary>
        /// 車体重心の目標前進速度・目標ヨー角速度から、左右車輪の目標線速度を求める（逆運動学）。
        ///
        /// 特殊なケース:
        ///  ・前進  : linearSpeed &gt; 0, angularSpeed = 0
        ///  ・後退  : linearSpeed &lt; 0, angularSpeed = 0
        ///  ・停止  : linearSpeed = 0, angularSpeed = 0
        ///  ・信地旋回（片輪停止）: 呼び出し側で片方の車輪速度を0に指定したい場合は
        ///                         ComputePivotTurnSpeeds を使うこと
        ///  ・超信地旋回（その場回転）: linearSpeed = 0, angularSpeed ≠ 0
        /// </summary>
        public void ComputeWheelSpeeds(float linearSpeed, float angularSpeed,
            out float leftWheelSpeed, out float rightWheelSpeed)
        {
            float halfTrack = _wheelTrack * 0.5f;
            leftWheelSpeed = linearSpeed - (angularSpeed * halfTrack);
            rightWheelSpeed = linearSpeed + (angularSpeed * halfTrack);
        }

        /// <summary>
        /// 信地旋回（片方の車輪を軸として、もう片方の車輪だけを動かす旋回）の
        /// 左右車輪速度を求める。
        /// </summary>
        /// <param name="wheelSpeed">動かす側の車輪の線速度（符号で前後を指定）</param>
        /// <param name="turnLeft">trueなら左車輪を軸に右車輪を動かす（左旋回）</param>
        public void ComputePivotTurnSpeeds(float wheelSpeed, bool turnLeft,
            out float leftWheelSpeed, out float rightWheelSpeed)
        {
            if (turnLeft)
            {
                leftWheelSpeed = 0.0f;
                rightWheelSpeed = wheelSpeed;
            }
            else
            {
                leftWheelSpeed = wheelSpeed;
                rightWheelSpeed = 0.0f;
            }
        }

        /// <summary>
        /// 超信地旋回（その場でのゼロ半径旋回）の左右車輪速度を求める。
        /// </summary>
        /// <param name="angularSpeed">目標ヨー角速度 [rad/s]（符号で回転方向を指定）</param>
        public void ComputeSpinInPlaceSpeeds(float angularSpeed,
            out float leftWheelSpeed, out float rightWheelSpeed)
        {
            float halfTrack = _wheelTrack * 0.5f;
            leftWheelSpeed = -angularSpeed * halfTrack;
            rightWheelSpeed = angularSpeed * halfTrack;
        }

        #endregion

        #region Visual Helpers

        /// <summary>
        /// 車輪の線速度から、車輪メッシュを見た目上回転させるための角速度 [rad/s] を求める。
        /// 完全に見た目（コスメティック）目的であり、物理的な移動には一切使用しない
        /// （課題必須条件「Transformでの位置変更禁止」は移動・旋回に対する制約であり、
        /// 子オブジェクトである車輪メッシュの回転アニメーションはこれに抵触しない）。
        /// </summary>
        public float LinearSpeedToWheelAngularSpeed(float wheelLinearSpeed)
        {
            return wheelLinearSpeed / _wheelRadius;
        }

        #endregion
    }
}
