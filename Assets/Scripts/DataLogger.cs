using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace SimulationRobotContest
{
    /// <summary>
    /// FixedUpdate毎に「時間・位置・速度・角速度・加速度・ジャーク」を記録し、
    /// CSVファイルへ保存するロガー。
    ///
    /// CSV列構成（GraphExporterと合意している固定フォーマット）:
    ///   time_s, pos_x_m, pos_z_m, speed_mps, angular_speed_rads, horizontal_accel_mps2, jerk_mps3
    /// </summary>
    public class DataLogger : MonoBehaviour
    {
        #region Constants

        private const string CsvHeader =
            "time_s,pos_x_m,pos_z_m,speed_mps,angular_speed_rads,horizontal_accel_mps2,jerk_mps3";

        private const string LogDirectoryName = "SimulationLogs";

        #endregion

        #region Inspector Fields

        [Header("出力設定")]
        [Tooltip("出力ファイル名のプレフィックス")]
        [SerializeField]
        private string fileNamePrefix = "run_log";

        #endregion

        #region Private State

        private StreamWriter _writer;
        private string _currentCsvPath;
        private bool _isLogging;

        #endregion

        #region Public Properties

        /// <summary>現在ログ記録中かどうか</summary>
        public bool IsLogging => _isLogging;

        /// <summary>直近に書き出したCSVファイルの絶対パス（未生成ならnull）</summary>
        public string LastCsvPath { get; private set; }

        /// <summary>記録したサンプル数</summary>
        public int SampleCount { get; private set; }

        #endregion

        #region Public Methods

        /// <summary>
        /// ログ記録を開始する。既存のログが開いていれば安全に閉じてから開始する。
        /// </summary>
        public void BeginLogging()
        {
            try
            {
                if (_isLogging)
                {
                    EndLogging();
                }

                string directory = GetLogDirectory();
                Directory.CreateDirectory(directory);

                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
                _currentCsvPath = Path.Combine(directory, $"{fileNamePrefix}_{timestamp}.csv");

                _writer = new StreamWriter(_currentCsvPath, append: false, encoding: Encoding.UTF8);
                _writer.WriteLine(CsvHeader);
                _writer.Flush();

                _isLogging = true;
                SampleCount = 0;

                Debug.Log($"[DataLogger] ログ記録を開始しました: {_currentCsvPath}");
            }
            catch (IOException ex)
            {
                Debug.LogError($"[DataLogger] ログファイルの作成に失敗しました: {ex.Message}");
                _isLogging = false;
            }
            catch (UnauthorizedAccessException ex)
            {
                Debug.LogError($"[DataLogger] ログファイルへの書き込み権限がありません: {ex.Message}");
                _isLogging = false;
            }
        }

        /// <summary>
        /// 1サンプル分のデータを記録する。RobotControllerのFixedUpdateから毎回呼び出す。
        /// </summary>
        public void LogSample(float time, Vector3 position, float speed, float angularSpeed,
            float horizontalAccel, float jerk)
        {
            if (!_isLogging || _writer == null)
            {
                return;
            }

            try
            {
                CultureInfo ci = CultureInfo.InvariantCulture;
                string line = string.Join(",",
                    time.ToString("F4", ci),
                    position.x.ToString("F4", ci),
                    position.z.ToString("F4", ci),
                    speed.ToString("F4", ci),
                    angularSpeed.ToString("F4", ci),
                    horizontalAccel.ToString("F4", ci),
                    jerk.ToString("F4", ci));

                _writer.WriteLine(line);
                SampleCount++;
            }
            catch (IOException ex)
            {
                Debug.LogError($"[DataLogger] サンプルの書き込みに失敗しました: {ex.Message}");
            }
        }

        /// <summary>
        /// ログ記録を終了し、ファイルをフラッシュ・クローズする。
        /// </summary>
        public void EndLogging()
        {
            if (!_isLogging)
            {
                return;
            }

            try
            {
                _writer?.Flush();
                _writer?.Close();
                LastCsvPath = _currentCsvPath;
                Debug.Log($"[DataLogger] ログ記録を終了しました（{SampleCount}サンプル）: {LastCsvPath}");
            }
            catch (IOException ex)
            {
                Debug.LogError($"[DataLogger] ログファイルのクローズに失敗しました: {ex.Message}");
            }
            finally
            {
                _writer = null;
                _isLogging = false;
            }
        }

        #endregion

        #region Private Methods

        private string GetLogDirectory()
        {
            return Path.Combine(Application.persistentDataPath, LogDirectoryName);
        }

        #endregion

        #region Unity Lifecycle

        private void OnDestroy()
        {
            // アプリ終了・シーン破棄時にファイルハンドルを確実に解放する
            if (_isLogging)
            {
                EndLogging();
            }
        }

        private void OnApplicationQuit()
        {
            if (_isLogging)
            {
                EndLogging();
            }
        }

        #endregion
    }
}
