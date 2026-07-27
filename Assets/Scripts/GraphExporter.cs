using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace SimulationRobotContest
{
    /// <summary>
    /// DataLoggerが出力したCSVを読み込み、速度・加速度・ジャークの時系列グラフを
    /// PNG画像として書き出す。外部ライブラリ（matplotlib等）に依存せず、
    /// Unity標準のTexture2D + ImageConversionのみでグラフ描画を完結させている。
    /// </summary>
    public class GraphExporter : MonoBehaviour
    {
        #region Inspector Fields

        [Header("画像サイズ")]
        [SerializeField] private int imageWidth = 800;
        [SerializeField] private int imageHeight = 400;

        [Header("余白（軸ラベル分のスペース）")]
        [SerializeField] private int marginLeft = 60;
        [SerializeField] private int marginBottom = 40;
        [SerializeField] private int marginTop = 20;
        [SerializeField] private int marginRight = 20;

        [Header("色")]
        [SerializeField] private Color backgroundColor = Color.white;
        [SerializeField] private Color axisColor = Color.black;
        [SerializeField] private Color lineColor = new Color(0.1f, 0.4f, 0.9f);
        [SerializeField] private Color gridColor = new Color(0.85f, 0.85f, 0.85f);
        [SerializeField] private Color limitLineColor = new Color(0.9f, 0.1f, 0.1f);

        [Header("加速度上限ライン（課題必須条件: 1.00 m/s^2）")]
        [SerializeField] private float accelerationLimit = 1.0f;

        #endregion

        #region Data Structure

        private struct CsvRow
        {
            public float Time;
            public float Speed;
            public float AngularSpeed;
            public float HorizontalAccel;
            public float Jerk;
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// CSVファイルを読み込み、同じフォルダに速度・加速度・ジャークのPNGグラフを出力する。
        /// </summary>
        /// <param name="csvPath">DataLoggerが出力したCSVファイルの絶対パス</param>
        /// <returns>生成に成功した場合true</returns>
        public bool ExportGraphs(string csvPath)
        {
            if (string.IsNullOrEmpty(csvPath))
            {
                Debug.LogError("[GraphExporter] csvPathがnullまたは空です。");
                return false;
            }

            if (!File.Exists(csvPath))
            {
                Debug.LogError($"[GraphExporter] CSVファイルが見つかりません: {csvPath}");
                return false;
            }

            List<CsvRow> rows;
            try
            {
                rows = ParseCsv(csvPath);
            }
            catch (Exception ex)
            {
                Debug.LogError($"[GraphExporter] CSVの解析に失敗しました: {ex.Message}");
                return false;
            }

            if (rows.Count == 0)
            {
                Debug.LogWarning("[GraphExporter] CSVにデータ行がありません。");
                return false;
            }

            string baseName = Path.GetFileNameWithoutExtension(csvPath);
            string directory = Path.GetDirectoryName(csvPath);
            if (string.IsNullOrEmpty(directory))
            {
                directory = Application.persistentDataPath;
            }

            var speedSeries = new List<Vector2>(rows.Count);
            var accelSeries = new List<Vector2>(rows.Count);
            var jerkSeries = new List<Vector2>(rows.Count);

            foreach (CsvRow row in rows)
            {
                speedSeries.Add(new Vector2(row.Time, row.Speed));
                accelSeries.Add(new Vector2(row.Time, row.HorizontalAccel));
                jerkSeries.Add(new Vector2(row.Time, row.Jerk));
            }

            bool success = true;
            success &= SaveLineGraph(speedSeries, "速度 [m/s]", Path.Combine(directory, $"{baseName}_speed.png"), null);
            success &= SaveLineGraph(accelSeries, "水平合成加速度 [m/s^2]", Path.Combine(directory, $"{baseName}_accel.png"), accelerationLimit);
            success &= SaveLineGraph(jerkSeries, "ジャーク [m/s^3]", Path.Combine(directory, $"{baseName}_jerk.png"), null);

            return success;
        }

        #endregion

        #region CSV Parsing

        private List<CsvRow> ParseCsv(string csvPath)
        {
            var rows = new List<CsvRow>();
            CultureInfo ci = CultureInfo.InvariantCulture;

            using (var reader = new StreamReader(csvPath))
            {
                string headerLine = reader.ReadLine(); // ヘッダ行を読み飛ばす
                if (headerLine == null)
                {
                    return rows;
                }

                string line;
                while ((line = reader.ReadLine()) != null)
                {
                    if (string.IsNullOrWhiteSpace(line))
                    {
                        continue;
                    }

                    string[] fields = line.Split(',');
                    if (fields.Length < 7)
                    {
                        continue; // 不完全な行はスキップ（防御的プログラミング）
                    }

                    rows.Add(new CsvRow
                    {
                        Time = float.Parse(fields[0], ci),
                        Speed = float.Parse(fields[3], ci),
                        AngularSpeed = float.Parse(fields[4], ci),
                        HorizontalAccel = float.Parse(fields[5], ci),
                        Jerk = float.Parse(fields[6], ci)
                    });
                }
            }

            return rows;
        }

        #endregion

        #region Graph Rendering

        /// <summary>
        /// (x, y) の点列から折れ線グラフを描画し、PNGとして保存する。
        /// </summary>
        /// <param name="optionalLimitLine">水平な上限ラインを描く場合はその値、不要ならnull</param>
        private bool SaveLineGraph(List<Vector2> points, string yAxisLabel, string outputPath, float? optionalLimitLine)
        {
            if (points == null || points.Count == 0)
            {
                Debug.LogWarning($"[GraphExporter] 描画データが空です: {outputPath}");
                return false;
            }

            var texture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGBA32, false);
            FillBackground(texture);

            int plotWidth = imageWidth - marginLeft - marginRight;
            int plotHeight = imageHeight - marginTop - marginBottom;

            float minX = points[0].x;
            float maxX = points[0].x;
            float minY = points[0].y;
            float maxY = points[0].y;

            foreach (Vector2 p in points)
            {
                if (p.x < minX) minX = p.x;
                if (p.x > maxX) maxX = p.x;
                if (p.y < minY) minY = p.y;
                if (p.y > maxY) maxY = p.y;
            }

            // 上限ラインが範囲外なら見えるようにY範囲を広げる
            if (optionalLimitLine.HasValue)
            {
                maxY = Mathf.Max(maxY, optionalLimitLine.Value);
            }

            // 全て同じ値の場合にゼロ除算しないよう最小レンジを確保する
            if (Mathf.Approximately(maxX, minX)) maxX = minX + 1f;
            if (Mathf.Approximately(maxY, minY)) maxY = minY + 1f;

            // 上下に少し余白を持たせる
            float yPadding = (maxY - minY) * 0.1f;
            minY -= yPadding;
            maxY += yPadding;
            if (minY > 0f) minY = 0f; // 加速度・速度は0を基準に含める

            DrawGrid(texture, plotWidth, plotHeight);
            DrawAxes(texture, plotWidth, plotHeight);

            if (optionalLimitLine.HasValue)
            {
                DrawLimitLine(texture, optionalLimitLine.Value, minY, maxY, plotWidth, plotHeight);
            }

            DrawDataSeries(texture, points, minX, maxX, minY, maxY, plotWidth, plotHeight);

            texture.Apply();

            byte[] pngBytes = texture.EncodeToPNG();
            UnityEngine.Object.Destroy(texture);

            try
            {
                File.WriteAllBytes(outputPath, pngBytes);
                Debug.Log($"[GraphExporter] グラフを保存しました（{yAxisLabel}）: {outputPath}");
            }
            catch (IOException ex)
            {
                Debug.LogError($"[GraphExporter] PNGの書き込みに失敗しました: {ex.Message}");
                return false;
            }

            return true;
        }

        private void FillBackground(Texture2D texture)
        {
            var pixels = new Color[imageWidth * imageHeight];
            for (int i = 0; i < pixels.Length; i++)
            {
                pixels[i] = backgroundColor;
            }
            texture.SetPixels(pixels);
        }

        private void DrawGrid(Texture2D texture, int plotWidth, int plotHeight)
        {
            const int divisions = 5;
            for (int i = 0; i <= divisions; i++)
            {
                int gx = marginLeft + Mathf.RoundToInt(plotWidth * (i / (float)divisions));
                DrawVerticalLine(texture, gx, marginTop, marginTop + plotHeight, gridColor);

                int gy = marginTop + Mathf.RoundToInt(plotHeight * (i / (float)divisions));
                DrawHorizontalLine(texture, marginLeft, marginLeft + plotWidth, gy, gridColor);
            }
        }

        private void DrawAxes(Texture2D texture, int plotWidth, int plotHeight)
        {
            // X軸（下端）
            DrawHorizontalLine(texture, marginLeft, marginLeft + plotWidth, marginTop + plotHeight, axisColor);
            // Y軸（左端）
            DrawVerticalLine(texture, marginLeft, marginTop, marginTop + plotHeight, axisColor);
        }

        private void DrawLimitLine(Texture2D texture, float limitValue, float minY, float maxY, int plotWidth, int plotHeight)
        {
            float t = Mathf.InverseLerp(minY, maxY, limitValue);
            int py = marginTop + plotHeight - Mathf.RoundToInt(t * plotHeight);
            DrawHorizontalLine(texture, marginLeft, marginLeft + plotWidth, py, limitLineColor);
        }

        private void DrawDataSeries(Texture2D texture, List<Vector2> points,
            float minX, float maxX, float minY, float maxY, int plotWidth, int plotHeight)
        {
            Vector2Int? previousPixel = null;

            foreach (Vector2 p in points)
            {
                float tx = Mathf.InverseLerp(minX, maxX, p.x);
                float ty = Mathf.InverseLerp(minY, maxY, p.y);

                int px = marginLeft + Mathf.RoundToInt(tx * plotWidth);
                int py = marginTop + plotHeight - Mathf.RoundToInt(ty * plotHeight);

                if (previousPixel.HasValue)
                {
                    DrawLine(texture, previousPixel.Value.x, previousPixel.Value.y, px, py, lineColor);
                }

                previousPixel = new Vector2Int(px, py);
            }
        }

        #endregion

        #region Pixel Drawing Primitives

        private void DrawHorizontalLine(Texture2D texture, int x0, int x1, int y, Color color)
        {
            if (y < 0 || y >= imageHeight) return;
            int xa = Mathf.Clamp(Mathf.Min(x0, x1), 0, imageWidth - 1);
            int xb = Mathf.Clamp(Mathf.Max(x0, x1), 0, imageWidth - 1);
            for (int x = xa; x <= xb; x++)
            {
                texture.SetPixel(x, y, color);
            }
        }

        private void DrawVerticalLine(Texture2D texture, int x, int y0, int y1, Color color)
        {
            if (x < 0 || x >= imageWidth) return;
            int ya = Mathf.Clamp(Mathf.Min(y0, y1), 0, imageHeight - 1);
            int yb = Mathf.Clamp(Mathf.Max(y0, y1), 0, imageHeight - 1);
            for (int y = ya; y <= yb; y++)
            {
                texture.SetPixel(x, y, color);
            }
        }

        /// <summary>Bresenhamのアルゴリズムによる線分描画</summary>
        private void DrawLine(Texture2D texture, int x0, int y0, int x1, int y1, Color color)
        {
            int dx = Mathf.Abs(x1 - x0);
            int dy = -Mathf.Abs(y1 - y0);
            int sx = x0 < x1 ? 1 : -1;
            int sy = y0 < y1 ? 1 : -1;
            int err = dx + dy;

            int x = x0;
            int y = y0;
            int safetyCounter = 0;
            const int safetyLimit = 100000; // 無限ループ防止のガード

            while (true)
            {
                if (x >= 0 && x < imageWidth && y >= 0 && y < imageHeight)
                {
                    texture.SetPixel(x, y, color);
                }

                if (x == x1 && y == y1) break;

                int e2 = 2 * err;
                if (e2 >= dy)
                {
                    err += dy;
                    x += sx;
                }
                if (e2 <= dx)
                {
                    err += dx;
                    y += sy;
                }

                safetyCounter++;
                if (safetyCounter > safetyLimit)
                {
                    Debug.LogWarning("[GraphExporter] DrawLineが安全上限に達したため中断しました。");
                    break;
                }
            }
        }

        #endregion
    }
}
