#!/bin/bash
export DISPLAY=:0
export LANG=ja_JP.utf8
export LC_ALL=ja_JP.utf8
export PYTHONIOENCODING=utf-8
source /opt/ros/humble/setup.bash

echo "=== 1. クリーンアップと初期配置 ==="
killall -9 mate-terminal python3 2>/dev/null || true
rm -f /tmp/mission_done.flag /tmp/demo_grape_harvester.mp4 /tmp/demo_grape_harvester.gif /tmp/ffmpeg.log

# ロボットをスタート地点 (y = -1.8m, 北向き 90度) にリセット
gz model -m happy_mini -x 0.0 -y -1.8 -z 0.01 -R 0.0 -P 0.0 -Y 1.5708
sleep 2

echo "=== 2. 録画開始 (画面全体) ==="
RES=$(xdpyinfo | grep dimensions | awk '{print $2}')
ffmpeg -y -f x11grab -framerate 25 -video_size $RES -i :0 -c:v libx264 -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" /tmp/demo_grape_harvester.mp4 > /tmp/ffmpeg.log 2>&1 &
FFMPEG_PID=$!
sleep 2

echo "=== 3. ターミナル起動と自律摘粒プログラム実行 ==="
# 実行用ラッパースクリプトを作成
cat << 'RUNEOF' > /tmp/run_node.sh
#!/bin/bash
export DISPLAY=:0
export LANG=ja_JP.utf8
export LC_ALL=ja_JP.utf8
export PYTHONIOENCODING=utf-8
source /opt/ros/humble/setup.bash
python3 -u /home/ubuntu/airobot_ws/src/grape_harvester.py
touch /tmp/mission_done.flag
sleep 15
RUNEOF
chmod +x /tmp/run_node.sh

# 右側領域に mate-terminal を起動
mate-terminal --geometry=72x35+692+35 --title="Physical AI - Grape Harvester Node" -e "/tmp/run_node.sh" &

echo "=== 4. ミッション実行中 (完了フラグ待機) ==="
MAX_WAIT=160
ELAPSED=0
while [ $ELAPSED -lt $MAX_WAIT ]; do
    if [ -f /tmp/mission_done.flag ]; then
        echo ">>> ミッション完了フラグを検知しました！余韻待機中 (4秒)..."
        sleep 4
        break
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done

echo "=== 5. 録画停止と正常終了 ==="
kill -INT $FFMPEG_PID || true
wait $FFMPEG_PID 2>/dev/null || true
sleep 1

# ロボット完全停止
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" > /dev/null 2>&1 || true

echo "=== 6. GIFプレビュー自動生成 ==="
ffmpeg -y -i /tmp/demo_grape_harvester.mp4 -vf "fps=6,scale=686:-1:flags=lanczos" /tmp/demo_grape_harvester.gif > /dev/null 2>&1 || true

echo "SUCCESS: 録画・GIF生成が完了しました！"
ls -lh /tmp/demo_grape_harvester.*
