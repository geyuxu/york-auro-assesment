#!/bin/bash
# 保存 Cartographer SLAM 状态（备用方法 - 不依赖 ROS2 daemon）

echo "💾 保存 Cartographer SLAM 状态（备用方法）"
echo "================================"
echo ""

# 配置
ROBOT_NS="${1:-robot1}"
SAVE_DIR="/workspaces/AURO2025/solution/cartographer_states"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
STATE_NAME="${2:-state_${TIMESTAMP}}"

# 创建保存目录
mkdir -p "$SAVE_DIR"

echo "📋 配置信息："
echo "  机器人命名空间: /$ROBOT_NS"
echo "  保存目录: $SAVE_DIR"
echo "  状态名称: $STATE_NAME"
echo ""

PBSTREAM_FILE="$SAVE_DIR/${STATE_NAME}.pbstream"

echo "⚠️  重要提示："
echo "  此方法需要您手动在 Cartographer 运行的终端执行命令"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "请在运行 Cartographer 的终端（终端 2）执行以下命令："
echo ""
echo "ros2 service call /$ROBOT_NS/finish_trajectory \\"
echo "  cartographer_ros_msgs/srv/FinishTrajectory \\"
echo "  \"{trajectory_id: 0}\""
echo ""
echo "sleep 2"
echo ""
echo "ros2 service call /$ROBOT_NS/write_state \\"
echo "  cartographer_ros_msgs/srv/WriteState \\"
echo "  \"{filename: '$PBSTREAM_FILE', include_unfinished_submaps: true}\""
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "执行完成后，按 Enter 继续保存其他信息..."
read

# 检查文件是否已生成
if [ -f "$PBSTREAM_FILE" ]; then
    echo "✅ Cartographer 状态保存成功: $PBSTREAM_FILE"
    FILE_SIZE=$(ls -lh "$PBSTREAM_FILE" | awk '{print $5}')
    echo "   文件大小: $FILE_SIZE"
else
    echo "❌ 未找到 .pbstream 文件"
    echo "   请确保上述命令执行成功"
    exit 1
fi
echo ""

# 2. 保存当前地图（用于可视化）
echo "💾 步骤 2/3: 保存地图图像（用于可视化）..."

MAP_FILE="$SAVE_DIR/${STATE_NAME}_map"

# 使用 Python 直接从话题读取（不依赖 ROS2 CLI）
python3 - <<PYTHON_SCRIPT
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import yaml
import numpy as np
from PIL import Image
import sys

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver_temp')
        self.map_data = None
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/$ROBOT_NS/map',
            self.map_callback,
            10)

    def map_callback(self, msg):
        self.map_data = msg
        self.save_map()
        rclpy.shutdown()

    def save_map(self):
        if self.map_data is None:
            return

        # Save PGM
        width = self.map_data.info.width
        height = self.map_data.info.height
        data = np.array(self.map_data.data).reshape((height, width))

        # Convert to image (0-100 -> 255-0, -1 -> 205)
        img_data = np.zeros((height, width), dtype=np.uint8)
        img_data[data == -1] = 205  # Unknown
        img_data[data >= 0] = ((100 - data[data >= 0]) * 255 / 100).astype(np.uint8)

        img = Image.fromarray(img_data, mode='L')
        img.save('$MAP_FILE.pgm')

        # Save YAML
        yaml_data = {
            'image': '${STATE_NAME}_map.pgm',
            'resolution': float(self.map_data.info.resolution),
            'origin': [
                float(self.map_data.info.origin.position.x),
                float(self.map_data.info.origin.position.y),
                0.0
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }

        with open('$MAP_FILE.yaml', 'w') as f:
            yaml.dump(yaml_data, f)

try:
    rclpy.init()
    saver = MapSaver()
    rclpy.spin_once(saver, timeout_sec=5.0)
    print("✅ 地图图像保存成功")
except Exception as e:
    print(f"⚠️  地图图像保存失败: {e}")
    sys.exit(0)  # 不影响主流程
PYTHON_SCRIPT

echo ""

# 3. 保存元数据
echo "💾 步骤 3/3: 保存元数据..."

METADATA_FILE="$SAVE_DIR/${STATE_NAME}_info.txt"

cat > "$METADATA_FILE" << EOF
Cartographer SLAM 状态保存信息
=====================================

保存时间: $(date)
机器人命名空间: $ROBOT_NS
状态名称: $STATE_NAME
保存方法: 手动方法（备用）

文件列表:
---------
1. SLAM 状态: ${STATE_NAME}.pbstream (主文件)
2. 地图图像: ${STATE_NAME}_map.pgm
3. 地图配置: ${STATE_NAME}_map.yaml
4. 元数据: ${STATE_NAME}_info.txt

如何继续建图:
-----------
bash 加载Cartographer状态.sh $ROBOT_NS $PBSTREAM_FILE
EOF

echo "✅ 元数据保存成功: $METADATA_FILE"
echo ""

# 创建最新状态的符号链接
LATEST_LINK="$SAVE_DIR/latest.pbstream"
if [ -L "$LATEST_LINK" ]; then
    rm "$LATEST_LINK"
fi
ln -s "$PBSTREAM_FILE" "$LATEST_LINK"

echo "🎉 完成！Cartographer 状态已保存"
echo "================================"
echo ""
echo "📂 保存的文件："
ls -lh "$SAVE_DIR" | grep "$STATE_NAME"
echo ""
