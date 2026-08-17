# FAST-LIVO2 Voxel Plane Map 可视化修复说明

## 目标

请基于当前工程：

```text
~/Desktop/fast_livo2_ws
```

修复 FAST-LIVO2 ROS2 版本中 `/planes` 的 Voxel Plane Map 可视化问题。

当前 RViz 报错：

```text
Duplicate Marker Check
Multiple Markers in the same MarkerArray message had the same ns and id: (plane, 0)
```

本次采用**最推荐方案**：

1. 让每一个 `VoxelPlane` 对象在第一次初始化时，无论当前是否被判定为平面，都获得一个**永久唯一 Marker ID**；
2. 保持同一个 `VoxelPlane` 后续使用相同的 `ns + id`；
3. 当该 voxel 从 plane 变成 non-plane 时，通过相同 ID 更新 Marker，而不是创建新的 ID；
4. 将 Marker 生命周期从 `0.01 s` 改为 `0.0 s`，避免 10 Hz LIO 下出现严重闪烁；
5. 不修改 LIO 状态估计、IMU、点云去畸变、Voxel Map 更新或匹配逻辑。

---

# 1. 问题根因

当前文件：

```text
src/FAST-LIVO2/src/voxel_map.cpp
```

中的 `VoxelOctoTree::init_plane()` 只有在点集被判断为平面时才分配唯一 ID：

```cpp
if (evalsReal(evalsMin) < planer_threshold_)
{
    ...
    plane->is_plane_ = true;
    plane->is_update_ = true;

    if (!plane->is_init_)
    {
        plane->id_ = voxel_plane_id;
        voxel_plane_id++;
        plane->is_init_ = true;
    }
}
else
{
    plane->is_update_ = true;
    plane->is_plane_ = false;
}
```

而 `VoxelPlane` 默认：

```cpp
int id_ = 0;
bool is_plane_ = false;
bool is_init_ = false;
bool is_update_ = false;
```

因此 non-plane 节点会出现：

```text
is_update_ = true
is_plane_  = false
id_        = 0
```

但 `GetUpdatePlane()` 又会把所有 `is_update_ == true` 的节点加入发布列表：

```cpp
if (current_octo->plane_ptr_->is_update_)
{
    plane_list.push_back(*current_octo->plane_ptr_);
}
```

于是同一个 `MarkerArray` 中可能出现：

```text
ns = "plane", id = 0
ns = "plane", id = 0
ns = "plane", id = 0
```

从而触发 RViz：

```text
Duplicate Marker Check
```

---

# 2. 推荐修复原则

不要简单过滤掉 non-plane 节点。

推荐：

> 每个 `VoxelPlane` 对象第一次参与 `init_plane()` 时就分配一个永久唯一 ID。

这样无论该节点当前是：

```text
plane
```

还是：

```text
non-plane
```

都拥有唯一的：

```text
ns = "plane"
id = unique_id
```

后续状态变化：

```text
plane
→ non-plane
→ plane
```

始终使用同一个 ID。

这样 RViz 才能正确更新同一个 Marker。

---

# 3. 修改 `VoxelOctoTree::init_plane()`

目标文件：

```text
src/FAST-LIVO2/src/voxel_map.cpp
```

找到：

```cpp
void VoxelOctoTree::init_plane(
    const std::vector<pointWithVar> &points,
    VoxelPlane *plane)
```

## 3.1 将 Marker ID 分配逻辑提前

当前类似：

```cpp
if (evalsReal(evalsMin) < planer_threshold_)
{
    ...
    plane->is_plane_ = true;
    plane->is_update_ = true;

    if (!plane->is_init_)
    {
        plane->id_ = voxel_plane_id;
        voxel_plane_id++;
        plane->is_init_ = true;
    }
}
else
{
    plane->is_update_ = true;
    plane->is_plane_ = false;
}
```

修改为：

```cpp
// Every VoxelPlane owns one persistent and globally unique
// visualization marker ID, regardless of its current plane state.
if (!plane->is_init_)
{
    plane->id_ = voxel_plane_id++;
    plane->is_init_ = true;
}

plane->is_update_ = true;

if (evalsReal(evalsMin) < planer_threshold_)
{
    ...
    plane->is_plane_ = true;
}
else
{
    plane->is_plane_ = false;
}
```

---

# 4. 删除原来的 plane-only ID 分配

原代码中位于：

```cpp
if (evalsReal(evalsMin) < planer_threshold_)
```

内部的：

```cpp
if (!plane->is_init_)
{
    plane->id_ = voxel_plane_id;
    voxel_plane_id++;
    plane->is_init_ = true;
}
```

必须删除。

最终整个函数中只能保留一处：

```cpp
if (!plane->is_init_)
{
    plane->id_ = voxel_plane_id++;
    plane->is_init_ = true;
}
```

---

# 5. 不要修改 `GetUpdatePlane()` 的筛选条件

当前：

```cpp
if (current_octo->plane_ptr_->is_update_)
{
    plane_list.push_back(*current_octo->plane_ptr_);
}
```

推荐保持不变。

不要改成：

```cpp
if (current_octo->plane_ptr_->is_update_ &&
    current_octo->plane_ptr_->is_plane_)
```

原因：

如果一个节点曾经是：

```text
plane
```

然后更新为：

```text
non-plane
```

仍然需要向 RViz 发布同一个 `id`，让旧 Marker 被隐藏或更新。

如果直接过滤 non-plane：

```text
旧 plane Marker
        ↓
节点变成 non-plane
        ↓
不再发送该 ID
        ↓
旧 Marker 可能永久残留
```

所以保留：

```cpp
is_update_
```

作为发布条件更稳妥。

---

# 6. 保留 `pubSinglePlane()` 的 ID 使用方式

当前类似：

```cpp
plane.ns = plane_ns;
plane.id = single_plane.id_;
```

保持不变。

因为修复后：

```text
single_plane.id_
```

已经保证唯一。

最终同一个节点会稳定对应：

```text
ns = "plane"
id = 0

ns = "plane"
id = 1

ns = "plane"
id = 2
```

不会再出现同一条消息内多个：

```text
(plane, 0)
```

---

# 7. 保留 non-plane 的 alpha=0 逻辑

当前 `pubVoxelMap()` 中类似：

```cpp
double alpha;

if (pub_plane_list[i].is_plane_)
{
    alpha = use_alpha;
}
else
{
    alpha = 0;
}
```

保持不变。

这样：

```text
plane
→ alpha = 0.8
→ 正常显示
```

```text
non-plane
→ alpha = 0
→ 使用同一个 ID 更新并隐藏旧 Marker
```

---

# 8. 修改 Marker lifetime

当前 `pubSinglePlane()` 中存在：

```cpp
plane.lifetime =
    rclcpp::Duration::from_seconds(0.01);
```

修改为：

```cpp
plane.lifetime =
    rclcpp::Duration::from_seconds(0.0);
```

推荐增加注释：

```cpp
// Keep the marker alive until another marker with the same
// namespace and ID updates/replaces it.
plane.lifetime = rclcpp::Duration::from_seconds(0.0);
```

原因：当前 MID360/LIO 大约 10 Hz，一帧约 100 ms，而原先 `0.01 s` 只有 10 ms，容易造成 Marker 大部分时间处于消失状态，表现为闪烁或难以观察。

---

# 9. 不要修改以下 LIO 核心代码

本次任务只修可视化。

禁止顺手修改：

```text
StateEstimation()
BuildVoxelMap()
UpdateVoxelMap()
BuildResidualListOMP()
build_single_residual()
IMU_Processing.cpp
Preprocess
LiDAR/IMU 时间同步
LiDAR-IMU 外参
Rcl / Pcl
acc_cov / gyr_cov
sigma_num
voxel_size
min_eigen_value
```

不要修改：

```text
/cloud_registered
/path
/aft_mapped_to_init
```

相关发布链。

当前 LIO 已经可以：

```text
静止稳定
旋转稳定
前后移动稳定
```

因此本次不得改动定位主链。

---

# 10. `/planes` 发布频率暂时不修改

当前：

```cpp
if (voxelmap_manager->config_setting_.is_pub_plane_map_)
{
    voxelmap_manager->pubVoxelMap();
}
```

暂时保持。

本次不要增加：

```cpp
frame_num % 5
```

之类的降频逻辑。

先验证 Marker ID 和 lifetime 修复正确。

如果后续发现 `/planes` 对 CPU 占用明显，再单独做第二次优化，将其降为 `1~2 Hz`。

---

# 11. 编译

修改完成后执行：

```bash
cd ~/Desktop/fast_livo2_ws

source /opt/ros/jazzy/setup.bash

rm -rf build/fast_livo
rm -rf install/fast_livo

colcon build   --packages-select fast_livo   --symlink-install   --cmake-clean-cache
```

成功后：

```bash
source install/setup.bash
```

---

# 12. 启动验证

确认 `livo.yaml` 中：

```yaml
publish:
  pub_plane_en: true
```

然后启动：

```bash
ros2 launch fast_livo mapping.launch.py
```

---

# 13. ROS2 Topic 验证

执行：

```bash
ros2 topic info /planes
```

期望：

```text
Type:
visualization_msgs/msg/MarkerArray

Publisher count: 1
```

然后：

```bash
ros2 topic hz /planes
```

确认持续有数据。

---

# 14. RViz 设置

RViz：

```text
Fixed Frame:
camera_init
```

添加：

```text
Add
→ MarkerArray
```

Topic：

```text
/planes
```

---

# 15. 成功判据

修改成功需要同时满足：

- [ ] `/planes` 能持续收到 `MarkerArray`；
- [ ] RViz 不再出现：
  ```text
  Duplicate Marker Check
  ```
- [ ] 不再出现：
  ```text
  Multiple Markers in the same MarkerArray message had the same ns and id
  ```
- [ ] 每个 Marker 的 `(ns, id)` 唯一；
- [ ] Voxel Plane Map 不再因为 `lifetime = 0.01` 明显闪烁；
- [ ] 旧 plane 变成 non-plane 时不会永久残留；
- [ ] `/cloud_registered` 正常；
- [ ] `/path` 正常；
- [ ] `/aft_mapped_to_init` 正常；
- [ ] LIO 静止、旋转、平移轨迹行为与修改前一致。

---

# 16. 建议增加临时重复 ID 自检

为了验证修复，可以在 `pubVoxelMap()` 发布前临时增加重复 ID 检查。

需要：

```cpp
#include <unordered_set>
```

临时调试代码：

```cpp
std::unordered_set<int> marker_ids;

for (const auto &marker : voxel_plane.markers)
{
    if (!marker_ids.insert(marker.id).second)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("voxel_map"),
            "Duplicate plane marker id detected: %d",
            marker.id);
    }
}
```

如果所有 Marker 都使用同一个：

```text
ns = "plane"
```

那么仅检查 `id` 就足够。

验证通过后，这段代码可以保留或删除。

---

# 17. Agent 执行原则

> 本次采用最小修改原则，仅修复 Voxel Plane Marker 的生命周期与唯一 ID 管理。
>
> 不修改已经稳定工作的 LIO 算法链。
>
> 核心修改只有两项：
>
> 1. 每个 `VoxelPlane` 第一次初始化时就分配永久唯一 `id_`，不再只给 plane 节点分配 ID；
> 2. `plane.lifetime` 从 `0.01 s` 修改为 `0.0 s`。
>
> 完成后重新编译，并通过 `/planes` + RViz 验证 Duplicate Marker 报错消失。
