# State-based agents

## 結論

目前 `common` image profile 支援的 **16 個 agent 全部都是
state-based**。它們的 driving model 不使用 CARLA RGB／depth／semantic
camera，也不使用 LiDAR 或 radar。執行時的 observation 可以完全由目前
PISA API 傳入的 scenario 與 state 建立，不需要 PISA 額外提供影像或點雲。

可用的完整 `pcla_agent` 名稱如下：

```text
plant2_plant2_0
plant2_plant2_1
plant2_plant2_2

carl_plant_0
carl_plant_1
carl_plant_2
carl_plant_3
carl_plant_4

carl_carl_0
carl_carl_1
carl_carlv11

carl_roach_0
carl_roach_1
carl_roach_2
carl_roach_3
carl_roach_4
```

## 各 family 的 observation

| Family | Agent names | Model 使用的 observation | CARLA sensor actor |
| --- | --- | --- | --- |
| Plant 2.0 | `plant2_plant2_0`, `plant2_plant2_1`, `plant2_plant2_2` | ego state、周遭 actor bounding boxes、route、speed limit | IMU、GNSS；另有 software speedometer。無 camera／LiDAR／radar |
| Plant 1.0 | `carl_plant_0` ～ `carl_plant_4` | ego state、周遭 actor bounding boxes、route tokens | IMU；另有 software speedometer。無 camera／LiDAR／radar |
| CaRL | `carl_carl_0`, `carl_carl_1`, `carl_carlv11` | 由 map、route、ego 與 actor state rasterize 的 semantic BEV，加上 kinematic/control measurements | 無；`sensors()` 回傳空陣列 |
| Roach | `carl_roach_0` ～ `carl_roach_4` | 由 map、route、ego 與 actor state rasterize 的 semantic BEV，加上 kinematic/control measurements | 無；`sensors()` 回傳空陣列 |

這裡的 semantic BEV 是由 simulator state 畫出的結構化 raster，**不是**
camera 影像，也不是 LiDAR point cloud。

## PISA API 資訊如何成為 observation

Wrapper 使用的資料來源是：

- `ScenarioPackData`：map／OpenDRIVE 與 route。
- `ObservationData.ego`：ego 的 pose、yaw、speed、acceleration、shape 等 state。
- `ObservationData.agents`：其他 road users 的類型、pose、yaw、speed、shape
  與可選的 tracking ID。
- step timestamp，以及 wrapper 前一步送出的 control（需要 temporal/control
  history 的 model 會使用）。

每個 step，wrapper 會把 PISA observation 同步到 shadow CARLA world：ego 與
其他 observed agents 會被更新成對應的 CARLA actor，route/map 則在 reset 時
建立。Agent 隨後從這些 actor/map state 產生 object tokens 或 semantic BEV。
因此 perception observation 的資訊來源仍是 PISA API，而不是 CARLA
camera／LiDAR 的量測。

Plant 1.0／2.0 所列的 IMU、GNSS 與 speedometer 只用來取得 ego kinematics：
它們的值由已同步的 ego actor state 產生。Plant 2.0 程式中雖保留一個
visualization RGB camera 選項，但 PCLA registry 會將 `PLANT_VIZ` 設成空字串；
Plant 1.0 的 common evaluation configs 也將 `visualize` 設為 `false`。所以這些
camera 在目前 common runtime 不會建立，也不是 model input。

## 重要邊界：state-based 不等於 CARLA-free

以上 agent 可以完全依賴 PISA API 所帶的資訊形成 observation，但目前的
實作仍然需要 shadow CARLA runtime，不能直接移除 CARLA。原因包括：

- agent code 會透過 CARLA actor/world API 查詢 pose、velocity、bounding box
  與附近 actors；
- route planner、road geometry、speed limit、traffic light／stop sign 等邏輯會
  查詢 CARLA map/world；
- Plant 的 IMU／GNSS 與 software speedometer 是由同步後的 ego actor 產生；
- CaRL／Roach 的 semantic BEV 是由 CARLA map 與 actor state rasterize 出來。

所以正確的能力描述是：

> 這 16 個 agent 不需要 visual、LiDAR 或 radar perception input；PISA 現有的
> scenario/state API 已足以提供它們所需的資訊，但 wrapper 目前仍以 shadow
> CARLA 作為 state adapter、地圖查詢與 observation builder。

## 不在此名單中的 agents

`PCLA/agents.json` 還包含 TransFuser、LAV、LBC、World-on-Rails、LMDrive、
SimLingo、NEAT、InterFuser 等 upstream entries。它們不是目前 `common` image
profile 支援的 agents，而且其 driving pipeline 會使用 camera、LiDAR 或其他
sensor perception input，因此不能只靠目前 PISA state observation 直接執行。

