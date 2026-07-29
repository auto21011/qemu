# 编写自定义 i2c 设备指南

本文档教你如何为 QEMU `i2c-custom` 设备编写一个**自己的 i2c 从设备模型**，
并通过 unix socket 与 QEMU 联动。`i2c-custom` 是 ast2600-evb 等 BMC 板
的一类通用挂载点，其行为完全由 Python 端实现（无需改 QEMU 源码、无需
重新编译）。

阅读本文档前请确认你已能跑通仓库里现有的 `i2c_custom_proxy.py` +
`tmp117_model.py` + `selftest.py`（参见同目录 `README.md` 的 Quick
start）。

---

## 1. 整体数据流

```
guest BMC 固件
    │  MMIO 读写 → aspeed_i2c 控制器（主控）
    ▼
i2c-custom  ─┐
    │ 4 字节帧头 + 可选载荷
    ▼  unix socket (chardev socket,server=on)
i2c_custom_proxy.py  ─┐
    │ 按帧切分后调用 device.send/recv
    ▼
你的 Device 子类（如 Ina219Model）

        ▲
        │ 2 字节回复长度 + 回复载荷（≤64 字节）
        │
i2c_custom_proxy.py
```

要点：
- 同一条 socket 连接上的多个请求按 `request → reply` 配对串行；不要在
  Python 端期望自发推送，反向数据流必须由 QEMU 端的 RECV 帧触发。
- QEMU 端的 i2c-custom 设备在 `I2C_FINISH`（master-write 完毕）时发
  SEND；在 `I2C_START_RECV`（master-read 开始）时发 RECV。也就是说
  `send()` 收到的是**一整笔** master-write，`recv()` 收到的是**一次**
  master-read 请求。

---

## 2. 线协议规范（必须对照）

> 与 `include/hw/sensor/i2c-custom.h` 保持镜像。所有多字节字段都按
> **大端**，长度单位是字节。

### 请求（QEMU → proxy）

| offset | 字段         | 取值                                       |
|--------|--------------|--------------------------------------------|
| 0      | `opcode`     | `1=SEND`, `2=RECV`, `3=PING`               |
| 1      | `tx_len`     | `0..64`                                    |
| 2      | `rx_len_max` | `0..64`（仅 RECV 有意义；SEND/PING 固定 0）|
| 3      | `address`    | i2c slave 地址（透传给 Python，仅用于日志）|
| 4..    | `tx_len` payload                                          |

PING 用于 QEMU `realize` 时握手一次；Python 端忽略它即可，但应正常
返回空回复、不要 NAK。

### 回复（proxy → QEMU）

| offset | 字段                 | 取值                              |
|--------|----------------------|-----------------------------------|
| 0..1   | `rx_len` (BE u16)    | 之后紧随的字节数；`0xFFFF` = NAK   |
| 2..    | `rx_len` payload                                       |

- `rx_len > 64`：`i2c_custom_proxy.reply()` 会自动转成 NAK，因此你
  只需返回合理长度的 `bytes`，长度越界由框架兜底。
- 抛出任何异常：框架捕获后会回 NAK（同一帧 QEMU 端会 NAK 整笔 i2c 事务）。
- Python 任何 *主动* 写 socket：不允许。框架只读 `recv()` 的返回值。

> C 端读 socket 时若 1 秒内拿不到完整回复，会把设备置为 `drained`
> 并对所有读写 NAK，直到下一次 `CHR_EVENT_OPENED` 重新接通。可参考
> C 端 `hw/sensor/i2c-custom.c` 的 `i2c_custom_request`。

---

## 3. `Device` 抽象类

```python
from i2c_custom_proxy import Device, register_device

class MySensor(Device):
    def send(self, address: int, payload: bytes) -> int:
        # 实现见下
        ...
    def recv(self, address: int, rx_len_max: int) -> bytes:
        # 实现见下
        ...
    # ping() 默认是 no-op，可不重写
```

三个方法都**同步执行**（socket 帧已经切好分给你，你只需算出答案并
return）。一旦你 `raise Exception`，本帧被框架转换为 NAK。

### 3.1 `send(address, payload) -> int`

- 触发时机：master 完成一次 i2c **写**事务（master 拉地址 + 若干字节
  写入 + STOP），QEMU 端在 I2C_FINISH 时一帧把所有字节传给你。
- `payload` 长度 = 这次写事务里 master 写出的总字节数（含 register
  pointer 等），上限 64 字节，超过会被 QEMU 截断并 NAK。
- 返回值：**目前视为保留位**，固定返回 `0`。QEMU 端 SEND 帧的
  `rx_len_max` 一定是 0，所以你返回的字节不会被消费。（这一区别于
  `tmp117_model.py` 早期文档的描述，即“返回 int rx_len”那段，只是
  描述上的预留，C 端当前不读它。）
- 在 `send()` 中保存好待用状态。例如：master 写完寄存器指针，你的
  `recv()` 后续就会被调用——你可以把 `self.ptr = payload[0]` 缓存
  起来，`recv()` 返回时按 `self.ptr` 决定读取内容。

### 3.2 `recv(address, rx_len_max) -> bytes`

- 触发时机：master 准备一次 i2c **读**事务，QEMU 端在 I2C_START_RECV
  时一次性调用你，预读 64 字节填进 QEMU 的 rx 缓冲；之后 master 取
  几个字节就由 QEMU 内部负责透出。
- `rx_len_max` 固定是 `64`（C 端常量 `I2C_CUSTOM_MAX_XFER`）。回的字节
  数量 <=它即可；超过会被框架转 NAK。
- 返回 `b""` 合法（表示无读取），但**实际写法要小心**：如果回空，
  QEMU rx_buf 留作全 0xFF，正好契合“i2c 读不到设备时返回 0xFF”的
  行为，所以读到“约定 0xFF”是常见协议里的“NACK/无数据”含义。
- 典型模板：

  ```python
  def recv(self, address, rx_len_max):
      return self._read_reg(self.ptr)   # 返回 bytes，长度 ≤ rx_len_max
  ```

### 3.3 `ping(address)` （可选）

- 触发时机：QEMU `realize`（即 `-device i2c-custom,...` 在命令行
  parse 完成后）会发一次 PING。Python 端**空回复**即可。
- 用于你自己做“初始化”或“自检”；不在 ping 里抛异常是惯例。

---

## 4. 最小例子：EEPROM 24C02

下面是一个 256 字节两线串行 EEPROM 的最小实现，便于看清最清晰
的 send/recv 配合。EEPROM 的写时序：`[A2..A0]` 高位是 byte address
的高位、剩余是 7-bit slaveaddr；这里为简单忽略 address 字段，按
单一 256 字节空间实现。

```python
# eeprom_24c02.py
from i2c_custom_proxy import Device, register_device


class Eeprom24C02(Device):
    """24C02: 256B 两线 EEPROM，16-byte page write。"""
    SIZE = 256

    def __init__(self, **_):
        self.mem = bytearray(b"\xff" * self.SIZE)
        self.ptr = 0

    def send(self, address, payload):
        if not payload:
            return 0
        self.ptr = payload[0] % self.SIZE
        if len(payload) > 1:
            # page write：writes wrap within the 16-byte page
            for b in payload[1:]:
                self.mem[self.ptr] = b
                self.ptr = (self.ptr & ~0x0F) | ((self.ptr + 1) & 0x0F)
        return 0

    def recv(self, address, rx_len_max):
        # 顺序读，跨 page 边界也连续
        out = bytearray(rx_len_max)
        for i in range(rx_len_max):
            out[i] = self.mem[self.ptr % self.SIZE]
            self.ptr = (self.ptr + 1) % self.SIZE
        return bytes(out)

    def ping(self, address):
        # 自检：填一段 0xAA、读回、复位
        for i in range(64, 128):
            self.mem[i] = 0xAA
        # 再清回去（不影响后续干净启动）
        for i in range(64, 128):
            self.mem[i] = 0xFF


register_device("EEPROM_24C02", Eeprom24C02)
```

启动：

```bash
python3 i2c_custom_proxy.py /tmp/i2c.sock EEPROM_24C02
```

为这一模型写一个最小自检脚本类似 `selftest.py` 即可：直接构造 client
帧发 `SEND(0x10, 0xAABB)`、`RECV` 读回前 2 字节验证 `0xAA / 0xBB`。

---

## 5. 进阶：状态机式 PMIC（pmic_example.py）

对于时序敏感的设备，建议把每次 SEND 理解为“向状态机喂入事件”，把
状态保存在实例里。下面是 PMIC 框架示意，覆盖“先写命令字 → 后续某
条件返回 telemetry”的模式：

```python
# pmic_example.py
from i2c_custom_proxy import Device, register_device
import struct


class PmicExample(Device):
    """
    示意 PMIC：3 字节命令字 + 1 字节参数；telemetry 16-bit BE。
    """
    CMD_READ_VOUT  = 0x88
    CMD_READ_TEMP  = 0x8A
    CMD_WRITE_SEQ  = 0x10

    def __init__(self, **_):
        self.reg_write_seq = 0
        self.vout_uv = 1200000   # 1.20 V
        self.temp_mc = 25000     # 25.0 C
        self.last_cmd = 0
        self.pending_rx = b""

    def send(self, address, payload):
        if not payload:
            return 0
        cmd = payload[0]
        self.last_cmd = cmd
        if cmd == self.CMD_WRITE_SEQ and len(payload) >= 2:
            self.reg_write_seq = payload[1]
        # PMbus telemetry 模型：master 写 cmd 后会立刻 RECV
        return 0

    def recv(self, address, rx_len_max):
        if self.last_cmd == self.CMD_READ_VOUT:
            return struct.pack(">H", self.vout_uv // 1000)  # mV
        if self.last_cmd == self.CMD_READ_TEMP:
            return struct.pack(">H", self.temp_mc // 10)   # deci-C
        # 不识别 → 返回 0xffff 让 master 看到 NACK-ish
        return b"\xff\xff"

    def ping(self, address):
        # 启动自检：旋一圈状态
        self.last_cmd = 0
        self.pending_rx = b""


register_device("PMIC_EXAMPLE", PmicExample)
```

要点：
- 每个 `send()` 都先记下 `last_cmd`，状态保存到实例里，
  Cruise 在下一次 `recv()` 里消费。
- 返回固定的 0xffff 表示“未知命令”，这样 master 看到的就是 0xFF，
  便于 guest 软件自检“有没有这个寄存器”。

---

## 6. 把你的模型接入代理

有两种风格：

### 6.1 在 `i2c_custom_proxy.main` 的链中 import

修改 `i2c_custom_proxy.py` 末尾几行：

```python
def main(argv):
    ...
    import tmp117_model  # noqa: F401  (registers "TMP117")
    import pmic_example  # noqa: F401  (registers "PMIC_EXAMPLE")
    ...
```

启动就直接用：

```bash
python3 i2c_custom_proxy.py /tmp/i2c.sock PMIC_EXAMPLE
```

### 6.2 用单行 wrapper

不想动 `i2c_custom_proxy.py`：

```python
# serve_pmic.py
import i2c_custom_proxy
import pmic_example
import sys
sys.exit(i2c_custom_proxy.main(["serve_pmic.py",
                                 "/tmp/i2c.sock", "PMIC_EXAMPLE"]))
```

或者一行 shell：

```bash
python3 -c "import i2c_custom_proxy as p; import pmic_example as _; \
import sys; sys.exit(p.main(['prog', '/tmp/i2c.sock', 'PMIC_EXAMPLE']))"
```

---

## 7. 配合 QEMU 与 ast2600-evb

```bash
qemu-system-aarch64 -M ast2600-evb \
  -chardev socket,id=ic0,path=/tmp/i2c.sock,server=on,wait=off \
  -device i2c-custom,address=0x48,chardev=ic0,bus=aspeed.i2c.bus8 \
  -serial mon:stdio -nographic
```

- `address=0x48`：你希望 master 寻址的 7-bit 从机地址，QEMU 端会
  按 i2c 总线的 7-bit 地址做匹配；只有匹配的才会触发本设备的事件。
- `bus=aspeed.i2c.bus8`：Soc 上的 i2c bus 号；`-M ast2600-evb` 一共
  16 条 (`bus0..bus15`)，其他机器若有 i2c 也可以挂。
- `server=on,wait=off`：QEMU 是 socket 服务端，Python 是客户端——
  QEMU 端等待 Python 先连接。如果反过来（Python 先 listen、QEMU 是
  client）则改成 `server=off,reconnect-ms=1000`（去掉 `wait`）。

---

## 8. 自测：让你的模型有 self-test

借鉴同目录 `selftest.py`：直接用 socket 起一个 in-process 代理
+ client，直接 `recv_exact()` 构造帧、走 send/recv 验证模型行为。
关键约定：

- 请求帧：`struct.pack("!BBBB", op, tx_len, rx_max, addr)` 加
  tx payload。
- 回复：`struct.pack("!H", rx_len)` + payload；`rx_len == 0xFFFF`
  是 NAK。
- 不要 zigzag 处理单字节：所有长度都是字节数，不涉及 bit 字段。

把你的 selftest 命名为 `tests/qemu/i2c-custom-proxy/<dev>_selftest.py`，
入口写 `python3 ...selftest.py; echo $?` 退出码，CI 工作流可一并拉。

---

## 9. 调试技巧

- **先离线 selftest** 再进 QEMU。selftest 是纯 socket 模拟，不依赖
  QEMU；39 毫秒并通过则说明模型状态机正确。
- **打开 trace**：QEMU 端加 `-trace i2c_custom_*` 可以看到 opcode、
  帧长等输出。trace events 在 `hw/sensor/trace-events`：
  - `i2c_custom_event_op`
  - `i2c_custom_send`
  - `i2c_custom_recv`
- **保持 send/recv 不阻塞**：所有耗时工作（如等待单独线程、HTTP）
  交给后台线程，send/recv **同步返回**——任何超过 1 秒的返回都会
  被视为 timeout 把设备 drained。
- **状态序列**：master-write 之后不会自动跟着 master-read；这取决
  于 guest 软件行为。不要在 `send()` 里假设“下面一定 recv”。

---

## 10. 同目录参考文件

| 文件                  | 角色                                                     |
|-----------------------|----------------------------------------------------------|
| `i2c_custom_proxy.py` | unix socket 服务端；帧切分、Device 分发、Unicode NAK    |
| `tmp117_model.py`     | TMP117 温度传感器模型实现（register-pointer 风格）      |
| `selftest.py`         | in-process self-test，覆盖 PING/SEND/RECV/NAK 路径      |
| `README.md`           | 用户视角的“我有这台机/怎么挂个 i2c 设备”阅读资料        |
| `i2c-custom-device-development.md`（本文档）| 开发者视角的“怎么写一个新 i2c 设备模型” |

## 11. 上游 i2c-custom 头文件查阅

| 文件                                 | 关键内容                          |
|--------------------------------------|-----------------------------------|
| `include/hw/sensor/i2c-custom.h`     | `I2C_CUSTOM_MAX_XFER = 64`、opcode 常量 |
| `hw/sensor/i2c-custom.c`             | 设备实现；可对照阅读 `i2c_custom_event` 与 `i2c_custom_request` 看出 `send/recv` 的触发时机 |
| `docs/superpowers/specs/2026-07-29-i2c-custom-design.md` | 设计文档             |
