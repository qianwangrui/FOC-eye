# 工具链手册

> Linux (Ubuntu) 环境下用 VS Code / Windsurf + CMSIS-DAP 调试器开发 STM32G431RB。
> 本文档记录所有工具、配置、常用命令，以及开发过程中踩过的坑和解决办法。

---

## 1. 硬件 / 软件概览

| 项 | 值 |
|------|------|
| MCU | STM32G431RBT6 (Cortex-M4, 170 MHz) |
| 调试器 | CMSIS-DAP_LU |
| 主机 OS | Ubuntu Linux |
| 编辑器 | VS Code / Windsurf（推荐，我用的这个） |
| 调试插件 | `Cortex-debug` |
| 构建系统 | GNU Make |

---

## 2. 工具链安装

### 2.1 ARM GCC 交叉编译器

```bash
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi
# 验证
arm-none-eabi-gcc --version    # 应为 10.3.1 或更新
```

### 2.2 GDB

```bash
sudo apt install gdb-multiarch
```

### 2.3 OpenOCD — **必须装 0.12+，不要用 apt 默认的 0.11！**

系统 apt 里的 OpenOCD 0.11 对 CMSIS-DAPv2 支持有 bug（见 §6 踩坑记录）。必须用 xPack 预编译版本。

```bash
cd /tmp
wget https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.12.0-2/xpack-openocd-0.12.0-2-linux-x64.tar.gz
tar xf xpack-openocd-0.12.0-2-linux-x64.tar.gz
sudo mv xpack-openocd-0.12.0-2 /opt/openocd-0.12
# 验证
/opt/openocd-0.12/bin/openocd --version    # 应显示 xPack 0.12.0+
```

安装后的路径（项目硬编码依赖）：

```
/opt/openocd-0.12/bin/openocd
```

### 2.4 ST-Link 工具（备用烧录，可选）

```bash
sudo apt install stlink-tools       # st-flash 命令
# 或 STM32CubeProgrammer CLI（ST 官方）
```

---

## 3. Git 配置

### 3.1 首次配置（全局）

```bash
git config --global user.name  "qianwangrui"
git config --global user.email "1457263737@qq.com"
git config --global core.editor "vim"           # 或 nano / code --wait
git config --global init.defaultBranch main
git config --global pull.rebase false           # merge 策略
```

### 3.2 SSH Key（与 GitHub 通信）

```bash
# 生成
ssh-keygen -t ed25519 -C "1457263737@qq.com"   # 一路回车
# 查看公钥（贴到 GitHub → Settings → SSH keys）
cat ~/.ssh/id_ed25519.pub
# 测试
ssh -T git@github.com                           # 看到 "Hi qianwangrui!" 即成功
```

### 3.3 本仓库当前配置

```
remote origin : 填写你在github上面的仓库地址（推荐SSH格式）
default branch: main
user.name     : qianwangrui
user.email    : 1457263737@qq.com
```

### 3.4 常用命令速查

| 场景 | 命令 |
|------|------|
| 克隆仓库 | `git clone git@github.com:qianwangrui/FOC-eye.git` |
| 查看状态 | `git status` |
| 查看改动 | `git diff` / `git diff --staged` |
| 暂存文件 | `git add <file>` / `git add .` |
| 提交 | `git commit -m "feat: xxx"` |
| 推送 | `git push` |
| 拉取更新 | `git pull` |
| 查看历史 | `git log --oneline --graph -n 20` |
| 撤销未暂存改动 | `git restore <file>` |
| 取消暂存 | `git restore --staged <file>` |
| 回滚最近一次提交（保留改动） | `git reset --soft HEAD~1` |
| 创建分支 | `git switch -c feature/xxx` |
| 切换分支 | `git switch main` |
| 合并分支 | `git merge feature/xxx` |
| 打标签 | `git tag -a v0.1 -m "first working"` |
| 推标签 | `git push --tags` |

### 3.5 推荐 commit message 风格

```
feat:     新功能
fix:      修 bug
refactor: 重构
docs:     文档
chore:    杂项/构建
perf:     性能优化
```

示例：

```
feat(MT6701): add SSI frame parser with status/CRC
fix(main):    remove stale MT6701_GPIO_Init call
```

---

## 4. 项目结构

```
FOC_eye/
├── Src/                        应用源文件
│   ├── main.c
│   ├── qwr_FOC_peri_init.c    PWM / 时钟 / GPIO 等外设
│   ├── qwr_MT6701_driver.c    磁编码器驱动（SSI）
│   └── qwr_INA240_driver.c    电流采样驱动（ADC）
├── Inc/                        头文件
├── Drivers/                    STM32 HAL / CMSIS
├── Startup/                    startup_stm32g431xx.s
├── build/                      构建产物（.o .elf .bin .hex）
├── STM32G431RBTX_FLASH.ld      链接脚本
├── Makefile
├── .vscode/
│   ├── launch.json             调试配置
│   ├── tasks.json              构建任务
│   └── STM32G431xx.svd         寄存器定义(去官网可以下载)
└── docs/
    └── TOOLCHAIN.md            （本文档）
```

---

## 5. 编译与烧录

### 5.1 Makefile 目标

| 命令 | 作用 |
|------|------|
| `make` | 编译，输出 `build/FOC_eye.elf / .hex / .bin` |
| `make clean` | 删除 `build/` 目录 |
| `make flash` | 用 `st-flash` 烧录（需 ST-Link） |
| `make flash-cube` | 用 `STM32_Programmer_CLI` 烧录 |
| `make flash-ocd` | 用 OpenOCD + ST-Link 烧录 |
| **`make flash-dap`** | **用 OpenOCD 0.12 + CMSIS-DAP 烧录（当前主流程）** |

### 5.2 典型工作流

```bash
make            # 编译
make flash-dap  # 烧录
```

### 5.3 OpenOCD 路径变量

Makefile 里固定使用：

```makefile
OPENOCD = /opt/openocd-0.12/bin/openocd
```

如果装在其他地方，改这一行。

---

## 6. 调试（VS Code / Windsurf + Cortex-Debug）

调试需要在项目根目录下创建 `.vscode/` 文件夹，里面放 3 个文件：

| 文件 | 作用 |
|------|------|
| `launch.json` | 调试启动配置（按 F5 时读这个） |
| `tasks.json` | 任务定义（编译命令、调试前的准备工作） |
| `STM32G431xx.svd` | 芯片寄存器定义文件，调试时用来在"外设 (Peripherals)"窗口直接看寄存器 |

### 6.1 `launch.json` —— 调试启动配置

完整内容（直接复制到 `.vscode/launch.json`）：

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "DAPLink Debug",
            "type": "cortex-debug",
            "request": "launch",
            "servertype": "openocd",
            "cwd": "${workspaceFolder}",
            "executable": "${workspaceFolder}/build/FOC_eye.elf",
            "configFiles": [
                "interface/cmsis-dap.cfg",
                "target/stm32g4x.cfg"
            ],
            "serverpath": "/opt/openocd-0.12/bin/openocd",
            "svdFile": "${workspaceFolder}/.vscode/STM32G431xx.svd",
            "runToEntryPoint": "main",
            "breakAfterReset": true,
            "preLaunchTask": "build",
            "device": "STM32G431RB",
            "gdbPath": "gdb-multiarch",
            "toolchainPrefix": "arm-none-eabi",
            "objdumpPath": "/usr/bin/arm-none-eabi-objdump"
        }
    ]
}
```

字段含义：

| 字段 | 说明 | 你要不要改 |
|------|------|----------|
| `name` | 调试配置的显示名（左上角下拉框看到的） | 随意 |
| `type` | 必须是 `cortex-debug` | 不改 |
| `request` | `launch` = 烧录后启动；`attach` = 附加到正在运行的程序 | 一般 `launch` |
| `servertype` | GDB Server 类型：`openocd`/`jlink`/`stutil`/`pyocd` 等 | 用 OpenOCD 就 `openocd` |
| `executable` | 编译产物 .elf 路径 | 改成你项目的名字 |
| `configFiles` | OpenOCD 加载的两个 .cfg：调试器接口 + 目标芯片 | 见下表 |
| `serverpath` | OpenOCD 可执行文件绝对路径 | **必须**改成你装 OpenOCD 的路径 |
| `svdFile` | 寄存器定义文件路径 | 改成你芯片对应的 SVD |
| `runToEntryPoint` | 烧录完自动跑到此函数停下，通常 `main` | 不改 |
| `breakAfterReset` | 复位后停在第一条指令 | `true` 方便调试 |
| `preLaunchTask` | 启动调试前先跑哪个 task（在 `tasks.json` 里定义） | 通常 `build` |
| `device` | 芯片型号（决定外设窗口的过滤） | 改成你的芯片 |
| `gdbPath` | GDB 可执行文件 | Linux 用 `gdb-multiarch` |
| `toolchainPrefix` | 工具链前缀 | 不改 |

`configFiles` 怎么选：

| 调试器 | interface 配置 |
|--------|---------------|
| ST-Link | `interface/stlink.cfg` |
| CMSIS-DAP | `interface/cmsis-dap.cfg` |
| J-Link | `interface/jlink.cfg` |

| 芯片系列 | target 配置 |
|---------|------------|
| STM32G4x | `target/stm32g4x.cfg` |
| STM32F1x | `target/stm32f1x.cfg` |
| STM32F4x | `target/stm32f4x.cfg` |
| STM32H7x | `target/stm32h7x.cfg` |

> 这些 .cfg 是 OpenOCD 自带的，不用自己写，直接引用即可。

### 6.2 `tasks.json` —— 任务定义

完整内容（`.vscode/tasks.json`）：

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "build",
            "type": "shell",
            "command": "make",
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "problemMatcher": {
                "owner": "gcc",
                "fileLocation": ["relative", "${workspaceFolder}"],
                "pattern": {
                    "regexp": "^(.*):(\\d+):(\\d+):\\s+(warning|error):\\s+(.*)$",
                    "file": 1,
                    "line": 2,
                    "column": 3,
                    "severity": 4,
                    "message": 5
                }
            }
        }
    ]
}
```

字段含义：

| 字段 | 说明 |
|------|------|
| `label` | 任务名（被 `launch.json` 的 `preLaunchTask` 引用） |
| `type: shell` | 在 shell 里执行命令 |
| `command: make` | 实际跑的命令 |
| `group.isDefault: true` | 按 `Ctrl+Shift+B` 默认跑这个 |
| `problemMatcher` | 用正则解析编译输出，把 warning/error 显示到 VS Code "问题"面板，可点击跳转 |

### 6.3 `STM32G431xx.svd` —— 芯片寄存器定义

**SVD 是什么**：System View Description，一份 XML 文件，描述芯片所有外设寄存器的地址、位域、含义。Cortex-Debug 加载它后，可以**展开 GPIO/SPI/TIM 等外设直接看每个寄存器的值**，并能看到每一位的名字含义。**调试硬件 bug 神器**。

**下载方法**：

1. 进 ST 官网 [Keil MDK Pack](https://www.keil.arm.com/packs/) 或 [GitHub cmsis-svd 仓库](https://github.com/cmsis-svd/cmsis-svd-data)
2. 找你芯片对应的 .svd 文件（例如 `STM32G431xx.svd`）
3. 放到 `.vscode/STM32G431xx.svd`
4. 在 `launch.json` 里 `svdFile` 字段引用它

**或者从 ST CubeMX 安装目录提取**：

```bash
# CubeMX 安装后，SVD 在这里：
find ~/STM32CubeMX -name "*.svd" 2>/dev/null
find /opt/st -name "STM32G4*.svd" 2>/dev/null
# 找到后直接复制到 .vscode/
```

**或者用 GitHub 直接下**（推荐，最快）：

```bash
mkdir -p .vscode
wget -O .vscode/STM32G431xx.svd \
  https://raw.githubusercontent.com/cmsis-svd/cmsis-svd-data/main/data/STMicro/STM32G431xx.svd
```

> SVD 不是必须的，没有它也能调试，只是看不到寄存器视图。

### 6.4 启动调试

1. 插好 CMSIS-DAP，确认电源
2. 打开任意 `.c` 文件，按 **F5**
3. 第一次会先跑 `preLaunchTask` 里的 `build`（自动 make）
4. 编译成功后启动 OpenOCD → GDB → 烧录 → 停在 `main`
5. F10 单步、F11 步入、F5 继续、Shift+F5 停止

调试视图能看到：
- 左侧 **变量** 面板（local/global）
- 左侧 **XPERIPHERALS** 面板（如果配了 SVD）
- 左侧 **寄存器** 面板（CPU 寄存器 R0~R15、xPSR）
- 底部 **调试控制台**（可以输入 GDB 命令，如 `print x` `info registers`）

---

## 7. 踩坑记录

### 坑：UART 输出乱码 —— `HSE_VALUE` 与实际晶振不匹配

**现象**：printf 重定向到 USART1，打开 tio/minicom 看到大片 `???? gE??Oe??` 这种乱码。

**根因**：`Inc/stm32g4xx_hal_conf.h` 默认 `HSE_VALUE = 24000000UL`，但板上实际晶振是 **8 MHz**。HAL 用 `HSE_VALUE` 计算 PCLK2，进而算 UART BRR。结果实际波特率 = 设定波特率 ×（实际晶振 / HSE_VALUE）。
- 代码设 115200，实际跑成 `115200 × 8/24 ≈ 38400`
- PC 用 115200 解 38400 的码流 → 全是乱码

**诊断方法**：在 PC 端依次试 `make serial SERIAL_BAUD=38400` / `9600` / `19200`，哪个能解出清楚字符就反推实际时钟比例。

**修复**：
```c
// Inc/stm32g4xx_hal_conf.h
#define HSE_VALUE    (8000000UL)   // 改成你板子实际晶振值
```

**经验**：
1. 任何"接错就乱码"问题，**第一反应是查时钟链**（HSE_VALUE / PLL / PCLK / 外设时钟源），而不是怀疑 USB-TTL 转换器
2. STM32G4 系列默认 `HSE_VALUE` 是 24 MHz（Nucleo 板），自制板/小板大多是 8 MHz，要改
3. UART 出问题时，先在 PC 上**遍历常见波特率**比改代码快得多

**为什么 PWM 之前一直是对的，UART 才暴露这个 bug？**

| 类型 | 依赖 `HSE_VALUE`？ | 例子 |
|------|------------------|------|
| **纯硬件分频** | ❌ 不依赖 | TIM PWM、ADC 采样、PLL 倍频本身 |
| **软件算分频** | ✅ 依赖 | UART BRR、`HAL_Delay`、I2C、SysTick |

PLL 硬件直接对 HSE 引脚的真实 8 MHz 做倍频 → 实际 SYSCLK 是正确的 170 MHz；TIM 也只是数真实时钟的脉冲，所以 PWM 频率不受影响。但 HAL 用错误的 `HSE_VALUE` 计算 `SystemCoreClock = 510 MHz`，所有靠这个变量算出来的分频值都偏 3×。所以改之前 `HAL_Delay(100)` 实际等了 300 ms（你看不出来，因为没有参照），UART 波特率掉到 38400。

---

## 8. 常用命令速查

### 编译 / 烧录

```bash
make                      # 编译
make clean && make        # 全量编译
make flash-dap            # 烧录（CMSIS-DAP + OpenOCD 0.12）
make flash-cube           # 烧录（STM32CubeProgrammer）
```

### 手动调试 OpenOCD 连接

```bash
# 测试能否连上芯片（不进入 GDB）
/opt/openocd-0.12/bin/openocd \
    -f interface/cmsis-dap.cfg \
    -f target/stm32g4x.cfg \
    -c "init; halt; exit"
```

### 手动用 GDB 调试

```bash
# 终端 A：启 OpenOCD
/opt/openocd-0.12/bin/openocd \
    -f interface/cmsis-dap.cfg \
    -f target/stm32g4x.cfg

# 终端 B：连 GDB
gdb-multiarch build/FOC_eye.elf
(gdb) target extended-remote :3333
(gdb) monitor reset halt
(gdb) load
(gdb) break main
(gdb) continue
```

### 查看 USB 调试器

```bash
lsusb | grep c251                               # 确认 CMSIS-DAP 插上
ls /sys/bus/usb/drivers/cdc_acm/ | grep :       # 看有没有被 cdc_acm 占
dmesg | tail -20                                # 看内核日志
```

### 查看编译产物大小

```bash
arm-none-eabi-size build/FOC_eye.elf
```

### 查看汇编 / map

```bash
less build/FOC_eye.map                          # 链接布局
less build/main.lst                             # 带汇编的源码清单
arm-none-eabi-objdump -d build/FOC_eye.elf | less
```

---

## 9. 快速上手（新机器 / 新人）

```bash
# 1. 装工具
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi gdb-multiarch

# 2. 装 OpenOCD 0.12（重要！）
cd /tmp && wget https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.12.0-2/xpack-openocd-0.12.0-2-linux-x64.tar.gz
tar xf xpack-openocd-0.12.0-2-linux-x64.tar.gz
sudo mv xpack-openocd-0.12.0-2 /opt/openocd-0.12

# 3. 配置 Git
git config --global user.name  "qianwangrui"
git config --global user.email "1457263737@qq.com"

# 4. 克隆项目
git clone git@github.com:qianwangrui/FOC-eye.git
cd FOC-eye

# 5. 编译 + 烧录
make
make flash-dap

# 6. VS Code 打开项目，F5 调试
```


---

**最后更新**：2026-05-02
