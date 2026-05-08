#pragma once
#include <QByteArray>
#include <QList>
#include <QMap>
#include <QString>
#include <array>

// ============================================================
// Ms901mStreamParser.h — ATK-MS901M 二进制串口协议流式解析器
//
// 帧格式：0x55 0x55 <ID> <LEN> <DATA[LEN]> <CHECKSUM>
// 校验和 = sum(所有字节[除最后一字节]) & 0xFF
//
// 合并输出快照（19 元素 double 数组）索引：
//   [0-2]   ax, ay, az       (m/s², 来自 0x03 帧)
//   [3-5]   gx, gy, gz       (rad/s, 来自 0x03 帧)
//   [6-9]   q0, q1, q2, q3   (来自 0x02 帧，[w,x,y,z])
//   [10-12] mx, my, mz       (原始值, 来自 0x04 帧)
//   [13]    温度             (°C, 来自 0x04 帧)
//   [14-16] roll, pitch, yaw (°, 来自 0x01 帧)
//   [17]    气压             (Pa, 来自 0x05 帧)
//   [18]    海拔             (m, 来自 0x05 帧)
// ============================================================

// 快照数组类型：19 个 double 元素
using Snapshot19 = std::array<double, 19>;

class Ms901mStreamParser {
public:
    // 构造函数：
    //   accFsr  - 加速度计满量程（G），用于量程转换
    //   gyroFsr - 陀螺仪满量程（°/s），用于量程转换
    explicit Ms901mStreamParser(int accFsr = 4, int gyroFsr = 2000);

    // 将原始字节流追加到内部缓冲区，解析所有完整帧
    // 每收到一个 0x03（陀螺仪+加速度）帧时触发一次快照合并输出
    // 返回：本次调用产生的快照列表（可能为空）
    QList<Snapshot19> feed(const QByteArray& data);

    // 将快照格式化为人类可读的调试字符串
    static QString formatDebug(const Snapshot19& snap);

    // 重置内部缓冲区和最新帧状态
    void reset();

private:
    // 帧头字节对
    static constexpr uint8_t HEADER_BYTE = 0x55;

    int m_accFsr;   // 加速度计满量程（G）
    int m_gyroFsr;  // 陀螺仪满量程（°/s）

    QByteArray m_buffer;    // 流式字节缓冲区
    int m_parseOffset = 0;  // 当前解析游标（避免重复 mid）

    // 各帧类型的最新解析结果（按 frame_id 索引）
    struct AttitudeData   { double roll, pitch, yaw; };
    struct QuaternionData { double q0, q1, q2, q3;  };
    struct GyroAccData    { double ax,ay,az,gx,gy,gz; };
    struct MagTempData    { double mx,my,mz,temp;   };
    struct BaroAltData    { double pressure,altitude; };

    bool          m_hasAttitude   = false;
    bool          m_hasQuaternion = false;
    bool          m_hasGyroAcc    = false;
    bool          m_hasMagTemp    = false;
    bool          m_hasBaroAlt    = false;

    AttitudeData   m_attitude{};
    QuaternionData m_quaternion{};
    GyroAccData    m_gyroAcc{};
    MagTempData    m_magTemp{};
    BaroAltData    m_baroAlt{};

    // 低字节在前的有符号 16 位整数转换
    static int16_t toInt16(uint8_t low, uint8_t high);

    // 低字节在前的有符号 32 位整数转换
    static int32_t toInt32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);

    // 从缓冲区头部尝试提取一个完整有效帧
    // 返回 true 表示成功，frame_id 和 payload 通过参数输出
    bool tryExtractFrame(uint8_t& frameId, QByteArray& payload);

    // 根据帧 ID 解析载荷并更新最新帧状态
    // 返回 true 表示触发快照（0x03 帧）
    bool parseFrame(uint8_t frameId, const QByteArray& payload);

    // 解析姿态角帧（ID=0x01，LEN=6）
    bool parseAttitude(const QByteArray& d);

    // 解析四元数帧（ID=0x02，LEN=8）
    bool parseQuaternion(const QByteArray& d);

    // 解析陀螺仪+加速度计帧（ID=0x03，LEN=12）
    bool parseGyroAcc(const QByteArray& d);

    // 解析磁力计+温度帧（ID=0x04，LEN=8）
    bool parseMagTemp(const QByteArray& d);

    // 解析气压计+海拔帧（ID=0x05，LEN=10）
    bool parseBaroAlt(const QByteArray& d);

    // 将各帧最新数据合并为 19 元素快照
    Snapshot19 buildSnapshot() const;
};
